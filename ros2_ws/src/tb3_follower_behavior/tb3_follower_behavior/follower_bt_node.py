"""ROS 2 node that runs the follower behavior tree."""
from __future__ import annotations

import time

import py_trees
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tb3_follower_msgs.msg import PersonDetection

from tb3_follower_behavior.behaviours.helpers import (
    KEY_DETECTION,
    KEY_LAST_SEEN_TIME,
    KEY_SCAN,
    TwistPublisher,
)
from tb3_follower_behavior.control import ControlParams, ObstacleParams
from tb3_follower_behavior.tree import build_tree


# Map the active behaviour-tree leaf (tip) name -> a compact state label that the
# dashboard understands. Published on /follower/state each tick.
_TIP_TO_STATE = {
    "AvoidObstacle": "avoid",
    "TurnToFace": "orient",
    "Approach": "approach",
    "HoldPosition": "hold",
    "Stop": "stop",
    "RotateInPlace": "searching",
    "Idle": "stopped",
}


class FollowerBTNode(Node):
    def __init__(self) -> None:
        super().__init__("follower_bt_node")

        # ----- Params -----
        self.declare_parameter("target_distance", 1.0)
        self.declare_parameter("close_threshold", 0.8)
        self.declare_parameter("far_threshold", 1.2)
        self.declare_parameter("max_linear_speed", 0.22)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("k_linear", 0.4)
        self.declare_parameter("k_angular", 1.5)
        self.declare_parameter("person_lost_timeout", 1.0)
        self.declare_parameter("search_yaw_rate", 0.3)
        self.declare_parameter("tick_rate_hz", 10.0)
        self.declare_parameter("detection_topic", "/person/detection")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        # ----- Obstacle avoidance (LiDAR) -----
        self.declare_parameter("obstacle_avoidance_enabled", True)
        self.declare_parameter("obstacle_stop_distance", 0.4)
        self.declare_parameter("obstacle_front_half_rad", 0.35)
        self.declare_parameter("obstacle_person_margin_rad", 0.26)
        self.declare_parameter("avoid_yaw_rate", 0.6)
        self.declare_parameter("camera_hfov_rad", 1.085)
        self.declare_parameter("scan_topic", "/scan")
        # ----- Orient-to-face + state reporting -----
        self.declare_parameter("orient_offset_threshold", 0.22)
        self.declare_parameter("follower_state_topic", "/follower/state")

        gp = self.get_parameter
        params = ControlParams(
            target_distance=float(gp("target_distance").value),
            close_threshold=float(gp("close_threshold").value),
            far_threshold=float(gp("far_threshold").value),
            max_linear_speed=float(gp("max_linear_speed").value),
            max_angular_speed=float(gp("max_angular_speed").value),
            k_linear=float(gp("k_linear").value),
            k_angular=float(gp("k_angular").value),
        )
        obstacle_params = ObstacleParams(
            enabled=bool(gp("obstacle_avoidance_enabled").value),
            stop_distance=float(gp("obstacle_stop_distance").value),
            front_half_rad=float(gp("obstacle_front_half_rad").value),
            person_margin_rad=float(gp("obstacle_person_margin_rad").value),
            avoid_yaw_rate=float(gp("avoid_yaw_rate").value),
            camera_hfov_rad=float(gp("camera_hfov_rad").value),
        )
        person_lost_timeout = float(gp("person_lost_timeout").value)
        search_yaw_rate = float(gp("search_yaw_rate").value)
        orient_offset_threshold = float(gp("orient_offset_threshold").value)
        tick_period = 1.0 / float(gp("tick_rate_hz").value)

        # ----- Publisher + Twist wrapper -----
        self.cmd_pub = self.create_publisher(Twist, gp("cmd_vel_topic").value, 10)
        twist_pub = TwistPublisher(publisher=self.cmd_pub)
        # State publisher for the dashboard (searching/approach/hold/stop/avoid/orient)
        self.state_pub = self.create_publisher(String, gp("follower_state_topic").value, 10)

        # ----- Detection subscriber -> blackboard -----
        self.bb = py_trees.blackboard.Client(name="follower_bt_node")
        self.bb.register_key(KEY_DETECTION, access=py_trees.common.Access.WRITE)
        self.bb.register_key(KEY_LAST_SEEN_TIME, access=py_trees.common.Access.WRITE)
        self.bb.register_key(KEY_SCAN, access=py_trees.common.Access.WRITE)
        # Pre-seed
        self.bb.set(KEY_DETECTION, None)
        self.bb.set(KEY_LAST_SEEN_TIME, None)
        self.bb.set(KEY_SCAN, None)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PersonDetection, gp("detection_topic").value,
            self._on_detection, sensor_qos
        )
        self.create_subscription(
            LaserScan, gp("scan_topic").value, self._on_scan, sensor_qos
        )

        # ----- Build tree -----
        self.root = build_tree(
            twist_pub=twist_pub,
            params=params,
            obstacle_params=obstacle_params,
            person_lost_timeout=person_lost_timeout,
            search_yaw_rate=search_yaw_rate,
            orient_offset_threshold=orient_offset_threshold,
        )
        self.tree = py_trees.trees.BehaviourTree(self.root)
        self.tree.setup(timeout=5.0)

        # ----- Tick timer -----
        self.create_timer(tick_period, self._tick)

        self.get_logger().info(
            f"follower_bt_node ready. Ticking at {1.0/tick_period:.1f} Hz."
        )
        print(py_trees.display.ascii_tree(self.root))

    def _on_detection(self, msg: PersonDetection) -> None:
        # Only latch a POSITIVE detection. Keeping the last good detection (and
        # its timestamp) lets the FOLLOW branch keep approaching through brief
        # YOLO drop-outs / a laggy stream, until person_lost_timeout expires and
        # SEARCH takes over. Storing detected=false frames here caused the robot
        # to fall back to SEARCH on every missed frame ("detects but won't follow").
        if msg.detected:
            self.bb.set(KEY_DETECTION, msg)
            self.bb.set(KEY_LAST_SEEN_TIME, time.monotonic())

    def _on_scan(self, msg: LaserScan) -> None:
        self.bb.set(KEY_SCAN, msg)

    def _tick(self) -> None:
        self.tree.tick()
        self._publish_state()

    def _publish_state(self) -> None:
        """Publish the active behaviour (tree tip) as a compact label for the GUI."""
        tip = self.root.tip()
        name = tip.name if tip is not None else ""
        msg = String()
        msg.data = _TIP_TO_STATE.get(name, "stopped")
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
