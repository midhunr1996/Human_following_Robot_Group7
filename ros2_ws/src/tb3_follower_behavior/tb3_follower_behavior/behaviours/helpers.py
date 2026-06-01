"""Shared blackboard schema + a TwistPublisher class injected into actions."""
from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Twist


# Blackboard keys (single source of truth — import these everywhere)
KEY_DETECTION = "person/detection"          # tb3_follower_msgs/PersonDetection
KEY_DISTANCE = "person/distance"            # float
KEY_LAST_SEEN_TIME = "person/last_seen_t"   # float, monotonic seconds


@dataclass
class TwistPublisher:
    """Thin wrapper passed into actions so they can publish without holding a Node."""
    publisher: object  # rclpy.publisher.Publisher

    def send(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)

    def stop(self) -> None:
        self.send(0.0, 0.0)
