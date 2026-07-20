"""Integration test for the behaviour tree.

Runs in the VM only (imports py_trees + tb3_follower_msgs).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import pytest

from sensor_msgs.msg import LaserScan
from tb3_follower_msgs.msg import PersonDetection
from tb3_follower_behavior.control import ControlParams, ObstacleParams
from tb3_follower_behavior.behaviours.helpers import (
    KEY_DETECTION,
    KEY_LAST_SEEN_TIME,
    KEY_SCAN,
    TwistPublisher,
)
from tb3_follower_behavior.tree import build_tree
import py_trees


@dataclass
class FakePublisher:
    """Stand-in for rclpy.publisher.Publisher — records last published Twist."""
    last_linear_x: float = 0.0
    last_angular_z: float = 0.0
    publish_count: int = 0

    def publish(self, msg) -> None:  # ROS-style API
        self.last_linear_x = float(msg.linear.x)
        self.last_angular_z = float(msg.angular.z)
        self.publish_count += 1


@pytest.fixture
def params() -> ControlParams:
    return ControlParams(
        target_distance=1.0,
        close_threshold=0.8,
        far_threshold=1.2,
        max_linear_speed=0.22,
        max_angular_speed=1.0,
        k_linear=0.4,
        k_angular=1.5,
    )


@pytest.fixture
def obstacle_params() -> ObstacleParams:
    return ObstacleParams(
        enabled=True,
        stop_distance=0.4,
        front_half_rad=0.35,
        person_margin_rad=0.26,
        avoid_yaw_rate=0.6,
        camera_hfov_rad=1.085,
    )


@pytest.fixture
def tree_and_pub(params, obstacle_params):
    pub = FakePublisher()
    twist_pub = TwistPublisher(publisher=pub)
    root = build_tree(
        twist_pub=twist_pub,
        params=params,
        obstacle_params=obstacle_params,
        person_lost_timeout=1.0,
        search_yaw_rate=0.3,
    )
    # Seed blackboard with the keys the tree expects.
    bb = py_trees.blackboard.Client(name="test_seed")
    bb.register_key(KEY_DETECTION, access=py_trees.common.Access.WRITE)
    bb.register_key(KEY_LAST_SEEN_TIME, access=py_trees.common.Access.WRITE)
    bb.register_key(KEY_SCAN, access=py_trees.common.Access.WRITE)
    bb.set(KEY_DETECTION, None)
    bb.set(KEY_LAST_SEEN_TIME, None)
    bb.set(KEY_SCAN, None)
    return root, pub, bb


def _make_scan(front_range=None, n: int = 360) -> LaserScan:
    """360-beam scan, all clear (10 m) except an optional obstacle straight ahead."""
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_increment = (2 * math.pi) / n
    scan.range_min = 0.0
    scan.range_max = 12.0
    ranges = [10.0] * n
    if front_range is not None:
        fwd = int(round((0.0 - scan.angle_min) / scan.angle_increment)) % n
        ranges[fwd] = float(front_range)
    scan.ranges = ranges
    return scan


def _make_detection(*, detected: bool, distance: float = -1.0, cx: float = 0.5) -> PersonDetection:
    msg = PersonDetection()
    msg.detected = detected
    msg.distance = float(distance)
    msg.bbox_cx = float(cx)
    msg.bbox_cy = 0.5
    msg.bbox_w = 0.2
    msg.bbox_h = 0.4
    msg.confidence = 0.9
    return msg


class TestTreeBranches:
    def test_no_detection_ever_rotates(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        root.tick_once()
        assert pub.last_linear_x == pytest.approx(0.0)
        assert pub.last_angular_z == pytest.approx(0.3)  # search yaw rate

    def test_person_too_close_stops(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=0.5, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        root.tick_once()
        assert pub.last_linear_x == pytest.approx(0.0)
        assert pub.last_angular_z == pytest.approx(0.0)

    def test_person_too_far_approaches(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=2.0, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        root.tick_once()
        assert pub.last_linear_x > 0.0
        assert pub.last_angular_z == pytest.approx(0.0)

    def test_person_far_and_offset_steers(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=2.0, cx=0.3))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        root.tick_once()
        assert pub.last_linear_x > 0.0
        # cx=0.3 -> person to the LEFT -> +angular (turn left in ROS convention)
        assert pub.last_angular_z > 0.0

    def test_person_in_range_holds_position(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=1.0, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        root.tick_once()
        assert pub.last_linear_x == pytest.approx(0.0)
        assert pub.last_angular_z == pytest.approx(0.0)

    def test_stale_detection_triggers_search(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        # Detection from 5 seconds ago -> stale (timeout is 1s)
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=1.0, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic() - 5.0)
        root.tick_once()
        assert pub.last_angular_z == pytest.approx(0.3)

    def test_lost_then_found_resumes(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        # Phase 1: no person -> SEARCH
        root.tick_once()
        assert pub.last_angular_z == pytest.approx(0.3)
        # Phase 2: fresh detection appears
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=2.0, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        root.tick_once()
        assert pub.last_linear_x > 0.0  # FOLLOW branch took over


class TestObstacleAvoidance:
    def test_no_obstacle_follows(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=2.0, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        bb.set(KEY_SCAN, _make_scan(front_range=None))  # clear ahead
        root.tick_once()
        assert pub.last_linear_x > 0.0                  # FOLLOW/approach, not avoid

    def test_obstacle_ahead_triggers_avoid(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        # Person far off to the left (bbox cx=0.1 -> bearing ~+0.43 rad, outside cone)
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=2.0, cx=0.1))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        bb.set(KEY_SCAN, _make_scan(front_range=0.3))   # 0.3 m obstacle dead ahead
        root.tick_once()
        assert pub.last_linear_x == pytest.approx(0.0)  # AVOID stops forward motion
        assert abs(pub.last_angular_z) == pytest.approx(0.6)  # spins (avoid_yaw_rate)

    def test_person_ahead_not_avoided(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        # Person dead ahead (cx=0.5 -> bearing 0); the forward return IS the person.
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=2.0, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        bb.set(KEY_SCAN, _make_scan(front_range=0.3))
        root.tick_once()
        # person's bearing excludes that beam -> not an obstacle -> FOLLOW approaches
        assert pub.last_linear_x > 0.0

    def test_off_center_far_orients(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        # Far AND well off-center (cx=0.9, offset 0.4 > 0.22 threshold) -> rotate to face.
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=2.0, cx=0.9))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        bb.set(KEY_SCAN, _make_scan(front_range=None))
        root.tick_once()
        assert pub.last_linear_x == pytest.approx(0.0)   # rotate in place, no forward
        assert pub.last_angular_z < 0.0                  # cx=0.9 (right) -> turn right

    def test_centered_far_approaches(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=2.0, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        bb.set(KEY_SCAN, _make_scan(front_range=None))
        root.tick_once()
        assert pub.last_linear_x > 0.0                   # centered -> approach, not orient

    def test_too_close_beats_orient(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        # Off-center AND too close -> the safety Stop branch wins over orient.
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=0.5, cx=0.9))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        bb.set(KEY_SCAN, _make_scan(front_range=None))
        root.tick_once()
        assert pub.last_linear_x == pytest.approx(0.0)
        assert pub.last_angular_z == pytest.approx(0.0)  # Stop = zero twist


class TestFollowRegression:
    def test_disabled_avoidance_follows(self, params):
        # With avoidance disabled, an obstacle ahead must NOT pre-empt FOLLOW.
        pub = FakePublisher()
        root = build_tree(
            twist_pub=TwistPublisher(publisher=pub), params=params,
            obstacle_params=ObstacleParams(
                enabled=False, stop_distance=0.4, front_half_rad=0.35,
                person_margin_rad=0.26, avoid_yaw_rate=0.6, camera_hfov_rad=1.085,
            ),
            person_lost_timeout=1.0, search_yaw_rate=0.3,
        )
        bb = py_trees.blackboard.Client(name="test_seed_disabled")
        for k in (KEY_DETECTION, KEY_LAST_SEEN_TIME, KEY_SCAN):
            bb.register_key(k, access=py_trees.common.Access.WRITE)
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=2.0, cx=0.1))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        bb.set(KEY_SCAN, _make_scan(front_range=0.3))
        root.tick_once()
        assert pub.last_linear_x > 0.0                  # avoidance off -> still follows
