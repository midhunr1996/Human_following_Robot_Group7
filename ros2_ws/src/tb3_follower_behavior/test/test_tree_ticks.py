"""Integration test for the behaviour tree.

Runs in the VM only (imports py_trees + tb3_follower_msgs).
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import pytest

from tb3_follower_msgs.msg import PersonDetection
from tb3_follower_behavior.control import ControlParams
from tb3_follower_behavior.behaviours.helpers import (
    KEY_DETECTION,
    KEY_LAST_SEEN_TIME,
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
def tree_and_pub(params):
    pub = FakePublisher()
    twist_pub = TwistPublisher(publisher=pub)
    root = build_tree(
        twist_pub=twist_pub,
        params=params,
        person_lost_timeout=1.0,
        search_yaw_rate=0.3,
    )
    # Seed blackboard with the keys the tree expects.
    bb = py_trees.blackboard.Client(name="test_seed")
    bb.register_key(KEY_DETECTION, access=py_trees.common.Access.WRITE)
    bb.register_key(KEY_LAST_SEEN_TIME, access=py_trees.common.Access.WRITE)
    bb.set(KEY_DETECTION, None)
    bb.set(KEY_LAST_SEEN_TIME, None)
    return root, pub, bb


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
