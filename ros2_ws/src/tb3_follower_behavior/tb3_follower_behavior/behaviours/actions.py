"""Action behaviours — publish Twist via injected TwistPublisher."""
from __future__ import annotations

import py_trees

from tb3_follower_behavior.behaviours.helpers import (
    KEY_DETECTION,
    KEY_DISTANCE,
    TwistPublisher,
)
from tb3_follower_behavior.control import (
    ControlParams,
    compute_approach_twist,
)


class Stop(py_trees.behaviour.Behaviour):
    """Publish zero Twist. Returns SUCCESS."""

    def __init__(self, twist_pub: TwistPublisher, name: str = "Stop"):
        super().__init__(name)
        self.twist_pub = twist_pub

    def update(self) -> py_trees.common.Status:
        self.twist_pub.stop()
        return py_trees.common.Status.SUCCESS


class Approach(py_trees.behaviour.Behaviour):
    """Drive toward the person using the pure controller. Returns RUNNING while driving."""

    def __init__(
        self,
        twist_pub: TwistPublisher,
        params: ControlParams,
        name: str = "Approach",
    ):
        super().__init__(name)
        self.twist_pub = twist_pub
        self.params = params
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_DETECTION, access=py_trees.common.Access.READ)
        self.bb.register_key(KEY_DISTANCE, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            det = self.bb.get(KEY_DETECTION)
            dist = float(self.bb.get(KEY_DISTANCE))
        except (KeyError, TypeError):
            self.twist_pub.stop()
            return py_trees.common.Status.FAILURE
        if det is None or not getattr(det, "detected", False):
            self.twist_pub.stop()
            return py_trees.common.Status.FAILURE
        v, w = compute_approach_twist(
            distance=dist,
            bbox_cx=float(det.bbox_cx),
            params=self.params,
        )
        self.twist_pub.send(v, w)
        return py_trees.common.Status.RUNNING


class HoldPosition(py_trees.behaviour.Behaviour):
    """In-range: stop linear motion but keep yaw-tracking the person.

    angular.z scales with bbox_cx offset only (no forward velocity).
    """

    def __init__(
        self,
        twist_pub: TwistPublisher,
        params: ControlParams,
        name: str = "HoldPosition",
    ):
        super().__init__(name)
        self.twist_pub = twist_pub
        self.params = params
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_DETECTION, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            det = self.bb.get(KEY_DETECTION)
            if det is None or not getattr(det, "detected", False):
                self.twist_pub.stop()
                return py_trees.common.Status.FAILURE
            bbox_cx = float(det.bbox_cx)
        except (KeyError, TypeError, AttributeError):
            self.twist_pub.stop()
            return py_trees.common.Status.FAILURE
        # Reuse approach controller with distance=target so linear is 0.
        _, w = compute_approach_twist(
            distance=self.params.target_distance,
            bbox_cx=bbox_cx,
            params=self.params,
        )
        self.twist_pub.send(0.0, w)
        return py_trees.common.Status.RUNNING


class RotateInPlace(py_trees.behaviour.Behaviour):
    """Spin to search. Returns RUNNING forever (until parent re-evaluates)."""

    def __init__(self, twist_pub: TwistPublisher, yaw_rate: float, name: str = "RotateInPlace"):
        super().__init__(name)
        self.twist_pub = twist_pub
        self.yaw_rate = yaw_rate

    def update(self) -> py_trees.common.Status:
        self.twist_pub.send(0.0, self.yaw_rate)
        return py_trees.common.Status.RUNNING


class Idle(py_trees.behaviour.Behaviour):
    """Safety net: zero Twist, returns RUNNING."""

    def __init__(self, twist_pub: TwistPublisher, name: str = "Idle"):
        super().__init__(name)
        self.twist_pub = twist_pub

    def update(self) -> py_trees.common.Status:
        self.twist_pub.stop()
        return py_trees.common.Status.RUNNING
