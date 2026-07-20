"""Condition (guard) behaviours — read-only, never publish."""
from __future__ import annotations

import time

import py_trees

from tb3_follower_behavior.behaviours.helpers import (
    KEY_DETECTION,
    KEY_DISTANCE,
    KEY_LAST_SEEN_TIME,
    KEY_SCAN,
    KEY_AVOID,
)
from tb3_follower_behavior.obstacle import assess_obstacle


class IsPersonDetected(py_trees.behaviour.Behaviour):
    """SUCCESS if a fresh detection (within timeout) exists on the blackboard."""

    def __init__(self, name: str = "IsPersonDetected", timeout_s: float = 1.0):
        super().__init__(name)
        self.timeout_s = timeout_s
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_LAST_SEEN_TIME, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            last_t = self.bb.get(KEY_LAST_SEEN_TIME)
        except KeyError:
            return py_trees.common.Status.FAILURE
        if last_t is None:
            return py_trees.common.Status.FAILURE
        age = time.monotonic() - float(last_t)
        return (
            py_trees.common.Status.SUCCESS
            if age <= self.timeout_s
            else py_trees.common.Status.FAILURE
        )


class IsOffCenter(py_trees.behaviour.Behaviour):
    """SUCCESS if a detected person is horizontally off-center by more than
    ``threshold`` (in normalized bbox_cx units, where 0.5 is dead ahead).

    Used to trigger a rotate-in-place "turn to face" before approaching, so the
    robot orients toward a person at the edge of frame instead of driving off at
    an angle.
    """

    def __init__(self, threshold: float, name: str = "IsOffCenter"):
        super().__init__(name)
        self.threshold = threshold
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_DETECTION, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            det = self.bb.get(KEY_DETECTION)
        except KeyError:
            det = None
        if det is None or not getattr(det, "detected", False):
            return py_trees.common.Status.FAILURE
        return (
            py_trees.common.Status.SUCCESS
            if abs(float(det.bbox_cx) - 0.5) > self.threshold
            else py_trees.common.Status.FAILURE
        )


class ReadDistance(py_trees.behaviour.Behaviour):
    """Copies detection.distance from the blackboard into KEY_DISTANCE.

    Returns SUCCESS if a detection exists, FAILURE otherwise.
    """

    def __init__(self, name: str = "ReadDistance"):
        super().__init__(name)
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_DETECTION, access=py_trees.common.Access.READ)
        self.bb.register_key(KEY_DISTANCE, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        try:
            det = self.bb.get(KEY_DETECTION)
        except KeyError:
            return py_trees.common.Status.FAILURE
        if det is None or not getattr(det, "detected", False):
            return py_trees.common.Status.FAILURE
        self.bb.set(KEY_DISTANCE, float(det.distance))
        return py_trees.common.Status.SUCCESS


class DistanceWithin(py_trees.behaviour.Behaviour):
    """SUCCESS if blackboard distance ∈ [lo, hi]. Used as IsTooClose / IsTooFar / InRange."""

    def __init__(self, name: str, lo: float, hi: float):
        super().__init__(name)
        self.lo = lo
        self.hi = hi
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_DISTANCE, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            d = float(self.bb.get(KEY_DISTANCE))
        except (KeyError, TypeError):
            return py_trees.common.Status.FAILURE
        return (
            py_trees.common.Status.SUCCESS
            if self.lo <= d <= self.hi
            else py_trees.common.Status.FAILURE
        )


class IsObstacleAhead(py_trees.behaviour.Behaviour):
    """SUCCESS if a non-person obstacle is within stop_distance ahead.

    Reads the latest LaserScan + current detection from the blackboard, runs the
    pure ``assess_obstacle`` geometry (excluding the person's bearing so we never
    'avoid' the person we follow), and stashes the assessment in KEY_AVOID for the
    AvoidObstacle action. Returns FAILURE when avoidance is disabled or no scan.
    """

    def __init__(self, params, name: str = "IsObstacleAhead"):
        super().__init__(name)
        self.params = params
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_SCAN, access=py_trees.common.Access.READ)
        self.bb.register_key(KEY_DETECTION, access=py_trees.common.Access.READ)
        self.bb.register_key(KEY_AVOID, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        if not self.params.enabled:
            return py_trees.common.Status.FAILURE
        try:
            scan = self.bb.get(KEY_SCAN)
        except KeyError:
            scan = None
        if scan is None:
            return py_trees.common.Status.FAILURE

        person_bearing = None
        try:
            det = self.bb.get(KEY_DETECTION)
        except KeyError:
            det = None
        if det is not None and getattr(det, "detected", False):
            # bbox_cx -> yaw: 0.5 (center) => 0, left of image (<0.5) => +yaw (left)
            person_bearing = (0.5 - float(det.bbox_cx)) * self.params.camera_hfov_rad

        assessment = assess_obstacle(
            ranges=list(scan.ranges),
            angle_min=float(scan.angle_min),
            angle_increment=float(scan.angle_increment),
            range_max=float(scan.range_max),
            front_half_rad=self.params.front_half_rad,
            stop_distance=self.params.stop_distance,
            person_bearing_rad=person_bearing,
            person_margin_rad=self.params.person_margin_rad,
        )
        self.bb.set(KEY_AVOID, assessment)
        return (
            py_trees.common.Status.SUCCESS
            if assessment.present
            else py_trees.common.Status.FAILURE
        )


class PersonLostTimer(py_trees.behaviour.Behaviour):
    """SUCCESS if it has been MORE than timeout_s since we last saw a person."""

    def __init__(self, name: str = "PersonLostTimer", timeout_s: float = 1.0):
        super().__init__(name)
        self.timeout_s = timeout_s
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_LAST_SEEN_TIME, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            last_t = self.bb.get(KEY_LAST_SEEN_TIME)
        except KeyError:
            last_t = None
        if last_t is None:
            return py_trees.common.Status.SUCCESS  # never seen — definitely "lost"
        age = time.monotonic() - float(last_t)
        return (
            py_trees.common.Status.SUCCESS
            if age > self.timeout_s
            else py_trees.common.Status.FAILURE
        )
