"""Condition (guard) behaviours — read-only, never publish."""
from __future__ import annotations

import time

import py_trees

from tb3_follower_behavior.behaviours.helpers import (
    KEY_DETECTION,
    KEY_DISTANCE,
    KEY_LAST_SEEN_TIME,
)


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
