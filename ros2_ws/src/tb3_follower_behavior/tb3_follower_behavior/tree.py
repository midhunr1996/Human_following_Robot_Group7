"""Behavior tree assembly. Pure-Python (no rclpy) — the BT node injects publishers."""
from __future__ import annotations

import py_trees

from tb3_follower_behavior.control import ControlParams
from tb3_follower_behavior.behaviours.helpers import TwistPublisher
from tb3_follower_behavior.behaviours.guards import (
    IsPersonDetected,
    ReadDistance,
    DistanceWithin,
    PersonLostTimer,
)
from tb3_follower_behavior.behaviours.actions import (
    Stop,
    Approach,
    HoldPosition,
    RotateInPlace,
    Idle,
)


def build_tree(
    *,
    twist_pub: TwistPublisher,
    params: ControlParams,
    person_lost_timeout: float,
    search_yaw_rate: float,
) -> py_trees.behaviour.Behaviour:
    """Returns the root behaviour of the follower tree.

    Root (Selector)
    ├── FOLLOW (Sequence)
    │   ├── IsPersonDetected
    │   ├── ReadDistance
    │   └── Selector
    │       ├── Sequence(IsTooClose, Stop)
    │       ├── Sequence(IsTooFar, Approach)
    │       └── Sequence(InRange, HoldPosition)
    ├── SEARCH (Sequence)
    │   ├── PersonLostTimer
    │   └── RotateInPlace
    └── Idle
    """
    # ----- FOLLOW branch -----
    distance_selector = py_trees.composites.Selector(
        name="DistanceSelector", memory=False
    )
    distance_selector.add_children([
        py_trees.composites.Sequence(name="TooClose->Stop", memory=False, children=[
            DistanceWithin(name="IsTooClose", lo=-float("inf"), hi=params.close_threshold),
            Stop(twist_pub=twist_pub),
        ]),
        py_trees.composites.Sequence(name="TooFar->Approach", memory=False, children=[
            DistanceWithin(name="IsTooFar", lo=params.far_threshold, hi=float("inf")),
            Approach(twist_pub=twist_pub, params=params),
        ]),
        py_trees.composites.Sequence(name="InRange->Hold", memory=False, children=[
            DistanceWithin(name="InRange", lo=params.close_threshold, hi=params.far_threshold),
            HoldPosition(twist_pub=twist_pub, params=params),
        ]),
    ])

    follow = py_trees.composites.Sequence(name="FOLLOW", memory=False, children=[
        IsPersonDetected(timeout_s=person_lost_timeout),
        ReadDistance(),
        distance_selector,
    ])

    # ----- SEARCH branch -----
    search = py_trees.composites.Sequence(name="SEARCH", memory=False, children=[
        PersonLostTimer(timeout_s=person_lost_timeout),
        RotateInPlace(twist_pub=twist_pub, yaw_rate=search_yaw_rate),
    ])

    # ----- Root selector -----
    root = py_trees.composites.Selector(name="Root", memory=False, children=[
        follow,
        search,
        Idle(twist_pub=twist_pub),
    ])
    return root
