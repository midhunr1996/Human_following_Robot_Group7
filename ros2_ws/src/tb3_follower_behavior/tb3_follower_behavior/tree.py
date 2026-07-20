"""Behavior tree assembly. Pure-Python (no rclpy) — the BT node injects publishers."""
from __future__ import annotations

import py_trees

from tb3_follower_behavior.control import ControlParams, ObstacleParams
from tb3_follower_behavior.behaviours.helpers import TwistPublisher
from tb3_follower_behavior.behaviours.guards import (
    IsPersonDetected,
    ReadDistance,
    DistanceWithin,
    PersonLostTimer,
    IsObstacleAhead,
    IsOffCenter,
)
from tb3_follower_behavior.behaviours.actions import (
    Stop,
    Approach,
    HoldPosition,
    RotateInPlace,
    AvoidObstacle,
    TurnToFace,
    Idle,
)


def build_tree(
    *,
    twist_pub: TwistPublisher,
    params: ControlParams,
    obstacle_params: ObstacleParams,
    person_lost_timeout: float,
    search_yaw_rate: float,
    orient_offset_threshold: float = 0.22,
) -> py_trees.behaviour.Behaviour:
    """Returns the root behaviour of the follower tree.

    Root (Selector)
    ├── AVOID (Sequence)          # highest priority — collision safety
    │   ├── IsObstacleAhead       # non-person obstacle within stop_distance
    │   └── AvoidObstacle         # stop + spin toward clearer side
    ├── FOLLOW (Sequence)
    │   ├── IsPersonDetected
    │   ├── ReadDistance
    │   └── Selector
    │       ├── Sequence(IsTooClose, Stop)
    │       ├── Sequence(IsOffCenter, TurnToFace)   # rotate in place to face
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
        py_trees.composites.Sequence(name="OffCenter->TurnToFace", memory=False, children=[
            IsOffCenter(threshold=orient_offset_threshold),
            TurnToFace(twist_pub=twist_pub, params=params),
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

    # ----- AVOID branch (highest priority: collision safety) -----
    avoid = py_trees.composites.Sequence(name="AVOID", memory=False, children=[
        IsObstacleAhead(params=obstacle_params),
        AvoidObstacle(twist_pub=twist_pub, params=obstacle_params),
    ])

    # ----- Root selector -----
    root = py_trees.composites.Selector(name="Root", memory=False, children=[
        avoid,
        follow,
        search,
        Idle(twist_pub=twist_pub),
    ])
    return root
