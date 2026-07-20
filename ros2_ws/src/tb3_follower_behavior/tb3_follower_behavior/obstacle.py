"""Pure LiDAR obstacle-avoidance helpers for the follower. No ROS imports.

Tested by test_obstacle.py — runnable anywhere with pytest. The behaviour-tree
guard/action extract raw fields from a LaserScan and delegate the geometry here.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def _norm(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _valid(r: float, range_max: float) -> bool:
    """A LaserScan range is usable if finite, positive and within sensor range."""
    return (
        r is not None
        and not math.isnan(r)
        and not math.isinf(r)
        and r > 0.0
        and r < range_max
    )


def sector_min_range(
    ranges,
    angle_min: float,
    angle_increment: float,
    center_rad: float,
    half_width_rad: float,
    range_max: float,
) -> float:
    """Smallest valid range within +/- half_width_rad of center_rad. inf if none."""
    best = float("inf")
    for i, r in enumerate(ranges):
        theta = angle_min + i * angle_increment
        if abs(_norm(theta - center_rad)) <= half_width_rad and _valid(r, range_max):
            if r < best:
                best = r
    return best


@dataclass(frozen=True)
class ObstacleAssessment:
    present: bool      # True if a non-person obstacle is within stop_distance ahead
    front_min: float   # nearest in-front (non-person) range, meters; inf if clear
    turn_sign: float   # +1 => steer left, -1 => steer right (toward the clearer side)


def assess_obstacle(
    *,
    ranges,
    angle_min: float,
    angle_increment: float,
    range_max: float,
    front_half_rad: float,
    stop_distance: float,
    person_bearing_rad: float | None = None,
    person_margin_rad: float = 0.0,
    side_half_rad: float = 0.9,
) -> ObstacleAssessment:
    """Decide whether to avoid, and which way to turn.

    Scans a frontal cone (+/- front_half_rad around straight ahead, 0 rad).
    Beams within person_margin_rad of the tracked person's bearing are ignored so
    we don't try to "avoid" the person we're following. If the nearest remaining
    obstacle is closer than stop_distance, present=True and turn_sign points toward
    whichever side (left = +, right = -) currently has more clearance.
    """
    front_min = float("inf")
    for i, r in enumerate(ranges):
        theta = _norm(angle_min + i * angle_increment)
        if abs(theta) > front_half_rad:
            continue
        if (
            person_bearing_rad is not None
            and abs(_norm(theta - person_bearing_rad)) <= person_margin_rad
        ):
            continue  # this beam is the person, not an obstacle
        if _valid(r, range_max) and r < front_min:
            front_min = r

    # Clearance to each side, EXCLUDING the central cone so a dead-ahead obstacle
    # doesn't bias the choice: left = (front_half, side_half], right = mirror.
    side_center = (front_half_rad + side_half_rad) / 2.0
    side_hw = max((side_half_rad - front_half_rad) / 2.0, 0.0)
    left_min = sector_min_range(
        ranges, angle_min, angle_increment, +side_center, side_hw, range_max
    )
    right_min = sector_min_range(
        ranges, angle_min, angle_increment, -side_center, side_hw, range_max
    )
    turn_sign = 1.0 if left_min >= right_min else -1.0

    return ObstacleAssessment(
        present=front_min < stop_distance,
        front_min=front_min,
        turn_sign=turn_sign,
    )
