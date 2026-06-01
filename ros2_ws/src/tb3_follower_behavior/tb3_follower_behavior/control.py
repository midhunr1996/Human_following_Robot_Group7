"""Pure controller logic for the follower behavior tree.

No ROS imports — tested directly with pytest.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ControlParams:
    target_distance: float
    close_threshold: float
    far_threshold: float
    max_linear_speed: float
    max_angular_speed: float
    k_linear: float
    k_angular: float


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def compute_approach_twist(
    *,
    distance: float,
    bbox_cx: float,
    params: ControlParams,
) -> tuple[float, float]:
    """Return (linear_x, angular_z) for the APPROACH action.

    - Linear: proportional to (distance - target_distance), clamped to [0, max_linear_speed].
      We never back up here; the STOP branch in the BT handles too-close.
    - Angular: proportional to -(bbox_cx - 0.5). Negative because in ROS, positive
      angular.z is a LEFT turn, and bbox_cx < 0.5 means the person is to the LEFT
      of frame center, so we want to turn left (positive angular).
    """
    lin_err = distance - params.target_distance
    v = clamp(params.k_linear * lin_err, 0.0, params.max_linear_speed)

    ang_err = -(bbox_cx - 0.5)
    w = clamp(params.k_angular * ang_err, -params.max_angular_speed, params.max_angular_speed)

    return v, w
