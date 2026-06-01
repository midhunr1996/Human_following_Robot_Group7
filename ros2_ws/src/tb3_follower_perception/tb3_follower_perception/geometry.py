"""Pure geometry helpers for the perception node. No ROS imports.

Tested by test_geometry.py — runnable on any machine with numpy + pytest.
"""
from __future__ import annotations

import math


def visual_distance(*, person_height_m: float, bbox_h_px: float, focal_px: float) -> float:
    """Estimate distance to a person using pinhole projection.

    d = (H * f) / h_px

    where H is the assumed person height in meters, f is camera focal length in
    pixels, and h_px is the bounding box height in pixels.
    """
    if bbox_h_px <= 0:
        raise ValueError("bbox_h_px must be > 0")
    if focal_px <= 0:
        raise ValueError("focal_px must be > 0")
    return (person_height_m * focal_px) / bbox_h_px


def bbox_cx_to_lidar_index(
    *,
    bbox_cx: float,
    camera_hfov_rad: float,
    scan_angle_min: float,
    scan_angle_increment: float,
    num_beams: int,
) -> int:
    """Map a normalized bbox center x ([0,1]) to a LaserScan beam index.

    Convention: bbox_cx=0.5 maps to yaw 0 (straight ahead). bbox_cx=0 (left edge
    of image) maps to +HFOV/2 yaw (object to robot's LEFT, positive yaw in ROS).
    bbox_cx=1 maps to -HFOV/2 (right side).
    """
    yaw = (0.5 - bbox_cx) * camera_hfov_rad
    raw_idx = (yaw - scan_angle_min) / scan_angle_increment
    idx = int(round(raw_idx))
    return idx % num_beams


def fuse_distance(*, visual_d: float, lidar_d: float, lidar_range_max: float) -> float:
    """Prefer LiDAR if it returned a valid in-range reading, else fall back to visual."""
    if math.isnan(lidar_d) or math.isinf(lidar_d):
        return visual_d
    if lidar_d <= 0 or lidar_d >= lidar_range_max:
        return visual_d
    return lidar_d
