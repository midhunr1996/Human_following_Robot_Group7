"""Pure-math tests for tb3_follower_perception.geometry — no ROS deps."""
import math
import pytest

from tb3_follower_perception.geometry import (
    visual_distance,
    bbox_cx_to_lidar_index,
    fuse_distance,
    nearest_index,
)


class TestVisualDistance:
    def test_far_person_small_bbox(self):
        # 1.7m person, bbox 60px tall in a 480-px image, focal 525px (Waffle Pi default-ish)
        d = visual_distance(person_height_m=1.7, bbox_h_px=60, focal_px=525)
        assert d == pytest.approx(14.875, rel=1e-3)

    def test_close_person_large_bbox(self):
        d = visual_distance(person_height_m=1.7, bbox_h_px=400, focal_px=525)
        assert d == pytest.approx(2.23125, rel=1e-3)

    def test_zero_bbox_raises(self):
        with pytest.raises(ValueError):
            visual_distance(person_height_m=1.7, bbox_h_px=0, focal_px=525)

    def test_negative_focal_raises(self):
        with pytest.raises(ValueError):
            visual_distance(person_height_m=1.7, bbox_h_px=100, focal_px=-1)


class TestBboxToLidarIndex:
    def test_center_maps_to_zero_yaw(self):
        # 360-beam scan, angle_min=-pi, angle_increment=2pi/360
        # bbox_cx=0.5 (image center) should map to forward-facing beam.
        # Camera HFOV assumed 1.085 rad (62.2 deg, Waffle Pi Raspberry Pi cam default).
        idx = bbox_cx_to_lidar_index(
            bbox_cx=0.5,
            camera_hfov_rad=1.085,
            scan_angle_min=-math.pi,
            scan_angle_increment=(2 * math.pi) / 360,
            num_beams=360,
        )
        # Forward = yaw 0 = scan index where angle_min + i*incr == 0 => i = pi / incr
        assert idx == 180

    def test_left_edge_negative_yaw(self):
        idx = bbox_cx_to_lidar_index(
            bbox_cx=0.0,
            camera_hfov_rad=1.085,
            scan_angle_min=-math.pi,
            scan_angle_increment=(2 * math.pi) / 360,
            num_beams=360,
        )
        # bbox_cx=0 means person at left edge of frame -> +HFOV/2 yaw (object to robot's left)
        # +0.5425 rad ≈ +31 deg ≈ index 180 + 31 = 211
        assert idx == pytest.approx(211, abs=1)

    def test_right_edge_positive_yaw(self):
        idx = bbox_cx_to_lidar_index(
            bbox_cx=1.0,
            camera_hfov_rad=1.085,
            scan_angle_min=-math.pi,
            scan_angle_increment=(2 * math.pi) / 360,
            num_beams=360,
        )
        # bbox_cx=1 -> person on robot's right -> -31 deg -> index 180 - 31 = 149
        assert idx == pytest.approx(149, abs=1)


class TestFuseDistance:
    def test_lidar_valid_overrides_visual(self):
        d = fuse_distance(visual_d=5.0, lidar_d=2.0, lidar_range_max=10.0)
        assert d == 2.0

    def test_lidar_above_max_falls_back_to_visual(self):
        d = fuse_distance(visual_d=5.0, lidar_d=10.5, lidar_range_max=10.0)
        assert d == 5.0

    def test_lidar_nan_falls_back_to_visual(self):
        d = fuse_distance(visual_d=5.0, lidar_d=float("nan"), lidar_range_max=10.0)
        assert d == 5.0

    def test_lidar_inf_falls_back_to_visual(self):
        d = fuse_distance(visual_d=5.0, lidar_d=float("inf"), lidar_range_max=10.0)
        assert d == 5.0


class TestNearestIndex:
    def test_picks_smallest(self):
        assert nearest_index([5.0, 2.0, 8.0]) == 1

    def test_single_entry(self):
        assert nearest_index([3.3]) == 0

    def test_skips_nan_and_inf(self):
        assert nearest_index([float("nan"), float("inf"), 4.0, 2.5]) == 3

    def test_skips_nonpositive(self):
        # -1.0 is the "unknown distance" sentinel; must never win.
        assert nearest_index([-1.0, 0.0, 6.0]) == 2

    def test_all_invalid_falls_back_to_first(self):
        assert nearest_index([float("nan"), -1.0]) == 0

    def test_empty_raises(self):
        with pytest.raises(ValueError):
            nearest_index([])

    def test_tie_keeps_first(self):
        assert nearest_index([2.0, 2.0, 5.0]) == 0
