"""Pure controller tests — no ROS deps."""
import pytest

from tb3_follower_behavior.control import (
    ControlParams,
    compute_approach_twist,
    clamp,
)


@pytest.fixture
def default_params() -> ControlParams:
    return ControlParams(
        target_distance=1.0,
        close_threshold=0.8,
        far_threshold=1.2,
        max_linear_speed=0.22,
        max_angular_speed=1.0,
        k_linear=0.4,
        k_angular=1.5,
    )


class TestClamp:
    def test_within_range(self):
        assert clamp(0.5, -1.0, 1.0) == 0.5

    def test_above(self):
        assert clamp(5.0, -1.0, 1.0) == 1.0

    def test_below(self):
        assert clamp(-5.0, -1.0, 1.0) == -1.0


class TestApproachTwist:
    def test_far_person_drives_forward(self, default_params):
        v, w = compute_approach_twist(distance=2.0, bbox_cx=0.5, params=default_params)
        # error = 2.0 - 1.0 = 1.0; k_lin*err = 0.4; clamped to 0.22 max
        assert v == pytest.approx(0.22, abs=1e-6)
        assert w == pytest.approx(0.0, abs=1e-6)

    def test_far_person_steers_toward_offset(self, default_params):
        # bbox_cx=0.3 -> person to LEFT of frame -> turn LEFT (positive angular.z in ROS)
        v, w = compute_approach_twist(distance=1.5, bbox_cx=0.3, params=default_params)
        assert v > 0.0
        # k_ang * -(0.3 - 0.5) = 1.5 * 0.2 = 0.3
        assert w == pytest.approx(0.3, abs=1e-6)

    def test_far_person_steers_right_for_right_offset(self, default_params):
        v, w = compute_approach_twist(distance=1.5, bbox_cx=0.7, params=default_params)
        assert w == pytest.approx(-0.3, abs=1e-6)

    def test_angular_clamped(self, default_params):
        # Extreme offset -> would saturate
        v, w = compute_approach_twist(distance=1.5, bbox_cx=0.0, params=default_params)
        # k_ang * 0.5 = 0.75, under cap 1.0
        assert w == pytest.approx(0.75, abs=1e-6)

    def test_at_target_distance_no_forward(self, default_params):
        # distance equals target -> linear error is 0
        v, w = compute_approach_twist(distance=1.0, bbox_cx=0.5, params=default_params)
        assert v == pytest.approx(0.0, abs=1e-6)
        assert w == pytest.approx(0.0, abs=1e-6)

    def test_below_target_no_backup(self, default_params):
        # If person is closer than target, we don't back up — STOP branch handles that.
        # compute_approach_twist must clamp linear to >= 0.
        v, _ = compute_approach_twist(distance=0.3, bbox_cx=0.5, params=default_params)
        assert v >= 0.0
