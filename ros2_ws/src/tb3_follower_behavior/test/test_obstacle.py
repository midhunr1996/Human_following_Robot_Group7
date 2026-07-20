"""Pure-math tests for tb3_follower_behavior.obstacle — no ROS deps."""
import math

from tb3_follower_behavior.obstacle import (
    sector_min_range,
    assess_obstacle,
    ObstacleAssessment,
)


# A 360-beam scan, angle_min=-pi, increment=2pi/360, so beam i is at
# angle -pi + i*(2pi/360). Forward (0 rad) is beam 180.
N = 360
ANGLE_MIN = -math.pi
INC = (2 * math.pi) / N
RANGE_MAX = 12.0


def _blank(fill=10.0):
    return [fill] * N


def _idx(angle_rad):
    return int(round((angle_rad - ANGLE_MIN) / INC)) % N


class TestSectorMinRange:
    def test_finds_min_in_sector(self):
        r = _blank()
        r[_idx(0.0)] = 0.5           # obstacle straight ahead
        d = sector_min_range(r, ANGLE_MIN, INC, 0.0, 0.3, RANGE_MAX)
        assert d == 0.5

    def test_ignores_outside_sector(self):
        r = _blank()
        r[_idx(math.pi / 2)] = 0.2   # obstacle to the left, outside a forward cone
        d = sector_min_range(r, ANGLE_MIN, INC, 0.0, 0.3, RANGE_MAX)
        assert d == 10.0

    def test_skips_invalid(self):
        r = _blank()
        r[_idx(0.0)] = float("inf")
        r[_idx(0.05)] = 0.0          # <=0 invalid
        r[_idx(-0.05)] = float("nan")
        d = sector_min_range(r, ANGLE_MIN, INC, 0.0, 0.3, RANGE_MAX)
        assert d == 10.0             # all forward beams invalid -> fall to fill


class TestAssessObstacle:
    def _assess(self, ranges, **kw):
        return assess_obstacle(
            ranges=ranges, angle_min=ANGLE_MIN, angle_increment=INC,
            range_max=RANGE_MAX, front_half_rad=0.35, stop_distance=0.5, **kw
        )

    def test_clear_path(self):
        a = self._assess(_blank())
        assert isinstance(a, ObstacleAssessment)
        assert a.present is False
        assert a.front_min == 10.0     # valid returns, just farther than stop_distance

    def test_all_invalid_front_is_inf(self):
        r = _blank(fill=float("inf"))  # nothing in range anywhere
        a = self._assess(r)
        assert a.present is False
        assert math.isinf(a.front_min)

    def test_obstacle_ahead_present(self):
        r = _blank()
        r[_idx(0.0)] = 0.3           # 0.3 m ahead < stop_distance 0.5
        a = self._assess(r)
        assert a.present is True
        assert a.front_min == 0.3

    def test_obstacle_beyond_stop_distance_not_present(self):
        r = _blank()
        r[_idx(0.0)] = 0.8           # ahead but farther than stop_distance
        a = self._assess(r)
        assert a.present is False

    def test_person_bearing_excluded(self):
        # The only close return is exactly where the person is -> not an obstacle.
        r = _blank()
        r[_idx(0.1)] = 0.3
        a = self._assess(r, person_bearing_rad=0.1, person_margin_rad=0.2)
        assert a.present is False

    def test_obstacle_next_to_person_still_detected(self):
        # Person at +0.1 rad (excluded); a separate obstacle at -0.2 rad is real.
        r = _blank()
        r[_idx(0.1)] = 0.3           # person
        r[_idx(-0.2)] = 0.25         # obstacle on the other side of the cone
        a = self._assess(r, person_bearing_rad=0.1, person_margin_rad=0.15)
        assert a.present is True
        assert a.front_min == 0.25

    def test_turns_toward_clearer_side(self):
        # Obstacle ahead; right side blocked, left side open -> steer left (+1).
        r = _blank(fill=10.0)
        r[_idx(0.0)] = 0.3
        for ang in (-0.3, -0.4, -0.5, -0.6):
            r[_idx(ang)] = 0.4       # clutter on the right
        a = self._assess(r)
        assert a.present is True
        assert a.turn_sign == 1.0

    def test_turns_right_when_left_blocked(self):
        r = _blank(fill=10.0)
        r[_idx(0.0)] = 0.3
        for ang in (0.3, 0.4, 0.5, 0.6):
            r[_idx(ang)] = 0.4       # clutter on the left
        a = self._assess(r)
        assert a.turn_sign == -1.0
