"""Unit tests for the pure helpers in fixed_wing_adapter
(CLAIMS C20). Covers the quaternion yaw helper, the angle wrap
used by the psidot heading integrator, the ENU->compass map for
GUIDED_CHANGE_HEADING, the Dubins-airplane control-input clip
(the differential-game admissible set), and the climb->ALTITUDE
arg helper. No ROS runtime: module-level pure functions only.
"""

import math

from rufus_sim_adapters.fixed_wing_adapter import (
    _wrap_angle,
    _yaw_from_quat,
    dubins_airplane_clip,
    enu_yaw_to_compass_deg,
    guided_alt_from_climb,
)


def _close(a, b, tol=1e-9):
    return math.isclose(a, b, abs_tol=tol)


def test_yaw_identity_quat_is_zero():
    assert _close(_yaw_from_quat(0.0, 0.0, 0.0, 1.0), 0.0)


def test_yaw_plus_90_about_z():
    h = math.sqrt(0.5)
    assert _close(_yaw_from_quat(0.0, 0.0, h, h), math.pi / 2)


def test_yaw_minus_90_about_z():
    h = math.sqrt(0.5)
    assert _close(_yaw_from_quat(0.0, 0.0, -h, h), -math.pi / 2)


def test_wrap_zero_unchanged():
    assert _close(_wrap_angle(0.0), 0.0)


def test_wrap_small_positive_unchanged():
    assert _close(_wrap_angle(1.0), 1.0)


def test_wrap_above_pi_folds_negative():
    # 3*pi/2 -> -pi/2
    assert _close(_wrap_angle(3.0 * math.pi / 2), -math.pi / 2)


def test_wrap_below_neg_pi_folds_positive():
    # -3*pi/2 -> +pi/2
    assert _close(_wrap_angle(-3.0 * math.pi / 2), math.pi / 2)


def test_wrap_two_pi_is_zero():
    assert _close(_wrap_angle(2.0 * math.pi), 0.0)


# --- enu_yaw_to_compass_deg (GUIDED_CHANGE_HEADING param2) -------

def test_compass_east_is_090():
    assert _close(enu_yaw_to_compass_deg(0.0), 90.0)


def test_compass_north_is_000():
    assert _close(enu_yaw_to_compass_deg(math.pi / 2), 0.0)


def test_compass_west_is_270():
    assert _close(enu_yaw_to_compass_deg(math.pi), 270.0)


def test_compass_south_is_180():
    assert _close(enu_yaw_to_compass_deg(-math.pi / 2), 180.0)


def test_compass_range_is_0_360():
    for psi in (-3.0, -1.0, 0.0, 1.0, 2.0, 3.0, 6.0):
        d = enu_yaw_to_compass_deg(psi)
        assert 0.0 <= d < 360.0


# --- dubins_airplane_clip (differential-game admissible set) -----
# Fixture bounds: v in [10, 25], lat_accel = 9.0 (g*tan(bank)),
# climb up 0.2 rad, descent 0.15 rad.
_VMIN, _VMAX, _LAT = 10.0, 25.0, 9.0
_GUP, _GDN = 0.2, 0.15


def _clip(v, pdot, climb):
    return dubins_airplane_clip(
        v, pdot, climb, _VMIN, _VMAX, _LAT, _GUP, _GDN)


def test_clip_interior_passthrough_no_sat():
    # psidot_max at V=15 is 9/15 = 0.6; climb_max = 15*sin(0.2).
    V, pdot, climb, sa, st, sc = _clip(15.0, 0.3, 1.0)
    assert _close(V, 15.0) and _close(pdot, 0.3)
    assert _close(climb, 1.0)
    assert not (sa or st or sc)


def test_clip_airspeed_below_min():
    V, _, _, sa, _, _ = _clip(4.0, 0.0, 0.0)
    assert _close(V, _VMIN) and sa


def test_clip_airspeed_above_max():
    V, _, _, sa, _, _ = _clip(99.0, 0.0, 0.0)
    assert _close(V, _VMAX) and sa


def test_clip_turn_rate_speed_coupled():
    # At V=15 the cap is 9/15 = 0.6 rad/s.
    _, pdot, _, _, st, _ = _clip(15.0, 5.0, 0.0)
    assert _close(pdot, 0.6) and st
    _, pdotn, _, _, stn, _ = _clip(15.0, -5.0, 0.0)
    assert _close(pdotn, -0.6) and stn


def test_clip_turn_rate_uses_clipped_speed():
    # Commanded V=99 clips to v_max=25; cap = 9/25 = 0.36, NOT
    # 9/99. This is the speed-coupled coupling the game must see.
    V, pdot, _, sa, st, _ = _clip(99.0, 1.0, 0.0)
    assert _close(V, _VMAX) and sa
    assert _close(pdot, _LAT / _VMAX) and st


def test_clip_climb_up_speed_coupled():
    V = 20.0
    cap = V * math.sin(_GUP)
    _, _, climb, _, _, sc = _clip(V, 0.0, 50.0)
    assert _close(climb, cap) and sc


def test_clip_descent_uses_descent_angle():
    V = 20.0
    cap = V * math.sin(_GDN)
    _, _, climb, _, _, sc = _clip(V, 0.0, -50.0)
    assert _close(climb, -cap) and sc


def test_clip_climb_bound_uses_clipped_speed():
    # V=99 -> 25; climb cap = 25*sin(0.2), not 99*sin(0.2).
    V, _, climb, _, _, sc = _clip(99.0, 0.0, 100.0)
    assert _close(climb, _VMAX * math.sin(_GUP)) and sc


def test_clip_no_false_turn_sat_at_min_speed():
    # At v_min=10 cap is 9/10 = 0.9; 0.9 exactly is not over.
    _, pdot, _, _, st, _ = _clip(10.0, 0.9, 0.0)
    assert _close(pdot, 0.9) and not st


# --- guided_alt_from_climb (climb -> GUIDED_CHANGE_ALTITUDE) -----

def test_alt_level_floors_rate_above_zero():
    alt, rate = guided_alt_from_climb(40.0, 0.0)
    assert _close(alt, 40.0)          # hold current
    assert _close(rate, 0.1)          # 0 would mean "max rate"


def test_alt_climb_sets_target_and_rate():
    alt, rate = guided_alt_from_climb(40.0, 1.5, alt_horizon=5.0)
    assert _close(alt, 47.5)          # 40 + 1.5*5
    assert _close(rate, 1.5)


def test_alt_descent_sign_and_magnitude_rate():
    alt, rate = guided_alt_from_climb(40.0, -2.0, alt_horizon=5.0)
    assert _close(alt, 30.0)          # 40 + (-2)*5
    assert _close(rate, 2.0)          # magnitude, floored
