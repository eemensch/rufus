"""Unit tests for rufus_sim_game.agent_params.

Covers (a) `translate()` shaping FCU-param dicts from high_level
+ fcu blocks, including the high_level→fcu precedence rule; and
(b) `apply_high_level_to_capability()` patching a Capability
message with the option-by-option couplings (rover v_min coupled
to v_max, quad bank → lateral_accel via g·tan, plane v_max
re-deriving vz_max_up via the stored climb angle, ...).
"""

import math

import pytest

from rufus_sim_msgs.msg import AgentState, Capability

from rufus_sim_game.agent_params import (
    GRAVITY, ParameterOverrideError,
    apply_high_level_to_capability, translate,
)


# ---------- translate() ----------


def test_translate_rover_v_max_writes_two_params():
    out = translate('rover', {'v_max': 0.5}, None)
    assert out == {'WP_SPEED': 0.5, 'CRUISE_SPEED': 0.5}


def test_translate_rover_yaw_rate_converts_radians_to_deg():
    out = translate('rover', {'yaw_rate_max': math.pi}, None)
    assert out == pytest.approx({'ATC_STR_RAT_MAX': 180.0})


def test_translate_rover_min_turn_radius_writes_turn_radius():
    out = translate('rover', {'min_turn_radius': 1.5}, None)
    assert out == pytest.approx({'TURN_RADIUS': 1.5})


def test_translate_quad_bank_to_deg():
    out = translate('quad', {'bank_angle_max': math.radians(20)},
                    None)
    assert 'ATC_ANGLE_MAX' in out
    assert out['ATC_ANGLE_MAX'] == pytest.approx(20.0)


def test_translate_fcu_overrides_high_level_when_same_param():
    # high_level v_max writes WP_SPEED; fcu writes WP_SPEED again
    # at a different value; the fcu value must win.
    out = translate('rover', {'v_max': 1.0}, {'WP_SPEED': 2.5})
    assert out['WP_SPEED'] == 2.5
    assert out['CRUISE_SPEED'] == 1.0


def test_translate_unknown_high_level_raises():
    with pytest.raises(ParameterOverrideError, match='unknown'):
        translate('rover', {'fly_mode': 'whatever'}, None)


def test_translate_unknown_platform_raises():
    with pytest.raises(ParameterOverrideError,
                       match='unknown platform'):
        translate('submarine', {'v_max': 1.0}, None)


def test_translate_non_numeric_raises():
    with pytest.raises(ParameterOverrideError, match='not numeric'):
        translate('rover', {'v_max': 'fast'}, None)


# ---------- apply_high_level_to_capability() ----------


def _rover_cap() -> Capability:
    cap = Capability()
    cap.platform = AgentState.PLATFORM_ROVER
    cap.v_max = 1.0
    cap.v_min = -1.0
    cap.yaw_rate_max = 2.0
    cap.lateral_accel_max = 0.6 * GRAVITY  # ATC_TURN_MAX_G=0.6
    cap.min_turn_radius = 0.9  # native ArduRover TURN_RADIUS
    return cap


def _quad_cap() -> Capability:
    cap = Capability()
    cap.platform = AgentState.PLATFORM_QUADROTOR
    cap.v_max = 10.0
    cap.v_min = -10.0
    cap.vz_max_up = 2.5
    cap.vz_max_down = 1.5
    cap.yaw_rate_max = math.radians(90.0)
    cap.bank_angle_max = math.radians(30.0)
    cap.lateral_accel_max = GRAVITY * math.tan(cap.bank_angle_max)
    return cap


def _plane_cap() -> Capability:
    cap = Capability()
    cap.platform = AgentState.PLATFORM_FIXED_WING
    cap.v_min = 9.0
    cap.v_max = 22.0
    cap.bank_angle_max = math.radians(45.0)
    cap.climb_angle_max = math.radians(20.0)
    cap.vz_max_up = 22.0 * math.sin(cap.climb_angle_max)
    cap.vz_max_down = 22.0 * math.sin(math.radians(25.0))
    cap.lateral_accel_max = GRAVITY * math.tan(cap.bank_angle_max)
    return cap


def test_capability_patch_rover_v_max_couples_v_min():
    cap = _rover_cap()
    apply_high_level_to_capability('rover', {'v_max': 0.5}, cap)
    assert cap.v_max == pytest.approx(0.5)
    assert cap.v_min == pytest.approx(-0.5)


def test_capability_patch_rover_yaw_rate_recouples_lateral_accel():
    cap = _rover_cap()
    cap.lateral_accel_max = 0.0    # force a low baseline
    apply_high_level_to_capability(
        'rover', {'yaw_rate_max': 1.0, 'v_max': 2.0}, cap)
    # lateral_accel_max = max(prev, yaw_rate * v_max)
    assert cap.yaw_rate_max == 1.0
    assert cap.v_max == 2.0
    assert cap.lateral_accel_max == pytest.approx(2.0)


def test_capability_patch_rover_min_turn_radius_widen_ok():
    cap = _rover_cap()                       # native 0.9
    apply_high_level_to_capability(
        'rover', {'min_turn_radius': 1.5}, cap)
    assert cap.min_turn_radius == pytest.approx(1.5)


def test_capability_patch_rover_min_turn_radius_equal_native_ok():
    cap = _rover_cap()                       # native 0.9
    apply_high_level_to_capability(
        'rover', {'min_turn_radius': 0.9}, cap)
    assert cap.min_turn_radius == pytest.approx(0.9)


def test_capability_patch_rover_min_turn_radius_below_native_raises():
    cap = _rover_cap()                       # native 0.9
    with pytest.raises(ParameterOverrideError, match='below'):
        apply_high_level_to_capability(
            'rover', {'min_turn_radius': 0.5}, cap)


def test_capability_patch_quad_bank_recouples_lateral_accel():
    cap = _quad_cap()
    apply_high_level_to_capability(
        'quad', {'bank_angle_max': math.radians(20.0)}, cap)
    assert cap.bank_angle_max == pytest.approx(math.radians(20.0))
    assert cap.lateral_accel_max == pytest.approx(
        GRAVITY * math.tan(math.radians(20.0)))


def test_capability_patch_quad_v_max_couples_v_min():
    cap = _quad_cap()
    apply_high_level_to_capability('quad', {'v_max': 6.0}, cap)
    assert cap.v_max == 6.0
    assert cap.v_min == -6.0


def test_capability_patch_plane_v_max_rescales_climb_ceiling():
    cap = _plane_cap()
    apply_high_level_to_capability('plane', {'v_max': 30.0}, cap)
    assert cap.v_max == 30.0
    # vz_max_up should track v_max * sin(climb_angle_max).
    assert cap.vz_max_up == pytest.approx(
        30.0 * math.sin(cap.climb_angle_max))


def test_capability_patch_plane_climb_angle_recouples_vz():
    cap = _plane_cap()
    apply_high_level_to_capability(
        'plane', {'climb_angle_max': math.radians(10.0)}, cap)
    assert cap.climb_angle_max == pytest.approx(math.radians(10.0))
    assert cap.vz_max_up == pytest.approx(
        cap.v_max * math.sin(math.radians(10.0)))


def test_capability_patch_unknown_option_raises():
    cap = _rover_cap()
    with pytest.raises(ParameterOverrideError, match='unknown'):
        apply_high_level_to_capability(
            'rover', {'tractor_beam': 1.0}, cap)


def test_capability_patch_empty_is_noop():
    cap = _rover_cap()
    snapshot = (cap.v_max, cap.v_min, cap.yaw_rate_max,
                cap.lateral_accel_max)
    apply_high_level_to_capability('rover', None, cap)
    apply_high_level_to_capability('rover', {}, cap)
    assert (cap.v_max, cap.v_min, cap.yaw_rate_max,
            cap.lateral_accel_max) == snapshot
