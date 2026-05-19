"""Unit tests for the pure helpers in rover_adapter.

First automated coverage of rufus_sim_adapters (CLAIMS C20). Covers
the body->world rotation and the command clamp/saturation logic
extracted from the Node methods, plus the quaternion yaw helper.
No ROS runtime: these import the module-level pure functions only.
"""

import math

from mavros_msgs.msg import PositionTarget

from rufus_sim_adapters.rover_adapter import (
    _yaw_from_quat,
    body_to_world_xy,
    clamp_command,
    rover_setpoint,
)


def _close(a, b, tol=1e-9):
    return math.isclose(a, b, abs_tol=tol)


# --- _yaw_from_quat ------------------------------------------------

def test_yaw_identity_quat_is_zero():
    assert _close(_yaw_from_quat(0.0, 0.0, 0.0, 1.0), 0.0)


def test_yaw_plus_90_about_z():
    h = math.sqrt(0.5)  # sin/cos of 45 deg = quat for 90 deg yaw
    assert _close(_yaw_from_quat(0.0, 0.0, h, h), math.pi / 2)


def test_yaw_minus_90_about_z():
    h = math.sqrt(0.5)
    assert _close(_yaw_from_quat(0.0, 0.0, -h, h), -math.pi / 2)


def test_yaw_180_about_z():
    assert _close(abs(_yaw_from_quat(0.0, 0.0, 1.0, 0.0)), math.pi)


# --- body_to_world_xy ----------------------------------------------

def test_rotation_identity_at_zero_yaw():
    x, y = body_to_world_xy(0.5, -0.3, 0.0)
    assert _close(x, 0.5) and _close(y, -0.3)


def test_rotation_plus_90_maps_forward_to_north():
    # body +x at yaw=+90deg points to world +y (ENU north)
    x, y = body_to_world_xy(1.0, 0.0, math.pi / 2)
    assert _close(x, 0.0) and _close(y, 1.0)


def test_rotation_minus_90_maps_forward_to_south():
    x, y = body_to_world_xy(1.0, 0.0, -math.pi / 2)
    assert _close(x, 0.0) and _close(y, -1.0)


def test_rotation_45_deg_diagonal():
    x, y = body_to_world_xy(1.0, 0.0, math.pi / 4)
    assert _close(x, math.sqrt(0.5)) and _close(y, math.sqrt(0.5))


def test_rotation_lateral_component():
    # body +y (left) at yaw=+90deg points to world -x (ENU west)
    x, y = body_to_world_xy(0.0, 1.0, math.pi / 2)
    assert _close(x, -1.0) and _close(y, 0.0)


# --- clamp_command -------------------------------------------------

def test_clamp_within_bounds_no_saturation():
    vx, wz, sl, sa = clamp_command(1.0, 0.5, 2.0, 2.0)
    assert _close(vx, 1.0) and _close(wz, 0.5)
    assert sl is False and sa is False


def test_clamp_vx_over_vmax():
    vx, wz, sl, sa = clamp_command(5.0, 0.0, 2.0, 2.0)
    assert _close(vx, 2.0) and sl is True and sa is False


def test_clamp_vx_under_neg_vmax():
    vx, wz, sl, sa = clamp_command(-5.0, 0.0, 2.0, 2.0)
    assert _close(vx, -2.0) and sl is True


def test_clamp_wz_over_wmax():
    vx, wz, sl, sa = clamp_command(0.0, 9.0, 2.0, 2.0)
    assert _close(wz, 2.0) and sa is True and sl is False


def test_clamp_exactly_at_bound_not_saturated():
    vx, wz, sl, sa = clamp_command(2.0, -2.0, 2.0, 2.0)
    assert _close(vx, 2.0) and _close(wz, -2.0)
    assert sl is False and sa is False


# --- rover_setpoint (S1.4 reverse-bug fix) -------------------------

# Velocity + yaw_rate valid; position, accel, yaw ignored.
_EXPECTED_MASK = (
    PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY
    | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX
    | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW
)


def test_rover_setpoint_mask_is_velocity_plus_yawrate():
    tm, frame, vx, wz, sl, sa = rover_setpoint(0.5, 0.0, 2.0, 2.0)
    # identical to what setpoint_velocity sent (1479): pos/accel/
    # yaw ignored, velocity + yaw_rate active — still velocity-level
    assert tm == _EXPECTED_MASK == 1479
    assert not tm & PositionTarget.IGNORE_VX
    assert not tm & PositionTarget.IGNORE_YAW_RATE


def test_rover_setpoint_frame_is_body_ned():
    # BODY_NED is the fix: ArduRover's is_negative(packet.vx)
    # then tests body-forward, not world NED-north.
    _, frame, *_ = rover_setpoint(0.5, 0.0, 2.0, 2.0)
    assert frame == PositionTarget.FRAME_BODY_NED == 8


def test_rover_setpoint_passes_through_within_envelope():
    tm, frame, vx, wz, sl, sa = rover_setpoint(0.5, -0.3, 2.0, 2.0)
    assert _close(vx, 0.5) and _close(wz, -0.3)
    assert sl is False and sa is False


def test_rover_setpoint_preserves_reverse_sign():
    # a genuine reverse command must stay negative (ArduRover
    # handles direction from body-forward sign, not a heuristic)
    _, _, vx, wz, _, _ = rover_setpoint(-0.4, 0.0, 2.0, 2.0)
    assert _close(vx, -0.4)


def test_rover_setpoint_clamps_and_flags_saturation():
    _, _, vx, wz, sl, sa = rover_setpoint(9.0, -9.0, 2.0, 2.0)
    assert _close(vx, 2.0) and _close(wz, -2.0)
    assert sl is True and sa is True


def test_rover_setpoint_pure_yaw_zero_speed():
    # vx=0, wz!=0 -> speed 0 + turn rate (spin in place); the
    # mask is unchanged, ArduRover branch handles speed==0
    _, _, vx, wz, _, _ = rover_setpoint(0.0, 0.8, 2.0, 2.0)
    assert _close(vx, 0.0) and _close(wz, 0.8)
