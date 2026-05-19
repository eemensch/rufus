"""Unit tests for the pure helpers in quad_adapter (CLAIMS C20).

Covers the quaternion yaw helper and the body->world rotation,
which is identical in convention to the rover adapter's. No ROS
runtime: module-level pure functions only.
"""

import math

from rufus_sim_adapters.quad_adapter import _yaw_from_quat, body_to_world_xy


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


def test_rotation_identity_at_zero_yaw():
    x, y = body_to_world_xy(0.7, -0.2, 0.0)
    assert _close(x, 0.7) and _close(y, -0.2)


def test_rotation_plus_90_forward_to_north():
    x, y = body_to_world_xy(1.0, 0.0, math.pi / 2)
    assert _close(x, 0.0) and _close(y, 1.0)


def test_rotation_minus_90_forward_to_south():
    x, y = body_to_world_xy(1.0, 0.0, -math.pi / 2)
    assert _close(x, 0.0) and _close(y, -1.0)


def test_rotation_45_diagonal():
    x, y = body_to_world_xy(1.0, 0.0, math.pi / 4)
    assert _close(x, math.sqrt(0.5)) and _close(y, math.sqrt(0.5))


def test_rotation_lateral_component():
    x, y = body_to_world_xy(0.0, 1.0, math.pi / 2)
    assert _close(x, -1.0) and _close(y, 0.0)
