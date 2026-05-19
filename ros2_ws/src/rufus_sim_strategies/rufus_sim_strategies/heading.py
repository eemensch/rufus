"""Shared helpers for converting a desired world-frame heading
into a Twist on the agent's `cmd_vel` frame.

Reference strategies that share the same control structure
(pursue / flee toward a heading at a chosen speed) call
`twist_from_heading` so the platform-frame branching lives in
one place. The Twist returned matches the `cmd_vel` frame
contract from `docs/control.md`:

  - rover, quad: body frame; angular.z is the yaw-rate command,
    linear.x is the body-forward speed; quad's vertical axis is
    handled by `linear.z` and rotates with the agent's yaw so a
    world-frame target altitude needs the rotated form.
  - plane: body-frame Dubins-airplane control inputs; linear.x
    is airspeed, angular.z is turn rate, linear.z is climb rate
    (the strategy turns toward the heading via a capped turn
    rate, it does not emit a world velocity vector).

These helpers do not mutate any input.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
from rufus_sim_msgs.msg import AgentState, Capability


def yaw_from_quaternion(qw: float, qx: float, qy: float,
                        qz: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _yaw(state: AgentState) -> float:
    o = state.pose.orientation
    return yaw_from_quaternion(o.w, o.x, o.y, o.z)


def rover_twist(my_state: AgentState, target_world_heading: float,
                cap: Capability, *, v_factor: float, k_psi: float
                ) -> Twist:
    """Body-frame Twist for a skid-steer rover heading toward a
    desired world-frame heading.

    `linear.x` is forward speed scaled by `cos(heading_err)` so the
    rover slows or reverses when grossly off-axis (rather than
    arcing). `angular.z` is proportional yaw rate, capped at
    the rover's `yaw_rate_max`.
    """
    err = wrap_pi(target_world_heading - _yaw(my_state))
    out = Twist()
    out.linear.x = v_factor * cap.v_max * math.cos(err)
    yaw_cap = cap.yaw_rate_max or float('inf')
    out.angular.z = max(-yaw_cap, min(yaw_cap, k_psi * err))
    return out


def quad_twist(my_state: AgentState, target_world_pos: tuple,
               cap: Capability, *, v_factor: float, k_pos: float,
               k_psi: float) -> Twist:
    """Body-frame Twist for a quad targeting a world-frame
    position.

    Computes a world-frame proportional-on-error velocity, caps
    at `v_factor * v_max`, rotates into body frame using current
    yaw. `angular.z` aims the quad along the horizontal velocity
    so a strategy that wants to "face the target" gets it for
    free; pure pursuit doesn't strictly need this but planes and
    cameras downstream do.
    """
    dx = target_world_pos[0] - my_state.pose.position.x
    dy = target_world_pos[1] - my_state.pose.position.y
    dz = target_world_pos[2] - my_state.pose.position.z
    psi = _yaw(my_state)
    vx_w = k_pos * dx
    vy_w = k_pos * dy
    norm = math.hypot(vx_w, vy_w)
    speed_cap = v_factor * cap.v_max
    if norm > speed_cap:
        vx_w *= speed_cap / norm
        vy_w *= speed_cap / norm
    c = math.cos(-psi)
    s = math.sin(-psi)
    out = Twist()
    out.linear.x = c * vx_w - s * vy_w
    out.linear.y = s * vx_w + c * vy_w
    vz_cap_up = cap.vz_max_up or speed_cap
    vz_cap_down = cap.vz_max_down or speed_cap
    vz = k_pos * dz
    if vz > vz_cap_up:
        vz = vz_cap_up
    elif vz < -vz_cap_down:
        vz = -vz_cap_down
    out.linear.z = vz
    desired_yaw = math.atan2(dy, dx)
    yaw_err = wrap_pi(desired_yaw - psi)
    yaw_cap = cap.yaw_rate_max or float('inf')
    out.angular.z = max(-yaw_cap, min(yaw_cap, k_psi * yaw_err))
    return out


def plane_twist(my_state: AgentState, target_world_heading: float,
                cap: Capability, *, v_factor: float, k_psi: float,
                vz_world: float = 0.0) -> Twist:
    """Body-frame Dubins-airplane control Twist for a fixed-wing
    turning toward a desired world-frame heading.

    The strategy commands the kinematic-model inputs directly,
    matching the fixed_wing adapter contract:

    - `linear.x` = airspeed `max(v_min, v_factor * v_max)` so a
      0.5 factor on a plane with v_min=9 still flies.
    - `angular.z` = turn rate, proportional to the heading error
      (`k_psi * err`), capped at the speed-coupled coordinated-
      turn rate `lateral_accel_max / V` (the documented
      `psi_dot_vel(V)`; the adapter re-clips with the same
      bound). The plane cannot side-slip, so heading is closed
      only through this turn rate, not a velocity vector.
    - `linear.z` = climb rate, clipped to the vertical envelope;
      defaults to level flight.
    """
    speed = max(cap.v_min, v_factor * cap.v_max)
    err = wrap_pi(target_world_heading - _yaw(my_state))
    out = Twist()
    out.linear.x = speed
    if cap.lateral_accel_max:
        psidot_cap = cap.lateral_accel_max / speed
    else:
        psidot_cap = cap.yaw_rate_max or float('inf')
    out.angular.z = max(-psidot_cap, min(psidot_cap, k_psi * err))
    vz_cap_up = cap.vz_max_up or speed
    vz_cap_down = cap.vz_max_down or speed
    out.linear.z = max(-vz_cap_down, min(vz_cap_up, vz_world))
    return out
