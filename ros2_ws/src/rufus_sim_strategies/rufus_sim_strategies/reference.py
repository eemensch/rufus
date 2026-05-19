"""Reference strategies for Stage 6.

  - `PurePursuitPursuer` — head straight at a named target
    (memoryless).
  - `ConstantBearingEvader` — maintain a fixed angle (default
    π, i.e. directly away) from the line of sight to a named
    threat (memoryless).
  - `LeadPursuer` — pursue a target using a finite-differenced
    target-velocity estimate to aim at a lead point. Stateful;
    holds the previous measurement for dt and target-velocity
    computation.

The terminology follows `strategy.py`: each strategy implements
`control(measurement) -> Twist`. The shared helpers that map a
desired world-frame heading or position to a per-platform
`Twist` live in `heading.py`.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
from rufus_sim_msgs.msg import AgentState

from .heading import (
    plane_twist, quad_twist, rover_twist, wrap_pi,
)
from .registry import register
from .strategy import Measurement, Strategy


def _platform_twist_to_position(
        my_state: AgentState, target_xyz: tuple,
        cap, *, v_factor: float, k_psi: float, k_pos: float
) -> Twist:
    """Compute a Twist that drives `my_state` toward
    `target_xyz` (world ENU), per-platform.

    Used by both PurePursuitPursuer and LeadPursuer; factored
    out so the only difference between them is the choice of
    target point (current vs leading).
    """
    platform = cap.platform
    me_x = my_state.pose.position.x
    me_y = my_state.pose.position.y
    me_z = my_state.pose.position.z
    if platform == AgentState.PLATFORM_ROVER:
        heading = math.atan2(target_xyz[1] - me_y,
                             target_xyz[0] - me_x)
        return rover_twist(my_state, heading, cap,
                           v_factor=v_factor, k_psi=k_psi)
    if platform == AgentState.PLATFORM_QUADROTOR:
        return quad_twist(my_state, target_xyz, cap,
                          v_factor=v_factor, k_pos=k_pos,
                          k_psi=k_psi)
    if platform == AgentState.PLATFORM_FIXED_WING:
        heading = math.atan2(target_xyz[1] - me_y,
                             target_xyz[0] - me_x)
        return plane_twist(my_state, heading, cap,
                           v_factor=v_factor, k_psi=k_psi,
                           vz_world=k_pos * (target_xyz[2] - me_z))
    raise ValueError(f"unsupported platform {platform}")


# ---------------------------------------------------------------
# PurePursuitPursuer (memoryless)
# ---------------------------------------------------------------


class PurePursuitPursuer(Strategy):
    """Drive toward a named target's *current* position.

    Memoryless: ignores history, looks only at the latest
    measurement.

    Required params:
      target          (str)    agent_id of the target.

    Optional params:
      v_factor        (float)  speed cap as a fraction of v_max
                               (default 1.0).
      k_psi           (float)  yaw-rate gain on heading error
                               (default 2.0).
      k_pos           (float)  position-error gain for quad
                               z-axis and plane vz_world
                               (default 0.7).
    """

    def reset(self) -> None:
        if 'target' not in self.params:
            raise ValueError(
                f"{type(self).__name__} ({self.agent_id!r}) "
                f"requires `target` param"
            )
        self._target_id: str = str(self.params['target'])
        self._v_factor: float = float(
            self.params.get('v_factor', 1.0))
        self._k_psi: float = float(self.params.get('k_psi', 2.0))
        self._k_pos: float = float(self.params.get('k_pos', 0.7))

    def control(self, measurement: Measurement) -> Twist:
        if self._target_id not in measurement.agents:
            return Twist()
        target = measurement.agents[self._target_id]
        return _platform_twist_to_position(
            measurement.my_state,
            (target.pose.position.x,
             target.pose.position.y,
             target.pose.position.z),
            measurement.my_capability,
            v_factor=self._v_factor,
            k_psi=self._k_psi,
            k_pos=self._k_pos,
        )


register('pure_pursuit_pursuer', PurePursuitPursuer)


# ---------------------------------------------------------------
# LeadPursuer (stateful — finite-differences target velocity)
# ---------------------------------------------------------------


class LeadPursuer(Strategy):
    """Lead-pursuit: aim at a point ahead of the target.

    Stateful: keeps the previous `Measurement` so that the
    target's world-frame velocity can be finite-differenced
    each tick. The lead point is

        p_lead = p_target + lead_time_s * v_target

    and the strategy then runs the same per-platform
    pursuit-to-position conversion as `PurePursuitPursuer`.
    Demonstrates the stateful pattern; for slow / non-evading
    targets `lead_time_s = 0` collapses to memoryless pure
    pursuit.

    Required params:
      target          (str)    agent_id of the target.

    Optional params:
      lead_time_s     (float)  seconds to project ahead
                               (default 1.0).
      v_factor        (float)  speed cap as a fraction of v_max
                               (default 1.0).
      k_psi, k_pos    same as PurePursuitPursuer.
    """

    def reset(self) -> None:
        if 'target' not in self.params:
            raise ValueError(
                f"{type(self).__name__} ({self.agent_id!r}) "
                f"requires `target` param"
            )
        self._target_id: str = str(self.params['target'])
        self._lead_time_s: float = float(
            self.params.get('lead_time_s', 1.0))
        self._v_factor: float = float(
            self.params.get('v_factor', 1.0))
        self._k_psi: float = float(self.params.get('k_psi', 2.0))
        self._k_pos: float = float(self.params.get('k_pos', 0.7))
        # Strategy state: cached previous measurement. The first
        # call sees `_prev is None` and falls back to memoryless
        # pure pursuit (no velocity estimate yet).
        self._prev: Measurement | None = None

    def control(self, measurement: Measurement) -> Twist:
        if self._target_id not in measurement.agents:
            self._prev = measurement
            return Twist()
        target = measurement.agents[self._target_id]
        tx = target.pose.position.x
        ty = target.pose.position.y
        tz = target.pose.position.z

        vx = vy = vz = 0.0
        if (self._prev is not None
                and self._target_id in self._prev.agents):
            dt = measurement.sim_time_s - self._prev.sim_time_s
            if dt > 1e-6:
                prev_target = self._prev.agents[self._target_id]
                vx = (tx - prev_target.pose.position.x) / dt
                vy = (ty - prev_target.pose.position.y) / dt
                vz = (tz - prev_target.pose.position.z) / dt

        lead_xyz = (
            tx + vx * self._lead_time_s,
            ty + vy * self._lead_time_s,
            tz + vz * self._lead_time_s,
        )
        out = _platform_twist_to_position(
            measurement.my_state,
            lead_xyz,
            measurement.my_capability,
            v_factor=self._v_factor,
            k_psi=self._k_psi,
            k_pos=self._k_pos,
        )
        self._prev = measurement
        return out


register('lead_pursuer', LeadPursuer)


# ---------------------------------------------------------------
# ConstantBearingEvader (memoryless)
# ---------------------------------------------------------------


class ConstantBearingEvader(Strategy):
    """Keep a fixed angle from the line of sight to a named
    threat.

    Default `bearing_offset = π` flees directly away. `±π/2`
    produce orthogonal escape (orbit-style); arbitrary offsets
    are accepted and not normalised.

    Required params:
      threat          (str)    agent_id of the pursuer.

    Optional params:
      bearing_offset  (float)  angle in rad between LOS to the
                               threat and the evader's desired
                               world heading (default π).
      v_factor, k_psi, k_pos
                               same meaning as
                               `PurePursuitPursuer`.
      flee_distance_m (float)  for quad/plane, the runner picks
                               a virtual waypoint
                               `flee_distance_m` ahead in the
                               desired direction (default 100 m).
    """

    def reset(self) -> None:
        if 'threat' not in self.params:
            raise ValueError(
                f"{type(self).__name__} ({self.agent_id!r}) "
                f"requires `threat` param"
            )
        self._threat_id: str = str(self.params['threat'])
        self._bearing_offset: float = float(
            self.params.get('bearing_offset', math.pi))
        self._v_factor: float = float(
            self.params.get('v_factor', 1.0))
        self._k_psi: float = float(self.params.get('k_psi', 2.0))
        self._k_pos: float = float(self.params.get('k_pos', 0.7))
        self._flee_dist: float = float(
            self.params.get('flee_distance_m', 100.0))

    def control(self, measurement: Measurement) -> Twist:
        if self._threat_id not in measurement.agents:
            return Twist()
        threat = measurement.agents[self._threat_id]
        me = measurement.my_state
        cap = measurement.my_capability
        platform = cap.platform

        dx = threat.pose.position.x - me.pose.position.x
        dy = threat.pose.position.y - me.pose.position.y
        los_to_threat = math.atan2(dy, dx)
        desired_heading = wrap_pi(
            los_to_threat + self._bearing_offset)

        if platform == AgentState.PLATFORM_ROVER:
            return rover_twist(me, desired_heading, cap,
                               v_factor=self._v_factor,
                               k_psi=self._k_psi)
        if platform == AgentState.PLATFORM_QUADROTOR:
            target_x = (me.pose.position.x
                        + self._flee_dist * math.cos(desired_heading))
            target_y = (me.pose.position.y
                        + self._flee_dist * math.sin(desired_heading))
            target_z = me.pose.position.z   # hold altitude
            return quad_twist(me, (target_x, target_y, target_z),
                              cap,
                              v_factor=self._v_factor,
                              k_pos=self._k_pos,
                              k_psi=self._k_psi)
        if platform == AgentState.PLATFORM_FIXED_WING:
            return plane_twist(me, desired_heading, cap,
                               v_factor=self._v_factor,
                               k_psi=self._k_psi,
                               vz_world=0.0)
        raise ValueError(
            f"{type(self).__name__} ({self.agent_id!r}): "
            f"unsupported platform {platform}"
        )


register('constant_bearing_evader', ConstantBearingEvader)
