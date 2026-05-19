"""Unit tests for the reference strategies.

Synthesises `Measurement` inputs without running ROS, calls
`control`, asserts on the returned `Twist`. The strategy
classes are pure Python so no node spin is needed.

Terminology mirrors `strategy.py`: the test fixture builds
`Measurement` records and exercises `Strategy.control`.
"""

import math

import pytest

from geometry_msgs.msg import Pose, Quaternion, Twist
from rufus_sim_msgs.msg import AgentState, Capability

from rufus_sim_strategies.reference import (
    ConstantBearingEvader, LeadPursuer, PurePursuitPursuer,
)
from rufus_sim_strategies.strategy import Measurement


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


def _agent(aid: str, x: float, y: float, *,
           z: float = 0.0, yaw: float = 0.0,
           platform: int = AgentState.PLATFORM_ROVER) -> AgentState:
    s = AgentState()
    s.agent_id = aid
    s.platform = platform
    s.pose = Pose()
    s.pose.position.x = x
    s.pose.position.y = y
    s.pose.position.z = z
    s.pose.orientation = _yaw_to_quat(yaw)
    return s


def _rover_capability() -> Capability:
    cap = Capability()
    cap.platform = AgentState.PLATFORM_ROVER
    cap.v_max = 1.0
    cap.v_min = -1.0
    cap.yaw_rate_max = 2.0
    return cap


def _quad_capability() -> Capability:
    cap = Capability()
    cap.platform = AgentState.PLATFORM_QUADROTOR
    cap.v_max = 5.0
    cap.v_min = -5.0
    cap.vz_max_up = 2.0
    cap.vz_max_down = 1.5
    cap.yaw_rate_max = 1.5
    return cap


def _plane_capability() -> Capability:
    cap = Capability()
    cap.platform = AgentState.PLATFORM_FIXED_WING
    cap.v_min = 9.0
    cap.v_max = 22.0
    cap.vz_max_up = 7.5
    cap.vz_max_down = 9.3
    cap.lateral_accel_max = 9.81   # g*tan(45 deg); psidot cap /V
    return cap


def _measurement(agents: dict, my_id: str, cap: Capability,
                 sim_time_s: float = 1.0) -> Measurement:
    return Measurement(
        sim_time_s=sim_time_s,
        agents=agents,
        my_state=agents[my_id],
        my_capability=cap,
        active_predicates=(),
        episode_id='test',
    )


# ----- PurePursuitPursuer -----


def test_pp_rover_target_dead_ahead():
    me = _agent('R0', 0.0, 0.0, yaw=0.0)
    target = _agent('R1', 10.0, 0.0)
    s = PurePursuitPursuer(agent_id='R0',
                           params={'target': 'R1', 'k_psi': 2.0,
                                   'v_factor': 1.0})
    s.reset()
    twist = s.control(_measurement({'R0': me, 'R1': target},
                                   'R0', _rover_capability()))
    assert twist.linear.x == pytest.approx(1.0)
    assert twist.angular.z == pytest.approx(0.0, abs=1e-9)


def test_pp_rover_target_to_left_turns_left():
    me = _agent('R0', 0.0, 0.0, yaw=0.0)
    target = _agent('R1', 0.0, 5.0)
    s = PurePursuitPursuer(agent_id='R0',
                           params={'target': 'R1', 'k_psi': 2.0})
    s.reset()
    twist = s.control(_measurement({'R0': me, 'R1': target},
                                   'R0', _rover_capability()))
    assert twist.linear.x == pytest.approx(0.0, abs=1e-9)
    assert twist.angular.z == pytest.approx(2.0)


def test_pp_rover_target_behind_reverses():
    me = _agent('R0', 0.0, 0.0, yaw=0.0)
    target = _agent('R1', -3.0, 0.0)
    s = PurePursuitPursuer(agent_id='R0',
                           params={'target': 'R1'})
    s.reset()
    twist = s.control(_measurement({'R0': me, 'R1': target},
                                   'R0', _rover_capability()))
    assert twist.linear.x == pytest.approx(-1.0)


def test_pp_plane_dead_ahead_no_turn():
    # Target east, plane facing east: heading error 0, so the
    # Dubins control is pure cruise (linear.x = airspeed, no
    # turn rate). linear.y is unused for the plane contract.
    me = _agent('P0', 0.0, 0.0, z=30.0,
                platform=AgentState.PLATFORM_FIXED_WING)
    target = _agent('Q0', 100.0, 0.0, z=30.0)
    s = PurePursuitPursuer(agent_id='P0',
                           params={'target': 'Q0',
                                   'v_factor': 0.5})
    s.reset()
    twist = s.control(_measurement({'P0': me, 'Q0': target},
                                   'P0', _plane_capability()))
    speed = max(9.0, 0.5 * 22.0)
    assert twist.linear.x == pytest.approx(speed)
    assert twist.angular.z == pytest.approx(0.0, abs=1e-9)
    assert twist.linear.z == pytest.approx(0.0, abs=1e-9)


def test_pp_plane_off_axis_turn_rate_capped():
    # Target due north, plane facing east: heading error +pi/2,
    # k_psi=2 -> commanded psidot ~= pi, must clip to the speed-
    # coupled coordinated-turn cap lateral_accel_max / V.
    me = _agent('P0', 0.0, 0.0, z=30.0,
                platform=AgentState.PLATFORM_FIXED_WING)
    target = _agent('Q0', 0.0, 100.0, z=30.0)
    s = PurePursuitPursuer(agent_id='P0',
                           params={'target': 'Q0',
                                   'v_factor': 0.5})
    s.reset()
    twist = s.control(_measurement({'P0': me, 'Q0': target},
                                   'P0', _plane_capability()))
    speed = max(9.0, 0.5 * 22.0)
    psidot_cap = 9.81 / speed
    assert twist.linear.x == pytest.approx(speed)
    assert twist.angular.z == pytest.approx(psidot_cap)


def test_pp_quad_descends_toward_lower_target():
    me = _agent('Q0', 0.0, 0.0, z=10.0, yaw=0.0,
                platform=AgentState.PLATFORM_QUADROTOR)
    target = _agent('Q1', 0.0, 0.0, z=2.0,
                    platform=AgentState.PLATFORM_QUADROTOR)
    s = PurePursuitPursuer(agent_id='Q0',
                           params={'target': 'Q1', 'k_pos': 0.5})
    s.reset()
    twist = s.control(_measurement({'Q0': me, 'Q1': target},
                                   'Q0', _quad_capability()))
    assert twist.linear.z == pytest.approx(-1.5)


def test_pp_missing_target_returns_zero_twist():
    me = _agent('R0', 0.0, 0.0)
    s = PurePursuitPursuer(agent_id='R0',
                           params={'target': 'R99'})
    s.reset()
    twist = s.control(_measurement({'R0': me}, 'R0',
                                   _rover_capability()))
    assert twist == Twist()


def test_pp_requires_target_param():
    s = PurePursuitPursuer(agent_id='R0', params={})
    with pytest.raises(ValueError, match='target'):
        s.reset()


# ----- LeadPursuer (stateful) -----


def test_lead_first_tick_falls_back_to_pure_pursuit():
    me = _agent('R0', 0.0, 0.0, yaw=0.0)
    target = _agent('R1', 10.0, 0.0)
    s = LeadPursuer(agent_id='R0',
                    params={'target': 'R1', 'lead_time_s': 2.0})
    s.reset()
    twist = s.control(_measurement({'R0': me, 'R1': target},
                                   'R0', _rover_capability(),
                                   sim_time_s=0.0))
    # No prior measurement -> v_target = 0 -> aim at current
    # target position; same answer as pure pursuit.
    assert twist.linear.x == pytest.approx(1.0)
    assert twist.angular.z == pytest.approx(0.0, abs=1e-9)


def test_lead_second_tick_uses_target_velocity():
    # Target moving at +1 m/s in +x. Pursuer at origin with
    # heading +x. Lead time = 2 s -> lead point at target_x +
    # 1*2 = +12. Heading from (0,0) to (12,0) is still 0;
    # angular.z stays 0 but the strategy is now consuming its
    # internal target-velocity estimate.
    me = _agent('R0', 0.0, 0.0, yaw=0.0)
    s = LeadPursuer(
        agent_id='R0',
        params={'target': 'R1', 'lead_time_s': 2.0,
                'v_factor': 1.0},
    )
    s.reset()
    s.control(_measurement(
        {'R0': me, 'R1': _agent('R1', 10.0, 0.0)},
        'R0', _rover_capability(), sim_time_s=0.0))
    twist = s.control(_measurement(
        {'R0': me, 'R1': _agent('R1', 11.0, 0.0)},
        'R0', _rover_capability(), sim_time_s=1.0))
    # Lead point still on +x axis -> heading_err = 0.
    assert twist.linear.x == pytest.approx(1.0)
    assert twist.angular.z == pytest.approx(0.0, abs=1e-9)


def test_lead_target_lateral_motion_steers_pursuer():
    # Target moves +1 m/s in +y. Pursuer at origin, heading +x.
    # On the second tick the lead point is offset in +y; the
    # pursuer must turn toward it.
    me = _agent('R0', 0.0, 0.0, yaw=0.0)
    s = LeadPursuer(
        agent_id='R0',
        params={'target': 'R1', 'lead_time_s': 2.0,
                'v_factor': 1.0, 'k_psi': 1.0},
    )
    s.reset()
    s.control(_measurement(
        {'R0': me, 'R1': _agent('R1', 10.0, 0.0)},
        'R0', _rover_capability(), sim_time_s=0.0))
    twist = s.control(_measurement(
        {'R0': me, 'R1': _agent('R1', 10.0, 1.0)},
        'R0', _rover_capability(), sim_time_s=1.0))
    # Target velocity y = 1 m/s, lead_time = 2 s -> lead point
    # (10, 0 + 1*2 = 2) -> bearing > 0 (target above horizon)
    # -> angular.z > 0.
    assert twist.angular.z > 0.0


def test_lead_state_persists_across_calls():
    # Sanity: the strategy stashes the measurement on self.
    # Without that, dt computation breaks on every call.
    me = _agent('R0', 0.0, 0.0)
    target = _agent('R1', 10.0, 0.0)
    s = LeadPursuer(agent_id='R0',
                    params={'target': 'R1'})
    s.reset()
    assert s._prev is None
    s.control(_measurement({'R0': me, 'R1': target},
                           'R0', _rover_capability()))
    assert s._prev is not None


# ----- ConstantBearingEvader -----


def test_cb_rover_flees_directly_away():
    me = _agent('R1', 0.0, 0.0, yaw=0.0)
    threat = _agent('R0', -5.0, 0.0)
    s = ConstantBearingEvader(agent_id='R1',
                              params={'threat': 'R0'})
    s.reset()
    twist = s.control(_measurement({'R0': threat, 'R1': me},
                                   'R1', _rover_capability()))
    assert twist.linear.x == pytest.approx(1.0)
    assert twist.angular.z == pytest.approx(0.0, abs=1e-9)


def test_cb_rover_threat_in_front_turns_around():
    me = _agent('R1', 0.0, 0.0, yaw=0.0)
    threat = _agent('R0', 5.0, 0.0)
    s = ConstantBearingEvader(agent_id='R1',
                              params={'threat': 'R0'})
    s.reset()
    twist = s.control(_measurement({'R0': threat, 'R1': me},
                                   'R1', _rover_capability()))
    assert twist.linear.x == pytest.approx(-1.0)


def test_cb_rover_orthogonal_offset():
    me = _agent('R1', 0.0, 0.0, yaw=0.0)
    threat = _agent('R0', 5.0, 0.0)
    s = ConstantBearingEvader(
        agent_id='R1',
        params={'threat': 'R0', 'bearing_offset': math.pi / 2},
    )
    s.reset()
    twist = s.control(_measurement({'R0': threat, 'R1': me},
                                   'R1', _rover_capability()))
    assert twist.linear.x == pytest.approx(0.0, abs=1e-9)
    assert twist.angular.z == pytest.approx(2.0)


def test_cb_plane_flees_via_capped_turn_rate():
    # Threat east, plane facing east: flee heading is west, so
    # the error is pi. A plane cannot fly backward; it commands
    # forward airspeed and reorients via the turn rate, clipped
    # to the speed-coupled coordinated-turn cap.
    me = _agent('P0', 0.0, 0.0, z=30.0,
                platform=AgentState.PLATFORM_FIXED_WING)
    threat = _agent('Q0', 100.0, 0.0, z=30.0)
    s = ConstantBearingEvader(agent_id='P0',
                              params={'threat': 'Q0'})
    s.reset()
    twist = s.control(_measurement({'P0': me, 'Q0': threat},
                                   'P0', _plane_capability()))
    speed = max(9.0, 1.0 * 22.0)
    psidot_cap = 9.81 / speed
    assert twist.linear.x == pytest.approx(speed)
    assert twist.angular.z == pytest.approx(psidot_cap)


def test_cb_missing_threat_returns_zero_twist():
    me = _agent('R1', 0.0, 0.0)
    s = ConstantBearingEvader(agent_id='R1',
                              params={'threat': 'R99'})
    s.reset()
    twist = s.control(_measurement({'R1': me}, 'R1',
                                   _rover_capability()))
    assert twist == Twist()


def test_cb_requires_threat_param():
    s = ConstantBearingEvader(agent_id='R1', params={})
    with pytest.raises(ValueError, match='threat'):
        s.reset()


# ----- ABC contract -----


def test_strategy_params_dict_is_isolated_from_caller():
    raw = {'target': 'R1'}
    s = PurePursuitPursuer(agent_id='R0', params=raw)
    raw['target'] = 'mutated'
    s.reset()
    assert s._target_id == 'R1'
