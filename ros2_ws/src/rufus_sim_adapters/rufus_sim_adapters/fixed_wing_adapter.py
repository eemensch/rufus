#!/usr/bin/env python3
"""Fixed-wing adapter for pursuit-evasion simulation.

Bridges <agent_ns>/cmd_vel (TwistStamped) onto ArduPlane
GUIDED-mode control via MAVROS, and republishes agent state in
the rufus_sim_msgs schema.

Strategy contract — the Dubins-airplane kinematic-model control
inputs, commanded and constrained directly so a pursuit-evasion
min-max can be solved over exactly this admissible set:

    cmd_vel.twist.linear.x  = airspeed V        (m/s)
    cmd_vel.twist.angular.z = turn rate psidot  (rad/s, ENU/CCW)
    cmd_vel.twist.linear.z  = climb rate        (m/s, +up)

    V        in [v_min, v_max]
    |psidot| <= g*tan(bank_max)/V    (speed-coupled coord turn)
    |climb|  <= V*sin(climb_*_angle)

`_project` clips to this set (pure helper `dubins_airplane_clip`)
and reports per-bound saturation. psidot is a first-class
control input — it is NOT synthesised from a heading-to-a-carrot
or a world velocity vector. Like the rover, the body-frame
angular rate is the game input; the adapter does not hide it.

Realization layer (adapter -> FCU, unchanged mechanism): the
`MAV_CMD_GUIDED_CHANGE_{SPEED,HEADING,ALTITUDE}` slew commands,
NOT a `SET_POSITION_TARGET` velocity setpoint. Verified from
firmware (`GCS_MAVLink_Plane.cpp`): ArduPlane's
`handle_set_position_target_local_ned` is an altitude-only stub
that ignores velocity entirely, so a velocity setpoint is a
silent no-op (the plane loiters/circles, never tracking). The
`GUIDED_CHANGE_*` family (gated by
`AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED`, default-on and
confirmed compiled in the vendored SITL binary) is ArduPlane's
purpose-built continuous companion-control API. Because HEADING
is heading- not rate-addressable, the adapter integrates the
commanded `psidot` into a heading target (`_setpoint_tick`,
re-seeded from pose while cmd_vel is stale); SPEED takes V;
ALTITUDE takes a target alt + climb rate from `climb`. ArduPlane
independently bounds the heading slew by its coordinated-turn
accel limit (the same g·tan(bank)/V cap the clip enforces, so
the FCU and the admissible set agree). Saturation flags
`airspeed`/`turn_rate`/`climb_rate` reflect which clip bounds
were active.

Hover commands (V_des < v_min) are not refused; the adapter
clamps to v_min and raises `SaturationFlags.airspeed`. The
strategy is responsible for not commanding hovers it cares about.

The bring-up state machine reproduces the canonical zephyr
takeoff procedure from `external/ardupilot_gazebo/README.md`
(`mode fbwa` → `arm throttle` → `rc 3 1800` → `mode circle`),
adapted for our GUIDED setpoint pipeline. The full sequence:

  1. set `ARMING_CHECK=0` (skip pre-arm gates).
  2. set `MAV_GCS_SYSID=1` so ArduPlane recognises MAVROS as
     the GCS and honours its RC overrides. Without this, AP
     ignores `/mavros/rc/override` (its default `MAV_GCS_SYSID`
     is 255 but MAVROS sends with sysid 1).
  3. switch mode to FBWA.
  4. arm.
  5. publish RC override on channel 3 at PWM 1800 every
     bring-up tick (well within AP's `RC_OVERRIDE_TIME` of 3 s).
     The zephyr SDF spawns the airframe on its side
     (roll = -90 deg) so throttle = vertical thrust; the plane
     climbs straight up.
  6. once local altitude exceeds the threshold (default 40 m),
     release the override and switch mode to GUIDED.
  7. READY.

ArduPlane parameter names in this firmware tree (4.6-dev,
HEAD `3d313de9`) are the *legacy* AIRSPEED_MIN/CRUISE/MAX,
ROLL_LIMIT_DEG, PTCH_LIM_*_DEG, NAVL1_PERIOD/DAMPING.
ArduPlane has not yet been migrated to the SI-suffix naming
that copter and rover went through (see project memory note);
this adapter would need name updates if pinning to a future AP
release that completes the rename.
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from geometry_msgs.msg import (
    Twist,
    TwistStamped,
    PoseStamped,
)
from sensor_msgs.msg import Imu
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import (
    ParamPull, ParamSetV2, SetMode, CommandBool, StreamRate,
    CommandLong, CommandInt,
)

from rufus_sim_msgs.msg import AgentState, Capability, SaturationFlags


GRAVITY = 9.81

# ArduPlane GUIDED slew commands (common MAVLink; consumed by
# GCS_MAVLink_Plane.cpp handle_command_*_guided_slew_commands,
# gated by AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED=1).
_MAV_CMD_GUIDED_CHANGE_SPEED = 43000
_MAV_CMD_GUIDED_CHANGE_ALTITUDE = 43001
_MAV_CMD_GUIDED_CHANGE_HEADING = 43002
_SPEED_TYPE_AIRSPEED = 0          # firmware rejects groundspeed
_HEADING_TYPE_COURSE_OVER_GROUND = 0   # course = integrated psidot
_MAV_FRAME_GLOBAL_RELATIVE_ALT = 6     # alt relative to home


class BringUpState(Enum):
    WAIT_CONNECT = auto()
    REQUESTING_STREAMS = auto()
    PULLING_PARAMS = auto()
    DISABLING_CHECKS = auto()
    SETTING_GCS_SYSID = auto()
    SETTING_MODE = auto()
    ARMING = auto()
    TAKING_OFF = auto()
    SETTING_GUIDED = auto()
    READY = auto()


class FixedWingAdapter(Node):
    def __init__(self) -> None:
        super().__init__('fixed_wing_adapter')

        # --- parameters
        self.declare_parameter('agent_id', 'F0')
        self.declare_parameter('agent_ns', '')
        self.declare_parameter('mavros_ns', '/mavros')
        self.declare_parameter('role', AgentState.ROLE_NEUTRAL)
        self.declare_parameter('disable_arming_check', True)
        self.declare_parameter('cmd_timeout_s', 1.0)
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('state_rate_hz', 50.0)
        self.declare_parameter('takeoff_alt_threshold_m', 40.0)
        self.declare_parameter('takeoff_throttle_pwm', 1800)
        # MAV_GCS_SYSID must match the MAVLink sender id used by
        # this MAVROS instance (`system_id` in mavros_node);
        # otherwise ArduPlane silently drops RC overrides during
        # takeoff because they appear to come from the "wrong" GCS.
        # The single-agent default is sysid 1; multi-agent runs
        # pass `240 + instance` from the launch.
        self.declare_parameter('mav_gcs_sysid', 1)

        self.agent_id: str = self.get_parameter('agent_id').value
        self.agent_ns: str = self.get_parameter('agent_ns').value
        self.mavros_ns: str = self.get_parameter('mavros_ns').value
        self.role: int = int(self.get_parameter('role').value)
        self.disable_arming_check: bool = bool(
            self.get_parameter('disable_arming_check').value
        )
        self.cmd_timeout_s: float = float(
            self.get_parameter('cmd_timeout_s').value
        )
        self.takeoff_alt_threshold_m: float = float(
            self.get_parameter('takeoff_alt_threshold_m').value
        )
        self.takeoff_throttle_pwm: int = int(
            self.get_parameter('takeoff_throttle_pwm').value
        )
        self.mav_gcs_sysid: int = int(
            self.get_parameter('mav_gcs_sysid').value
        )

        # --- runtime state
        self._latest_cmd: Twist = Twist()
        self._latest_cmd_time = self.get_clock().now()
        # GUIDED_CHANGE_* are continuous-slew commands; decimate the
        # setpoint timer down to this cadence to avoid saturating
        # the MAVROS command service (3 calls per send).
        self._last_slew_time = self.get_clock().now()
        # Commanded heading (rad, world ENU): the strategy commands
        # turn rate psidot; the adapter integrates it here and feeds
        # the heading to the GUIDED_CHANGE_HEADING realization. None
        # until first seeded from pose; re-seeded from pose whenever
        # cmd_vel goes stale (cruise) so resume starts from the
        # actual heading, not a drifted integral.
        self._cmd_heading: float | None = None
        self._fcu_state: State | None = None
        self._latest_pose: PoseStamped | None = None
        self._latest_twist_body: TwistStamped | None = None
        self._latest_imu: Imu | None = None
        self._caps: Capability | None = None
        self._saturation = SaturationFlags()
        self._bring_up = BringUpState.WAIT_CONNECT
        self._inflight_request = False
        # Cruise airspeed used as a representative point for the
        # speed-dependent yaw-rate-max derivation; populated in
        # _on_caps_params_done.
        self._airspeed_cruise = 12.0
        # Asymmetric pitch envelope; populated in
        # _on_caps_params_done.
        self._climb_angle_up = math.radians(20.0)
        self._climb_angle_down = math.radians(25.0)

        # --- QoS
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # --- topic helpers
        def m(name: str) -> str:
            return f'{self.mavros_ns}/{name}'

        def a(name: str) -> str:
            ns = self.agent_ns.rstrip('/')
            return f'{ns}/{name}' if ns else f'/{name}'

        # --- publishers
        # No setpoint_velocity publisher: ArduPlane GUIDED ignores
        # SET_POSITION_TARGET velocity (firmware-verified stub).
        # The GUIDED_CHANGE_* slew commands are sent via the MAVROS
        # command services created below.
        self._pub_rc_override = self.create_publisher(
            OverrideRCIn, m('rc/override'), 10
        )
        self._pub_state = self.create_publisher(
            AgentState, a('state'), 10
        )
        self._pub_capability = self.create_publisher(
            Capability, a('capability'), latched_qos
        )

        # --- subscribers
        self.create_subscription(
            TwistStamped, a('cmd_vel'), self._cmd_vel_cb, 10
        )
        self.create_subscription(
            State, m('state'), self._fcu_state_cb, 10
        )
        self.create_subscription(
            PoseStamped, m('local_position/pose'),
            self._pose_cb, sensor_qos,
        )
        self.create_subscription(
            TwistStamped, m('local_position/velocity_body'),
            self._twist_cb, sensor_qos,
        )
        # local_position/velocity_body carries linear only; the
        # body-frame angular rate comes from the imu plugin (same
        # MAVROS gap as the rover; Stage 1 Blocker 1 / plan S1.2).
        # Fixed-wing does not command yaw, but a faithful state
        # report still needs the actual body rates.
        self.create_subscription(
            Imu, m('imu/data'), self._imu_cb, sensor_qos,
        )

        # --- service clients
        # GUIDED_CHANGE_* slew commands. SPEED/HEADING are
        # params-only (COMMAND_LONG); ALTITUDE needs frame+z
        # (COMMAND_INT). `command` plugin is in the allowlist.
        self._cli_cmd = self.create_client(
            CommandLong, m('cmd/command')
        )
        self._cli_cmd_int = self.create_client(
            CommandInt, m('cmd/command_int')
        )
        self._cli_pull = self.create_client(
            ParamPull, m('param/pull')
        )
        self._cli_param_set = self.create_client(
            ParamSetV2, m('param/set')
        )
        self._cli_param_get = self.create_client(
            GetParameters, m('param/get_parameters')
        )
        self._cli_set_mode = self.create_client(
            SetMode, m('set_mode')
        )
        self._cli_arm = self.create_client(
            CommandBool, m('cmd/arming')
        )
        self._cli_set_stream = self.create_client(
            StreamRate, m('set_stream_rate')
        )

        # --- timers
        sp_period = 1.0 / float(
            self.get_parameter('setpoint_rate_hz').value
        )
        st_period = 1.0 / float(
            self.get_parameter('state_rate_hz').value
        )
        self.create_timer(sp_period, self._setpoint_tick)
        self.create_timer(st_period, self._state_tick)
        self.create_timer(0.5, self._bring_up_tick)

        self.get_logger().info(
            f'fixed_wing_adapter started for agent_id={self.agent_id}, '
            f'mavros_ns={self.mavros_ns}, agent_ns={self.agent_ns!r}'
        )

    # ------------------------------------------------------------------
    # subscriber callbacks
    # ------------------------------------------------------------------

    def _cmd_vel_cb(self, msg: TwistStamped) -> None:
        self._latest_cmd = msg.twist
        self._latest_cmd_time = self.get_clock().now()

    def _fcu_state_cb(self, msg: State) -> None:
        self._fcu_state = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._latest_pose = msg

    def _twist_cb(self, msg: TwistStamped) -> None:
        self._latest_twist_body = msg

    def _imu_cb(self, msg: Imu) -> None:
        self._latest_imu = msg

    # ------------------------------------------------------------------
    # bring-up state machine
    # ------------------------------------------------------------------

    def _bring_up_tick(self) -> None:
        if self._inflight_request:
            return

        s = self._bring_up
        if s is BringUpState.WAIT_CONNECT:
            # service_is_ready guards against an rclpy race where
            # call_async on an undiscovered service silently never
            # resolves; without this gate the adapter occasionally
            # gets stuck post-startup under multi-agent load.
            if self._fcu_state is not None \
                    and self._fcu_state.connected \
                    and self._cli_set_stream.service_is_ready():
                self.get_logger().info(
                    'FCU connected; requesting streams'
                )
                self._inflight_request = True
                req = StreamRate.Request(
                    stream_id=0, message_rate=10, on_off=True,
                )
                fut = self._cli_set_stream.call_async(req)
                fut.add_done_callback(self._on_streams_done)

        elif s is BringUpState.REQUESTING_STREAMS:
            pass

        elif s is BringUpState.PULLING_PARAMS:
            pass

        elif s is BringUpState.DISABLING_CHECKS:
            self._inflight_request = True
            req = ParamSetV2.Request()
            req.force_set = True
            req.param_id = 'ARMING_CHECK'
            req.value = ParameterValue(type=2, integer_value=0)
            fut = self._cli_param_set.call_async(req)
            fut.add_done_callback(self._on_arming_check_done)

        elif s is BringUpState.SETTING_GCS_SYSID:
            self._inflight_request = True
            req = ParamSetV2.Request()
            req.force_set = True
            req.param_id = 'MAV_GCS_SYSID'
            req.value = ParameterValue(
                type=2, integer_value=self.mav_gcs_sysid)
            fut = self._cli_param_set.call_async(req)
            fut.add_done_callback(self._on_gcs_sysid_done)

        elif s is BringUpState.SETTING_MODE:
            self._inflight_request = True
            fut = self._cli_set_mode.call_async(
                SetMode.Request(custom_mode='FBWA')
            )
            fut.add_done_callback(self._on_set_mode_done)

        elif s is BringUpState.ARMING:
            self._inflight_request = True
            fut = self._cli_arm.call_async(
                CommandBool.Request(value=True)
            )
            fut.add_done_callback(self._on_arm_done)

        elif s is BringUpState.TAKING_OFF:
            # Refresh RC throttle override every tick. AP's
            # `RC_OVERRIDE_TIME` defaults to 3 s; 0.5 s ticks keep
            # the override alive comfortably.
            self._publish_rc_override(self.takeoff_throttle_pwm)
            if self._latest_pose is not None and \
                    self._latest_pose.pose.position.z >= \
                    self.takeoff_alt_threshold_m:
                self.get_logger().info(
                    f'altitude reached '
                    f'({self._latest_pose.pose.position.z:.2f} m); '
                    f'releasing throttle override and switching '
                    f'to GUIDED'
                )
                self._publish_rc_override_release()
                self._bring_up = BringUpState.SETTING_GUIDED

        elif s is BringUpState.SETTING_GUIDED:
            self._inflight_request = True
            fut = self._cli_set_mode.call_async(
                SetMode.Request(custom_mode='GUIDED')
            )
            fut.add_done_callback(self._on_set_guided_done)

        elif s is BringUpState.READY:
            pass

    def _on_streams_done(self, fut) -> None:
        self._inflight_request = False
        try:
            fut.result()
        except Exception as e:
            self.get_logger().warning(f'set_stream_rate failed: {e}')
        self.get_logger().info('streams requested; pulling params')
        self._inflight_request = True
        fut2 = self._cli_pull.call_async(
            ParamPull.Request(force_pull=True)
        )
        fut2.add_done_callback(self._on_pull_done)
        self._bring_up = BringUpState.REQUESTING_STREAMS

    def _on_pull_done(self, fut) -> None:
        self._inflight_request = False
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f'param/pull failed: {e}')
            return
        self.get_logger().info(
            f'pulled {res.param_received} FCU params'
        )
        self._bring_up = BringUpState.PULLING_PARAMS
        self._fetch_capability_params()

    def _fetch_capability_params(self) -> None:
        self._inflight_request = True
        # Legacy ArduPlane names (still valid in 4.6-dev). If a
        # future AP release migrates these to SI-suffix forms
        # (cf. WP_SPD/ATC_ANGLE_MAX in copter), update both this
        # list and the unit handling below.
        names = [
            'AIRSPEED_MIN', 'AIRSPEED_CRUISE', 'AIRSPEED_MAX',
            'ROLL_LIMIT_DEG',
            'PTCH_LIM_MAX_DEG', 'PTCH_LIM_MIN_DEG',
        ]
        req = GetParameters.Request(names=names)
        fut = self._cli_param_get.call_async(req)
        fut.add_done_callback(
            lambda f: self._on_caps_params_done(f, names)
        )

    def _on_caps_params_done(self, fut, names: list[str]) -> None:
        self._inflight_request = False
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f'param/get_parameters failed: {e}')
            return
        params: dict[str, float] = {}
        for name, val in zip(names, res.values):
            if val.type == 3:
                params[name] = val.double_value
            elif val.type == 2:
                params[name] = float(val.integer_value)
            else:
                params[name] = float('nan')

        airspeed_min = params.get('AIRSPEED_MIN', 9.0)
        airspeed_cruise = params.get('AIRSPEED_CRUISE', 12.0)
        airspeed_max = params.get('AIRSPEED_MAX', 22.0)
        roll_limit_deg = params.get('ROLL_LIMIT_DEG', 45.0)
        ptch_max_deg = params.get('PTCH_LIM_MAX_DEG', 20.0)
        ptch_min_deg = params.get('PTCH_LIM_MIN_DEG', -25.0)

        if math.isnan(airspeed_min) or airspeed_min <= 0.0:
            airspeed_min = 9.0
        if math.isnan(airspeed_cruise) or airspeed_cruise <= 0.0:
            airspeed_cruise = 12.0
        if math.isnan(airspeed_max) or airspeed_max <= 0.0:
            airspeed_max = 22.0
        if math.isnan(roll_limit_deg) or roll_limit_deg <= 0.0:
            roll_limit_deg = 45.0
        if math.isnan(ptch_max_deg) or ptch_max_deg <= 0.0:
            ptch_max_deg = 20.0
        if math.isnan(ptch_min_deg):
            ptch_min_deg = -25.0

        self._airspeed_cruise = airspeed_cruise
        self._climb_angle_up = math.radians(ptch_max_deg)
        self._climb_angle_down = math.radians(abs(ptch_min_deg))

        cap = Capability()
        cap.header.stamp = self.get_clock().now().to_msg()
        cap.header.frame_id = 'map'
        cap.agent_id = self.agent_id
        cap.platform = AgentState.PLATFORM_FIXED_WING

        cap.v_max = airspeed_max
        cap.v_min = airspeed_min  # fixed-wing: positive lower bound

        # Achievable vertical rates at v_max; conservative upper
        # bounds. Actual climb rate at any moment is V * sin(pitch).
        cap.vz_max_up = airspeed_max * math.sin(self._climb_angle_up)
        cap.vz_max_down = airspeed_max * math.sin(self._climb_angle_down)

        bank_max_rad = math.radians(roll_limit_deg)
        cap.bank_angle_max = bank_max_rad
        cap.lateral_accel_max = GRAVITY * math.tan(bank_max_rad)
        # yaw_rate_max varies with airspeed for a coordinated turn.
        # Report the value at cruise; strategies can compute
        # lateral_accel_max / V on demand.
        cap.yaw_rate_max = cap.lateral_accel_max / airspeed_cruise

        cap.climb_angle_max = self._climb_angle_up
        # Turn radius is V^2 / lateral_accel_max — speed-coupled,
        # not a single constant; 0 = "use lateral_accel_max".
        cap.min_turn_radius = 0.0

        cap.source = (
            f'AIRSPEED_MIN={airspeed_min}; '
            f'AIRSPEED_CRUISE={airspeed_cruise}; '
            f'AIRSPEED_MAX={airspeed_max}; '
            f'ROLL_LIMIT_DEG={roll_limit_deg}; '
            f'PTCH_LIM_MAX_DEG={ptch_max_deg}; '
            f'PTCH_LIM_MIN_DEG={ptch_min_deg}'
        )

        self._caps = cap
        self._pub_capability.publish(cap)
        self.get_logger().info(
            f'Capability: V=[{cap.v_min:.1f}, {cap.v_max:.1f}] m/s, '
            f'vz_up={cap.vz_max_up:.2f} m/s, '
            f'vz_dn={cap.vz_max_down:.2f} m/s, '
            f'bank_max={math.degrees(cap.bank_angle_max):.1f} deg, '
            f'climb_angle_max='
            f'{math.degrees(cap.climb_angle_max):.1f} deg, '
            f'yaw_rate_at_cruise={cap.yaw_rate_max:.2f} rad/s'
        )

        if self.disable_arming_check:
            self._bring_up = BringUpState.DISABLING_CHECKS
        else:
            self._bring_up = BringUpState.SETTING_GCS_SYSID

    def _on_arming_check_done(self, fut) -> None:
        self._inflight_request = False
        self.get_logger().info('ARMING_CHECK set request issued')
        self._bring_up = BringUpState.SETTING_GCS_SYSID

    def _on_gcs_sysid_done(self, fut) -> None:
        self._inflight_request = False
        # Without MAV_GCS_SYSID matching the MAVROS sender ID, AP
        # silently drops every RC override message we publish, so
        # the throttle stays at minimum and the plane never moves.
        self.get_logger().info(
            f'MAV_GCS_SYSID={self.mav_gcs_sysid} set request '
            f'issued (needed for RC override pass-through)'
        )
        self._bring_up = BringUpState.SETTING_MODE

    def _on_set_mode_done(self, fut) -> None:
        self._inflight_request = False
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f'set_mode FBWA failed: {e}')
            return
        if res.mode_sent:
            self.get_logger().info('FBWA mode set')
            self._bring_up = BringUpState.ARMING
        else:
            self.get_logger().warning(
                'FBWA set rejected; retrying'
            )

    def _on_arm_done(self, fut) -> None:
        self._inflight_request = False
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f'arming failed: {e}')
            return
        if res.success and self._fcu_state is not None \
                and self._fcu_state.armed:
            self.get_logger().info(
                'armed in FBWA; pushing RC throttle override'
            )
            self._bring_up = BringUpState.TAKING_OFF
        else:
            self.get_logger().warning(
                'arm acknowledged but FCU not armed; retrying'
            )

    def _on_set_guided_done(self, fut) -> None:
        self._inflight_request = False
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f'set_mode GUIDED failed: {e}')
            return
        if res.mode_sent:
            self.get_logger().info('GUIDED mode set; READY')
            self._bring_up = BringUpState.READY
        else:
            self.get_logger().warning('GUIDED set rejected; retrying')

    # ------------------------------------------------------------------
    # RC override helpers
    # ------------------------------------------------------------------

    def _publish_rc_override(self, throttle_pwm: int) -> None:
        msg = OverrideRCIn()
        # roll/pitch/yaw centred (no command); throttle pushed high.
        # Channels beyond 4 left at 0 (CHAN_RELEASE) so AP keeps
        # whatever it had before, e.g. its mode-switch channel.
        msg.channels[0] = 1500
        msg.channels[1] = 1500
        msg.channels[2] = int(throttle_pwm)
        msg.channels[3] = 1500
        for i in range(4, 18):
            msg.channels[i] = 0
        self._pub_rc_override.publish(msg)

    def _publish_rc_override_release(self) -> None:
        msg = OverrideRCIn()
        for i in range(18):
            msg.channels[i] = 0  # CHAN_RELEASE
        self._pub_rc_override.publish(msg)

    # ------------------------------------------------------------------
    # cyclic timers
    # ------------------------------------------------------------------

    def _setpoint_tick(self) -> None:
        if self._bring_up is not BringUpState.READY:
            return
        if self._caps is None:
            return

        if self._latest_pose is None:
            return

        now = self.get_clock().now()
        # Decimate to ~5 Hz: GUIDED_CHANGE_* are continuous-slew
        # (guided_timeout is seconds), and 3 command-service calls
        # per send at 20 Hz would saturate the MAVROS command path.
        # The decimated interval is also the psidot integration step.
        dt = (now - self._last_slew_time).nanoseconds * 1e-9
        if dt < 0.2:
            return
        self._last_slew_time = now

        psi_pose = _yaw_from_quat(
            self._latest_pose.pose.orientation.x,
            self._latest_pose.pose.orientation.y,
            self._latest_pose.pose.orientation.z,
            self._latest_pose.pose.orientation.w,
        )

        age = (now - self._latest_cmd_time).nanoseconds * 1e-9
        stale = age > self.cmd_timeout_s
        if stale:
            V, psidot, climb = self._cruise_setpoint()
        else:
            V, psidot, climb = self._project(
                self._latest_cmd, self._caps)

        # Integrate the commanded turn rate into a heading target
        # (the GUIDED_CHANGE_HEADING realization is heading-, not
        # rate-, addressable). Re-seed from the actual pose heading
        # while stale or before the first command so resume starts
        # from where the plane is, not a drifted integral.
        if stale or self._cmd_heading is None:
            self._cmd_heading = psi_pose
        else:
            self._cmd_heading = _wrap_angle(
                self._cmd_heading + psidot * dt)

        z_now = self._latest_pose.pose.position.z
        heading_deg = enu_yaw_to_compass_deg(self._cmd_heading)
        alt_target, climb_rate = guided_alt_from_climb(z_now, climb)
        self._send_guided_slew(
            V, heading_deg, alt_target, climb_rate,
            self._caps.lateral_accel_max,
        )

    def _send_guided_slew(self, V: float, heading_deg: float,
                          alt_target: float, climb_rate: float,
                          lat_accel: float) -> None:
        """Fire the GUIDED_CHANGE_{SPEED,HEADING,ALTITUDE} triplet.

        Fire-and-forget async: these are continuously refreshed,
        and awaiting 3 services per tick would stall the timer.
        """
        if self._cli_cmd.service_is_ready():
            sp = CommandLong.Request()
            sp.command = _MAV_CMD_GUIDED_CHANGE_SPEED
            sp.param1 = float(_SPEED_TYPE_AIRSPEED)
            sp.param2 = float(V)
            sp.param3 = 0.0   # accel 0 -> ArduPlane paces via TECS
            self._cli_cmd.call_async(sp).add_done_callback(
                self._slew_done)

            hd = CommandLong.Request()
            hd.command = _MAV_CMD_GUIDED_CHANGE_HEADING
            hd.param1 = float(_HEADING_TYPE_COURSE_OVER_GROUND)
            hd.param2 = float(heading_deg)          # [0, 360)
            # param3 = lateral-accel limit; ArduPlane turns it into
            # the bank cap -> coordinated-turn rate g·tan(bank)/V,
            # the same cap the Dubins-airplane bench ideal uses.
            hd.param3 = float(max(lat_accel, 0.05))
            self._cli_cmd.call_async(hd).add_done_callback(
                self._slew_done)

        if self._cli_cmd_int.service_is_ready():
            al = CommandInt.Request()
            al.frame = _MAV_FRAME_GLOBAL_RELATIVE_ALT
            al.command = _MAV_CMD_GUIDED_CHANGE_ALTITUDE
            al.param3 = float(climb_rate)           # vertical speed
            al.z = float(alt_target)                # m, rel home
            self._cli_cmd_int.call_async(al).add_done_callback(
                self._slew_done)

    def _slew_done(self, fut) -> None:
        # Drain the future so rclpy doesn't warn; a denied slew
        # (e.g. transient not-in-GUIDED) self-corrects next tick.
        try:
            fut.result()
        except Exception:
            pass

    def _cruise_setpoint(self) -> tuple[float, float, float]:
        """Control triple to hold heading at cruise on stale
        cmd_vel: (V=cruise airspeed, psidot=0, climb=0).

        A fixed-wing cannot hover, so a stale command must not
        decay to zero; holding cruise straight-and-level keeps the
        plane airborne. psidot=0 holds whatever heading the
        integrator was re-seeded to (see _setpoint_tick). No bound
        is active, so saturation flags clear.
        """
        self._saturation.linear_velocity = False
        self._saturation.angular_velocity = False
        self._saturation.airspeed = False
        self._saturation.turn_rate = False
        self._saturation.climb_rate = False
        return self._airspeed_cruise, 0.0, 0.0

    def _project(self, cmd: Twist,
                 cap: Capability) -> tuple[float, float, float]:
        """Clip the commanded Dubins-airplane control inputs to the
        admissible set. Strategy contract (body frame):

          cmd.linear.x  = airspeed V       (m/s)
          cmd.angular.z = turn rate psidot (rad/s, ENU/CCW)
          cmd.linear.z  = climb rate       (m/s, +up)

        These ARE the differential-game control inputs, commanded
        and constrained directly (no carrot/heading-hold layer):

          V        in [v_min, v_max]
          |psidot| <= g*tan(bank_max)/V       (cap.lateral_accel_max
                      is g*tan(bank_max); the cap is speed-coupled)
          |climb|  <= V*sin(climb_*_angle)

        Returns the clipped (V, psidot, climb) and sets the
        per-bound saturation flags. The adapter integrates psidot
        into a heading target downstream (_setpoint_tick); it does
        not synthesise psidot from a velocity vector.
        """
        V, psidot, climb, sat_air, sat_turn, sat_climb = (
            dubins_airplane_clip(
                cmd.linear.x, cmd.angular.z, cmd.linear.z,
                cap.v_min, cap.v_max, cap.lateral_accel_max,
                self._climb_angle_up, self._climb_angle_down))

        self._saturation.linear_velocity = False
        self._saturation.angular_velocity = False
        self._saturation.airspeed = sat_air
        self._saturation.turn_rate = sat_turn
        self._saturation.climb_rate = sat_climb
        return V, psidot, climb

    def _state_tick(self) -> None:
        if self._latest_pose is None:
            return

        msg = AgentState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._latest_pose.header.frame_id or 'map'
        msg.agent_id = self.agent_id
        msg.role = self.role
        msg.platform = AgentState.PLATFORM_FIXED_WING
        msg.pose = self._latest_pose.pose
        # Body-frame twist: linear from local_position/velocity_body
        # (whose angular component MAVROS leaves at zero), body
        # rates from the imu plugin. Stage 1 Blocker 1 / plan S1.2.
        if self._latest_twist_body is not None:
            msg.twist.linear = self._latest_twist_body.twist.linear
        if self._latest_imu is not None:
            msg.twist.angular = self._latest_imu.angular_velocity
        msg.saturation = self._saturation
        self._pub_state.publish(msg)


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def _wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def enu_yaw_to_compass_deg(psi_rad: float) -> float:
    """ENU heading (0=East, CCW, radians) -> MAVLink course/
    compass degrees in [0, 360) (0=North, clockwise), as
    GUIDED_CHANGE_HEADING.param2 requires. Pure; unit-tested."""
    return (90.0 - math.degrees(psi_rad)) % 360.0


def dubins_airplane_clip(v_cmd: float, psidot_cmd: float,
                         climb_cmd: float, v_min: float,
                         v_max: float, lat_accel: float,
                         climb_up_angle: float,
                         climb_dn_angle: float
                         ) -> tuple[float, float, float,
                                    bool, bool, bool]:
    """Clip the Dubins-airplane *control inputs* to the platform's
    admissible set and report which bounds bound. This is the
    differential-game control set for the fixed-wing:

      V        in [v_min, v_max]
      |psidot| <= lat_accel / V          (= g*tan(bank_max)/V;
                  the speed-coupled coordinated-turn rate cap)
      |climb|  <= V*sin(climb_*_angle)   (speed-coupled, from the
                  pitch/flight-path envelope)

    All three bounds are speed-coupled, so V is clipped first and
    the psidot/climb bounds use the clipped V. Returns
    (V, psidot, climb, sat_air, sat_turn, sat_climb). Pure;
    unit-tested. The strategy commands (airspeed, turn-rate,
    climb-rate) directly so a min-max can be solved over exactly
    this set; the adapter does not hide psidot."""
    sat_air = False
    V = v_cmd
    if V < v_min:
        V, sat_air = v_min, True
    elif V > v_max:
        V, sat_air = v_max, True

    psidot_max = lat_accel / max(V, 1e-3)
    sat_turn = False
    psidot = psidot_cmd
    if psidot > psidot_max:
        psidot, sat_turn = psidot_max, True
    elif psidot < -psidot_max:
        psidot, sat_turn = -psidot_max, True

    climb_up_max = V * math.sin(climb_up_angle)
    climb_dn_max = V * math.sin(climb_dn_angle)
    sat_climb = False
    climb = climb_cmd
    if climb > climb_up_max:
        climb, sat_climb = climb_up_max, True
    elif climb < -climb_dn_max:
        climb, sat_climb = -climb_dn_max, True

    return V, psidot, climb, sat_air, sat_turn, sat_climb


def guided_alt_from_climb(z_now: float, climb: float, *,
                          alt_horizon: float = 5.0,
                          min_rate: float = 0.1
                          ) -> tuple[float, float]:
    """GUIDED_CHANGE_ALTITUDE args from a (clipped) climb rate:
    (target_alt_m, climb_rate). target = current relative alt +
    climb*horizon (a climb intent the FCU paces at climb_rate);
    climb_rate floored above 0 because ArduPlane treats a 0
    vertical-speed arg as "max rate". Pure; unit-tested."""
    return z_now + climb * alt_horizon, max(abs(climb), min_rate)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedWingAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
