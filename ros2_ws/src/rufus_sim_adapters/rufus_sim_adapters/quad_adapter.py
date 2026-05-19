#!/usr/bin/env python3
"""Quadrotor adapter for pursuit-evasion simulation.

Bridges <agent_ns>/cmd_vel (TwistStamped, body frame) onto
ArduCopter GUIDED-mode velocity setpoints via MAVROS, and
republishes agent state in the rufus_sim_msgs schema.

Honours linear.x/y/z body-frame velocity and angular.z yaw rate.
Pulls WP_SPD, WP_SPD_UP, WP_SPD_DN, ATC_RATE_Y_MAX, and
ATC_ANGLE_MAX from the FCU at bring-up to populate Capability.
These names are valid for ArduCopter 4.7+; the legacy
WPNAV_SPEED*/ANGLE_MAX were retired in the SI-suffix migration
(units also changed: cm/s -> m/s for speeds, cdeg -> deg for
angles). When pinning an older AP tree, the legacy names need
to be substituted back here and in `_fetch_capability_params`.

The bring-up state machine adds a TAKING_OFF step compared to
the rover: after arming, the adapter issues MAV_CMD_NAV_TAKEOFF
and gates READY on the local-position altitude exceeding the
threshold, since ArduCopter in GUIDED on the ground will not
honour velocity setpoints without an explicit takeoff.

The topic contract mirrors the rover adapter; consult
docs/control.md for the contract and docs/operations.md for
the SITL chain.

Frame note: MAVROS apm_config.yaml configures the
setpoint_velocity plugin with `mav_frame: LOCAL_NED`, which is
a *world*-frame (LOCAL relative to the EKF home, not body).
The adapter therefore rotates the incoming body-frame `linear`
into world ENU before publishing to
`setpoint_velocity/cmd_vel_unstamped`, using the latest yaw
from `local_position/pose`. Yaw rate (`angular.z`) is left
unrotated: for a level rotorcraft body wz approximately equals
world wz, and the FCU interprets the MAVLink `yaw_rate` field
as the heading rate regardless of frame. Without this rotation
a body-frame command of (0.5, 0, 0) issued to an iris spawned
at yaw=90 deg would be interpreted by the FCU as 0.5 m/s east
rather than 0.5 m/s along body-x (north), producing the 5.66 m
position drift observed in the Stage 2 step_vx bench.
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
from mavros_msgs.msg import State
from mavros_msgs.srv import (
    ParamPull, ParamSetV2, SetMode, CommandBool, CommandTOL, StreamRate,
)

from rufus_sim_msgs.msg import AgentState, Capability, SaturationFlags


GRAVITY = 9.81


class BringUpState(Enum):
    WAIT_CONNECT = auto()
    REQUESTING_STREAMS = auto()
    PULLING_PARAMS = auto()
    DISABLING_CHECKS = auto()
    SETTING_MODE = auto()
    ARMING = auto()
    TAKING_OFF = auto()
    READY = auto()


class QuadAdapter(Node):
    def __init__(self) -> None:
        super().__init__('quad_adapter')

        # --- parameters
        self.declare_parameter('agent_id', 'Q0')
        self.declare_parameter('agent_ns', '')
        self.declare_parameter('mavros_ns', '/mavros')
        self.declare_parameter('role', AgentState.ROLE_NEUTRAL)
        self.declare_parameter('disable_arming_check', True)
        self.declare_parameter('cmd_timeout_s', 1.0)
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('state_rate_hz', 50.0)
        self.declare_parameter('takeoff_altitude_m', 5.0)
        self.declare_parameter('takeoff_alt_threshold_m', 4.0)

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
        self.takeoff_altitude_m: float = float(
            self.get_parameter('takeoff_altitude_m').value
        )
        self.takeoff_alt_threshold_m: float = float(
            self.get_parameter('takeoff_alt_threshold_m').value
        )

        # --- runtime state
        self._latest_cmd: Twist = Twist()
        self._latest_cmd_time = self.get_clock().now()
        self._fcu_state: State | None = None
        self._latest_pose: PoseStamped | None = None
        self._latest_twist_body: TwistStamped | None = None
        self._latest_imu: Imu | None = None
        self._caps: Capability | None = None
        self._saturation = SaturationFlags()
        self._bring_up = BringUpState.WAIT_CONNECT
        self._inflight_request = False
        self._takeoff_issued = False

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
        self._pub_setpoint = self.create_publisher(
            Twist, m('setpoint_velocity/cmd_vel_unstamped'), 10
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
        # body-frame yaw rate comes from the imu plugin (same
        # MAVROS gap as the rover; Stage 1 Blocker 1 / plan S1.2).
        self.create_subscription(
            Imu, m('imu/data'), self._imu_cb, sensor_qos,
        )

        # --- service clients
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
        self._cli_takeoff = self.create_client(
            CommandTOL, m('cmd/takeoff')
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
            f'quad_adapter started for agent_id={self.agent_id}, '
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

        elif s is BringUpState.SETTING_MODE:
            self._inflight_request = True
            fut = self._cli_set_mode.call_async(
                SetMode.Request(custom_mode='GUIDED')
            )
            fut.add_done_callback(self._on_set_mode_done)

        elif s is BringUpState.ARMING:
            self._inflight_request = True
            fut = self._cli_arm.call_async(
                CommandBool.Request(value=True)
            )
            fut.add_done_callback(self._on_arm_done)

        elif s is BringUpState.TAKING_OFF:
            if self._latest_pose is not None and \
                    self._latest_pose.pose.position.z >= \
                    self.takeoff_alt_threshold_m:
                self.get_logger().info(
                    f'altitude reached '
                    f'({self._latest_pose.pose.position.z:.2f} m); '
                    f'READY'
                )
                self._bring_up = BringUpState.READY
            elif not self._takeoff_issued:
                self._inflight_request = True
                self._takeoff_issued = True
                fut = self._cli_takeoff.call_async(
                    CommandTOL.Request(
                        altitude=self.takeoff_altitude_m
                    )
                )
                fut.add_done_callback(self._on_takeoff_done)

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
        # Names valid for ArduCopter 4.7+ post-rename:
        # WPNAV_SPEED* -> WP_SPD* (cm/s -> m/s) and
        # ANGLE_MAX -> ATC_ANGLE_MAX (cdeg -> deg). Legacy names
        # silently return ParameterValue(type=0) and the fallbacks
        # below fire without warning.
        names = [
            'WP_SPD', 'WP_SPD_UP', 'WP_SPD_DN',
            'ATC_RATE_Y_MAX', 'ATC_ANGLE_MAX',
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
        params = {}
        for name, val in zip(names, res.values):
            if val.type == 3:
                params[name] = val.double_value
            elif val.type == 2:
                params[name] = float(val.integer_value)
            else:
                params[name] = float('nan')

        cap = Capability()
        cap.header.stamp = self.get_clock().now().to_msg()
        cap.header.frame_id = 'map'
        cap.agent_id = self.agent_id
        cap.platform = AgentState.PLATFORM_QUADROTOR

        # WP_SPD, _UP, _DN are in m/s. Renamed from WPNAV_SPEED* in the
        # ArduPilot 4.7+ unit-suffix migration; the units were changed
        # from cm/s to m/s in the same change.
        wp_spd = params.get('WP_SPD', float('nan'))
        wp_spd_up = params.get('WP_SPD_UP', float('nan'))
        wp_spd_dn = params.get('WP_SPD_DN', float('nan'))
        cap.v_max = (
            wp_spd if not math.isnan(wp_spd) and wp_spd > 0 else 10.0
        )
        cap.v_min = -cap.v_max  # quad can fly backward in body frame
        cap.vz_max_up = (
            wp_spd_up if not math.isnan(wp_spd_up) and wp_spd_up > 0
            else 2.5
        )
        cap.vz_max_down = (
            wp_spd_dn if not math.isnan(wp_spd_dn) and wp_spd_dn > 0
            else 1.5
        )

        # ATC_RATE_Y_MAX in deg/s. 0 means "no explicit cap"; fall back
        # to a safe default derived from typical ArduCopter behaviour.
        atc_rate_y = params.get('ATC_RATE_Y_MAX', 0.0)
        if math.isnan(atc_rate_y) or atc_rate_y <= 0.0:
            atc_rate_y = 90.0  # deg/s, ArduCopter typical
        cap.yaw_rate_max = math.radians(atc_rate_y)

        # ATC_ANGLE_MAX in degrees. Renamed from ANGLE_MAX (cdeg) in
        # the same ArduPilot 4.7+ migration as WP_SPD*.
        angle_max_deg = params.get('ATC_ANGLE_MAX', 30.0)
        if math.isnan(angle_max_deg) or angle_max_deg <= 0.0:
            angle_max_deg = 30.0
        cap.bank_angle_max = math.radians(angle_max_deg)
        cap.lateral_accel_max = GRAVITY * math.tan(cap.bank_angle_max)

        cap.climb_angle_max = 0.0  # not meaningful for hover-capable quad
        cap.min_turn_radius = 0.0  # holonomic: yaw decoupled from speed

        cap.source = (
            f'WP_SPD={wp_spd}; WP_SPD_UP={wp_spd_up}; '
            f'WP_SPD_DN={wp_spd_dn}; ATC_RATE_Y_MAX={atc_rate_y}; '
            f'ATC_ANGLE_MAX={angle_max_deg}'
        )

        self._caps = cap
        self._pub_capability.publish(cap)
        self.get_logger().info(
            f'Capability: v_max={cap.v_max:.2f} m/s, '
            f'vz_up={cap.vz_max_up:.2f} m/s, vz_dn={cap.vz_max_down:.2f} m/s, '
            f'yaw_rate_max={cap.yaw_rate_max:.2f} rad/s, '
            f'bank_max={math.degrees(cap.bank_angle_max):.1f} deg'
        )

        if self.disable_arming_check:
            self._bring_up = BringUpState.DISABLING_CHECKS
        else:
            self._bring_up = BringUpState.SETTING_MODE

    def _on_arming_check_done(self, fut) -> None:
        self._inflight_request = False
        self.get_logger().info('ARMING_CHECK set request issued')
        self._bring_up = BringUpState.SETTING_MODE

    def _on_set_mode_done(self, fut) -> None:
        self._inflight_request = False
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f'set_mode failed: {e}')
            return
        if res.mode_sent:
            self.get_logger().info('GUIDED mode set')
            self._bring_up = BringUpState.ARMING
        else:
            self.get_logger().warning('GUIDED set rejected; retrying')

    def _on_arm_done(self, fut) -> None:
        self._inflight_request = False
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f'arming failed: {e}')
            return
        if res.success and self._fcu_state is not None \
                and self._fcu_state.armed:
            self.get_logger().info('armed; commanding takeoff')
            self._bring_up = BringUpState.TAKING_OFF
            self._takeoff_issued = False
        else:
            self.get_logger().warning(
                'arm acknowledged but FCU not armed; retrying'
            )

    def _on_takeoff_done(self, fut) -> None:
        self._inflight_request = False
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f'takeoff service failed: {e}')
            self._takeoff_issued = False
            return
        if res.success:
            self.get_logger().info('takeoff acknowledged; climbing')
        else:
            self.get_logger().warning(
                f'takeoff rejected; result={res.result}; retrying'
            )
            self._takeoff_issued = False

    # ------------------------------------------------------------------
    # cyclic timers
    # ------------------------------------------------------------------

    def _setpoint_tick(self) -> None:
        if self._bring_up is not BringUpState.READY:
            return
        if self._caps is None:
            return

        now = self.get_clock().now()
        age = (now - self._latest_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout_s:
            cmd_body = Twist()
        else:
            cmd_body = self._clamp(self._latest_cmd, self._caps)

        cmd_world = self._body_to_world(cmd_body)
        self._pub_setpoint.publish(cmd_world)

    def _body_to_world(self, cmd_body: Twist) -> Twist:
        """Rotate body-frame linear velocity into world ENU using
        the latest pose's yaw. See module docstring for why this is
        necessary (MAVROS LOCAL_NED frame).

        Falls back to passing the command through unrotated when no
        pose has arrived yet — this only happens during bring-up
        before the EKF reports, where the adapter holds zero
        setpoint anyway.
        """
        out = Twist()
        out.linear.z = cmd_body.linear.z
        out.angular.z = cmd_body.angular.z
        if self._latest_pose is None:
            out.linear.x = cmd_body.linear.x
            out.linear.y = cmd_body.linear.y
            return out
        ao = self._latest_pose.pose.orientation
        psi = _yaw_from_quat(ao.x, ao.y, ao.z, ao.w)
        out.linear.x, out.linear.y = body_to_world_xy(
            cmd_body.linear.x, cmd_body.linear.y, psi
        )
        return out

    def _clamp(self, cmd: Twist, cap: Capability) -> Twist:
        out = Twist()

        # Horizontal vector capped to v_max in 2-norm; preserves
        # heading of the commanded body-frame velocity.
        vx = cmd.linear.x
        vy = cmd.linear.y
        vh = math.hypot(vx, vy)
        if vh > cap.v_max and vh > 0.0:
            scale = cap.v_max / vh
            out.linear.x = vx * scale
            out.linear.y = vy * scale
            self._saturation.linear_velocity = True
        else:
            out.linear.x = vx
            out.linear.y = vy
            self._saturation.linear_velocity = False

        # Vertical: asymmetric envelope (climb up, descent down).
        vz = cmd.linear.z
        if vz > cap.vz_max_up:
            out.linear.z = cap.vz_max_up
            self._saturation.climb_rate = True
        elif vz < -cap.vz_max_down:
            out.linear.z = -cap.vz_max_down
            self._saturation.climb_rate = True
        else:
            out.linear.z = vz
            self._saturation.climb_rate = False

        # Yaw rate.
        wz = cmd.angular.z
        wmax = cap.yaw_rate_max
        clamped_wz = max(-wmax, min(wmax, wz))
        out.angular.z = clamped_wz
        self._saturation.angular_velocity = (clamped_wz != wz)

        # Fixed-wing-only flags.
        self._saturation.airspeed = False
        self._saturation.turn_rate = False

        return out

    def _state_tick(self) -> None:
        if self._latest_pose is None:
            return

        msg = AgentState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._latest_pose.header.frame_id or 'map'
        msg.agent_id = self.agent_id
        msg.role = self.role
        msg.platform = AgentState.PLATFORM_QUADROTOR
        msg.pose = self._latest_pose.pose
        # Body-frame twist: linear from local_position/velocity_body
        # (whose angular component MAVROS leaves at zero), yaw rate
        # from the imu plugin. Stage 1 Blocker 1 / plan S1.2.
        if self._latest_twist_body is not None:
            msg.twist.linear = self._latest_twist_body.twist.linear
        if self._latest_imu is not None:
            msg.twist.angular = self._latest_imu.angular_velocity
        msg.saturation = self._saturation
        self._pub_state.publish(msg)


def _yaw_from_quat(qx: float, qy: float, qz: float,
                   qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def body_to_world_xy(vx_b: float, vy_b: float,
                      psi: float) -> tuple[float, float]:
    """Rotate a body-frame planar velocity into world ENU by yaw
    `psi` (rad). Same convention as the rover adapter; pure,
    unit-tested in test_quad_adapter."""
    c, s = math.cos(psi), math.sin(psi)
    return (vx_b * c - vy_b * s, vx_b * s + vy_b * c)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QuadAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
