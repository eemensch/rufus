#!/usr/bin/env python3
"""Rover adapter for pursuit-evasion simulation.

Bridges <agent_ns>/cmd_vel (TwistStamped, body frame) onto
ArduRover GUIDED-mode velocity setpoints via MAVROS, and republishes
agent state in the rufus_sim_msgs schema.

The adapter runs a small bring-up state machine on startup: wait for
FCU connect, pull FCU params, publish a latched Capability message,
optionally disable arming checks, set GUIDED mode, arm. After that it
forwards cmd_vel at a fixed rate (the FCU drops the setpoint on
timeout) and publishes AgentState.

Frame note: the adapter publishes a body-frame
`mavros_msgs/PositionTarget` on `setpoint_raw/local` with
`coordinate_frame = MAV_FRAME_BODY_NED`. It does NOT rotate the
command into world ENU. Reason: ArduRover's
`SET_POSITION_TARGET_LOCAL_NED` handler infers drive direction
from `is_negative(packet.vx)` (Rover/GCS_MAVLink_Rover.cpp).
In a world frame `packet.vx` is the NED-north component, so a
rover not heading due north gets its forward/backward sign from
noise and drives backward (diagnosed on the S1.4 baseline:
`step_vx` cmd +0.5 -> steady actual -0.5). In body frame
`packet.vx` is body-forward, so the sign is correct. The
type_mask (velocity + yaw_rate valid; position, accel, yaw
ignored) routes ArduRover to `set_desired_turn_rate_and_speed`,
the unicycle (speed, turn-rate) mode, which is correct for
straight, arc, and spin-in-place commands alike. See
`rover_setpoint()` below and docs/control.md.
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
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import (
    ParamPull, ParamSetV2, SetMode, CommandBool, StreamRate,
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
    READY = auto()


class RoverAdapter(Node):
    def __init__(self) -> None:
        super().__init__('rover_adapter')

        # --- parameters
        self.declare_parameter('agent_id', 'P0')
        self.declare_parameter('agent_ns', '')
        self.declare_parameter('mavros_ns', '/mavros')
        self.declare_parameter('role', AgentState.ROLE_NEUTRAL)
        self.declare_parameter('disable_arming_check', True)
        self.declare_parameter('cmd_timeout_s', 1.0)
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('state_rate_hz', 50.0)
        # bounds enforced before forwarding to MAVROS; if None use FCU values
        self.declare_parameter('vmax_override', -1.0)
        self.declare_parameter('wmax_override', -1.0)

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
            PositionTarget, m('setpoint_raw/local'), 10
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
        # MAVROS local_position/velocity_body carries linear only;
        # body-frame yaw rate comes from the imu plugin. See the
        # mavros_pluginlists.yaml note (Stage 1 Blocker 1 / S1.2).
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
            f'rover_adapter started for agent_id={self.agent_id}, '
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
            # waiting for callback
            pass

        elif s is BringUpState.PULLING_PARAMS:
            # waiting for capability fetch in callback chain
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
        names = [
            'WP_SPEED', 'CRUISE_SPEED', 'GUID_SPEED_MAX',
            'ATC_STR_RAT_MAX', 'ATC_TURN_MAX_G', 'TURN_RADIUS',
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
            # MAVROS returns FCU floats as type=3 (PARAMETER_DOUBLE)
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
        cap.platform = AgentState.PLATFORM_ROVER

        wp_speed = params.get('WP_SPEED', float('nan'))
        cruise = params.get('CRUISE_SPEED', float('nan'))
        guid = params.get('GUID_SPEED_MAX', float('nan'))
        speeds = [v for v in (wp_speed, cruise, guid)
                  if not math.isnan(v) and v > 0.0]
        cap.v_max = max(speeds) if speeds else 1.0
        cap.v_min = -cap.v_max  # skid-steer rover can reverse

        cap.vz_max_up = 0.0
        cap.vz_max_down = 0.0

        atc_str_rat = params.get('ATC_STR_RAT_MAX', 0.0)
        cap.yaw_rate_max = math.radians(atc_str_rat)

        atc_turn_g = params.get('ATC_TURN_MAX_G', 0.0)
        cap.lateral_accel_max = max(
            atc_turn_g * GRAVITY,
            cap.yaw_rate_max * cap.v_max,
        )

        cap.bank_angle_max = 0.0
        cap.climb_angle_max = 0.0

        # Dubins-car coupling |omega| <= |v| / R_min. ArduRover's
        # GUIDED/steering path enforces TURN_RADIUS as the tightest
        # arc even on a skid-steer chassis (pure pivot is AUTO-only;
        # see docs/control.md). Default 0.9 m = ArduRover
        # Parameters.cpp default, also set in r1_rover.param.
        turn_radius = params.get('TURN_RADIUS', float('nan'))
        cap.min_turn_radius = (
            turn_radius
            if not math.isnan(turn_radius) and turn_radius > 0.0
            else 0.9
        )

        cap.source = (
            f'WP_SPEED={wp_speed}; CRUISE_SPEED={cruise}; '
            f'GUID_SPEED_MAX={guid}; ATC_STR_RAT_MAX={atc_str_rat}; '
            f'ATC_TURN_MAX_G={atc_turn_g}; TURN_RADIUS={turn_radius}'
        )

        self._caps = cap
        self._pub_capability.publish(cap)
        self.get_logger().info(
            f'Capability: v_max={cap.v_max:.2f} m/s, '
            f'yaw_rate_max={cap.yaw_rate_max:.2f} rad/s, '
            f'min_turn_radius={cap.min_turn_radius:.2f} m'
        )

        if self.disable_arming_check:
            self._bring_up = BringUpState.DISABLING_CHECKS
        else:
            self._bring_up = BringUpState.SETTING_MODE

    def _on_arming_check_done(self, fut) -> None:
        self._inflight_request = False
        # ParamSetV2 may report success=False but still take effect; do
        # not block on its return value.
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
            self.get_logger().info('armed; READY')
            self._bring_up = BringUpState.READY
        else:
            # request acknowledged but FCU not actually armed (prearm
            # rejection); retry next bring-up tick.
            self.get_logger().warning(
                'arm acknowledged but FCU not armed; retrying'
            )

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
            vx_in, wz_in = 0.0, 0.0          # stop on stale cmd
        else:
            vx_in = self._latest_cmd.linear.x
            wz_in = self._latest_cmd.angular.z

        tm, frame, vx, wz, sat_lin, sat_ang = rover_setpoint(
            vx_in, wz_in,
            self._caps.v_max, self._caps.yaw_rate_max,
        )
        self._saturation.linear_velocity = sat_lin
        self._saturation.angular_velocity = sat_ang
        self._saturation.airspeed = False
        self._saturation.turn_rate = False
        self._saturation.climb_rate = False

        msg = PositionTarget()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.coordinate_frame = frame
        msg.type_mask = tm
        msg.velocity.x = vx
        msg.yaw_rate = wz
        self._pub_setpoint.publish(msg)

    def _state_tick(self) -> None:
        if self._latest_pose is None:
            return

        msg = AgentState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._latest_pose.header.frame_id or 'map'
        msg.agent_id = self.agent_id
        msg.role = self.role
        msg.platform = AgentState.PLATFORM_ROVER
        msg.pose = self._latest_pose.pose
        # Body-frame twist: linear from local_position/velocity_body
        # (whose angular component MAVROS leaves at zero), yaw rate
        # from the imu plugin's body-frame gyro. Stage 1 Blocker 1 /
        # plan S1.2.
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
    `psi` (rad). Pure; unit-tested in test_rover_adapter.

    Retained as a tested geometry utility; the rover setpoint
    path no longer rotates (it sends MAV_FRAME_BODY_NED — see
    `rover_setpoint`). `_yaw_from_quat` likewise. Dedupe across
    adapters is a tracked follow-up, not done here to keep this
    change bounded.
    """
    c, s = math.cos(psi), math.sin(psi)
    return (vx_b * c - vy_b * s, vx_b * s + vy_b * c)


def clamp_command(vx: float, wz: float, vmax: float,
                  wmax: float) -> tuple[float, float, bool, bool]:
    """Clamp (vx, wz) to the symmetric envelope
    [-vmax, vmax] x [-wmax, wmax]. Returns (clamped_vx,
    clamped_wz, linear_saturated, angular_saturated). Pure;
    unit-tested in test_rover_adapter."""
    clamped_vx = max(-vmax, min(vmax, vx))
    clamped_wz = max(-wmax, min(wmax, wz))
    return (clamped_vx, clamped_wz,
            clamped_vx != vx, clamped_wz != wz)


def rover_setpoint(vx: float, wz: float, vmax: float,
                   wmax: float) -> tuple[int, int, float, float,
                                         bool, bool]:
    """Body-frame ArduRover GUIDED setpoint for the skid rover.

    Returns (type_mask, coordinate_frame, vx_clamped,
    wz_clamped, sat_linear, sat_angular).

    Sent in MAV_FRAME_BODY_NED, never world LOCAL_NED:
    ArduRover infers drive direction from `is_negative(packet.vx)`
    (Rover/GCS_MAVLink_Rover.cpp). World-frame makes that the
    NED-north sign, so a non-north-facing rover reverses; body
    frame makes it body-forward, which is correct. The fixed
    type_mask (velocity + yaw_rate valid; position, accel and
    yaw ignored) routes to `set_desired_turn_rate_and_speed`,
    the unicycle (speed, turn-rate) mode that handles straight,
    arc and spin-in-place commands uniformly. Pure;
    unit-tested in test_rover_adapter.
    """
    vx_c, wz_c, sat_lin, sat_ang = clamp_command(
        vx, wz, vmax, wmax)
    type_mask = (
        PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY
        | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX
        | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        | PositionTarget.IGNORE_YAW
    )
    return (type_mask, PositionTarget.FRAME_BODY_NED,
            vx_c, wz_c, sat_lin, sat_ang)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoverAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
