"""Per-agent strategy_runner ROS node.

One process per agent. Loads a `Strategy` subclass by name from
`rufus_sim_strategies.registry`, then runs the standard
hand-off-and-tick loop:

  1. Wait for `/game/role_assignments` (TRANSIENT_LOCAL); the
     runner latches that only after every agent's `ready_when`
     held for its dwell, so it is the strategies' "go" signal.
  2. Wait for the next `/game/state` to arrive.
  3. Call `Strategy.reset()` exactly once.
  4. On every `/game/state` thereafter, build a `Measurement`,
     call `Strategy.control(measurement)`, and publish the
     returned `Twist` on `/<agent_id>/cmd_vel` (wrapped in a
     fresh-stamped `TwistStamped`).

Terminology follows control-systems convention (see
`strategy.py`): the per-tick input bundle is a `Measurement`,
the strategy's mapping is `control`, and the returned `Twist`
is the control output.

ROS parameters
--------------

  agent_id     (string, required)
  strategy_type(string, required)  registry key
  params_yaml  (string, optional)  YAML-encoded dict passed to
                                   `Strategy.params`. The
                                   episode launch encodes the
                                   per-agent `strategy: params`
                                   block here.

Topics
------

Subscribed:
  /game/state                   rufus_sim_msgs/GameState
  /game/role_assignments        rufus_sim_msgs/RoleAssignment
                                (TRANSIENT_LOCAL)
  /<agent_id>/capability        rufus_sim_msgs/Capability
                                (TRANSIENT_LOCAL)

Published:
  /<agent_id>/cmd_vel           geometry_msgs/TwistStamped
"""

from __future__ import annotations

import yaml

import rclpy
from geometry_msgs.msg import TwistStamped
from rufus_sim_msgs.msg import (
    Capability, GameState, RoleAssignment,
)
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
    QoSReliabilityPolicy,
)

from .registry import UnknownStrategy, get
from .strategy import Measurement


class StrategyRunner(Node):

    def __init__(self) -> None:
        super().__init__('strategy_runner')

        self.declare_parameter('agent_id', '')
        self.declare_parameter('strategy_type', '')
        self.declare_parameter('params_yaml', '')

        self._agent_id: str = self.get_parameter(
            'agent_id').get_parameter_value().string_value
        strategy_type: str = self.get_parameter(
            'strategy_type').get_parameter_value().string_value
        params_yaml: str = self.get_parameter(
            'params_yaml').get_parameter_value().string_value

        if not self._agent_id:
            raise RuntimeError(
                'strategy_runner requires `agent_id` parameter')
        if not strategy_type:
            raise RuntimeError(
                'strategy_runner requires `strategy_type` parameter')

        try:
            cls = get(strategy_type)
        except UnknownStrategy as e:
            raise RuntimeError(str(e)) from e

        params = yaml.safe_load(params_yaml) if params_yaml else {}
        if params is None:
            params = {}
        if not isinstance(params, dict):
            raise RuntimeError(
                f"params_yaml must decode to a dict, "
                f"got {type(params).__name__}"
            )

        self._strategy = cls(agent_id=self._agent_id, params=params)

        # Hand-off contract: silent until role_assignments latches
        # and the next /game/state arrives. `_started` flips on
        # the first call to select_action.
        self._handed_off: bool = False
        self._started: bool = False
        self._capability: Capability | None = None

        latched = QoSProfile(
            depth=16,   # holds N agents' RoleAssignment
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        cap_qos = QoSProfile(
            depth=4,    # adapter + runner override may both latch
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self._pub_cmd = self.create_publisher(
            TwistStamped, f'/{self._agent_id}/cmd_vel', 10)
        self.create_subscription(
            RoleAssignment, '/game/role_assignments',
            self._on_role, latched)
        self.create_subscription(
            Capability, f'/{self._agent_id}/capability',
            self._on_capability, cap_qos)
        self.create_subscription(
            GameState, '/game/state', self._on_game_state, 10)

        self.get_logger().info(
            f"strategy_runner up: agent_id={self._agent_id!r}, "
            f"strategy={type(self._strategy).__name__}, "
            f"params={params}"
        )

    # ----- subscribers -----

    def _on_role(self, msg: RoleAssignment) -> None:
        if msg.agent_id != self._agent_id:
            return
        if not self._handed_off:
            self._handed_off = True
            self.get_logger().info(
                'role_assignment received; awaiting next '
                '/game/state to begin commanding'
            )

    def _on_capability(self, msg: Capability) -> None:
        # Two publishers latch on this topic (adapter + runner
        # override after parameter resync). Pick the one with the
        # later header.stamp so we always end up with the
        # post-override envelope when an override exists.
        if self._capability is None or self._stamp_s(
                msg.header.stamp) > self._stamp_s(
                    self._capability.header.stamp):
            self._capability = msg

    def _on_game_state(self, msg: GameState) -> None:
        if not self._handed_off:
            return
        if self._capability is None:
            return
        agents = {a.agent_id: a for a in msg.agents}
        if self._agent_id not in agents:
            self.get_logger().warning(
                f"GameState does not include agent_id "
                f"{self._agent_id!r}; ignoring tick"
            )
            return
        if not self._started:
            try:
                self._strategy.reset()
            except Exception as e:
                self.get_logger().error(
                    f"strategy reset failed: {e}"
                )
                return
            self._started = True
            self.get_logger().info('strategy started')

        measurement = Measurement(
            sim_time_s=(msg.sim_time.sec
                        + msg.sim_time.nanosec * 1e-9),
            agents=agents,
            my_state=agents[self._agent_id],
            my_capability=self._capability,
            active_predicates=tuple(msg.active_predicates),
            episode_id=msg.episode_id,
        )

        try:
            twist = self._strategy.control(measurement)
        except Exception as e:
            self.get_logger().error(
                f"strategy.control raised: {e}; sending zero "
                f"command this tick"
            )
            self._publish_zero(msg.header.frame_id)
            return

        self._publish(msg.header.frame_id, twist)

    # ----- helpers -----

    def _publish(self, _frame_in: str, twist) -> None:
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        # cmd_vel frame is per-platform: rover/quad use
        # 'base_link'; plane uses 'map' (world ENU). The
        # adapter on the receiving end keys on the platform
        # rather than the frame_id, so any reasonable label is
        # accepted; we set the conventional value here for
        # external observers (rosbag2 replay, etc.).
        out.header.frame_id = self._cmd_vel_frame()
        out.twist = twist
        self._pub_cmd.publish(out)

    def _publish_zero(self, _frame_in: str) -> None:
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._cmd_vel_frame()
        self._pub_cmd.publish(out)

    def _cmd_vel_frame(self) -> str:
        # Avoid importing AgentState constants here; the int
        # values are stable in rufus_sim_msgs and the frame string
        # is informational only.
        if self._capability is None:
            return 'base_link'
        return 'map' if self._capability.platform == 2 else 'base_link'

    @staticmethod
    def _stamp_s(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = StrategyRunner()
    except (RuntimeError, UnknownStrategy) as e:
        rclpy.logging.get_logger('strategy_runner').error(str(e))
        rclpy.shutdown()
        raise SystemExit(2) from e
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
