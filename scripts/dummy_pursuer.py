"""One-shot dummy strategy: drive R0 toward R1 using pure pursuit.

Subscribes to /game/state for the world-frame poses of both
agents, computes the world-frame heading toward R1 from R0's
position, and publishes a body-frame /R0/cmd_vel TwistStamped
that turns R0 to face R1 while moving forward at v_max.
Stops once the runner emits /game/termination_event.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from rufus_sim_msgs.msg import (
    GameState, RoleAssignment, TerminationEvent,
)
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
    QoSReliabilityPolicy,
)


def yaw_of(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class DummyPursuer(Node):
    def __init__(self):
        super().__init__('dummy_pursuer')

        self._pub = self.create_publisher(
            TwistStamped, '/R0/cmd_vel', 10)
        self.create_subscription(GameState, '/game/state',
                                 self._on_state, 10)
        latched = QoSProfile(
            depth=4,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            RoleAssignment, '/game/role_assignments',
            self._on_role_assignment, latched,
        )
        self.create_subscription(
            TerminationEvent, '/game/termination_event',
            self._on_termination, latched,
        )
        self._terminated = False
        self._game_started = False

    def _on_role_assignment(self, msg: RoleAssignment):
        # Hand-off contract: the runner publishes role_assignments
        # once every agent has reached its ready state. Until then
        # the runner owns each agent's cmd_vel; strategies must
        # stay silent or they will fight the warmup driver.
        if not self._game_started:
            self.get_logger().info(
                f'role_assignments received '
                f'(agent_id={msg.agent_id!r}, role={msg.role}); '
                f'taking over /R0/cmd_vel'
            )
        self._game_started = True

    def _on_state(self, msg: GameState):
        if self._terminated or not self._game_started:
            return
        agents = {a.agent_id: a for a in msg.agents}
        if 'R0' not in agents or 'R1' not in agents:
            return
        r0 = agents['R0'].pose.position
        r1 = agents['R1'].pose.position
        psi = yaw_of(agents['R0'].pose.orientation)

        dx = r1.x - r0.x
        dy = r1.y - r0.y
        bearing = math.atan2(dy, dx)
        heading_err = wrap(bearing - psi)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        # Body-frame: forward at v_max while turning to align.
        cmd.twist.linear.x = 1.0
        cmd.twist.angular.z = max(-2.0, min(2.0, 2.0 * heading_err))
        self._pub.publish(cmd)

    def _on_termination(self, msg: TerminationEvent):
        self.get_logger().info(
            f'termination received: predicate_id={msg.predicate_id!r}, '
            f'outcome={msg.outcome!r}, sim_t={msg.sim_time.sec}.'
            f'{msg.sim_time.nanosec:09d}'
        )
        self._terminated = True
        # Stop driving on termination.
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        self._pub.publish(cmd)
        rclpy.shutdown()


def main():
    rclpy.init()
    n = DummyPursuer()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == '__main__':
    main()
