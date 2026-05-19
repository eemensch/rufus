"""Custom gz->ROS bridge for `/world/<world>/dynamic_pose/info`.

`ros_gz_bridge`'s `gz.msgs.Pose_V → tf2_msgs/TFMessage` converter
drops the per-pose `name` field, leaving every
`TransformStamped.child_frame_id` empty. That worked under an
earlier index-based fallback because `gz` happened to emit
top-level model poses first for the worlds we tested. Under
worlds with nested `<include>`s — e.g. the iris model, which
includes `iris_with_standoffs` as a child — `gz` interleaves
each parent model with its children, and the index-based
mapping picks up child poses (origin-anchored, all zeros) in
place of the second top-level model.

This node uses `gz.transport13` directly to subscribe to the
gz topic, picks the `name` field out of each `gz.msgs.Pose`,
and republishes a `tf2_msgs/TFMessage` whose
`TransformStamped.child_frame_id` is set to the gz `name`. The
`episode_runner` then matches by id rather than by index.
"""

from __future__ import annotations

import argparse
import sys
import threading

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    Quaternion as QuaternionMsg,
    TransformStamped, Vector3 as Vector3Msg,
)
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode


class WorldPoseBridge(Node):

    def __init__(self) -> None:
        super().__init__('world_pose_bridge')

        self.declare_parameter('world_name', '')
        self.declare_parameter('out_topic', '/game/world_pose')

        world_name = self.get_parameter(
            'world_name').get_parameter_value().string_value
        out_topic = self.get_parameter(
            'out_topic').get_parameter_value().string_value
        if not world_name:
            raise RuntimeError(
                'world_pose_bridge requires `world_name` parameter')

        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(TFMessage, out_topic, qos)

        self._gz = GzNode()
        gz_topic = f'/world/{world_name}/dynamic_pose/info'
        ok = self._gz.subscribe(Pose_V, gz_topic, self._on_gz_msg)
        if not ok:
            raise RuntimeError(
                f'failed to subscribe to gz topic {gz_topic!r}')
        self.get_logger().info(
            f"bridging gz topic {gz_topic!r} -> ROS topic "
            f"{out_topic!r} with names preserved as "
            f"`child_frame_id`"
        )
        # Keep a strong reference; gz-transport's Python wrapper
        # drops subscriptions when the wrapper is GC'd.
        self._lock = threading.Lock()

    def _on_gz_msg(self, msg: Pose_V) -> None:
        out = TFMessage()
        for pose in msg.pose:
            t = TransformStamped()
            t.header.stamp = TimeMsg(
                sec=msg.header.stamp.sec,
                nanosec=msg.header.stamp.nsec,
            )
            t.header.frame_id = ''
            # The model name is what the runner matches on;
            # nested-model entries (`iris_with_standoffs`,
            # `base_link`, rotor links) come through with their
            # own names but the runner ignores anything not
            # listed in the manifest's agent_ids.
            t.child_frame_id = pose.name
            t.transform.translation = Vector3Msg(
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z,
            )
            t.transform.rotation = QuaternionMsg(
                x=pose.orientation.x,
                y=pose.orientation.y,
                z=pose.orientation.z,
                w=pose.orientation.w,
            )
            out.transforms.append(t)
        with self._lock:
            self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = WorldPoseBridge()
    except (RuntimeError, KeyError) as e:
        rclpy.logging.get_logger('world_pose_bridge').error(str(e))
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
