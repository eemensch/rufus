"""Launch the episode runner with a gz<->ROS world-pose bridge.

Reads the episode YAML at launch time, follows its `manifest:`
reference (resolving `package://<pkg>/...` URIs through the ament
index) to discover the world_name, and brings up:

  - `parameter_bridge` mapping
    `gz: /world/<world>/dynamic_pose/info` →
    `ROS: /game/world_pose` (`tf2_msgs/msg/TFMessage`). Each model's
    pose lands as a `TransformStamped` whose `child_frame_id` is the
    gz model name (= the manifest's `agent_id`). The bridge is the
    only world-frame ground truth available — mavros
    `local_position/pose` is per-agent EKF-relative and would
    collapse to (0,0) at startup.
  - `episode_runner` consuming that topic plus
    `/<agent_id>/state` per agent.

The chain (gz, SITL, MAVROS, adapters) must already be running per
`docs/operations.md`; this launch only brings up the game-side
consumers.
"""

from pathlib import Path

import yaml

from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_BRIDGE_TOPIC = '/game/world_pose'


def _resolve_path(reference, base):
    if reference.startswith('package://'):
        rest = reference[len('package://'):]
        pkg, rel = rest.split('/', 1)
        try:
            share = get_package_share_directory(pkg)
        except PackageNotFoundError as e:
            raise RuntimeError(
                f'manifest reference {reference!r}: package '
                f'{pkg!r} not on the ament index'
            ) from e
        return Path(share) / rel
    p = Path(reference)
    return p if p.is_absolute() else (base / p).resolve()


def _build_actions(context):
    episode_path = LaunchConfiguration(
        'episode_path').perform(context)
    if not episode_path:
        raise RuntimeError(
            'episode_path argument is required '
            '(absolute path to an episode YAML)'
        )
    episode_path_abs = Path(episode_path).resolve()
    episode = yaml.safe_load(episode_path_abs.read_text())
    manifest_path = _resolve_path(
        episode['manifest'], episode_path_abs.parent)
    manifest = yaml.safe_load(manifest_path.read_text())
    world_name = manifest['world_name']

    gz_topic = f'/world/{world_name}/dynamic_pose/info'

    # Custom Python bridge instead of ros_gz_bridge: the standard
    # gz.msgs.Pose_V -> tf2_msgs/TFMessage converter strips the
    # per-pose `name` field. Worlds with nested model includes
    # (e.g. iris's `iris_with_standoffs` child) interleave parent
    # and child poses in the Pose_V; without the names, the
    # episode_runner cannot tell Q1's model pose from Q0's child
    # link. The custom bridge keeps `name` -> `child_frame_id`.
    pose_bridge = Node(
        package='rufus_sim_game',
        executable='world_pose_bridge',
        name='world_pose_bridge',
        parameters=[{
            'world_name': manifest['world_name'],
            'out_topic': _BRIDGE_TOPIC,
            'use_sim_time': True,
        }],
        output='screen',
    )

    tick_rate_hz = float(episode.get('tick_rate_hz', 50.0))
    runner = Node(
        package='rufus_sim_game',
        executable='episode_runner',
        name='episode_runner',
        output='screen',
        parameters=[{
            'episode_path': str(episode_path_abs),
            'world_pose_topic': _BRIDGE_TOPIC,
            'tick_rate_hz': tick_rate_hz,
            'use_sim_time': True,
        }],
    )

    return [pose_bridge, runner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'episode_path',
            description='Absolute path to the episode YAML.',
        ),
        OpaqueFunction(function=_build_actions),
    ])
