"""Launch an episode plus its strategies plus a rosbag2 record.

Reads the episode YAML, spawns:

  - `parameter_bridge` mapping gz `dynamic_pose/info` ->
    `/game/world_pose` (same setup as `episode.launch.py` from
    rufus_sim_game, replicated here so this launch is
    self-contained).
  - `episode_runner` (rufus_sim_game).
  - One `strategy_runner` (rufus_sim_strategies) per agent that
    has a `strategy:` block in the episode YAML. Strategy
    params are encoded as a YAML string and passed via the
    `params_yaml` ROS parameter.
  - `ros2 bag record` capturing the game-side topics + every
    `/<agent_id>/{state,cmd_vel,capability}`. The bag goes
    under the `bag_dir` launch argument (default
    `/tmp/rufus_sim_bags/<episode_name>_<timestamp>`).

The chain (gz, SITL per agent, MAVROS, adapters) must already
be running per `docs/operations.md`; this launch is the
game-and-strategies side.
"""

from datetime import datetime
from pathlib import Path

import yaml

from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
)
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
    bag_dir_arg = LaunchConfiguration('bag_dir').perform(context)
    record_bag = LaunchConfiguration(
        'record_bag').perform(context).lower() in ('true', '1')

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
    agent_ids = [a['id'] for a in manifest['agents']]

    actions = []

    # Custom Python bridge instead of ros_gz_bridge — see the
    # commentary in rufus_sim_game/launch/episode.launch.py for why.
    actions.append(Node(
        package='rufus_sim_game',
        executable='world_pose_bridge',
        name='world_pose_bridge',
        parameters=[{
            'world_name': world_name,
            'out_topic': _BRIDGE_TOPIC,
            'use_sim_time': True,
        }],
        output='screen',
    ))

    # Episode YAML's `tick_rate_hz` (if present) wins over the
    # launch default; the runner re-validates either way.
    tick_rate_hz = float(episode.get('tick_rate_hz', 50.0))
    actions.append(Node(
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
    ))

    for aid, entry in (episode.get('agents') or {}).items():
        entry = entry or {}
        sblock = entry.get('strategy')
        if not sblock:
            continue
        stype = sblock.get('type')
        if not stype:
            raise RuntimeError(
                f"agent {aid!r}: `strategy.type` is required "
                f"when a `strategy:` block is present"
            )
        params_yaml = yaml.safe_dump(sblock.get('params') or {},
                                     default_flow_style=False)
        actions.append(Node(
            package='rufus_sim_strategies',
            executable='strategy_runner',
            name=f'strategy_runner_{aid}',
            output='screen',
            parameters=[{
                'agent_id': aid,
                'strategy_type': stype,
                'params_yaml': params_yaml,
                'use_sim_time': True,
            }],
        ))

    if record_bag:
        if bag_dir_arg:
            bag_dir = bag_dir_arg
        else:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            bag_dir = (
                f'/tmp/rufus_sim_bags/{episode["name"]}_{stamp}')
        topics = [
            '/game/state',
            '/game/role_assignments',
            '/game/termination_event',
            '/clock',
        ]
        for aid in agent_ids:
            topics.extend([
                f'/{aid}/state',
                f'/{aid}/cmd_vel',
                f'/{aid}/capability',
            ])
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '--use-sim-time',
                 '-o', bag_dir,
                 *topics],
            output='screen',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'episode_path',
            description='Absolute path to the episode YAML.',
        ),
        DeclareLaunchArgument(
            'bag_dir', default_value='',
            description='Output directory for rosbag2 record. '
                        'Default: /tmp/rufus_sim_bags/'
                        '<episode_name>_<timestamp>',
        ),
        DeclareLaunchArgument(
            'record_bag', default_value='true',
            description='Whether to spawn a rosbag2 record '
                        'process for the game-side topics.',
        ),
        OpaqueFunction(function=_build_actions),
    ])
