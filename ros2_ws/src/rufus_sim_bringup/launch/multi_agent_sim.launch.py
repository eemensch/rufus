"""Bring up the ROS-side chain for a multi-agent shared world.

Reads the same YAML agent manifest that `rufus_sim_worlds` uses to
generate the world SDF (see `rufus_sim_worlds/scripts/generate_world.py`).
For each agent in the manifest, spawns one MAVROS process and one
type-specific adapter, namespaced by the agent id.

Five-or-more ROS nodes per run:

  - parameter_bridge: gz `/world/<world_name>/clock` -> ROS
    `/clock` so all downstream nodes can run on sim time.
  - mavros_node x N: one per agent under `/mavros_<instance>`
    with `use_sim_time=true`. `tgt_system = instance + 1` matches
    the per-instance `MAV_SYSID` set by the chained `.parm` file;
    the MAVROS sender `system_id = 240 + instance` keeps each
    UAS heartbeat distinguishable.
  - <type>_adapter x N: one per agent in the agent's namespace
    (`/<id>`), pointing at its MAVROS namespace via `mavros_ns`.

Outside this launch (started manually per `docs/operations.md`):

  - gz sim with the generated world (`<world_name>.sdf`). Both
    the per-instance gz model `<fdm_port_in>` and the per-instance
    `MAV_SYSID` are derived from `instance` in the manifest, so
    the same YAML file is the single source of truth for both
    sides of the chain.
  - ardurover/arducopter/arduplane SITL, one per agent:
    `-I <instance>` shifts MAVLink TCP and FDM UDP ports by
    10·instance; chain `config/sysid_overrides/sysid_<n>.parm`
    (n = instance + 1) so the SITL emits heartbeats with the
    matching sysid. The legacy `SYSID_THISMAV` parameter name is
    silently ignored under AP 4.7+ — use `MAV_SYSID`.

Manifest path: pass `manifest:=<abs path>` to override; default is
the installed `rufus_sim_worlds/config/agents/two_rovers.yaml`.
"""

from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Maps the role string in the manifest to the integer role used by
# rufus_sim_msgs (ROLE_PURSUER = 1, ROLE_EVADER = 2).
ROLE_BY_NAME = {'pursuer': 1, 'evader': 2}

# Maps agent type to the ROS 2 package + executable that drives it.
ADAPTER_BY_TYPE = {
    'rover': ('rufus_sim_adapters', 'rover_adapter'),
    'quad': ('rufus_sim_adapters', 'quad_adapter'),
    'plane': ('rufus_sim_adapters', 'fixed_wing_adapter'),
}


def _mavros(namespace, fcu_url, system_id, target_system,
            mavros_share, bringup_share):
    return Node(
        package='mavros',
        executable='mavros_node',
        namespace=namespace,
        output='screen',
        parameters=[
            # Custom allowlist replaces upstream apm_pluginlists.
            # See rufus_sim_bringup/config/mavros_pluginlists.yaml for
            # rationale; the upstream default loads ~40 plugins,
            # half of which our adapters never touch and which
            # together push a 4-MAVROS run into CPU saturation.
            PathJoinSubstitution(
                [bringup_share, 'config', 'mavros_pluginlists.yaml']),
            PathJoinSubstitution(
                [mavros_share, 'launch', 'apm_config.yaml']),
            {
                'fcu_url': fcu_url,
                'gcs_url': '',
                'tgt_system': target_system,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
                'use_sim_time': True,
                'system_id': system_id,
                # Drop FCU<->ROS clock sync. With use_sim_time=true
                # and a clock_bridge sourcing /clock from gz, ROS
                # time is already authoritative. Default 10 Hz
                # timesync against the FCU otherwise produces
                # `RTT too high for timesync` storms under
                # multi-agent CPU contention.
                'time.timesync_rate': 0.0,
                'time.system_time_rate': 0.0,
            },
        ],
    )


def _adapter(agent_type, agent_id, agent_ns, mavros_ns, role,
             extra_params=None):
    pkg, exe = ADAPTER_BY_TYPE[agent_type]
    params = {
        'agent_id': agent_id,
        'agent_ns': agent_ns,
        'mavros_ns': mavros_ns,
        'role': role,
        'use_sim_time': True,
    }
    if extra_params:
        params.update(extra_params)
    return Node(
        package=pkg,
        executable=exe,
        name=f'{exe}_{agent_id}',
        output='screen',
        parameters=[params],
    )


def _clock_bridge(world_name):
    topic = f'/world/{world_name}/clock'
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[f'{topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[(topic, '/clock')],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )


def _build_actions(context):
    manifest_path = LaunchConfiguration('manifest').perform(context)
    manifest = yaml.safe_load(Path(manifest_path).read_text())

    mavros_share = get_package_share_directory('mavros')
    bringup_share = get_package_share_directory('rufus_sim_bringup')
    actions = [_clock_bridge(manifest['world_name'])]

    for agent in manifest['agents']:
        agent_type = agent['type']
        if agent_type not in ADAPTER_BY_TYPE:
            raise ValueError(
                f"unsupported agent type {agent_type!r} in manifest")
        instance = int(agent['instance'])
        agent_id = agent['id']
        mavros_system_id = 240 + instance
        actions.append(_mavros(
            namespace=f'/mavros_{instance}',
            fcu_url=f'tcp://localhost:{5760 + 10 * instance}',
            system_id=mavros_system_id,
            target_system=instance + 1,
            mavros_share=mavros_share,
            bringup_share=bringup_share,
        ))
        # ArduPlane gates RC overrides on MAV_GCS_SYSID matching
        # the MAVLink sender; pass the per-MAVROS sysid through so
        # the fixed-wing adapter sets it correctly during bring-up.
        extra = {}
        if agent_type == 'plane':
            extra['mav_gcs_sysid'] = mavros_system_id
        actions.append(_adapter(
            agent_type=agent_type,
            agent_id=agent_id,
            agent_ns=f'/{agent_id}',
            mavros_ns=f'/mavros_{instance}',
            role=ROLE_BY_NAME[agent['role']],
            extra_params=extra,
        ))
    return actions


def generate_launch_description():
    default_manifest = PathJoinSubstitution([
        FindPackageShare('rufus_sim_worlds'),
        'config', 'agents', 'two_rovers.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'manifest', default_value=default_manifest,
            description='YAML agent manifest path. Default: '
                        'rufus_sim_worlds/config/agents/two_rovers.yaml.'),
        OpaqueFunction(function=_build_actions),
    ])
