"""Bring up the ROS-side chain for the zephyr fixed-wing simulation.

Starts:
  - parameter_bridge: gz `/world/zephyr_minimal/clock` -> ROS `/clock`
  - mavros_node: ArduPlane MAVLink bridge with use_sim_time=true

Assumes gz sim is already running with `zephyr_minimal.sdf` and that
arduplane SITL is listening on TCP 5760. Both are launched
manually per `docs/operations.md`.

The combination of a /clock source plus use_sim_time on MAVROS
makes MAVROS read time from the gz physics clock (which is what
the FCU also reports), eliminating the "Time jump detected"
warnings observed with default wall-clock MAVROS bring-up.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


WORLD_NAME = 'zephyr_minimal'


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    mavros_share = FindPackageShare('mavros')

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            f'/world/{WORLD_NAME}/clock'
            '@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            (f'/world/{WORLD_NAME}/clock', '/clock'),
        ],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [mavros_share, 'launch', 'apm_pluginlists.yaml']
            ),
            PathJoinSubstitution(
                [mavros_share, 'launch', 'apm_config.yaml']
            ),
            {
                'fcu_url': fcu_url,
                'gcs_url': '',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
                'use_sim_time': True,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url', default_value='tcp://localhost:5760'
        ),
        clock_bridge,
        mavros,
    ])
