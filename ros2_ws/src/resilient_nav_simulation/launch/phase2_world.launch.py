"""Launch the phase 2 Gazebo world, clock bridge, and system heartbeat."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Build the phase 2 simulation launch description."""
    simulation_share = Path(
        get_package_share_directory('resilient_nav_simulation')
    )
    ros_gz_sim_share = Path(get_package_share_directory('ros_gz_sim'))

    default_world_path = simulation_share / 'worlds' / 'phase2_world.sdf'
    world_path = LaunchConfiguration('world')
    bridge_config = simulation_share / 'config' / 'bridge.yaml'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_sim_share / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        parameters=[{'config_file': str(bridge_config)}],
    )

    system_heartbeat = Node(
        package='resilient_nav_monitor',
        executable='system_heartbeat',
        name='system_heartbeat',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=str(default_world_path),
            description='Absolute SDF world path; defaults to the Phase 2 world.',
        ),
        gazebo,
        clock_bridge,
        system_heartbeat,
    ])
