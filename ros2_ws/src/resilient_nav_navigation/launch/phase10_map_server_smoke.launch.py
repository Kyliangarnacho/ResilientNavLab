"""Launch only Nav2's Map Server and its lifecycle manager for Task 1.3."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Expose the frozen Phase 9 map without starting AMCL or simulation."""
    navigation_share = Path(
        get_package_share_directory('resilient_nav_navigation')
    )
    slam_share = Path(get_package_share_directory('resilient_nav_slam'))

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            str(navigation_share / 'config' / 'nav2_localization.yaml'),
            {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'autostart': autostart,
            'node_names': ['map_server'],
            'use_sim_time': use_sim_time,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='phase10_map_server_rviz',
        arguments=['-d', str(navigation_share / 'rviz' / 'phase10_localization.rviz')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=str(
                slam_share / 'maps' / 'phase9' / 'occupancy' / 'phase9_map.yaml'
            ),
            description='Frozen Phase 9 occupancy-map YAML.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Map-only smoke does not require a Gazebo clock.',
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Lifecycle-manage only map_server.',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Show the static map in the Phase 10 RViz view.',
        ),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Map Server and lifecycle-manager log level.',
        ),
        map_server,
        lifecycle_manager,
        rviz,
    ])
