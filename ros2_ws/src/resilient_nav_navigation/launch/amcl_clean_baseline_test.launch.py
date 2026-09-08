"""Observe AMCL in the saved-map world with no added test obstacles."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Start the clean localization baseline and the shared AMCL view."""
    navigation_share = Path(
        get_package_share_directory('resilient_nav_navigation')
    )
    simulation_share = Path(
        get_package_share_directory('resilient_nav_simulation')
    )
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                navigation_share
                / 'launch'
                / 'phase10_localization_smoke.launch.py'
            )
        ),
        launch_arguments={
            'world': str(
                simulation_share / 'worlds' / 'phase9_slam_world.sdf'
            ),
            'spawn_x': '-3.5',
            'spawn_y': '-3.5',
            'spawn_yaw': '0.0',
            'auto_initial_pose': 'true',
            'initial_pose_x': '0.0',
            'initial_pose_y': '0.0',
            'initial_pose_yaw': '0.0',
            'initial_pose_result': '/tmp/amcl_clean_initial_pose.json',
            'use_rviz': 'false',
            'log_level': log_level,
        }.items(),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='amcl_obstacle_test_rviz',
        output='screen',
        arguments=[
            '-d', str(navigation_share / 'rviz' / 'amcl_obstacle_test.rviz')
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        GroupAction(actions=[localization], scoped=True, forwarding=True),
        rviz,
    ])
