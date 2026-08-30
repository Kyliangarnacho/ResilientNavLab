"""Minimal reproducer for Map Server + localization Lifecycle Manager startup."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = Path(get_package_share_directory('resilient_nav_navigation'))
    output_path = LaunchConfiguration('output_path')
    timeout_sec = LaunchConfiguration('timeout_sec')
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'phase10_localization.launch.py')
        ),
        launch_arguments={
            # No Gazebo/clock/bridge/benchmark process is included.  The
            # official manager still configures then activates Map Server and
            # AMCL using the frozen Phase 9 saved map.
            'use_sim_time': 'false',
            'use_rviz': 'false',
        }.items(),
    )
    probe = Node(
        package='resilient_nav_navigation',
        executable='phase10_localization_lifecycle_probe',
        name='phase10_localization_lifecycle_probe',
        output='screen',
        arguments=['--output-path', output_path, '--timeout-sec', timeout_sec],
        parameters=[{'use_sim_time': False}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('output_path'),
        DeclareLaunchArgument('timeout_sec', default_value='30.0'),
        localization,
        probe,
        RegisterEventHandler(
            OnProcessExit(
                target_action=probe,
                on_exit=[EmitEvent(event=Shutdown(reason='localization lifecycle probe finished'))],
            )
        ),
    ])
