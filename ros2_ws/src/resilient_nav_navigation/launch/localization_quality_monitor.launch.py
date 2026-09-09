"""Launch only the thin localization-quality monitor."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Keep Nav2 task policy out of the localization observation layer."""
    package_share = Path(
        get_package_share_directory('resilient_nav_navigation')
    )
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=str(
                package_share / 'config' / 'localization_quality_monitor.yaml'
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='resilient_nav_navigation',
            executable='localization_quality_monitor',
            name='localization_quality_monitor',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
