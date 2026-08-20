"""Launch only the healthy Slam Toolbox online_async mapping baseline."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Delegate lifecycle handling to the installed Slam Toolbox launch."""
    package_share = Path(get_package_share_directory('resilient_nav_slam'))
    slam_toolbox_share = Path(get_package_share_directory('slam_toolbox'))
    slam_params_file = LaunchConfiguration('slam_params_file')

    online_async = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(slam_toolbox_share / 'launch' / 'online_async_launch.py'),
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=str(
                package_share / 'config' / 'mapper_params_online_async.yaml',
            ),
            description='Jazzy Slam Toolbox mapping parameter file.',
        ),
        online_async,
    ])
