"""Optional evaluation-only Path and RViz overlay for Phase 8 trajectories."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start only visual consumers; the estimator graph must already be running."""
    fusion_share = Path(get_package_share_directory('resilient_nav_fusion'))
    use_rviz = LaunchConfiguration('use_rviz')
    paths = Node(
        package='resilient_nav_fusion',
        executable='trajectory_path_adapter',
        name='trajectory_path_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='phase8_trajectory_rviz',
        output='screen',
        arguments=['-d', str(fusion_share / 'config' / 'phase8_trajectories.rviz')],
        condition=IfCondition(use_rviz),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Open the minimal trajectory display when a GUI is available.',
        ),
        paths,
        rviz,
    ])
