"""Launch Phase 8 measurement adaptation and the adaptive EKF only."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Connect adapter-owned fusion inputs to an EKF without TF ownership."""
    fusion_share = Path(get_package_share_directory('resilient_nav_fusion'))

    measurement_adapter = Node(
        package='resilient_nav_fusion',
        executable='measurement_adapter',
        name='measurement_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    adaptive_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='adaptive_ekf_filter_node',
        output='screen',
        parameters=[str(fusion_share / 'config' / 'adaptive_ekf.yaml')],
        remappings=[('odometry/filtered', '/odometry/adaptive')],
    )

    return LaunchDescription([measurement_adapter, adaptive_ekf])
