"""Launch Phase 8 measurement adaptation and the adaptive EKF only."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Connect adapter-owned fusion inputs to an EKF without TF ownership."""
    fusion_share = Path(get_package_share_directory('resilient_nav_fusion'))
    adaptive_output_topic = LaunchConfiguration('adaptive_output_topic')
    publish_tf = LaunchConfiguration('publish_tf')

    lidar_odometry = Node(
        package='resilient_nav_fusion',
        executable='lidar_odometry',
        name='lidar_odometry',
        output='screen',
        parameters=[str(fusion_share / 'config' / 'lidar_odometry.yaml')],
    )

    measurement_adapter = Node(
        package='resilient_nav_fusion',
        executable='measurement_adapter',
        name='measurement_adapter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'rf_model_directory': str(
                fusion_share / 'models' / 'physical_reliability_rf_v2'
            ),
            'fallback_reliability_threshold': ParameterValue(
                LaunchConfiguration('fallback_reliability_threshold'),
                value_type=float,
            ),
        }],
    )

    adaptive_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='adaptive_ekf_filter_node',
        output='screen',
        parameters=[
            str(fusion_share / 'config' / 'adaptive_ekf.yaml'),
            {
                'publish_tf': ParameterValue(publish_tf, value_type=bool),
            },
        ],
        remappings=[('odometry/filtered', adaptive_output_topic)],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'fallback_reliability_threshold', default_value='0.10'
        ),
        DeclareLaunchArgument(
            'adaptive_output_topic', default_value='/odometry/adaptive'
        ),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        lidar_odometry,
        measurement_adapter,
        adaptive_ekf,
    ])
