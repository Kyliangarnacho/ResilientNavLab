"""Start the BRNE real-data shadow overlay without modifying Phase 10."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Compose adapter, shadow planner, and only the synthetic pedestrian source."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    runtime_profile = (
        Path(get_package_share_directory('resilient_nav_brne'))
        / 'config'
        / 'brne_v1_runtime.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='resilient_nav_brne',
            executable='brne_shadow_input_adapter',
            name='brne_shadow_input_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_frequency_hz': 10.0,
                'odom_max_age_sec': 0.25,
                'odom_max_receipt_age_sec': 0.5,
                'plan_max_age_sec': 1.5,
                'plan_max_receipt_age_sec': 3.0,
                'tf_timeout_sec': 0.05,
                'local_goal_distance': 0.8,
                'maximum_path_nearest_distance': 0.75,
            }],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_shadow_node',
            name='brne_shadow_node',
            output='screen',
            parameters=[str(runtime_profile), {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_synthetic_pedestrian_source',
            name='brne_synthetic_pedestrian_source',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
