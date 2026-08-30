"""Start evaluation-only Ground Truth and AMCL absolute-metric processes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Keep Ground Truth out of Map Server, AMCL, and the main smoke launch."""
    fusion_share = Path(get_package_share_directory('resilient_nav_fusion'))
    output_path = LaunchConfiguration('output_path')
    map_to_odom_x = LaunchConfiguration('map_to_odom_x')
    map_to_odom_y = LaunchConfiguration('map_to_odom_y')
    map_to_odom_yaw = LaunchConfiguration('map_to_odom_yaw')

    ground_truth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fusion_share / 'launch' / 'phase8_ground_truth.launch.py')
        )
    )
    evaluator = Node(
        package='resilient_nav_navigation',
        executable='phase10_amcl_evaluator',
        name='phase10_amcl_evaluator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_path': output_path,
            'map_to_odom_x': map_to_odom_x,
            'map_to_odom_y': map_to_odom_y,
            'map_to_odom_yaw': map_to_odom_yaw,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_path', default_value='/tmp/phase10_amcl_evaluation.json'
        ),
        DeclareLaunchArgument('map_to_odom_x', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_y', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_yaw', default_value='0.0'),
        ground_truth,
        evaluator,
    ])
