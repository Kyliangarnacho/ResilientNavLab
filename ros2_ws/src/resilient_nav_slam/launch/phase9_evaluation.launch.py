"""Evaluation-only Phase 9 adapters; SLAM core launches remain independent."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Expose Ground Truth and SLAM outputs only on /evaluation/* topics."""
    fusion_share = Path(get_package_share_directory('resilient_nav_fusion'))
    output_path = LaunchConfiguration('output_path')
    run_label = LaunchConfiguration('run_label')
    max_samples = LaunchConfiguration('max_samples')
    ground_truth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fusion_share / 'launch' / 'phase8_ground_truth.launch.py')
        )
    )
    slam_pose_adapter = Node(
        package='resilient_nav_slam',
        executable='slam_pose_adapter',
        name='slam_pose_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    slam_map_adapter = Node(
        package='resilient_nav_slam',
        executable='slam_map_adapter',
        name='slam_map_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    evaluator = Node(
        package='resilient_nav_slam',
        executable='slam_evaluator',
        name='slam_evaluator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_path': output_path,
            'run_label': run_label,
            'max_samples': max_samples,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_path', default_value='',
            description='Optional JSON result path for this evaluation-only run.',
        ),
        DeclareLaunchArgument(
            'run_label', default_value='unspecified',
            description='Structured label written only by the evaluator.',
        ),
        DeclareLaunchArgument(
            'max_samples', default_value='12000',
            description='Bounded evaluation history size for long mapping runs.',
        ),
        ground_truth,
        slam_pose_adapter,
        slam_map_adapter,
        evaluator,
    ])
