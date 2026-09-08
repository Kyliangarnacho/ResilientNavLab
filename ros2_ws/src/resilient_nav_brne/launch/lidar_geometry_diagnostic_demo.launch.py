"""Minimal wall-rotation demo for evaluator-only GT/EKF LiDAR comparison."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    brne_share = Path(get_package_share_directory('resilient_nav_brne'))
    fusion_share = Path(get_package_share_directory('resilient_nav_fusion'))
    navigation_share = Path(get_package_share_directory('resilient_nav_navigation'))
    simulation_share = Path(get_package_share_directory('resilient_nav_simulation'))
    output_path = LaunchConfiguration('diagnostic_output')
    log_level = LaunchConfiguration('log_level')

    planner_costmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'phase10_planner_smoke.launch.py')
        ),
        launch_arguments={
            'world': str(simulation_share / 'worlds' / 'phase9_slam_world.sdf'),
            'spawn_x': '-3.5',
            'spawn_y': '-3.5',
            'spawn_yaw': '0.0',
            'auto_initial_pose': 'true',
            'initial_pose_x': '0.0',
            'initial_pose_y': '0.0',
            'initial_pose_yaw': '0.0',
            'global_obstacle_layer_enabled': 'false',
            'use_rviz': 'false',
            'log_level': log_level,
        }.items(),
    )
    ground_truth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fusion_share / 'launch' / 'phase8_ground_truth.launch.py')
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'diagnostic_output',
            default_value='/tmp/brne_lidar_geometry_diagnostic.json',
        ),
        DeclareLaunchArgument('log_level', default_value='info'),
        GroupAction(actions=[planner_costmap], scoped=True, forwarding=True),
        ground_truth,
        Node(
            package='resilient_nav_brne',
            executable='brne_lidar_dynamic_agent_node',
            name='brne_lidar_dynamic_agent_node',
            output='screen',
            parameters=[
                str(brne_share / 'config' / 'brne_v1_runtime.yaml'),
                {'use_sim_time': True},
            ],
        ),
        Node(
            package='resilient_nav_fusion',
            executable='lidar_projection_evaluator',
            name='lidar_projection_evaluator',
            output='screen',
            parameters=[{'use_sim_time': True, 'output_path': output_path}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='lidar_geometry_diagnostic_rviz',
            output='screen',
            arguments=['-d', str(brne_share / 'rviz' / 'lidar_geometry_diagnostic.rviz')],
            parameters=[{'use_sim_time': True}],
        ),
    ])
