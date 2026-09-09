"""One fault-aware Nav2 trial with evaluation-only GT evidence."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BAG_TOPICS = (
    '/clock', '/map', '/scan', '/faulted/scan', '/imu/data',
    '/faulted/imu/data', '/wheel/odometry', '/faulted/wheel/odometry',
    '/health/wheel', '/health/imu', '/health/scan',
    '/lidar/odometry/status', '/fusion/reliability', '/fusion/status',
    '/localization/quality', '/resilience/status', '/odometry/filtered',
    '/resilience/active_goal',
    '/amcl_pose', '/tf', '/tf_static', '/plan', '/received_global_plan',
    '/cmd_vel', '/behavior_tree_log', '/evaluation/ground_truth_pose',
    '/fault_injection/status',
)


def generate_launch_description():
    """Run one route through the Supervisor-gated action endpoint."""
    navigation_share = Path(
        get_package_share_directory('resilient_nav_navigation')
    )
    fusion_share = Path(get_package_share_directory('resilient_nav_fusion'))

    runtime = GroupAction(
        scoped=True,
        forwarding=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    navigation_share
                    / 'launch'
                    / 'fault_aware_navigation.launch.py'
                )
            ),
            launch_arguments={
                'physical_scenario': LaunchConfiguration(
                    'physical_scenario'
                ),
                'sensor_scenario_file': LaunchConfiguration(
                    'sensor_scenario_file'
                ),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'initial_pose_result': LaunchConfiguration(
                    'initial_pose_result'
                ),
                'use_recovery': LaunchConfiguration('use_recovery'),
                'zone_center_x': LaunchConfiguration('zone_center_x'),
                'zone_center_y': LaunchConfiguration('zone_center_y'),
                'zone_length_m': LaunchConfiguration('zone_length_m'),
                'zone_width_m': LaunchConfiguration('zone_width_m'),
                'low_friction_mu': LaunchConfiguration('low_friction_mu'),
                'roughness_height_m': LaunchConfiguration(
                    'roughness_height_m'
                ),
                'roughness_spacing_m': LaunchConfiguration(
                    'roughness_spacing_m'
                ),
                'disturbance_delay_sec': LaunchConfiguration(
                    'disturbance_delay_sec'
                ),
                'disturbance_pulse_count': LaunchConfiguration(
                    'disturbance_pulse_count'
                ),
                'disturbance_interval_sec': LaunchConfiguration(
                    'disturbance_interval_sec'
                ),
                'impact_duration_sec': LaunchConfiguration(
                    'impact_duration_sec'
                ),
                'impact_force_x_n': LaunchConfiguration('impact_force_x_n'),
                'impact_force_y_n': LaunchConfiguration('impact_force_y_n'),
            }.items(),
        )],
    )
    ground_truth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fusion_share / 'launch' / 'phase8_ground_truth.launch.py')
        )
    )
    runner = Node(
        package='resilient_nav_navigation',
        executable='phase10_navigation_benchmark_runner',
        name='fault_aware_navigation_benchmark_runner',
        output='screen',
        arguments=[
            '--scenario', LaunchConfiguration('scenario'),
            '--output-path', LaunchConfiguration('navigation_output'),
            '--timeout-sec', LaunchConfiguration('readiness_timeout_sec'),
            '--scenarios-file', LaunchConfiguration('scenarios_file'),
            '--action-name', '/resilient_navigate_to_pose',
            '--observe-path-safety-only',
        ],
        parameters=[{'use_sim_time': True}],
    )
    recorder = Node(
        package='resilient_nav_navigation',
        executable='phase10_navigation_benchmark_gt_recorder',
        name='fault_aware_navigation_benchmark_gt_recorder',
        output='screen',
        arguments=[
            '--output-path', LaunchConfiguration('ground_truth_output')
        ],
        parameters=[{'use_sim_time': True}],
    )
    bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '--storage', 'mcap',
            '--output', LaunchConfiguration('diagnostics_output'),
            *BAG_TOPICS,
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_diagnostics')),
    )
    shutdown = RegisterEventHandler(OnProcessExit(
        target_action=runner,
        on_exit=[EmitEvent(event=Shutdown(reason='benchmark runner finished'))],
    ))

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='simple_reachable'),
        DeclareLaunchArgument('physical_scenario', default_value='normal'),
        DeclareLaunchArgument(
            'sensor_scenario_file',
            default_value=str(
                fusion_share / 'config' / 'healthy_passthrough.yaml'
            ),
        ),
        DeclareLaunchArgument('navigation_output'),
        DeclareLaunchArgument('ground_truth_output'),
        DeclareLaunchArgument('diagnostics_output'),
        DeclareLaunchArgument('initial_pose_result'),
        DeclareLaunchArgument('record_diagnostics', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_recovery', default_value='false'),
        DeclareLaunchArgument('readiness_timeout_sec', default_value='120.0'),
        DeclareLaunchArgument('zone_center_x', default_value='-1.5'),
        DeclareLaunchArgument('zone_center_y', default_value='-3.5'),
        DeclareLaunchArgument('zone_length_m', default_value='2.0'),
        DeclareLaunchArgument('zone_width_m', default_value='1.0'),
        DeclareLaunchArgument('low_friction_mu', default_value='0.12'),
        DeclareLaunchArgument('roughness_height_m', default_value='0.008'),
        DeclareLaunchArgument('roughness_spacing_m', default_value='0.45'),
        DeclareLaunchArgument('disturbance_delay_sec', default_value='7.0'),
        DeclareLaunchArgument('disturbance_pulse_count', default_value='1'),
        DeclareLaunchArgument('disturbance_interval_sec', default_value='3.0'),
        DeclareLaunchArgument('impact_duration_sec', default_value='0.25'),
        DeclareLaunchArgument('impact_force_x_n', default_value='0.0'),
        DeclareLaunchArgument('impact_force_y_n', default_value='12.0'),
        DeclareLaunchArgument(
            'scenarios_file',
            default_value=str(
                navigation_share / 'config' / 'planner_smoke_scenarios.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'gz_partition', default_value='resilient_nav_fault_aware_healthy'
        ),
        DeclareLaunchArgument('ros_domain_id', default_value='58'),
        SetEnvironmentVariable(
            'GZ_PARTITION', LaunchConfiguration('gz_partition')
        ),
        SetEnvironmentVariable(
            'ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')
        ),
        runtime,
        ground_truth,
        recorder,
        bag,
        runner,
        shutdown,
    ])
