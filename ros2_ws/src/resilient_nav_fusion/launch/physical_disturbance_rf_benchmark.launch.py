"""Self-terminating fixed-vs-RF-adaptive physical disturbance benchmark."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BAG_TOPICS = (
    '/clock',
    '/cmd_vel',
    '/wheel/odometry',
    '/faulted/wheel/odometry',
    '/imu/data',
    '/faulted/imu/data',
    '/scan',
    '/faulted/scan',
    '/health/wheel',
    '/health/imu',
    '/lidar/odometry/status',
    '/fusion/reliability',
    '/fusion/status',
    '/odometry/fixed',
    '/odometry/adaptive',
    '/evaluation/ground_truth_pose',
    '/evaluation/localization_metrics',
    '/evaluation/localization_alignment_sweep',
)


def _passthrough(sensor: str) -> dict:
    base = {
        'use_sim_time': True,
        'status_topic': '/fault_injection/status',
        'enabled': False,
        'start_time_sec': 5.0,
        'end_time_sec': 15.0,
        'scenario_seed': 20260908,
        'scenario_id': f'{sensor}_benchmark_passthrough',
        'event_id': f'{sensor}_benchmark_passthrough_event',
    }
    if sensor == 'wheel':
        base.update({
            'input_topic': '/wheel/odometry',
            'output_topic': '/faulted/wheel/odometry',
            'model': 'freeze',
        })
    elif sensor == 'imu':
        base.update({
            'input_topic': '/imu/data',
            'output_topic': '/faulted/imu/data',
            'model': 'bias',
            'bias_rad_s': 0.0,
        })
    else:
        base.update({
            'input_topic': '/scan',
            'output_topic': '/faulted/scan',
            'model': 'sector_blindness',
            'sector_center_rad': 0.0,
            'sector_width_rad': 1.0,
        })
    return base


def generate_launch_description():
    """Run one parameterized world, route, A/B estimator graph, and bag."""
    fusion_share = Path(get_package_share_directory('resilient_nav_fusion'))
    fault_share = Path(get_package_share_directory('resilient_nav_fault_injection'))
    health_share = Path(get_package_share_directory('resilient_nav_health_assessment'))
    simulation_share = Path(get_package_share_directory('resilient_nav_simulation'))

    scene_arguments = {
        name: LaunchConfiguration(name)
        for name in (
            'scenario',
            'use_rviz',
            'zone_center_x',
            'zone_center_y',
            'zone_length_m',
            'zone_width_m',
            'low_friction_mu',
            'asymmetric_low_mu',
            'weak_wheel_side',
            'roughness_height_m',
            'roughness_spacing_m',
            'disturbance_delay_sec',
            'disturbance_pulse_count',
            'disturbance_interval_sec',
            'impact_duration_sec',
            'impact_force_x_n',
            'impact_force_y_n',
            'wheel_block_duration_sec',
            'wheel_block_force_n',
            'wheel_block_yaw_torque_nm',
            'blocked_wheel',
        )
    }
    scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / 'launch' / 'physical_disturbance_scene.launch.py')
        ),
        launch_arguments=scene_arguments.items(),
    )
    wheel_uncertainty = Node(
        package='resilient_nav_localization',
        executable='wheel_odometry_uncertainty',
        name='wheel_odometry_uncertainty',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/odom',
            'output_topic': '/wheel/odometry',
        }],
    )
    passthrough_nodes = [
        Node(
            package='resilient_nav_fault_injection',
            executable=f'{sensor}_fault_injector',
            name=f'{sensor}_benchmark_passthrough',
            output='screen',
            parameters=[_passthrough(sensor)],
        )
        for sensor in ('wheel', 'imu', 'scan')
    ]
    health = Node(
        package='resilient_nav_health_assessment',
        executable='sensor_health_monitor',
        name='sensor_health_monitor',
        output='screen',
        parameters=[
            str(health_share / 'config' / 'health_monitor.yaml'),
            {'use_sim_time': True},
        ],
    )
    fixed = Node(
        package='robot_localization',
        executable='ekf_node',
        name='faulted_ekf_filter_node',
        output='screen',
        parameters=[str(fault_share / 'config' / 'faulted_ekf.yaml')],
        remappings=[('odometry/filtered', '/odometry/fixed')],
    )
    adaptive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fusion_share / 'launch' / 'phase8_adaptive_ekf.launch.py')
        ),
        launch_arguments={
            'fallback_reliability_threshold': LaunchConfiguration(
                'fallback_reliability_threshold'
            ),
        }.items(),
    )
    truth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fusion_share / 'launch' / 'phase8_ground_truth.launch.py')
        )
    )
    evaluator = Node(
        package='resilient_nav_fusion',
        executable='localization_evaluator',
        name='physical_localization_evaluator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'fixed_topic': '/odometry/fixed',
            'adaptive_topic': '/odometry/adaptive',
            'evaluation_period_sec': 0.5,
            'alignment_sweep_windows_sec': [0.02, 0.03, 0.05],
        }],
    )
    bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '--storage', 'mcap',
            '--output', LaunchConfiguration('bag_output'), *BAG_TOPICS,
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_bag')),
    )
    route = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'resilient_nav_simulation',
            'physical_disturbance_route', '--profile',
            LaunchConfiguration('route_profile'),
        ],
        output='screen',
    )
    delayed_route = TimerAction(period=4.0, actions=[route])
    stop_after_route = RegisterEventHandler(OnProcessExit(
        target_action=route,
        on_exit=[
            LogInfo(msg='Physical RF benchmark route complete; flushing outputs.'),
            EmitEvent(event=Shutdown(reason='physical RF benchmark complete')),
        ],
    ))

    arguments = (
        ('scenario', 'normal'),
        ('use_rviz', 'false'),
        ('record_bag', 'true'),
        ('bag_output', '/tmp/physical_disturbance_rf_benchmark'),
        ('route_profile', 'standard'),
        ('fallback_reliability_threshold', '0.10'),
        ('zone_center_x', '-1.5'),
        ('zone_center_y', '-3.5'),
        ('zone_length_m', '2.0'),
        ('zone_width_m', '1.0'),
        ('low_friction_mu', '0.12'),
        ('asymmetric_low_mu', '0.15'),
        ('weak_wheel_side', 'left'),
        ('roughness_height_m', '0.008'),
        ('roughness_spacing_m', '0.45'),
        ('disturbance_delay_sec', '7.0'),
        ('disturbance_pulse_count', '1'),
        ('disturbance_interval_sec', '3.0'),
        ('impact_duration_sec', '0.25'),
        ('impact_force_x_n', '0.0'),
        ('impact_force_y_n', '12.0'),
        ('wheel_block_duration_sec', '0.60'),
        ('wheel_block_force_n', '3.0'),
        ('wheel_block_yaw_torque_nm', '0.08'),
        ('blocked_wheel', 'left'),
    )
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in arguments],
        scene,
        wheel_uncertainty,
        *passthrough_nodes,
        health,
        fixed,
        adaptive,
        truth,
        evaluator,
        bag,
        delayed_route,
        stop_after_route,
    ])
