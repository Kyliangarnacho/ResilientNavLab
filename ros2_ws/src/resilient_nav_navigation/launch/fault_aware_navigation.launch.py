"""Compose the runtime-only resilient localization and Nav2 goal chain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _ros_parameters(node_yaml):
    return node_yaml.get('ros__parameters', node_yaml)


def _passthrough(sensor: str) -> dict:
    parameters = {
        'use_sim_time': True,
        'status_topic': '/fault_injection/status',
        'enabled': False,
        'start_time_sec': 5.0,
        'end_time_sec': 15.0,
        'scenario_seed': 20260908,
        'scenario_id': f'{sensor}_navigation_passthrough',
        'event_id': f'{sensor}_navigation_passthrough_event',
    }
    if sensor == 'wheel':
        parameters.update({
            'input_topic': '/wheel/odometry',
            'output_topic': '/faulted/wheel/odometry',
            'model': 'freeze',
        })
    elif sensor == 'imu':
        parameters.update({
            'input_topic': '/imu/data',
            'output_topic': '/faulted/imu/data',
            'model': 'bias',
            'bias_rad_s': 0.0,
        })
    else:
        parameters.update({
            'input_topic': '/scan',
            'output_topic': '/faulted/scan',
            'model': 'sector_blindness',
            'sector_center_rad': 0.0,
            'sector_width_rad': 1.0,
        })
    return parameters


def _fault_nodes(context):
    """Load an optional sensor scenario without exposing it downstream."""
    scenario_path = Path(
        LaunchConfiguration('sensor_scenario_file').perform(context)
    )
    with scenario_path.open('r', encoding='utf-8') as stream:
        scenario = yaml.safe_load(stream) or {}

    specifications = (
        ('wheel', 'wheel_fault_injector', 'wheel_fault_injector'),
        ('imu', 'imu_fault_injector', 'imu_fault_injector'),
        ('scan', 'scan_fault_injector', 'scan_fault_injector'),
    )
    actions = []
    for sensor, key, executable in specifications:
        configured = scenario.get(key)
        if sensor == 'imu' and not isinstance(configured, dict):
            configured = scenario.get('imu_bias_injector')
            if isinstance(configured, dict):
                executable = 'imu_bias_injector'
        parameters = (
            _ros_parameters(configured)
            if isinstance(configured, dict)
            else _passthrough(sensor)
        )
        actions.append(Node(
            package='resilient_nav_fault_injection',
            executable=executable,
            name=f'{sensor}_navigation_fault_injector',
            output='screen',
            parameters=[parameters],
        ))
    return actions


def generate_launch_description():
    """Start one TF-owner chain and gate Nav2 goals above its BT."""
    navigation_share = Path(
        get_package_share_directory('resilient_nav_navigation')
    )
    simulation_share = Path(
        get_package_share_directory('resilient_nav_simulation')
    )
    fusion_share = Path(get_package_share_directory('resilient_nav_fusion'))
    health_share = Path(
        get_package_share_directory('resilient_nav_health_assessment')
    )

    scene_argument_names = (
        'base_world', 'entity_name', 'spawn_x', 'spawn_y', 'spawn_z',
        'spawn_yaw', 'zone_center_x', 'zone_center_y', 'zone_length_m',
        'zone_width_m', 'low_friction_mu', 'asymmetric_low_mu',
        'weak_wheel_side', 'roughness_height_m', 'roughness_spacing_m',
        'disturbance_delay_sec', 'disturbance_pulse_count',
        'disturbance_interval_sec', 'impact_duration_sec',
        'impact_force_x_n', 'impact_force_y_n', 'wheel_block_duration_sec',
        'wheel_block_force_n', 'wheel_block_yaw_torque_nm', 'blocked_wheel',
    )
    scene_arguments = {
        name: LaunchConfiguration(name) for name in scene_argument_names
    }
    scene_arguments.update({
        'scenario': LaunchConfiguration('physical_scenario'),
        'use_rviz': 'false',
        'odom_ros_topic': '/odom',
        'start_odom_tf_broadcaster': 'false',
    })
    scene = GroupAction(
        scoped=True,
        forwarding=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    simulation_share
                    / 'launch'
                    / 'physical_disturbance_scene.launch.py'
                )
            ),
            launch_arguments=scene_arguments.items(),
        )],
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
    adaptive_fusion = GroupAction(
        scoped=True,
        forwarding=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(fusion_share / 'launch' / 'phase8_adaptive_ekf.launch.py')
            ),
            launch_arguments={
                'fallback_reliability_threshold': LaunchConfiguration(
                    'fallback_reliability_threshold'
                ),
                'adaptive_output_topic': '/odometry/filtered',
                'publish_tf': 'true',
            }.items(),
        )],
    )
    localization = GroupAction(
        scoped=True,
        forwarding=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    navigation_share
                    / 'launch'
                    / 'phase10_localization.launch.py'
                )
            ),
            launch_arguments={
                'params_file': str(
                    navigation_share
                    / 'config'
                    / 'nav2_fault_aware_localization.yaml'
                ),
                'use_sim_time': 'true',
                'use_rviz': 'false',
                'use_composition': 'False',
                'use_respawn': 'false',
                'log_level': LaunchConfiguration('log_level'),
            }.items(),
        )],
    )
    initial_pose = Node(
        package='resilient_nav_navigation',
        executable='phase10_initial_pose_helper',
        name='phase10_initial_pose_helper',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('initial_pose_x'),
            '--y', LaunchConfiguration('initial_pose_y'),
            '--yaw', LaunchConfiguration('initial_pose_yaw'),
            '--result-path', LaunchConfiguration('initial_pose_result'),
        ],
        parameters=[{'use_sim_time': True}],
    )
    localization_quality = GroupAction(
        scoped=True,
        forwarding=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    navigation_share
                    / 'launch'
                    / 'localization_quality_monitor.launch.py'
                )
            )
        )],
    )
    nav2 = GroupAction(
        scoped=True,
        forwarding=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    navigation_share
                    / 'launch'
                    / 'phase10_bt_navigation_smoke.launch.py'
                )
            ),
            launch_arguments={
                'start_localization': 'false',
                'navigation_scan_topic': '/faulted/scan',
                'costmap_update_timeout': '0.3',
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_recovery': LaunchConfiguration('use_recovery'),
                'log_level': LaunchConfiguration('log_level'),
            }.items(),
        )],
    )
    supervisor = Node(
        package='resilient_nav_navigation',
        executable='resilience_supervisor',
        name='resilience_supervisor',
        output='screen',
        parameters=[
            str(navigation_share / 'config' / 'resilience_supervisor.yaml'),
            {'use_sim_time': True},
        ],
    )
    goal_gate = Node(
        package='resilient_nav_navigation',
        executable='resilient_goal_gate',
        name='resilient_goal_gate',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'upstream_action_name': '/resilient_navigate_to_pose',
            'downstream_action_name': '/navigate_to_pose',
            'navigation_scan_topic': '/faulted/scan',
            'replay_tf_target_frame': 'map',
            'replay_tf_confirmation_count': 2,
        }],
    )

    defaults = (
        ('physical_scenario', 'normal'),
        ('sensor_scenario_file', str(
            fusion_share / 'config' / 'healthy_passthrough.yaml'
        )),
        ('base_world', str(
            simulation_share / 'worlds' / 'phase9_slam_world.sdf'
        )),
        ('entity_name', 'resilient_nav_robot'),
        ('spawn_x', '-4.0'), ('spawn_y', '-3.5'), ('spawn_z', '0.25'),
        ('spawn_yaw', '0.0'),
        ('initial_pose_x', '0.0'), ('initial_pose_y', '0.0'),
        ('initial_pose_yaw', '0.0'),
        ('initial_pose_result', '/tmp/fault_aware_initial_pose.json'),
        ('use_rviz', 'false'), ('use_recovery', 'false'),
        ('log_level', 'info'), ('fallback_reliability_threshold', '0.10'),
        ('zone_center_x', '-1.5'), ('zone_center_y', '-3.5'),
        ('zone_length_m', '2.0'), ('zone_width_m', '1.0'),
        ('low_friction_mu', '0.12'), ('asymmetric_low_mu', '0.15'),
        ('weak_wheel_side', 'left'), ('roughness_height_m', '0.008'),
        ('roughness_spacing_m', '0.45'), ('disturbance_delay_sec', '7.0'),
        ('disturbance_pulse_count', '1'),
        ('disturbance_interval_sec', '3.0'),
        ('impact_duration_sec', '0.25'), ('impact_force_x_n', '0.0'),
        ('impact_force_y_n', '12.0'), ('wheel_block_duration_sec', '0.60'),
        ('wheel_block_force_n', '3.0'),
        ('wheel_block_yaw_torque_nm', '0.08'), ('blocked_wheel', 'left'),
    )
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in defaults
        ],
        scene,
        wheel_uncertainty,
        OpaqueFunction(function=_fault_nodes),
        health,
        adaptive_fusion,
        localization,
        initial_pose,
        localization_quality,
        nav2,
        supervisor,
        goal_gate,
    ])
