"""Run a self-terminating, healthy-only Phase 8 ROS graph smoke test."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Start pass-through health inputs, adaptive fusion, and a finite probe."""
    fusion_share = Path(get_package_share_directory('resilient_nav_fusion'))
    health_share = Path(
        get_package_share_directory('resilient_nav_health_assessment')
    )

    upstream = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(health_share / 'launch' / 'phase6_health_evaluation.launch.py')
        ),
        launch_arguments={
            'scenario_file': str(fusion_share / 'config' / 'healthy_passthrough.yaml'),
            'use_rviz': 'false',
            'record_bag': 'false',
        }.items(),
    )
    adapter = Node(
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
    probe = Node(
        package='resilient_nav_fusion',
        executable='healthy_smoke_probe',
        name='phase8_healthy_smoke_probe',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'required_samples': 3,
            'timeout_sec': 20.0,
            'evidence_output': '/tmp/phase8_healthy_smoke_evidence.json',
        }],
    )

    shutdown_after_probe = RegisterEventHandler(
        OnProcessExit(
            target_action=probe,
            on_exit=[
                EmitEvent(event=Shutdown(reason='phase8 healthy smoke completed'))
            ],
        )
    )

    return LaunchDescription([
        upstream,
        adapter,
        adaptive_ekf,
        probe,
        shutdown_after_probe,
    ])
