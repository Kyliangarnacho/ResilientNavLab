"""Launch phase 5 fault injection with phase 6 health evaluation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, SetParameter

from resilient_nav_health_assessment.evaluator_launch import create_health_evaluator


def generate_launch_description():
    """Start the phase 5 chain, health monitor, and health evaluator."""
    health_share = Path(
        get_package_share_directory('resilient_nav_health_assessment')
    )
    fault_share = Path(
        get_package_share_directory('resilient_nav_fault_injection')
    )
    scenario_file = LaunchConfiguration('scenario_file')
    use_rviz = LaunchConfiguration('use_rviz')
    record_bag = LaunchConfiguration('record_bag')

    phase5_fault_injection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fault_share / 'launch' / 'phase5_fault_injection.launch.py')
        ),
        launch_arguments={
            'scenario_file': scenario_file,
            'use_rviz': use_rviz,
            'record_bag': record_bag,
        }.items(),
    )

    health_monitor = Node(
        package='resilient_nav_health_assessment',
        executable='sensor_health_monitor',
        name='sensor_health_monitor',
        output='screen',
        parameters=[
            str(health_share / 'config' / 'health_monitor.yaml'),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_file',
            default_value=str(
                fault_share
                / 'config'
                / 'scenarios'
                / 'imu_bias_ekf_comparison.yaml'
            ),
            description='Phase 5 fault scenario YAML file.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Forwarded to the phase 5 RViz comparison view.',
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Forwarded to phase 5 bag recording.',
        ),
        DeclareLaunchArgument(
            'evaluator_output_json',
            default_value='/tmp/phase6_health_evaluation.json',
            description='Path written by health_evaluator when the launch stops.',
        ),
        DeclareLaunchArgument(
            'camera_health_topic',
            default_value='',
            description=(
                'Optional camera SensorHealth topic; empty preserves phase 6.'
            ),
        ),
        SetParameter(name='use_sim_time', value=True),
        phase5_fault_injection,
        health_monitor,
        OpaqueFunction(function=create_health_evaluator),
    ])
