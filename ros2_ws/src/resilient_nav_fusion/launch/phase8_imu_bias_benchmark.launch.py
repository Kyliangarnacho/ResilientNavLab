"""Controlled, self-terminating Phase 8 IMU comparison benchmark."""

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
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start a fixed/adaptive/truth benchmark from the existing Phase 6 scenario."""
    fusion_share = Path(get_package_share_directory('resilient_nav_fusion'))
    health_share = Path(get_package_share_directory('resilient_nav_health_assessment'))
    fault_share = Path(get_package_share_directory('resilient_nav_fault_injection'))
    output_json = LaunchConfiguration('output_json')
    scenario_file = LaunchConfiguration('scenario_file')
    benchmark_name = LaunchConfiguration('benchmark_name')
    fault_sensor = LaunchConfiguration('fault_sensor')
    fault_status_sensor = LaunchConfiguration('fault_status_sensor')
    health_topic = LaunchConfiguration('health_topic')
    health_output_json = LaunchConfiguration('health_output_json')
    motion_duration_sec = LaunchConfiguration('motion_duration_sec')
    benchmark_timeout_wall_sec = LaunchConfiguration('benchmark_timeout_wall_sec')
    require_path_evidence = LaunchConfiguration('require_path_evidence')
    min_complete_sim_time_sec = LaunchConfiguration('min_complete_sim_time_sec')

    phase6 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(health_share / 'launch' / 'phase6_health_evaluation.launch.py')
        ),
        launch_arguments={
            'scenario_file': scenario_file,
            'use_rviz': 'false',
            'record_bag': 'false',
            'evaluator_output_json': health_output_json,
            'camera_health_topic': '',
        }.items(),
    )
    adaptive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fusion_share / 'launch' / 'phase8_adaptive_ekf.launch.py')
        )
    )
    ground_truth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fusion_share / 'launch' / 'phase8_ground_truth.launch.py')
        )
    )
    evaluator = Node(
        package='resilient_nav_fusion',
        executable='localization_evaluator',
        name='localization_evaluator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'evaluation_period_sec': 0.5,
            'alignment_sweep_windows_sec': [0.02, 0.03, 0.05],
        }],
    )
    paths = Node(
        package='resilient_nav_fusion',
        executable='trajectory_path_adapter',
        name='trajectory_path_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    recorder = Node(
        package='resilient_nav_fusion',
        executable='imu_benchmark_runner',
        name='phase8_imu_benchmark_runner',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'benchmark_name': benchmark_name,
                'fault_sensor': fault_sensor,
                'fault_status_sensor': fault_status_sensor,
                'health_topic': health_topic,
                'output_json': output_json,
                'timeout_wall_sec': benchmark_timeout_wall_sec,
                'require_alignment_sensitivity': True,
                'require_path_evidence': require_path_evidence,
                'min_complete_sim_time_sec': min_complete_sim_time_sec,
            }
        ],
    )
    motion = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'run',
                    'resilient_nav_simulation',
                    'motion_test',
                    'arc',
                    '--linear-speed',
                    '0.20',
                    '--angular-speed',
                    '0.30',
                    '--duration',
                    motion_duration_sec,
                ],
                output='screen',
            )
        ],
    )
    shutdown_when_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=recorder,
            on_exit=[
                LogInfo(msg='Phase 8 sensor benchmark recorder exited; shutting down.'),
                EmitEvent(event=Shutdown(reason='benchmark recorder completed')),
            ],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_json',
            default_value='/tmp/phase8_imu_bias_benchmark.json',
            description='Benchmark result JSON; the recorder refuses partial success.',
        ),
        DeclareLaunchArgument(
            'min_complete_sim_time_sec',
            default_value='0.0',
            description='Minimum observed simulation time before recorder success.',
        ),
        DeclareLaunchArgument(
            'require_path_evidence',
            default_value='false',
            description='Require all evaluation-only Path streams before success.',
        ),
        DeclareLaunchArgument(
            'scenario_file',
            default_value=str(
                fault_share
                / 'config'
                / 'scenarios'
                / 'imu_bias_ekf_comparison.yaml'
            ),
            description='Existing Phase 5/6 IMU fault scenario to benchmark.',
        ),
        DeclareLaunchArgument(
            'benchmark_name',
            default_value='phase8_imu_bias',
            description='Evaluation-only label written to the benchmark record.',
        ),
        DeclareLaunchArgument(
            'fault_sensor',
            default_value='imu',
            description='FaultStatus sensor and health channel to observe.',
        ),
        DeclareLaunchArgument(
            'fault_status_sensor',
            default_value='',
            description='Optional FaultStatus.sensor override for logical targets.',
        ),
        DeclareLaunchArgument(
            'health_topic',
            default_value='/health/imu',
            description='Target SensorHealth topic for benchmark evidence.',
        ),
        DeclareLaunchArgument(
            'health_output_json',
            default_value='/tmp/phase8_imu_bias_health_evaluation.json',
            description='Phase 6 evaluator output path for this isolated run.',
        ),
        DeclareLaunchArgument(
            'motion_duration_sec',
            default_value='24.0',
            description='Bounded wall-clock motion duration for benchmark stimulus.',
        ),
        DeclareLaunchArgument(
            'benchmark_timeout_wall_sec',
            default_value='60.0',
            description='Wall-clock fail-closed timeout for the benchmark recorder.',
        ),
        phase6,
        adaptive,
        ground_truth,
        evaluator,
        paths,
        recorder,
        motion,
        shutdown_when_complete,
    ])
