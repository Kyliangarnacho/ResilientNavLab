"""One bounded Phase 10 run for BRNE Task 1 shadow-only closeout evidence."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Start one frozen Phase 10 run plus isolated BRNE processes and observer."""
    navigation_share = Path(get_package_share_directory('resilient_nav_navigation'))
    brne_share = Path(get_package_share_directory('resilient_nav_brne'))
    runtime_profile = brne_share / 'config' / 'brne_v1_runtime.yaml'
    venv_python = LaunchConfiguration('venv_python')
    observer = ExecuteProcess(
        cmd=[
            venv_python, '-c',
            'from resilient_nav_brne.brne_task1_closeout_observer import main; main()',
            '--ros-args', '-p', 'use_sim_time:=true', '-p', 'deadline_sec:=28.0',
        ],
        name='brne_task1_closeout_observer', output='screen',
    )
    adapter = ExecuteProcess(
        cmd=[
            venv_python, '-c',
            'from resilient_nav_brne.brne_shadow_input_adapter import main; main()',
            '--ros-args', '-p', 'use_sim_time:=true',
        ],
        name='brne_shadow_input_adapter', output='screen',
    )
    shadow = ExecuteProcess(
        cmd=[
            venv_python, '-c',
            'from resilient_nav_brne.brne_shadow_node import main; main()',
            '--ros-args', '--params-file', str(runtime_profile),
            '-p', 'use_sim_time:=true',
        ],
        name='brne_shadow_node', output='screen',
    )
    pedestrians = ExecuteProcess(
        cmd=[
            venv_python, '-c',
            'from resilient_nav_brne.brne_synthetic_pedestrian_source import main; main()',
            '--ros-args', '-p', 'use_sim_time:=true',
        ],
        name='brne_synthetic_pedestrian_source', output='screen',
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'phase10_bt_navigation_smoke.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false', 'use_recovery': 'false',
            'initial_pose_result': '/tmp/brne_task1_initial_pose.json',
        }.items(),
    )
    runner = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'resilient_nav_navigation', 'phase10_navigate_to_pose_probe',
            '--scenario', 'simple_reachable', '--result-path',
            '/tmp/brne_task1_navigation.json', '--timeout-sec', '90.0',
        ],
        name='brne_task1_phase10_runner', output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'venv_python',
            description='Python executable in the existing repository .venv (must contain Numba).',
        ),
        DeclareLaunchArgument('gz_partition', default_value='resilient_nav_brne_task1_closeout'),
        DeclareLaunchArgument('ros_domain_id', default_value='85'),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gz_partition')),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        # Start this passive recorder before any navigation action is sent.
        observer,
        adapter,
        shadow,
        pedestrians,
        navigation,
        runner,
        RegisterEventHandler(OnProcessExit(
            target_action=observer,
            on_exit=[EmitEvent(event=Shutdown(reason='BRNE Task 1 closeout observer finished'))],
        )),
    ])
