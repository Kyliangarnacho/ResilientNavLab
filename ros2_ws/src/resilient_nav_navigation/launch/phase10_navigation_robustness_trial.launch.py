"""Task 5 trial: the frozen Task 4 stack plus one Gazebo-only obstacle event."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = Path(get_package_share_directory('resilient_nav_navigation'))
    scenario = LaunchConfiguration('scenario')
    navigation_output = LaunchConfiguration('navigation_output')
    ground_truth_output = LaunchConfiguration('ground_truth_output')
    diagnostics_output = LaunchConfiguration('diagnostics_output')
    initial_pose_result = LaunchConfiguration('initial_pose_result')
    event_output = LaunchConfiguration('event_output')
    record_diagnostics = LaunchConfiguration('record_diagnostics')
    use_rviz = LaunchConfiguration('use_rviz')
    use_recovery = LaunchConfiguration('use_recovery')
    enable_obstacle_event = LaunchConfiguration('enable_obstacle_event')
    gz_partition = LaunchConfiguration('gz_partition')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    readiness_timeout_sec = LaunchConfiguration('readiness_timeout_sec')
    scenarios_file = LaunchConfiguration('scenarios_file')
    trial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'phase10_navigation_benchmark_trial.launch.py')
        ),
        launch_arguments={
            'scenario': scenario,
            'navigation_output': navigation_output,
            'ground_truth_output': ground_truth_output,
            'diagnostics_output': diagnostics_output,
            'initial_pose_result': initial_pose_result,
            'record_diagnostics': record_diagnostics,
            'use_rviz': use_rviz,
            'use_recovery': use_recovery,
            'gz_partition': gz_partition,
            'ros_domain_id': ros_domain_id,
            'readiness_timeout_sec': readiness_timeout_sec,
            'scenarios_file': scenarios_file,
        }.items(),
    )
    # ros_gz_bridge exposes Gazebo's documented entity services. The injector
    # performs the frozen scenario's create/delete events only; it never
    # observes Recovery or controls Nav2.
    entity_service_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='phase10_task5_entity_factory_bridge', output='screen',
        arguments=[
            '/world/resilient_lab/create@ros_gz_interfaces/srv/SpawnEntity',
            '/world/resilient_lab/remove@ros_gz_interfaces/srv/DeleteEntity',
        ],
        condition=IfCondition(enable_obstacle_event),
    )
    injector = Node(
        package='resilient_nav_navigation',
        executable='phase10_navigation_obstacle_event_injector',
        name='phase10_navigation_obstacle_event_injector', output='screen',
        arguments=[
            '--scenario', scenario,
            '--scenarios-file', scenarios_file,
            '--output-path', event_output,
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(enable_obstacle_event),
    )
    # The primary runner owns terminal trial shutdown.  The injector has no
    # control capability beyond its one official Gazebo create request.
    return LaunchDescription([
        DeclareLaunchArgument('scenario'),
        DeclareLaunchArgument('navigation_output'),
        DeclareLaunchArgument('ground_truth_output'),
        DeclareLaunchArgument('diagnostics_output'),
        DeclareLaunchArgument('initial_pose_result'),
        DeclareLaunchArgument('event_output'),
        DeclareLaunchArgument('record_diagnostics', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_recovery', default_value='false'),
        DeclareLaunchArgument('enable_obstacle_event', default_value='true'),
        DeclareLaunchArgument('gz_partition', default_value='resilient_nav_phase10_task5'),
        DeclareLaunchArgument('ros_domain_id', default_value='57'),
        DeclareLaunchArgument('readiness_timeout_sec', default_value='120.0'),
        DeclareLaunchArgument(
            'scenarios_file',
            default_value=str(navigation_share / 'config' / 'navigation_robustness_scenarios.yaml'),
        ),
        # Apply the same isolation to the parent bridge/injector as to the
        # included Task 4 trial; launch inclusion must not make this implicit.
        SetEnvironmentVariable('GZ_PARTITION', gz_partition),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        trial,
        entity_service_bridge,
        injector,
    ])
