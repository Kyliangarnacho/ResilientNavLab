"""One fresh-process Task 4 trial: frozen Nav2 chain plus evaluator-only GT."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = Path(get_package_share_directory('resilient_nav_navigation'))
    fusion_share = Path(get_package_share_directory('resilient_nav_fusion'))
    scenario = LaunchConfiguration('scenario')
    navigation_output = LaunchConfiguration('navigation_output')
    ground_truth_output = LaunchConfiguration('ground_truth_output')
    diagnostics_output = LaunchConfiguration('diagnostics_output')
    initial_pose_result = LaunchConfiguration('initial_pose_result')
    record_diagnostics = LaunchConfiguration('record_diagnostics')
    use_rviz = LaunchConfiguration('use_rviz')
    use_recovery = LaunchConfiguration('use_recovery')
    gz_partition = LaunchConfiguration('gz_partition')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    readiness_timeout_sec = LaunchConfiguration('readiness_timeout_sec')
    scenarios_file = LaunchConfiguration('scenarios_file')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(navigation_share / 'launch' / 'phase10_bt_navigation_smoke.launch.py')),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_recovery': use_recovery,
            # Keep the verified Task 3 chain unchanged.  Its official
            # navigation Lifecycle Manager owns configure/activate ordering.
            'initial_pose_result': initial_pose_result,
        }.items(),
    )
    ground_truth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(fusion_share / 'launch' / 'phase8_ground_truth.launch.py'))
    )
    runner = Node(
        package='resilient_nav_navigation', executable='phase10_navigation_benchmark_runner',
        name='phase10_navigation_benchmark_runner', output='screen',
        arguments=[
            '--scenario', scenario, '--output-path', navigation_output,
            '--timeout-sec', readiness_timeout_sec,
            '--scenarios-file', scenarios_file,
        ],
        parameters=[{'use_sim_time': True}],
    )
    recorder = Node(
        package='resilient_nav_navigation', executable='phase10_navigation_benchmark_gt_recorder',
        name='phase10_navigation_benchmark_gt_recorder', output='screen',
        arguments=['--output-path', ground_truth_output], parameters=[{'use_sim_time': True}],
    )
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--output', diagnostics_output,
             '/clock', '/map', '/scan', '/global_costmap/costmap_raw', '/local_costmap/costmap_raw', '/tf', '/tf_static', '/amcl_pose', '/plan', '/received_global_plan', '/cmd_vel', '/odometry/filtered', '/evaluation/ground_truth_pose', '/behavior_tree_log'],
        output='screen', condition=IfCondition(record_diagnostics),
    )
    shutdown_when_runner_finishes = RegisterEventHandler(
        OnProcessExit(target_action=runner, on_exit=[EmitEvent(event=Shutdown(reason='benchmark runner finished'))])
    )
    return LaunchDescription([
        DeclareLaunchArgument('scenario'),
        DeclareLaunchArgument('navigation_output'),
        DeclareLaunchArgument('ground_truth_output'),
        DeclareLaunchArgument('diagnostics_output'),
        DeclareLaunchArgument('initial_pose_result'),
        # rosbag is forensic evidence, not a prerequisite for this healthy
        # benchmark.  Leave it out of the localization discovery burst by
        # default; an explicitly requested diagnostic run can enable it.
        DeclareLaunchArgument('record_diagnostics', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        # Task 4 retains false. Task 5.3 only forwards its scenario-selected
        # official recovery profile through this existing trial overlay.
        DeclareLaunchArgument('use_recovery', default_value='false'),
        # Gazebo Transport is process-external.  Isolate the world and all
        # ros_gz bridges from stale simulations in the default partition.
        DeclareLaunchArgument('gz_partition', default_value='resilient_nav_phase10_task4'),
        # ROS 2 discovery is independent of Gazebo Transport.  Keep all
        # benchmark nodes and bags out of any default-domain leftovers.
        DeclareLaunchArgument('ros_domain_id', default_value='47'),
        # One broad application readiness deadline.  It covers only the
        # prerequisite facts needed to send NavigateToPose, not cold-start
        # latency as a benchmark metric.
        DeclareLaunchArgument('readiness_timeout_sec', default_value='120.0'),
        DeclareLaunchArgument(
            'scenarios_file',
            default_value=str(navigation_share / 'config' / 'planner_smoke_scenarios.yaml'),
        ),
        SetEnvironmentVariable('GZ_PARTITION', gz_partition),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        navigation,
        ground_truth,
        recorder,
        bag,
        runner,
        shutdown_when_runner_finishes,
    ])
