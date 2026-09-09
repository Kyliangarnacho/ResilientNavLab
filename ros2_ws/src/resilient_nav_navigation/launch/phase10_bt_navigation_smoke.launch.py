"""Compose healthy localization with Nav2's no-recovery NavigateToPose BT."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start Planner, Controller, then BT Navigator under one ordered manager."""
    navigation_share = Path(get_package_share_directory('resilient_nav_navigation'))
    nav2_bt_share = Path(get_package_share_directory('nav2_bt_navigator'))
    simulation_share = Path(get_package_share_directory('resilient_nav_simulation'))
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    auto_initial_pose = LaunchConfiguration('auto_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    initial_pose_result = LaunchConfiguration('initial_pose_result')
    use_rviz = LaunchConfiguration('use_rviz')
    use_recovery = LaunchConfiguration('use_recovery')
    start_localization = LaunchConfiguration('start_localization')
    navigation_scan_topic = LaunchConfiguration('navigation_scan_topic')
    costmap_update_timeout = LaunchConfiguration('costmap_update_timeout')
    log_level = LaunchConfiguration('log_level')

    controller_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'phase10_controller_smoke.launch.py')
        ),
        launch_arguments={
            'world': world,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_yaw': spawn_yaw,
            'use_rviz': 'false',
            'auto_initial_pose': auto_initial_pose,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
            'initial_pose_result': initial_pose_result,
            'log_level': log_level,
            'manage_planner': 'false',
            'manage_controller': 'false',
            # Task 5.3 alone avoids the Nav2 1.3.12 bounded RPP nearest-pose
            # failure on long, repeatedly replaced paths.  False preserves
            # the frozen Task 3 / Task 4 / Task 5.1--5.2 controller contract.
            'use_recovery_controller_profile': use_recovery,
            'start_localization': start_localization,
            'navigation_scan_topic': navigation_scan_topic,
            'costmap_update_timeout': costmap_update_timeout,
        }.items(),
    )
    bt_navigator_baseline = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            str(navigation_share / 'config' / 'nav2_bt_navigator.yaml'),
            {
                'use_sim_time': True,
                'default_nav_to_pose_bt_xml': str(
                    nav2_bt_share / 'behavior_trees' / 'navigate_w_replanning_time.xml'
                ),
            },
        ],
        arguments=['--ros-args', '--log-level', log_level],
        condition=UnlessCondition(use_recovery),
    )
    bt_navigator_recovery = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            str(navigation_share / 'config' / 'nav2_bt_navigator.yaml'),
            {
                'use_sim_time': True,
                'default_nav_to_pose_bt_xml': str(
                    nav2_bt_share / 'behavior_trees'
                    / 'navigate_to_pose_w_replanning_and_recovery.xml'
                ),
            },
        ],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(use_recovery),
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            str(navigation_share / 'config' / 'nav2_behaviors.yaml'),
            {'use_sim_time': True},
        ],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(use_recovery),
    )
    lifecycle_manager_baseline = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['planner_server', 'controller_server', 'bt_navigator'],
            'use_sim_time': True,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=UnlessCondition(use_recovery),
    )
    lifecycle_manager_recovery = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'planner_server', 'controller_server', 'behavior_server',
                'bt_navigator',
            ],
            'use_sim_time': True,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(use_recovery),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='phase10_bt_navigation_rviz',
        arguments=['-d', str(navigation_share / 'rviz' / 'phase10_bt_navigation.rviz')],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('/scan', navigation_scan_topic)],
        condition=IfCondition(use_rviz),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=str(simulation_share / 'worlds' / 'phase9_slam_world.sdf'),
        ),
        DeclareLaunchArgument('spawn_x', default_value='-3.5'),
        DeclareLaunchArgument('spawn_y', default_value='-3.5'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('auto_initial_pose', default_value='true'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_result', default_value='/tmp/phase10_initial_pose.json'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        # The frozen Task 3 baseline is no-Recovery.  Task 5.3 alone selects
        # the official recovery XML and Behavior Server through this opt-in.
        DeclareLaunchArgument('use_recovery', default_value='false'),
        DeclareLaunchArgument('start_localization', default_value='true'),
        DeclareLaunchArgument('navigation_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('costmap_update_timeout', default_value='0.3'),
        DeclareLaunchArgument('log_level', default_value='info'),
        # Keep forced-off child RViz values local to nested wrappers.
        GroupAction(actions=[controller_chain], scoped=True, forwarding=True),
        bt_navigator_baseline,
        bt_navigator_recovery,
        behavior_server,
        # The official lifecycle manager owns configure/activate ordering.
        # Do not add application-level startup sleeps around that contract.
        lifecycle_manager_baseline,
        lifecycle_manager_recovery,
        rviz,
    ])
