"""Compose localization, Planner, and Controller Server for Task 3.2 only."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start FollowPath, but deliberately omit BT and NavigateToPose."""
    navigation_share = Path(get_package_share_directory('resilient_nav_navigation'))
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
    manage_planner = LaunchConfiguration('manage_planner')
    manage_controller = LaunchConfiguration('manage_controller')
    use_recovery_controller_profile = LaunchConfiguration(
        'use_recovery_controller_profile'
    )
    start_localization = LaunchConfiguration('start_localization')
    navigation_scan_topic = LaunchConfiguration('navigation_scan_topic')
    costmap_update_timeout = LaunchConfiguration('costmap_update_timeout')
    log_level = LaunchConfiguration('log_level')

    planner_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'phase10_planner_smoke.launch.py')
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
            'manage_planner': manage_planner,
            'start_localization': start_localization,
            'planner_scan_topic': navigation_scan_topic,
        }.items(),
    )
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            str(navigation_share / 'config' / 'nav2_costmaps.yaml'),
            str(navigation_share / 'config' / 'nav2_controller.yaml'),
            {
                'use_sim_time': True,
                'costmap_update_timeout': ParameterValue(
                    costmap_update_timeout, value_type=float
                ),
                # RPP 1.3.12 defaults to half of the 6 m Local Costmap (3 m).
                # Preserve that frozen Task 3 value unless the Task 5.3-only
                # Recovery profile opts into the upstream full-path search.
                'FollowPath.max_robot_pose_search_dist': ParameterValue(
                    IfElseSubstitution(
                        condition=use_recovery_controller_profile,
                        if_value='-1.0',
                        else_value='3.0',
                    ),
                    value_type=float,
                ),
            },
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('/scan', navigation_scan_topic)],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['controller_server'],
            'use_sim_time': True,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(manage_controller),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='phase10_controller_rviz',
        arguments=['-d', str(navigation_share / 'rviz' / 'phase10_controller.rviz')],
        output='screen',
        parameters=[{'use_sim_time': True}],
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
        # Defaults preserve the independent Task 3.2 lifecycle behavior.
        # Task 3.3 sets both false and supplies one ordered manager for the
        # Planner, Controller, and BT Navigator together.
        DeclareLaunchArgument('manage_planner', default_value='true'),
        DeclareLaunchArgument('manage_controller', default_value='true'),
        # Negative is Nav2 1.3.12's documented request to search the complete
        # Path for the robot pose.  Only Task 5.3 passes true; all existing
        # standalone and no-Recovery launches retain the 3.0 m baseline.
        DeclareLaunchArgument(
            'use_recovery_controller_profile', default_value='false'
        ),
        DeclareLaunchArgument('start_localization', default_value='true'),
        DeclareLaunchArgument('navigation_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('costmap_update_timeout', default_value='0.3'),
        DeclareLaunchArgument('log_level', default_value='info'),
        # Keep the planner wrapper's forced-off child RViz argument local.
        GroupAction(actions=[planner_chain], scoped=True, forwarding=True),
        controller_server,
        lifecycle_manager,
        rviz,
    ])
