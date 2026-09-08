"""Compose healthy localization with Nav2 Planner Server and its Global Costmap."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Start ComputePathToPose only; no controller or navigation executor."""
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
    global_obstacle_layer_enabled = LaunchConfiguration(
        'global_obstacle_layer_enabled'
    )
    planner_scan_topic = LaunchConfiguration('planner_scan_topic')
    planner_plan_topic = LaunchConfiguration('planner_plan_topic')
    log_level = LaunchConfiguration('log_level')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'phase10_localization_smoke.launch.py')
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
        }.items(),
    )

    configured_costmaps = ParameterFile(
        RewrittenYaml(
            source_file=str(navigation_share / 'config' / 'nav2_costmaps.yaml'),
            param_rewrites={
                (
                    'global_costmap.global_costmap.ros__parameters.'
                    'obstacle_layer.enabled'
                ): global_obstacle_layer_enabled,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            configured_costmaps,
            str(navigation_share / 'config' / 'nav2_planner.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[
            ('/scan', planner_scan_topic),
            ('/plan', planner_plan_topic),
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['planner_server'],
            'use_sim_time': True,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(manage_planner),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='phase10_planner_rviz',
        arguments=['-d', str(navigation_share / 'rviz' / 'phase10_planner.rviz')],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=str(simulation_share / 'worlds' / 'phase9_slam_world.sdf'),
            description='The frozen Phase 9 Gazebo world used by Task 1.',
        ),
        DeclareLaunchArgument('spawn_x', default_value='-3.5'),
        DeclareLaunchArgument('spawn_y', default_value='-3.5'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('auto_initial_pose', default_value='true'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'initial_pose_result', default_value='/tmp/phase10_initial_pose.json'
        ),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        # Standalone Task 3.1 owns Planner Server's lifecycle.  The Task 3.3
        # BT wrapper disables this manager and starts all navigation servers
        # in one explicit ordered manager instead.
        DeclareLaunchArgument('manage_planner', default_value='true'),
        DeclareLaunchArgument(
            'global_obstacle_layer_enabled',
            default_value='true',
            description=(
                'Enable live LaserScan marking/clearing in the global costmap.'
            ),
        ),
        DeclareLaunchArgument('planner_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('planner_plan_topic', default_value='/plan'),
        DeclareLaunchArgument('log_level', default_value='info'),
        # Scope the child launch's forced-off Phase 4 RViz setting so it
        # cannot overwrite this wrapper's optional planner RViz argument.
        GroupAction(actions=[localization], scoped=True, forwarding=True),
        planner_server,
        lifecycle_manager,
        rviz,
    ])
