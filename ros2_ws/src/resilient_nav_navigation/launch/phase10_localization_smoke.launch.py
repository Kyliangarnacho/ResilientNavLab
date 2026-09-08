"""Compose the healthy Phase 9 simulation inputs with Nav2 AMCL only."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Keep the Task 1 TF owners explicit and exclude all navigation servers."""
    navigation_share = Path(
        get_package_share_directory('resilient_nav_navigation')
    )
    localization_share = Path(
        get_package_share_directory('resilient_nav_localization')
    )
    simulation_share = Path(
        get_package_share_directory('resilient_nav_simulation')
    )

    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_initial_pose = LaunchConfiguration('auto_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    initial_pose_result = LaunchConfiguration('initial_pose_result')
    log_level = LaunchConfiguration('log_level')

    healthy_inputs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / 'launch' / 'phase4_imu_lidar_demo.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false',
            'world': world,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_yaw': spawn_yaw,
            'odom_ros_topic': '/wheel/odometry/raw',
            'start_odom_tf_broadcaster': 'false',
        }.items(),
    )

    healthy_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[str(localization_share / 'config' / 'ekf.yaml')],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )

    wheel_odometry_uncertainty = Node(
        package='resilient_nav_localization',
        executable='wheel_odometry_uncertainty',
        name='wheel_odometry_uncertainty',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'phase10_localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz': use_rviz,
            # Preserve Nav2 Jazzy's capitalized Python boolean contract.
            'use_composition': 'False',
            'use_respawn': 'false',
            'log_level': log_level,
        }.items(),
    )

    initial_pose_helper = Node(
        package='resilient_nav_navigation',
        executable='phase10_initial_pose_helper',
        name='phase10_initial_pose_helper',
        output='screen',
        arguments=[
            '--x', initial_pose_x,
            '--y', initial_pose_y,
            '--yaw', initial_pose_yaw,
            '--result-path', initial_pose_result,
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(auto_initial_pose),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=str(simulation_share / 'worlds' / 'phase9_slam_world.sdf'),
            description='The frozen Phase 9 Gazebo world.',
        ),
        DeclareLaunchArgument('spawn_x', default_value='-3.5'),
        DeclareLaunchArgument('spawn_y', default_value='-3.5'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'initial_pose_x', default_value='0.0',
            description='Explicit AMCL initial-pose X in the saved map frame.',
        ),
        DeclareLaunchArgument(
            'initial_pose_y', default_value='0.0',
            description='Explicit AMCL initial-pose Y in the saved map frame.',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw', default_value='0.0',
            description='Explicit AMCL initial-pose yaw in the saved map frame.',
        ),
        DeclareLaunchArgument(
            'auto_initial_pose', default_value='false',
            description='Start the one-shot, active-state-gated initial-pose helper.',
        ),
        DeclareLaunchArgument(
            'initial_pose_result', default_value='/tmp/phase10_initial_pose.json',
            description='Writable evidence path for the optional helper.',
        ),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        # Keep the child launch's use_rviz=false from overwriting this wrapper's
        # use_rviz configuration before Nav2 localization is included.
        GroupAction(
            actions=[healthy_inputs],
            scoped=True,
            forwarding=True,
        ),
        wheel_odometry_uncertainty,
        healthy_ekf,
        nav2_localization,
        initial_pose_helper,
    ])
