"""Compose healthy localization with a standalone odom-frame Local Costmap."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start Local Costmap only; planning and control remain excluded."""
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
    log_level = LaunchConfiguration('log_level')
    inflation_radius = LaunchConfiguration('local_inflation_radius')
    cost_scaling_factor = LaunchConfiguration('local_cost_scaling_factor')

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

    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='local_costmap',
        name='local_costmap',
        output='screen',
        parameters=[
            str(navigation_share / 'config' / 'nav2_costmaps.yaml'),
            {
                'use_sim_time': True,
                'inflation_layer.inflation_radius': ParameterValue(
                    inflation_radius, value_type=float
                ),
                'inflation_layer.cost_scaling_factor': ParameterValue(
                    cost_scaling_factor, value_type=float
                ),
            },
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace='local_costmap',
        name='lifecycle_manager_local_costmap',
        output='screen',
        parameters=[{
            'autostart': True,
            # Standalone Costmap accepts lifecycle transitions but does not
            # establish Nav2's managed-node bond in this Jazzy baseline.
            'bond_timeout': 0.0,
            'node_names': ['local_costmap'],
            'use_sim_time': True,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='phase10_local_costmap_rviz',
        arguments=['-d', str(navigation_share / 'rviz' / 'phase10_local_costmap.rviz')],
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
        DeclareLaunchArgument('local_inflation_radius', default_value='0.55'),
        DeclareLaunchArgument('local_cost_scaling_factor', default_value='3.0'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        # Keep the localization child's forced-off RViz setting local to its
        # scope so it cannot alter this wrapper's optional RViz instance.
        GroupAction(actions=[localization], scoped=True, forwarding=True),
        local_costmap,
        lifecycle_manager,
        rviz,
    ])
