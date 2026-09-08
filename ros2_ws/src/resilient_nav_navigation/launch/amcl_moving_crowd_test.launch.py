"""Observe stationary AMCL while four new pedestrians move continuously."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


PEDESTRIANS = (
    ('amcl_test_pedestrian_1', -2.0, -4.7, 1.5707963267948966),
    ('amcl_test_pedestrian_2', -1.2, -2.3, -1.5707963267948966),
    ('amcl_test_pedestrian_3', -0.4, -4.5, 1.5707963267948966),
    ('amcl_test_pedestrian_4', 0.5, -2.5, -1.5707963267948966),
)


def generate_launch_description():
    """Start localization, four independent actors, and the AMCL RViz view."""
    navigation_share = Path(
        get_package_share_directory('resilient_nav_navigation')
    )
    simulation_share = Path(
        get_package_share_directory('resilient_nav_simulation')
    )
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                navigation_share
                / 'launch'
                / 'phase10_localization_smoke.launch.py'
            )
        ),
        launch_arguments={
            'world': str(
                simulation_share / 'worlds' / 'phase9_slam_world.sdf'
            ),
            'spawn_x': '-3.5',
            'spawn_y': '-3.5',
            'spawn_yaw': '0.0',
            'auto_initial_pose': 'true',
            'initial_pose_x': '0.0',
            'initial_pose_y': '0.0',
            'initial_pose_yaw': '0.0',
            'initial_pose_result': '/tmp/amcl_moving_crowd_initial_pose.json',
            'use_rviz': 'false',
            'log_level': log_level,
        }.items(),
    )

    model_file = (
        navigation_share / 'models' / 'amcl_crowd_pedestrian' / 'model.sdf'
    )
    spawners = [
        Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{name}',
            output='screen',
            parameters=[{
                'world': 'resilient_lab',
                'name': name,
                'allow_renaming': False,
                'file': str(model_file),
                'x': x,
                'y': y,
                'z': 0.60,
                'Y': yaw,
            }],
        )
        for name, x, y, yaw in PEDESTRIANS
    ]
    pose_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='amcl_crowd_pose_service_bridge',
        output='screen',
        arguments=[
            '/world/resilient_lab/set_pose@'
            'ros_gz_interfaces/srv/SetEntityPose'
        ],
    )
    oscillator = Node(
        package='resilient_nav_navigation',
        executable='amcl_crowd_oscillator',
        name='amcl_crowd_oscillator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'entity_names': [item[0] for item in PEDESTRIANS],
            'base_x': [item[1] for item in PEDESTRIANS],
            'base_y': [item[2] for item in PEDESTRIANS],
            'yaw': [item[3] for item in PEDESTRIANS],
            'speed': 0.25,
            'travel_distance': 2.0,
            'start_delay_sec': 7.0,
        }],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='amcl_obstacle_test_rviz',
        output='screen',
        arguments=[
            '-d', str(navigation_share / 'rviz' / 'amcl_obstacle_test.rviz')
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        GroupAction(actions=[localization], scoped=True, forwarding=True),
        pose_service_bridge,
        oscillator,
        TimerAction(period=5.0, actions=spawners),
        rviz,
    ])
