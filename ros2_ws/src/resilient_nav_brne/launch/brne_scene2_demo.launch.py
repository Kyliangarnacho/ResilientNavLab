"""BRNE Scene 2: sequential, ID-aware two-pedestrian crossing demo."""

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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    brne_share = Path(get_package_share_directory('resilient_nav_brne'))
    navigation_share = Path(get_package_share_directory('resilient_nav_navigation'))
    simulation_share = Path(get_package_share_directory('resilient_nav_simulation'))
    arm_brne = LaunchConfiguration('arm_brne')
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')
    runtime_profile = brne_share / 'config' / 'brne_v1_runtime.yaml'
    planner_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'phase10_planner_smoke.launch.py')
        ),
        launch_arguments={
            'world': str(simulation_share / 'worlds' / 'phase9_slam_world.sdf'),
            'spawn_x': '-3.5', 'spawn_y': '-3.5', 'spawn_yaw': '0.0',
            'auto_initial_pose': 'true',
            'initial_pose_x': '0.0', 'initial_pose_y': '0.0', 'initial_pose_yaw': '0.0',
            'manage_planner': 'true', 'planner_scan_topic': '/brne/static_scan',
            'use_rviz': 'false', 'log_level': log_level,
        }.items(),
    )
    pedestrian1 = brne_share / 'models' / 'brne_crossing_pedestrian.sdf'
    pedestrian2 = brne_share / 'models' / 'brne_crossing_pedestrian_2.sdf'
    spawn_pedestrian1 = Node(
        package='ros_gz_sim', executable='create', name='spawn_brne_scene2_pedestrian1', output='screen',
        parameters=[{'world': 'resilient_lab', 'name': 'brne_pedestrian', 'allow_renaming': False,
                     'file': str(pedestrian1), 'x': -2.5, 'y': -4.5, 'z': 0.60,
                     'Y': 1.5707963267948966}],
    )
    spawn_pedestrian2 = Node(
        package='ros_gz_sim', executable='create', name='spawn_brne_scene2_pedestrian2', output='screen',
        parameters=[{'world': 'resilient_lab', 'name': 'brne_pedestrian_2', 'allow_renaming': False,
                     'file': str(pedestrian2), 'x': -1.4, 'y': -2.5, 'z': 0.60,
                     'Y': -1.5707963267948966}],
    )
    bridge1 = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='brne_scene2_pedestrian1_bridge', output='screen',
        arguments=['/model/brne_pedestrian/joint/crossing_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
                   '/model/brne_pedestrian/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[('/model/brne_pedestrian/joint/crossing_joint/cmd_vel', '/brne/pedestrian/joint_velocity'),
                    ('/model/brne_pedestrian/odometry', '/brne/pedestrian/odometry')],
    )
    bridge2 = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='brne_scene2_pedestrian2_bridge', output='screen',
        arguments=['/model/brne_pedestrian_2/joint/crossing_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
                   '/model/brne_pedestrian_2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[('/model/brne_pedestrian_2/joint/crossing_joint/cmd_vel', '/brne/pedestrian2/joint_velocity'),
                    ('/model/brne_pedestrian_2/odometry', '/brne/pedestrian2/odometry')],
    )
    rviz = Node(package='rviz2', executable='rviz2', name='brne_scene2_demo_rviz', output='screen',
                arguments=['-d', str(brne_share / 'rviz' / 'brne_sensor_scene1_demo.rviz')],
                parameters=[{'use_sim_time': True}], condition=IfCondition(use_rviz))
    return LaunchDescription([
        DeclareLaunchArgument('arm_brne', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        GroupAction(actions=[planner_only], scoped=True, forwarding=True),
        # The nested Phase 10 launch creates Gazebo plus the robot.  Delay
        # Scene 2's independent actors until the world creation service is
        # available; otherwise `ros_gz_sim create` can exit before
        # `resilient_lab` exists on slower startup paths.
        TimerAction(period=5.0, actions=[spawn_pedestrian1, spawn_pedestrian2]),
        bridge1, bridge2, rviz,
        Node(package='resilient_nav_brne', executable='brne_periodic_planner',
             name='brne_periodic_planner', output='screen',
             parameters=[{'use_sim_time': True, 'goal_x': 2.8, 'goal_y': 0.0, 'goal_yaw': 0.0}]),
        Node(package='resilient_nav_brne', executable='brne_shadow_input_adapter',
             name='brne_shadow_input_adapter', output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='resilient_nav_brne', executable='brne_lidar_dynamic_agent_node',
             name='brne_scene2_lidar_dynamic_agent_node', output='screen',
             parameters=[str(runtime_profile), {'use_sim_time': True}]),
        Node(package='resilient_nav_brne', executable='brne_shadow_node', name='brne_shadow_node', output='screen',
             arguments=['--ros-args', '--log-level', log_level],
             parameters=[str(runtime_profile), {'use_sim_time': True}]),
        Node(package='resilient_nav_brne', executable='brne_crossing_pedestrian_driver',
             name='brne_scene2_pedestrian1_driver', output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='resilient_nav_brne', executable='brne_scene2_coordinator', name='brne_scene2_coordinator',
             output='screen', parameters=[{'use_sim_time': True,
                                            'robot_min_map_x': 0.8,
                                            'scene_delay_sec': 0.4}]),
        Node(package='resilient_nav_brne', executable='brne_crossing_pedestrian_driver',
             name='brne_scene2_pedestrian2_driver', output='screen', parameters=[{
                 'use_sim_time': True, 'command_topic': '/brne/pedestrian2/joint_velocity',
                 'odometry_topic': '/brne/pedestrian2/odometry', 'plan_topic': '/plan',
                 'ready_topic': '/brne/pedestrian2/ready', 'speed': 0.25,
                 'target_world_y': -4.50, 'world_y_direction': -1, 'start_delay_sec': 0.1,
                 'maximum_duration_sec': 10.0, 'input_timeout_sec': 0.5, 'publish_frequency_hz': 20.0,
             }]),
        Node(package='resilient_nav_brne', executable='brne_control_gate', name='brne_control_gate', output='screen',
             parameters=[{'use_sim_time': True, 'armed': ParameterValue(arm_brne, value_type=bool)}]),
    ])
