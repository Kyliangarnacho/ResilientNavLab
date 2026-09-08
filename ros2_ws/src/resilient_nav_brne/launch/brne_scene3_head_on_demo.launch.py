"""BRNE Scene 3: one sensor-observed pedestrian approaching head-on."""

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
            'spawn_x': '-3.5',
            'spawn_y': '-3.5',
            'spawn_yaw': '0.0',
            'auto_initial_pose': 'true',
            'initial_pose_x': '0.0',
            'initial_pose_y': '0.0',
            'initial_pose_yaw': '0.0',
            'manage_planner': 'true',
            'global_obstacle_layer_enabled': 'false',
            'use_rviz': 'false',
            'log_level': log_level,
        }.items(),
    )
    pedestrian_model = brne_share / 'models' / 'brne_crossing_pedestrian.sdf'
    spawn_pedestrian = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_brne_scene3_head_on_pedestrian',
        output='screen',
        parameters=[{
            'world': 'resilient_lab',
            'name': 'brne_pedestrian',
            'allow_renaming': False,
            'file': str(pedestrian_model),
            # Robot world (-3.5, -3.5) is odom (0, 0). The actor begins
            # 3.0 m straight ahead; yaw=pi maps model +X to world -X.
            'x': -0.5,
            'y': -3.5,
            'z': 0.60,
            'Y': 3.141592653589793,
        }],
    )
    pedestrian_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='brne_scene3_pedestrian_bridge',
        output='screen',
        arguments=[
            (
                '/model/brne_pedestrian/joint/crossing_joint/cmd_vel'
                '@std_msgs/msg/Float64]gz.msgs.Double'
            ),
            '/model/brne_pedestrian/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            (
                '/model/brne_pedestrian/joint/crossing_joint/cmd_vel',
                '/brne/pedestrian/joint_velocity',
            ),
            (
                '/model/brne_pedestrian/odometry',
                '/brne/pedestrian/odometry',
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('arm_brne', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        GroupAction(actions=[planner_only], scoped=True, forwarding=True),
        TimerAction(period=5.0, actions=[spawn_pedestrian]),
        pedestrian_bridge,
        Node(
            package='rviz2',
            executable='rviz2',
            name='brne_scene3_demo_rviz',
            output='screen',
            arguments=[
                '-d', str(brne_share / 'rviz' / 'brne_sensor_scene1_demo.rviz')
            ],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_periodic_planner',
            name='brne_scene3_periodic_planner',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'goal_x': 2.8,
                'goal_y': 0.0,
                'goal_yaw': 0.0,
            }],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_shadow_input_adapter',
            name='brne_scene3_shadow_input_adapter',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_lidar_dynamic_agent_node',
            name='brne_scene3_lidar_dynamic_agent_node',
            output='screen',
            parameters=[str(runtime_profile), {'use_sim_time': True}],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_shadow_node',
            name='brne_scene3_shadow_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[str(runtime_profile), {'use_sim_time': True}],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_crossing_pedestrian_driver',
            name='brne_scene3_head_on_pedestrian_driver',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'motion_axis': 'x',
                # Backwards-compatible scalar endpoint parameter, interpreted
                # on motion_axis=x by the generalized driver.
                'target_world_y': -3.5,
                'world_y_direction': -1,
                'speed': 0.20,
                'maximum_duration_sec': 18.0,
            }],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_control_gate',
            name='brne_scene3_control_gate',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'armed': ParameterValue(arm_brne, value_type=bool),
            }],
        ),
    ])
