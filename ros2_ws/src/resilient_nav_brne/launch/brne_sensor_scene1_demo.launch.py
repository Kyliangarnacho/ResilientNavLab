"""Run Scene 1 BRNE with pedestrian state derived only from robot LiDAR."""

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
    """Compose the sensor-input BRNE owner without a GT state adapter."""
    brne_share = Path(get_package_share_directory('resilient_nav_brne'))
    navigation_share = Path(
        get_package_share_directory('resilient_nav_navigation')
    )
    simulation_share = Path(
        get_package_share_directory('resilient_nav_simulation')
    )
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
        name='spawn_brne_sensor_scene1_pedestrian',
        output='screen',
        parameters=[{
            'world': 'resilient_lab',
            'name': 'brne_pedestrian',
            'allow_renaming': False,
            'file': str(pedestrian_model),
            # Robot world (-3.5, -3.5) is odom (0, 0). This is odom
            # (0.8, -1.0), preserving all other interaction settings.
            'x': -2.7,
            'y': -4.5,
            'z': 0.60,
            'Y': 1.5707963267948966,
        }],
    )
    pedestrian_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='brne_sensor_scene1_pedestrian_bridge',
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
                '/scenario/brne_pedestrian/odometry',
            ),
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='brne_sensor_scene1_demo_rviz',
        output='screen',
        arguments=['-d', str(brne_share / 'rviz' / 'brne_sensor_scene1_demo.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('arm_brne', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        GroupAction(actions=[planner_only], scoped=True, forwarding=True),
        spawn_pedestrian,
        pedestrian_bridge,
        rviz,
        Node(
            package='resilient_nav_brne',
            executable='brne_periodic_planner',
            name='brne_sensor_scene1_periodic_planner',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'goal_x': 2.0,
                'goal_y': 0.0,
                'goal_yaw': 0.0,
            }],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_shadow_input_adapter',
            name='brne_sensor_scene1_shadow_input_adapter',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_lidar_dynamic_agent_node',
            name='brne_lidar_dynamic_agent_node',
            output='screen',
            parameters=[str(runtime_profile), {'use_sim_time': True}],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_shadow_node',
            name='brne_sensor_scene1_shadow_node',
            output='screen',
            parameters=[str(runtime_profile), {'use_sim_time': True}],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_crossing_pedestrian_driver',
            name='brne_sensor_scene1_pedestrian_driver',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odometry_topic': '/scenario/brne_pedestrian/odometry',
                # Three metres at 0.25 m/s gives a 12 second crossing.
                'target_world_y': -1.5,
            }],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_control_gate',
            name='brne_sensor_scene1_control_gate',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'armed': ParameterValue(arm_brne, value_type=bool),
            }],
        ),
    ])
