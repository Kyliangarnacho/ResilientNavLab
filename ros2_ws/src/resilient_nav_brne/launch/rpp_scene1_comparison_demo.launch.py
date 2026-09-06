"""Run the frozen Phase 10 Navfn + BT + RPP chain in BRNE Scene 1."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Compose the RPP-only counterpart without any BRNE control owner."""
    brne_share = Path(get_package_share_directory('resilient_nav_brne'))
    navigation_share = Path(get_package_share_directory('resilient_nav_navigation'))
    simulation_share = Path(get_package_share_directory('resilient_nav_simulation'))
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'phase10_bt_navigation_smoke.launch.py')
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
            'use_rviz': use_rviz,
            'use_recovery': 'false',
            'log_level': log_level,
        }.items(),
    )
    pedestrian_model = brne_share / 'models' / 'brne_crossing_pedestrian.sdf'
    spawn_pedestrian = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_rpp_scene1_pedestrian',
        output='screen',
        parameters=[{
            'world': 'resilient_lab',
            'name': 'brne_pedestrian',
            'allow_renaming': False,
            'file': str(pedestrian_model),
            # Exactly the BRNE Scene 1 start: odom (0.5, -1.0).
            'x': -3.0,
            'y': -4.5,
            'z': 0.60,
            'Y': 1.5707963267948966,
        }],
    )
    pedestrian_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rpp_scene1_pedestrian_bridge',
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
                '/rpp/pedestrian/joint_velocity',
            ),
            ('/model/brne_pedestrian/odometry', '/rpp/pedestrian/odometry'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        GroupAction(actions=[navigation], scoped=True, forwarding=True),
        spawn_pedestrian,
        pedestrian_bridge,
        Node(
            package='resilient_nav_brne',
            executable='rpp_scene1_goal_coordinator',
            name='rpp_scene1_goal_coordinator',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='resilient_nav_brne',
            executable='brne_crossing_pedestrian_driver',
            name='rpp_scene1_pedestrian_driver',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'command_topic': '/rpp/pedestrian/joint_velocity',
                'odometry_topic': '/rpp/pedestrian/odometry',
                'plan_topic': '/plan',
                'ready_topic': '/rpp/ready',
                'speed': 0.25,
                'target_world_y': -2.50,
                'start_delay_sec': 0.1,
                'maximum_duration_sec': 10.0,
                'input_timeout_sec': 0.5,
                'publish_frequency_hz': 20.0,
            }],
        ),
    ])
