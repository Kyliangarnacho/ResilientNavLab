"""Spawn the stage 3 differential-drive robot in the phase 2 world."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the differential-drive robot spawn launch description."""
    simulation_share = Path(
        get_package_share_directory('resilient_nav_simulation')
    )
    description_share = Path(
        get_package_share_directory('resilient_nav_description')
    )

    entity_name = LaunchConfiguration('entity_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    cmd_vel_gz_topic = ['/model/', entity_name, '/cmd_vel']
    odometry_gz_topic = ['/model/', entity_name, '/odometry']
    joint_state_gz_topic = [
        '/world/resilient_lab/model/',
        entity_name,
        '/joint_state',
    ]

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            str(
                description_share
                / 'urdf'
                / 'resilient_nav_robot.urdf.xacro'
            ),
            ' gazebo_model_name:=',
            entity_name,
        ]),
        value_type=str,
    )

    phase2_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / 'launch' / 'phase2_world.launch.py')
        )
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
    )

    robot_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='robot_bridge',
        output='screen',
        arguments=[
            [
                *cmd_vel_gz_topic,
                '@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
            [
                *odometry_gz_topic,
                '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            [
                *joint_state_gz_topic,
                '@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
        ],
        remappings=[
            (cmd_vel_gz_topic, '/cmd_vel'),
            (odometry_gz_topic, '/odom'),
            (joint_state_gz_topic, '/joint_states'),
        ],
    )

    odom_tf_broadcaster = Node(
        package='resilient_nav_monitor',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_resilient_nav_robot',
        output='screen',
        parameters=[
            {
                'world': 'resilient_lab',
                'name': ParameterValue(entity_name, value_type=str),
                'allow_renaming': False,
                'topic': '/robot_description',
                'x': ParameterValue(spawn_x, value_type=float),
                'y': ParameterValue(spawn_y, value_type=float),
                'z': ParameterValue(spawn_z, value_type=float),
                'Y': ParameterValue(spawn_yaw, value_type=float),
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'entity_name',
            default_value='resilient_nav_robot',
            description='Gazebo entity name; duplicate names are rejected.',
        ),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='0.0',
            description='Initial robot X position in metres.',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='Initial robot Y position in metres.',
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.25',
            description=(
                'Initial robot Z position in metres for the drop test.'
            ),
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='Initial robot yaw in radians.',
        ),
        phase2_world,
        robot_state_publisher,
        robot_bridge,
        odom_tf_broadcaster,
        spawn_robot,
    ])
