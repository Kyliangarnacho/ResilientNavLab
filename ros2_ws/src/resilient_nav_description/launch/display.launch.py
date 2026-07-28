"""Display the stage 3 robot description and TF tree in RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the robot description display launch."""
    use_gui = LaunchConfiguration('use_gui')

    description_share = FindPackageShare('resilient_nav_description')
    xacro_file = PathJoinSubstitution(
        [description_share, 'urdf', 'resilient_nav_robot.urdf.xacro']
    )
    rviz_config = PathJoinSubstitution(
        [description_share, 'rviz', 'display.rviz']
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='false',
            description=(
                'Use joint_state_publisher_gui instead of '
                'joint_state_publisher.'
            ),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(use_gui),
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
