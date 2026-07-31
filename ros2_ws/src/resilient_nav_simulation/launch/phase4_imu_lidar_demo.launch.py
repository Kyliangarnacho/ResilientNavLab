"""Launch the stage 3 robot chain with stage 4 IMU and lidar bridges."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Reuse the stage 3 demo and add only the sensor bridge."""
    simulation_share = Path(
        get_package_share_directory('resilient_nav_simulation')
    )

    use_rviz = LaunchConfiguration('use_rviz')
    entity_name = LaunchConfiguration('entity_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    odom_ros_topic = LaunchConfiguration('odom_ros_topic')
    start_odom_tf_broadcaster = LaunchConfiguration(
        'start_odom_tf_broadcaster'
    )

    phase3_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / 'launch' / 'phase3_demo.launch.py')
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'entity_name': entity_name,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw,
            'odom_ros_topic': odom_ros_topic,
            'start_odom_tf_broadcaster': start_odom_tf_broadcaster,
        }.items(),
    )

    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        output='screen',
        parameters=[{
            'config_file': str(
                simulation_share
                / 'config'
                / 'phase4_sensor_bridge.yaml'
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start the existing stage 3 RViz display.',
        ),
        DeclareLaunchArgument(
            'entity_name',
            default_value='resilient_nav_robot',
            description='Gazebo entity name passed to the stage 3 demo.',
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
            description='Initial robot Z position in metres.',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='Initial robot yaw in radians.',
        ),
        DeclareLaunchArgument(
            'odom_ros_topic',
            default_value='/odom',
            description='ROS topic receiving raw Gazebo wheel odometry.',
        ),
        DeclareLaunchArgument(
            'start_odom_tf_broadcaster',
            default_value='true',
            description='Start the stage 3 odom to base TF broadcaster.',
        ),
        phase3_demo,
        sensor_bridge,
    ])
