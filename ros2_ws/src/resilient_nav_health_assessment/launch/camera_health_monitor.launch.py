"""Launch the standalone development camera health monitor."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start camera health with configurable topics and time source."""
    package_share = Path(
        get_package_share_directory('resilient_nav_health_assessment')
    )
    image_topic = LaunchConfiguration('image_topic')
    health_topic = LaunchConfiguration('health_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/c920/image_raw',
            description='sensor_msgs/Image input assessed by the monitor.',
        ),
        DeclareLaunchArgument(
            'health_topic',
            default_value='/health/camera',
            description='SensorHealth output topic.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use ROS simulation time instead of the system clock.',
        ),
        Node(
            package='resilient_nav_health_assessment',
            executable='camera_health_monitor',
            name='camera_health_monitor',
            output='screen',
            parameters=[
                str(package_share / 'config' / 'camera_health.yaml'),
                {
                    'image_topic': image_topic,
                    'health_topic': health_topic,
                    'use_sim_time': ParameterValue(
                        use_sim_time, value_type=bool
                    ),
                },
            ],
        ),
    ])
