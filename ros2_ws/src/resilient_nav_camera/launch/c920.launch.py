"""Launch the C920 baseline capture and its metadata-only probe."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the C920 capture chain and optionally its rectification branch."""
    config_file = get_package_share_directory('resilient_nav_camera')
    config_file += '/config/c920.yaml'
    camera_info_url = LaunchConfiguration('camera_info_url')
    enable_rectification = LaunchConfiguration('enable_rectification')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=(
                'package://resilient_nav_camera/config/c920_camera_info.yaml'),
            description='ROS camera calibration URL passed to usb_cam.',
        ),
        DeclareLaunchArgument(
            'enable_rectification',
            default_value='false',
            description='Start image_proc rectification and its metadata probe.',
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='/camera/c920',
            output='screen',
            parameters=[config_file, {'camera_info_url': camera_info_url}],
        ),
        Node(
            package='resilient_nav_camera',
            executable='c920_probe',
            name='c920_probe',
            output='screen',
        ),
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify',
            namespace='/camera/c920',
            output='screen',
            remappings=[('image', 'image_raw')],
            condition=IfCondition(enable_rectification),
        ),
        Node(
            package='resilient_nav_camera',
            executable='rectification_probe',
            name='rectification_probe',
            output='screen',
            condition=IfCondition(enable_rectification),
        ),
    ])
