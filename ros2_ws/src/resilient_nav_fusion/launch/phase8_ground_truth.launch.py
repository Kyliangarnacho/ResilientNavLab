"""Launch only the evaluation-only Gazebo ground-truth pose channel."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Bridge independent Gazebo world pose into evaluator-only ROS data."""
    gazebo_tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='phase8_ground_truth_tf_bridge',
        output='screen',
        arguments=[
            '/model/resilient_nav_robot/pose'
            '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            (
                '/model/resilient_nav_robot/pose',
                '/evaluation/gazebo_world_model_tf',
            )
        ],
    )
    adapter = Node(
        package='resilient_nav_fusion',
        executable='ground_truth_pose_adapter',
        name='ground_truth_pose_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([gazebo_tf_bridge, adapter])
