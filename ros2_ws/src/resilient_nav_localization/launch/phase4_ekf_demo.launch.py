"""Launch the complete stage 4 multisensor and EKF demonstration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Reuse the RGB-D chain and give odom TF ownership to the EKF."""
    localization_share = Path(
        get_package_share_directory('resilient_nav_localization')
    )
    simulation_share = Path(
        get_package_share_directory('resilient_nav_simulation')
    )

    use_sensor_rviz = LaunchConfiguration('use_sensor_rviz')
    entity_name = LaunchConfiguration('entity_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    rgbd_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                simulation_share
                / 'launch'
                / 'phase4_rgbd_demo.launch.py'
            )
        ),
        launch_arguments={
            'use_sensor_rviz': use_sensor_rviz,
            'entity_name': entity_name,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw,
            'odom_ros_topic': '/wheel/odometry/raw',
            'start_odom_tf_broadcaster': 'false',
        }.items(),
    )

    wheel_odometry_uncertainty = Node(
        package='resilient_nav_localization',
        executable='wheel_odometry_uncertainty',
        name='wheel_odometry_uncertainty',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[str(localization_share / 'config' / 'ekf.yaml')],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sensor_rviz',
            default_value='true',
            description='Start RViz with the stage 4 sensor configuration.',
        ),
        DeclareLaunchArgument(
            'entity_name',
            default_value='resilient_nav_robot',
            description='Gazebo entity name passed to the sensor demo.',
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
        rgbd_demo,
        wheel_odometry_uncertainty,
        ekf,
    ])
