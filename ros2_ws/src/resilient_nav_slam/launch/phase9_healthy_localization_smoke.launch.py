"""Launch the static Phase 9 healthy localization smoke chain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Compose healthy simulation inputs with the frozen localization TF owners."""
    localization_share = Path(
        get_package_share_directory('resilient_nav_localization')
    )
    simulation_share = Path(
        get_package_share_directory('resilient_nav_simulation')
    )
    slam_share = Path(get_package_share_directory('resilient_nav_slam'))

    use_rviz = LaunchConfiguration('use_rviz')
    world_path = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    use_start_pose = LaunchConfiguration('use_start_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    imu_lidar_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / 'launch' / 'phase4_imu_lidar_demo.launch.py'),
        ),
        launch_arguments={
            'use_rviz': 'false',
            'world': world_path,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_yaw': spawn_yaw,
            'odom_ros_topic': '/wheel/odometry',
            'start_odom_tf_broadcaster': 'false',
        }.items(),
    )

    healthy_simulation_inputs = GroupAction(
        actions=[imu_lidar_demo],
        scoped=True,
        forwarding=True,
    )

    healthy_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[str(localization_share / 'config' / 'ekf.yaml')],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(slam_share / 'launch' / 'phase9_localization.launch.py'),
        ),
        launch_arguments={
            'use_start_pose': use_start_pose,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='phase9_localization_rviz',
        arguments=['-d', str(slam_share / 'rviz' / 'phase9_mapping.rviz')],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start the existing Phase 9 SLAM RViz configuration.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=str(
                simulation_share / 'worlds' / 'phase9_slam_world.sdf'
            ),
            description='Phase 9 SDF world; override to retain another world.',
        ),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='-3.5',
            description='M8 mapping-route initial X position in metres.',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='-3.5',
            description='M8 mapping-route initial Y position in metres.',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='M8 mapping-route initial yaw in radians.',
        ),
        DeclareLaunchArgument(
            'use_start_pose',
            default_value='false',
            description=(
                'Pass a map-frame startup pose to Slam Toolbox before the '
                'first scan; false preserves the M10 dock-start baseline.'
            ),
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='-3.5',
            description='Map-frame X coordinate when use_start_pose is true.',
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='-3.5',
            description='Map-frame Y coordinate when use_start_pose is true.',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Map-frame yaw in radians when use_start_pose is true.',
        ),
        healthy_simulation_inputs,
        healthy_ekf,
        localization,
        rviz,
    ])
