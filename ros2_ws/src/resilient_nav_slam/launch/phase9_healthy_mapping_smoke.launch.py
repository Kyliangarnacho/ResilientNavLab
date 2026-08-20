"""Launch the minimal static Phase 9 healthy mapping smoke chain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Compose existing healthy inputs with their two frozen TF owners."""
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
    slam_params_file = LaunchConfiguration('slam_params_file')

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

    # The Phase 4 include also declares ``use_rviz``.  Scope its explicit
    # ``false`` value so it cannot disable this launch's Phase 9 RViz node.
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

    online_async_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(slam_share / 'launch' / 'phase9_online_async_mapping.launch.py'),
        ),
        launch_arguments={'slam_params_file': slam_params_file}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='phase9_mapping_rviz',
        arguments=['-d', str(slam_share / 'rviz' / 'phase9_mapping.rviz')],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start only the Phase 9 mapping RViz configuration.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=str(
                slam_share / 'config' / 'mapper_params_online_async.yaml'
            ),
            description='Mapping parameter file; defaults to the frozen baseline.',
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
            description='Phase 9 mapping-route initial X position in metres.',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='-3.5',
            description='Phase 9 mapping-route initial Y position in metres.',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='Phase 9 mapping-route initial yaw in radians.',
        ),
        healthy_simulation_inputs,
        healthy_ekf,
        online_async_mapping,
        rviz,
    ])
