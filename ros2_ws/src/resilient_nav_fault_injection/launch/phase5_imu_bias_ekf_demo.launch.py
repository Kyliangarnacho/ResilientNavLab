from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fault_injection_share = Path(
        get_package_share_directory('resilient_nav_fault_injection')
    )
    localization_share = Path(
        get_package_share_directory('resilient_nav_localization')
    )

    use_rviz = LaunchConfiguration('use_rviz')
    entity_name = LaunchConfiguration('entity_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    phase4_ekf_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                localization_share
                / 'launch'
                / 'phase4_ekf_demo.launch.py'
            )
        ),
        launch_arguments={
            'use_sensor_rviz': use_rviz,
            'entity_name': entity_name,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw,
        }.items(),
    )

    imu_bias_injector = Node(
        package='resilient_nav_fault_injection',
        executable='imu_bias_injector',
        name='imu_bias_injector',
        output='screen',
        parameters=[
            str(
                fault_injection_share
                / 'config'
                / 'scenarios'
                / 'imu_bias_demo.yaml'
            ),
        ],
    )

    wheel_fault_injector = Node(
        package='resilient_nav_fault_injection',
        executable='wheel_fault_injector',
        name='wheel_fault_injector',
        output='screen',
        parameters=[
            str(
                fault_injection_share
                / 'config'
                / 'scenarios'
                / 'wheel_freeze_demo.yaml'
            ),
            {
                'enabled': False,
                'input_topic': '/wheel/odometry',
                'output_topic': '/faulted/wheel/odometry',
                'model': 'freeze',
                'scenario_id': 'wheel_passthrough_for_faulted_ekf',
                'event_id': 'wheel_passthrough_001',
            },
        ],
    )

    faulted_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='faulted_ekf_filter_node',
        output='screen',
        parameters=[
            str(fault_injection_share / 'config' / 'faulted_ekf.yaml'),
        ],
        remappings=[('odometry/filtered', '/odometry/faulted')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Forwarded to the stage 4 sensor RViz argument.',
        ),
        DeclareLaunchArgument(
            'entity_name',
            default_value='resilient_nav_robot',
            description='Gazebo entity name passed to the stage 4 EKF demo.',
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
        phase4_ekf_demo,
        imu_bias_injector,
        wheel_fault_injector,
        faulted_ekf,
    ])
