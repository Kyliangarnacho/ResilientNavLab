from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fault_injection_share = FindPackageShare('resilient_nav_fault_injection')
    default_scenario_file = PathJoinSubstitution([
        fault_injection_share,
        'config',
        'scenarios',
        'imu_delay_demo.yaml',
    ])
    scenario_file = LaunchConfiguration('scenario_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    imu_fault_injector = Node(
        package='resilient_nav_fault_injection',
        executable='imu_fault_injector',
        name='imu_fault_injector',
        output='screen',
        parameters=[
            scenario_file,
            {
                'use_sim_time': ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_file',
            default_value=default_scenario_file,
            description='Path to an IMU fixed-delay scenario YAML file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use ROS simulation time for the IMU delay injector.',
        ),
        imu_fault_injector,
    ])
