from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fault_injection_share = FindPackageShare('resilient_nav_fault_injection')
    default_scenario_file = PathJoinSubstitution([
        fault_injection_share,
        'config',
        'scenarios',
        'wheel_freeze_demo.yaml',
    ])
    scenario_file = LaunchConfiguration('scenario_file')

    wheel_fault_injector = Node(
        package='resilient_nav_fault_injection',
        executable='wheel_fault_injector',
        name='wheel_fault_injector',
        output='screen',
        parameters=[scenario_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_file',
            default_value=default_scenario_file,
            description='Path to a wheel odometry freeze scenario YAML file.',
        ),
        wheel_fault_injector,
    ])
