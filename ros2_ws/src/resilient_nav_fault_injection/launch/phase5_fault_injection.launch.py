"""Launch the complete phase 5 fault injection comparison chain."""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


BAG_TOPICS = [
    '/clock',
    '/imu/data',
    '/faulted/imu/data',
    '/wheel/odometry',
    '/faulted/wheel/odometry',
    '/scan',
    '/faulted/scan',
    '/fault_injection/status',
    '/odometry/filtered',
    '/odometry/faulted',
    '/tf',
    '/tf_static',
]


def generate_launch_description():
    """Start healthy phase 4, scenario injectors, faulted EKF, RViz and bag."""
    fault_share = Path(
        get_package_share_directory('resilient_nav_fault_injection')
    )
    localization_share = Path(
        get_package_share_directory('resilient_nav_localization')
    )

    scenario_file = LaunchConfiguration('scenario_file')
    use_rviz = LaunchConfiguration('use_rviz')
    record_bag = LaunchConfiguration('record_bag')
    bag_output = LaunchConfiguration('bag_output')

    phase4_ekf_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                localization_share
                / 'launch'
                / 'phase4_ekf_demo.launch.py'
            )
        ),
        launch_arguments={
            'use_sensor_rviz': 'false',
            'entity_name': LaunchConfiguration('entity_name'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
    )

    faulted_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='faulted_ekf_filter_node',
        output='screen',
        parameters=[str(fault_share / 'config' / 'faulted_ekf.yaml')],
        remappings=[('odometry/filtered', '/odometry/faulted')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='phase5_rviz',
        output='screen',
        arguments=[
            '-d',
            str(fault_share / 'rviz' / 'phase5_fault_injection.rviz'),
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    bag_record = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'resilient_nav_fault_injection',
            'phase5_record_bag',
            '--scenario-file',
            scenario_file,
            '--output-root',
            bag_output,
            '--',
            *BAG_TOPICS,
        ],
        output='screen',
        condition=IfCondition(record_bag),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_file',
            default_value=str(
                fault_share
                / 'config'
                / 'scenarios'
                / 'imu_bias_ekf_comparison.yaml'
            ),
            description='Phase 5 scenario YAML file.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start the phase 5 RViz comparison view.',
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Record the phase 5 reproducibility topic set.',
        ),
        DeclareLaunchArgument(
            'bag_output',
            default_value='phase5_bags',
            description='Bag output root; a unique child directory is created.',
        ),
        DeclareLaunchArgument(
            'entity_name',
            default_value='resilient_nav_robot',
            description='Gazebo entity name passed to the stage 4 chain.',
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
        OpaqueFunction(function=_make_fault_nodes),
        faulted_ekf,
        rviz,
        bag_record,
    ])


def _make_fault_nodes(context):
    scenario_path = Path(LaunchConfiguration('scenario_file').perform(context))
    with scenario_path.open('r', encoding='utf-8') as scenario_stream:
        scenario = yaml.safe_load(scenario_stream) or {}

    actions = []
    actions.append(_imu_node(scenario))
    actions.append(_wheel_node(scenario))
    actions.append(_scan_node(scenario))
    return actions


def _imu_node(scenario):
    if 'imu_bias_injector' in scenario:
        return Node(
            package='resilient_nav_fault_injection',
            executable='imu_bias_injector',
            name='imu_bias_injector',
            output='screen',
            parameters=[_ros_parameters(scenario['imu_bias_injector'])],
        )
    if 'imu_fault_injector' in scenario:
        return Node(
            package='resilient_nav_fault_injection',
            executable='imu_fault_injector',
            name='imu_fault_injector',
            output='screen',
            parameters=[_ros_parameters(scenario['imu_fault_injector'])],
        )
    return Node(
        package='resilient_nav_fault_injection',
        executable='imu_fault_injector',
        name='imu_fault_injector',
        output='screen',
        parameters=[_passthrough_parameters('imu_passthrough')],
    )


def _wheel_node(scenario):
    if 'wheel_fault_injector' in scenario:
        return Node(
            package='resilient_nav_fault_injection',
            executable='wheel_fault_injector',
            name='wheel_fault_injector',
            output='screen',
            parameters=[_ros_parameters(scenario['wheel_fault_injector'])],
        )
    return Node(
        package='resilient_nav_fault_injection',
        executable='wheel_fault_injector',
        name='wheel_fault_injector',
        output='screen',
        parameters=[_passthrough_parameters('wheel_passthrough')],
    )


def _scan_node(scenario):
    if 'scan_fault_injector' in scenario:
        return Node(
            package='resilient_nav_fault_injection',
            executable='scan_fault_injector',
            name='scan_fault_injector',
            output='screen',
            parameters=[_ros_parameters(scenario['scan_fault_injector'])],
        )
    return Node(
        package='resilient_nav_fault_injection',
        executable='scan_fault_injector',
        name='scan_fault_injector',
        output='screen',
        parameters=[_passthrough_parameters('scan_passthrough')],
    )


def _passthrough_parameters(scenario_id):
    event_id = f'{scenario_id}_{datetime.utcnow().strftime("%Y%m%d%H%M%S")}'
    base = {
        'use_sim_time': True,
        'status_topic': '/fault_injection/status',
        'enabled': False,
        'start_time_sec': 5.0,
        'end_time_sec': 15.0,
        'scenario_id': scenario_id,
        'scenario_seed': 20260803,
        'event_id': event_id,
    }
    if scenario_id.startswith('imu'):
        base.update({
            'input_topic': '/imu/data',
            'output_topic': '/faulted/imu/data',
            'model': 'bias',
            'bias_rad_s': 0.0,
        })
    elif scenario_id.startswith('wheel'):
        base.update({
            'input_topic': '/wheel/odometry',
            'output_topic': '/faulted/wheel/odometry',
            'model': 'freeze',
        })
    else:
        base.update({
            'input_topic': '/scan',
            'output_topic': '/faulted/scan',
            'model': 'sector_blindness',
            'sector_center_rad': 0.0,
            'sector_width_rad': 1.0,
        })
    return base


def _ros_parameters(node_yaml):
    return node_yaml.get('ros__parameters', node_yaml)
