"""Launch the C920 capture and phase 7.2 camera health observation chain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Compose the unchanged C920 launch with optional health observers."""
    camera_share = Path(get_package_share_directory('resilient_nav_camera'))
    health_share = Path(
        get_package_share_directory('resilient_nav_health_assessment')
    )
    enable_rectification = LaunchConfiguration('enable_rectification')
    health_config = LaunchConfiguration('health_config')
    source_topic = LaunchConfiguration('source_topic')
    run_camera = LaunchConfiguration('run_camera')
    run_evaluator = LaunchConfiguration('run_evaluator')
    evaluator_output_json = LaunchConfiguration('evaluator_output_json')
    run_watch = LaunchConfiguration('run_watch')
    run_image_view = LaunchConfiguration('run_image_view')
    use_sim_time = LaunchConfiguration('use_sim_time')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(camera_share / 'launch' / 'c920.launch.py')
        ),
        launch_arguments={
            'enable_rectification': enable_rectification,
        }.items(),
        condition=IfCondition(run_camera),
    )
    monitor = Node(
        package='resilient_nav_health_assessment',
        executable='camera_health_monitor',
        name='camera_health_monitor',
        output='screen',
        parameters=[
            health_config,
            {
                'image_topic': source_topic,
                'health_topic': '/health/camera',
                'use_sim_time': use_sim_time,
            },
        ],
    )
    evaluator = Node(
        package='resilient_nav_health_assessment',
        executable='health_evaluator',
        name='health_evaluator',
        output='screen',
        parameters=[
            str(health_share / 'config' / 'health_evaluator.yaml'),
            {
                'camera_health_topic': '/health/camera',
                'output_json_path': evaluator_output_json,
                'use_sim_time': use_sim_time,
            },
        ],
        condition=IfCondition(run_evaluator),
    )
    watch = Node(
        package='resilient_nav_health_assessment',
        executable='camera_health_watch',
        name='camera_health_watch',
        output='screen',
        parameters=[
            {
                'health_topic': '/health/camera',
                'use_sim_time': use_sim_time,
            },
        ],
        condition=IfCondition(run_watch),
    )
    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_image_view',
        output='screen',
        arguments=[source_topic],
        remappings=[('image', source_topic)],
        condition=IfCondition(run_image_view),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_rectification',
            default_value='false',
            description='Forwarded unchanged to the existing C920 launch.',
        ),
        DeclareLaunchArgument(
            'health_config',
            default_value=str(health_share / 'config' / 'camera_health.yaml'),
            description='Camera health monitor parameter YAML.',
        ),
        DeclareLaunchArgument(
            'source_topic',
            default_value='/camera/c920/image_raw',
            description='Image topic observed by camera_health_monitor.',
        ),
        DeclareLaunchArgument(
            'run_camera',
            default_value='true',
            description='Include the existing C920 launch; disable for replay/tests.',
        ),
        DeclareLaunchArgument(
            'run_evaluator',
            default_value='false',
            description='Optionally evaluate /health/camera against FaultStatus.',
        ),
        DeclareLaunchArgument(
            'evaluator_output_json',
            default_value='/tmp/phase7_2_camera_health_evaluation.json',
            description='Optional camera evaluator JSON output path.',
        ),
        DeclareLaunchArgument(
            'run_watch',
            default_value='false',
            description='Optionally print camera SensorHealth state changes.',
        ),
        DeclareLaunchArgument(
            'run_image_view',
            default_value='false',
            description='Optionally view source_topic with rqt_image_view.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time for launched health nodes.',
        ),
        camera,
        monitor,
        evaluator,
        watch,
        image_view,
    ])
