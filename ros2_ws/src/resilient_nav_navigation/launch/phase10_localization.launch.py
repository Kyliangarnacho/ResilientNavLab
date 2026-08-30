"""Launch the official Nav2 localization stack with Phase 9's saved map."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Delegate Map Server, AMCL, and lifecycle control to Nav2 Jazzy."""
    navigation_share = Path(
        get_package_share_directory('resilient_nav_navigation')
    )
    nav2_bringup_share = Path(get_package_share_directory('nav2_bringup'))
    slam_share = Path(get_package_share_directory('resilient_nav_slam'))

    namespace = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_bringup_share / 'launch' / 'localization_launch.py'),
        ),
        launch_arguments={
            'namespace': namespace,
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': use_composition,
            'container_name': container_name,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='phase10_localization_rviz',
        arguments=['-d', str(navigation_share / 'rviz' / 'phase10_localization.rviz')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Forwarded to Nav2; the Phase 10 baseline uses no namespace.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=str(
                slam_share / 'maps' / 'phase9' / 'occupancy' / 'phase9_map.yaml'
            ),
            description=(
                'Phase 9 saved occupancy-map YAML. Any override is outside '
                'the frozen Phase 10 baseline.'
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(navigation_share / 'config' / 'nav2_localization.yaml'),
            description='Phase 10 localization-only Nav2 parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the existing Gazebo-to-ROS /clock bridge.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Let Nav2 lifecycle-manage Map Server then AMCL.',
        ),
        DeclareLaunchArgument(
            'use_composition',
            # Nav2 Jazzy's upstream launch evaluates this in PythonExpression.
            default_value='False',
            description='Keep separate nodes for the initial debugging baseline.',
        ),
        DeclareLaunchArgument(
            'container_name',
            default_value='nav2_container',
            description='Nav2 component container name when composition is enabled.',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='Do not hide a failed localization node by respawning it.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Forwarded Nav2 log level.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start the Phase 10 localization-only RViz view.',
        ),
        localization,
        rviz,
    ])
