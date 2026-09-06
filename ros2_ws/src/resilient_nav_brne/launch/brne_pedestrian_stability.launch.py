"""Launch only the Scene 1 world and constrained crossing-pedestrian verifier."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Keep the 60 s actor check independent from robot, Nav2, and BRNE control."""
    brne_share = Path(get_package_share_directory('resilient_nav_brne'))
    simulation_share = Path(get_package_share_directory('resilient_nav_simulation'))
    ros_gz_sim_share = Path(get_package_share_directory('ros_gz_sim'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_sim_share / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s ', str(simulation_share / 'worlds' / 'phase9_slam_world.sdf')],
            'on_exit_shutdown': 'true',
        }.items(),
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='brne_pedestrian_stability_clock_bridge',
        output='screen',
        parameters=[{'config_file': str(simulation_share / 'config' / 'bridge.yaml')}],
    )
    spawn_pedestrian = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_brne_pedestrian_stability',
        output='screen',
        parameters=[{
            'world': 'resilient_lab',
            'name': 'brne_pedestrian',
            'allow_renaming': False,
            'file': str(brne_share / 'models' / 'brne_crossing_pedestrian.sdf'),
            'x': -3.0,
            'y': -4.5,
            'z': 0.60,
            'Y': 1.5707963267948966,
        }],
    )
    pedestrian_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='brne_pedestrian_stability_bridge',
        output='screen',
        arguments=[
            (
                '/model/brne_pedestrian/joint/crossing_joint/cmd_vel'
                '@std_msgs/msg/Float64]gz.msgs.Double'
            ),
            '/model/brne_pedestrian/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            (
                '/model/brne_pedestrian/joint/crossing_joint/cmd_vel',
                '/brne/pedestrian/joint_velocity',
            ),
            ('/model/brne_pedestrian/odometry', '/brne/pedestrian/odometry'),
        ],
    )
    probe = Node(
        package='resilient_nav_brne',
        executable='brne_pedestrian_stability_probe',
        name='brne_pedestrian_stability_probe',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'observation_duration_sec': 60.0,
            'result_path': '/tmp/brne_pedestrian_stability_result.json',
        }],
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        spawn_pedestrian,
        pedestrian_bridge,
        probe,
        RegisterEventHandler(
            OnProcessExit(
                target_action=probe,
                on_exit=[EmitEvent(event=Shutdown(reason='pedestrian stability probe complete'))],
            )
        ),
    ])
