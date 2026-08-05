"""Shared launch helpers for the health evaluator."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_health_evaluator(context, *args, **kwargs):
    """Create an evaluator after resolving its output path in the launch context."""
    del args, kwargs
    health_share = Path(
        get_package_share_directory('resilient_nav_health_assessment')
    )
    output_json_path = LaunchConfiguration('evaluator_output_json').perform(context)
    return [
        Node(
            package='resilient_nav_health_assessment',
            executable='health_evaluator',
            name='health_evaluator',
            output='screen',
            parameters=[
                str(health_share / 'config' / 'health_evaluator.yaml'),
                {
                    'use_sim_time': True,
                    'output_json_path': output_json_path,
                },
            ],
        )
    ]
