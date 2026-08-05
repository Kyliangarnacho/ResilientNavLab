"""Launch only the health evaluator, without simulation or visualization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from resilient_nav_health_assessment.evaluator_launch import create_health_evaluator


def generate_launch_description():
    """Start a health evaluator with a runtime-resolved JSON output path."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'evaluator_output_json',
            default_value='/tmp/phase6_health_evaluation.json',
            description='Path written by health_evaluator when the launch stops.',
        ),
        OpaqueFunction(function=create_health_evaluator),
    ])
