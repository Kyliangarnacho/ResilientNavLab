"""Launch the Jazzy Slam Toolbox localization node with the M8 pose graph."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _as_bool(value, name):
    """Parse a ROS launch boolean without silently accepting a typo."""
    if value.lower() in ('true', '1'):
        return True
    if value.lower() in ('false', '0'):
        return False
    raise RuntimeError(f'{name} must be true or false, got {value!r}')


def _localization_actions(context):
    """Create the lifecycle node after resolving an optional start-pose guess."""
    slam_share = Path(get_package_share_directory('resilient_nav_slam'))
    posegraph_base = (
        slam_share / 'maps' / 'phase9' / 'posegraph' / 'phase9_posegraph'
    )
    use_start_pose = _as_bool(
        LaunchConfiguration('use_start_pose').perform(context),
        'use_start_pose',
    )

    startup_parameters = {
        'use_sim_time': True,
        'map_file_name': str(posegraph_base),
    }
    if use_start_pose:
        try:
            start_pose = [
                float(LaunchConfiguration('initial_pose_x').perform(context)),
                float(LaunchConfiguration('initial_pose_y').perform(context)),
                float(LaunchConfiguration('initial_pose_yaw').perform(context)),
            ]
            if not all(math.isfinite(value) for value in start_pose):
                raise ValueError('non-finite value')
            startup_parameters.update({
                'map_start_at_dock': False,
                'map_start_pose': start_pose,
            })
        except ValueError as error:
            raise RuntimeError(
                'initial_pose_x, initial_pose_y, and initial_pose_yaw '
                'must be finite numeric launch arguments'
            ) from error

    localization = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            str(slam_share / 'config' / 'mapper_params_localization.yaml'),
            startup_parameters,
        ],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(localization),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=localization,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] Slam Toolbox localization is activating.'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(localization),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
    )

    return [localization, configure, activate]


def generate_launch_description():
    """Load M8's pose graph, optionally beginning at a supplied map-frame pose."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_start_pose',
            default_value='false',
            description=(
                'Use the explicit map_start_pose below before the first scan; '
                'false preserves the M10 dock-start baseline.'
            ),
        ),
        DeclareLaunchArgument(
            'initial_pose_x', default_value='-3.5',
            description='Map-frame X coordinate for an explicit start pose.',
        ),
        DeclareLaunchArgument(
            'initial_pose_y', default_value='-3.5',
            description='Map-frame Y coordinate for an explicit start pose.',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw', default_value='0.0',
            description='Map-frame yaw in radians for an explicit start pose.',
        ),
        OpaqueFunction(function=_localization_actions),
    ])
