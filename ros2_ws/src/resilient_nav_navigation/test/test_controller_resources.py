"""Static contracts for the deliberately narrow Task 3.2 controller smoke."""

from __future__ import annotations

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_controller_uses_rpp_and_explicit_gazebo_twist_contract():
    parameters = yaml.safe_load((PACKAGE_ROOT / 'config' / 'nav2_controller.yaml').read_text())['controller_server']['ros__parameters']
    assert parameters['controller_plugins'] == ['FollowPath']
    assert parameters['FollowPath']['plugin'] == 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    assert parameters['odom_topic'] == '/odometry/filtered'
    assert parameters['enable_stamped_cmd_vel'] is False
    assert parameters['publish_zero_velocity'] is True
    assert parameters['FollowPath']['use_collision_detection'] is True
    assert parameters['FollowPath']['allow_reversing'] is False
    assert parameters['goal_checker']['plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert parameters['progress_checker']['plugin'] == 'nav2_controller::PoseProgressChecker'


def test_scenarios_allow_only_two_fixed_healthy_controller_runs():
    scenarios = yaml.safe_load((PACKAGE_ROOT / 'config' / 'planner_smoke_scenarios.yaml').read_text())['scenarios']
    for name in ('simple_reachable', 'static_obstacle_detour'):
        controller = scenarios[name]['controller']
        assert controller['action_timeout_sec'] > 0.0
        assert controller['max_cross_track_error_m'] > 0.0
    assert 'controller' not in scenarios['occupied_goal']


def test_controller_launch_reuses_planner_and_task2_costmaps_without_bt():
    source = (PACKAGE_ROOT / 'launch' / 'phase10_controller_smoke.launch.py').read_text()
    ast.parse(source)
    for required in (
        "package='nav2_controller'",
        "executable='controller_server'",
        "'node_names': ['controller_server']",
        "phase10_planner_smoke.launch.py",
        "nav2_costmaps.yaml",
        "nav2_controller.yaml",
        "DeclareLaunchArgument(\n            'use_recovery_controller_profile', default_value='false'",
        "'FollowPath.max_robot_pose_search_dist': ParameterValue(",
        "if_value='-1.0'",
        "else_value='3.0'",
        'GroupAction(actions=[planner_chain], scoped=True, forwarding=True)',
    ):
        assert required in source
    for forbidden in ("package='nav2_costmap_2d'", "package='nav2_bt_navigator'", "package='nav2_behaviors'", "package='nav2_velocity_smoother'"):
        assert forbidden not in source


def test_recovery_controller_search_override_is_opt_in_and_upstream_native():
    """Task 5.3 may search the full Path without changing the baseline YAML."""
    source = (PACKAGE_ROOT / 'launch' / 'phase10_controller_smoke.launch.py').read_text()
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_controller.yaml').read_text()
    )['controller_server']['ros__parameters']['FollowPath']

    assert 'max_robot_pose_search_dist' not in parameters
    assert "use_recovery_controller_profile = LaunchConfiguration(" in source
    assert 'IfElseSubstitution(' in source
    assert 'value_type=float' in source


def test_controller_probe_has_bounded_actions_and_zero_only_probe_command():
    source = (PACKAGE_ROOT / 'controller_probe.py').read_text()
    ast.parse(source)
    for required in (
        'ComputePathToPose',
        'FollowPath',
        'full_footprint_path_sweep',
        "'/global_costmap/costmap_raw'",
        "'/local_costmap/costmap_raw'",
        "'/lookahead_collision_arc'",
        'MAX_LINEAR_COMMAND_MPS',
        'MAX_ANGULAR_COMMAND_RADPS',
        'publish_zero_stop',
        'Twist()',
        "'/local_costmap/costmap_raw', self._on_local_raw, MAP_QOS",
    ):
        assert required in source
    for forbidden in ('NavigateToPose', 'set_parameters', 'fault_injection', 'motion_safety'):
        assert forbidden not in source
    assert "choices=['simple_reachable', 'static_obstacle_detour']" in source


def test_controller_rviz_is_observational_and_has_no_goal_tool():
    source = (PACKAGE_ROOT / 'rviz' / 'phase10_controller.rviz').read_text()
    for topic in ('/map', '/scan', '/global_costmap/costmap', '/local_costmap/costmap', '/plan', '/received_global_plan', '/lookahead_collision_arc'):
        assert topic in source
    assert 'rviz_default_plugins/Interact' in source
    for forbidden in ('SetGoal', 'Navigation2 Goal', 'NavigateToPose'):
        assert forbidden not in source
