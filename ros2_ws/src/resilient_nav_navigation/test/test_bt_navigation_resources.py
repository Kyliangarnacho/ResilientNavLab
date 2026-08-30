"""Static contracts for the narrow Phase 10 Task 3.3 BT integration."""

from __future__ import annotations

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_bt_config_is_navigate_to_pose_only_and_uses_project_tf_contract():
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_bt_navigator.yaml').read_text()
    )['bt_navigator']['ros__parameters']
    assert parameters['global_frame'] == 'map'
    assert parameters['robot_base_frame'] == 'base_footprint'
    assert parameters['odom_topic'] == '/odometry/filtered'
    assert parameters['bt_loop_duration'] == 10
    assert parameters['navigators'] == ['navigate_to_pose']
    assert parameters['navigate_to_pose']['plugin'] == 'nav2_bt_navigator::NavigateToPoseNavigator'
    assert parameters['error_code_names'] == ['compute_path_error_code', 'follow_path_error_code']
    assert 'behavior_server' not in parameters


def test_scenarios_define_predeclared_bt_replanning_and_goal_contracts():
    scenarios = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'planner_smoke_scenarios.yaml').read_text()
    )['scenarios']
    for name in ('simple_reachable', 'static_obstacle_detour'):
        contract = scenarios[name]['navigate_to_pose']
        assert contract['action_timeout_sec'] > 0.0
        assert contract['min_plan_updates'] >= 2
        assert contract['max_final_xy_error_m'] > 0.0
        assert contract['max_final_yaw_error_rad'] > 0.0
    assert 'navigate_to_pose' not in scenarios['occupied_goal']


def test_bt_launch_preserves_default_chain_and_has_opt_in_official_recovery_profile():
    source = (PACKAGE_ROOT / 'launch' / 'phase10_bt_navigation_smoke.launch.py').read_text()
    ast.parse(source)
    for required in (
        "package='nav2_bt_navigator'",
        "executable='bt_navigator'",
        "'node_names': ['planner_server', 'controller_server', 'bt_navigator']",
        "'manage_planner': 'false'",
        "'manage_controller': 'false'",
        "'navigate_w_replanning_time.xml'",
        "'navigate_to_pose_w_replanning_and_recovery.xml'",
        "package='nav2_behaviors'",
        "executable='behavior_server'",
        "'planner_server', 'controller_server', 'behavior_server'",
        "DeclareLaunchArgument('use_recovery', default_value='false')",
        "'use_recovery_controller_profile': use_recovery",
        'lifecycle_manager_baseline,',
        'lifecycle_manager_recovery,',
        "GroupAction(actions=[controller_chain], scoped=True, forwarding=True)",
    ):
        assert required in source
    for forbidden in ('nav2_costmap_2d', 'TimerAction', 'navigation_start_delay_sec'):
        assert forbidden not in source.lower()


def test_task53_behavior_server_uses_existing_frames_topics_and_safe_recovery_plugins():
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_behaviors.yaml').read_text()
    )['behavior_server']['ros__parameters']
    assert parameters['behavior_plugins'] == ['spin', 'backup', 'wait']
    assert parameters['spin']['plugin'] == 'nav2_behaviors::Spin'
    assert parameters['backup']['plugin'] == 'nav2_behaviors::BackUp'
    assert parameters['wait']['plugin'] == 'nav2_behaviors::Wait'
    assert parameters['local_frame'] == 'odom'
    assert parameters['global_frame'] == 'map'
    assert parameters['robot_base_frame'] == 'base_footprint'
    assert parameters['max_rotational_vel'] == 0.60
    assert parameters['rotational_acc_lim'] == 1.50


def test_existing_wrappers_keep_standalone_manager_defaults_and_opt_outs():
    planner = (PACKAGE_ROOT / 'launch' / 'phase10_planner_smoke.launch.py').read_text()
    controller = (PACKAGE_ROOT / 'launch' / 'phase10_controller_smoke.launch.py').read_text()
    assert "DeclareLaunchArgument('manage_planner', default_value='true')" in planner
    assert 'condition=IfCondition(manage_planner)' in planner
    assert "DeclareLaunchArgument('manage_planner', default_value='true')" in controller
    assert "DeclareLaunchArgument('manage_controller', default_value='true')" in controller
    assert 'condition=IfCondition(manage_controller)' in controller


def test_probe_only_sends_navigate_to_pose_and_keeps_safety_observational():
    source = (PACKAGE_ROOT / 'navigate_to_pose_probe.py').read_text()
    ast.parse(source)
    for required in (
        'NavigateToPose',
        "'/navigate_to_pose'",
        'BehaviorTreeLog',
        'full_footprint_path_sweep',
        "'/behavior_tree_log'",
        "'/received_global_plan'",
        "'/global_costmap/costmap_raw'",
        'publish_zero_stop',
        "request.behavior_tree = ''",
        'feedback.current_pose',
        'associated_feedback',
        'clock_contract',
        "Clock, '/clock'",
        'Buffer(cache_time=Duration(seconds=60.0))',
    ):
        assert required in source
    for forbidden in (
        'from nav2_msgs.action import ComputePathToPose',
        'from nav2_msgs.action import FollowPath',
        'ActionClient(self, ComputePathToPose',
        'ActionClient(self, FollowPath',
        'set_parameters',
        'fault_injection',
    ):
        assert forbidden not in source
    assert "choices=['simple_reachable', 'static_obstacle_detour']" in source
    # Runtime safety sweeps deliberately use the action's current_pose rather
    # than a short-lived late TF buffer. TF remains a readiness/final check.
    assert 'full_footprint_path_sweep(' in source
    assert 'final TF/feedback cross-check diverged' in source


def test_rviz_has_goal_tool_and_no_recovery_or_navigation_panel():
    source = (PACKAGE_ROOT / 'rviz' / 'phase10_bt_navigation.rviz').read_text()
    for topic in ('/map', '/scan', '/global_costmap/costmap', '/local_costmap/costmap', '/plan', '/received_global_plan'):
        assert topic in source
    assert 'nav2_rviz_plugins/GoalTool' in source
    for forbidden in ('Navigation 2', 'Recovery', 'Docking'):
        assert forbidden not in source
