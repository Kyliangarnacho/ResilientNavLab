"""Static contracts for the narrow Phase 10 Task 3.1 Planner integration."""

from __future__ import annotations

import ast
from pathlib import Path

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_planner_yaml_is_navfn_only_and_preserves_task2_costmap_contract():
    planner = yaml.safe_load((PACKAGE_ROOT / 'config' / 'nav2_planner.yaml').read_text())
    parameters = planner['planner_server']['ros__parameters']
    assert parameters['planner_plugins'] == ['GridBased']
    assert parameters['GridBased'] == {
        'plugin': 'nav2_navfn_planner::NavfnPlanner',
        'tolerance': 0.0,
        'use_astar': False,
        'allow_unknown': False,
    }
    costmaps = (PACKAGE_ROOT / 'config' / 'nav2_costmaps.yaml').read_text()
    assert 'global_frame: map' in costmaps
    assert 'robot_base_frame: base_footprint' in costmaps
    assert 'plugins: ["static_layer", "obstacle_layer", "inflation_layer"]' in costmaps


def test_smoke_scenarios_cover_reachable_detour_and_occupied_goal():
    scenarios = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'planner_smoke_scenarios.yaml').read_text()
    )['scenarios']
    assert set(scenarios) == {
        'simple_reachable',
        'static_obstacle_detour',
        'multi_turn_healthy',
        'occupied_goal',
    }
    assert scenarios['simple_reachable']['expected'] == 'success'
    assert scenarios['static_obstacle_detour']['require_direct_line_lethal'] is True
    assert scenarios['occupied_goal'] == {
        'goal': {'x': 5.05, 'y': 2.15, 'yaw': 0.0},
        'expected': 'failure',
        'expected_error_code': 'GOAL_OCCUPIED',
    }


def test_launch_is_scoped_planner_only_and_owns_no_second_costmap():
    source = (PACKAGE_ROOT / 'launch' / 'phase10_planner_smoke.launch.py').read_text()
    ast.parse(source)
    assert "package='nav2_planner'" in source
    assert "executable='planner_server'" in source
    assert "package='nav2_lifecycle_manager'" in source
    assert "node_names': ['planner_server']" in source
    assert 'GroupAction(actions=[localization], scoped=True, forwarding=True)' in source
    assert 'nav2_costmap_2d' not in source
    assert 'phase10_global_costmap_smoke' not in source
    assert 'phase10_costmaps_smoke' not in source
    assert 'RewrittenYaml(' in source
    assert "'global_obstacle_layer_enabled'" in source
    assert "'obstacle_layer.enabled'" in source
    assert "DeclareLaunchArgument('planner_scan_topic', default_value='/scan')" in source
    assert "DeclareLaunchArgument('planner_plan_topic', default_value='/plan')" in source
    assert "('/scan', planner_scan_topic)" in source
    assert "('/plan', planner_plan_topic)" in source
    for forbidden in ('controller_server', 'bt_navigator', 'NavigateToPose', 'cmd_vel'):
        assert forbidden not in source


def test_brne_override_disables_only_the_global_obstacle_layer():
    context = LaunchContext()
    context.launch_configurations['global_obstacle_layer_enabled'] = 'false'
    rewritten_path = RewrittenYaml(
        source_file=PACKAGE_ROOT / 'config' / 'nav2_costmaps.yaml',
        param_rewrites={
            (
                'global_costmap.global_costmap.ros__parameters.'
                'obstacle_layer.enabled'
            ): LaunchConfiguration('global_obstacle_layer_enabled'),
        },
        convert_types=True,
    ).perform(context)

    rewritten = yaml.safe_load(Path(rewritten_path).read_text())
    global_parameters = rewritten['global_costmap']['global_costmap'][
        'ros__parameters'
    ]
    local_parameters = rewritten['local_costmap']['local_costmap'][
        'ros__parameters'
    ]
    assert global_parameters['obstacle_layer']['enabled'] is False
    assert local_parameters['obstacle_layer']['enabled'] is True


def test_probe_checks_action_path_costs_and_cannot_command_motion():
    source = (PACKAGE_ROOT / 'planner_probe.py').read_text()
    ast.parse(source)
    for required in (
        'ComputePathToPose',
        'IsPathValid',
        'path_cost_evidence',
        'full_footprint_path_sweep',
        "'/global_costmap/costmap_raw'",
        'LETHAL_COST_THRESHOLD',
        "'/global_costmap/costmap'",
        "'/plan'",
    ):
        assert required in source
    for forbidden in ('create_publisher', 'cmd_vel', 'set_parameters', 'fault_injection'):
        assert forbidden not in source


def test_planner_rviz_only_visualizes_path_and_has_no_navigation_tool():
    source = (PACKAGE_ROOT / 'rviz' / 'phase10_planner.rviz').read_text()
    for topic in ('/map', '/scan', '/global_costmap/costmap', '/plan'):
        assert topic in source
    assert 'rviz_default_plugins/Interact' in source
    for forbidden in ('SetGoal', 'Navigation2 Goal', 'NavigateToPose'):
        assert forbidden not in source
