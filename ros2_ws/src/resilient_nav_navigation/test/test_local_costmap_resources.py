"""Static and pure-Python contracts for Phase 10 Task 2.3–2.4."""

import ast
import sys
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from costmap_contract import LETHAL_COST_THRESHOLD, cost_distribution
from costmap_experiment_evaluator import evaluate
from local_costmap_probe import rolling_summary


CONFIG_FILE = PACKAGE_ROOT / 'config' / 'nav2_costmaps.yaml'
PROFILE_FILE = PACKAGE_ROOT / 'config' / 'costmap_experiment_profiles.yaml'
LOCAL_LAUNCH = PACKAGE_ROOT / 'launch' / 'phase10_local_costmap_smoke.launch.py'
JOINT_LAUNCH = PACKAGE_ROOT / 'launch' / 'phase10_costmaps_smoke.launch.py'
LOCAL_RVIZ = PACKAGE_ROOT / 'rviz' / 'phase10_local_costmap.rviz'
JOINT_RVIZ = PACKAGE_ROOT / 'rviz' / 'phase10_costmaps.rviz'
LOCAL_PROBE = PACKAGE_ROOT / 'local_costmap_probe.py'
JOINT_PROBE = PACKAGE_ROOT / 'costmap_joint_probe.py'
EVALUATOR = PACKAGE_ROOT / 'costmap_experiment_evaluator.py'


def local_parameters():
    with CONFIG_FILE.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)['local_costmap']['local_costmap']['ros__parameters']


def test_local_costmap_is_odom_rolling_scan_only_contract():
    config = local_parameters()
    assert config['global_frame'] == 'odom'
    assert config['robot_base_frame'] == 'base_footprint'
    assert config['rolling_window'] is True
    assert config['width'] == 6 and config['height'] == 6
    assert config['resolution'] == 0.05
    assert config['track_unknown_space'] is False
    assert config['update_frequency'] == 5.0
    assert config['publish_frequency'] == 2.0
    assert config['plugins'] == ['obstacle_layer', 'inflation_layer']
    assert 'static_layer' not in config
    assert config['obstacle_layer']['plugin'] == 'nav2_costmap_2d::ObstacleLayer'
    assert config['obstacle_layer']['scan']['topic'] == '/scan'
    assert config['obstacle_layer']['scan']['marking'] is True
    assert config['obstacle_layer']['scan']['clearing'] is True
    assert config['obstacle_layer']['scan']['obstacle_max_range'] == 2.5
    assert config['obstacle_layer']['scan']['raytrace_max_range'] == 3.0
    assert 'observation_persistence' not in config['obstacle_layer']['scan']
    assert config['obstacle_layer']['scan']['expected_update_rate'] == 0.2
    assert config['obstacle_layer']['scan']['sensor_frame'] == 'lidar_link'
    assert config['inflation_layer'] == {
        'plugin': 'nav2_costmap_2d::InflationLayer',
        'inflation_radius': 0.55,
        'cost_scaling_factor': 3.0,
    }
    assert config['always_send_full_costmap'] is True


def test_local_and_joint_launches_scope_child_rviz_and_exclude_navigation_servers():
    for path, required in (
        (LOCAL_LAUNCH, ("'phase10_localization_smoke.launch.py'", "namespace='local_costmap'")),
        (JOINT_LAUNCH, ("'phase10_global_costmap_smoke.launch.py'", "namespace='local_costmap'")),
    ):
        source = path.read_text(encoding='utf-8')
        tree = ast.parse(source)
        assert "package='nav2_costmap_2d'" in source
        assert "executable='nav2_costmap_2d'" in source
        assert "package='nav2_lifecycle_manager'" in source
        assert "'bond_timeout': 0.0" in source
        assert "ParameterValue(" in source
        for item in required:
            assert item in source
        for forbidden in (
            'planner_server', 'controller_server', 'bt_navigator', 'slam_toolbox',
            'navigate_to_pose', '/cmd_vel', 'fault_injection', 'ground_truth',
        ):
            assert forbidden not in source.lower()
        groups = [
            call for call in ast.walk(tree)
            if isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == 'GroupAction'
        ]
        assert len(groups) == 1
        keywords = {keyword.arg: keyword.value for keyword in groups[0].keywords}
        assert isinstance(keywords['scoped'], ast.Constant) and keywords['scoped'].value
        assert isinstance(keywords['forwarding'], ast.Constant) and keywords['forwarding'].value


def test_costmap_launches_use_standard_autostart_lifecycle_managers():
    """Standalone Costmaps retain normal lifecycle startup without a gate."""
    for path in (
        PACKAGE_ROOT / 'launch' / 'phase10_global_costmap_smoke.launch.py',
        PACKAGE_ROOT / 'launch' / 'phase10_local_costmap_smoke.launch.py',
        JOINT_LAUNCH,
    ):
        source = path.read_text(encoding='utf-8')
        assert "'autostart': True" in source
        assert "'bond_timeout': 0.0" in source
        assert 'phase10_costmap_startup_gate' not in source
        assert 'costmap_gate_timeout_sec' not in source


def test_rviz_and_probes_make_local_and_joint_observability_explicit_and_read_only():
    local_rviz = LOCAL_RVIZ.read_text(encoding='utf-8')
    joint_rviz = JOINT_RVIZ.read_text(encoding='utf-8')
    for expected in (
        'Fixed Frame: map', 'Value: /local_costmap/costmap',
        'Value: /local_costmap/published_footprint', 'Value: /scan',
    ):
        assert expected in local_rviz
        assert expected in joint_rviz
    for expected in ('Value: /global_costmap/costmap', 'Value: /global_costmap/published_footprint'):
        assert expected in joint_rviz

    source = LOCAL_PROBE.read_text(encoding='utf-8')
    for expected in (
        "'odom'", "'base_footprint'", "'lidar_link'", 'rolling_summary',
        'roi_snapshot', 'effective_parameters', 'LETHAL_COST_THRESHOLD',
    ):
        assert expected in source
    for forbidden in ('create_publisher', 'cmd_vel', 'set_parameters', 'fault_injection'):
        assert forbidden not in source.lower()
    assert 'validate_costmap' in JOINT_PROBE.read_text(encoding='utf-8')
    assert 'validate_local_costmap' in JOINT_PROBE.read_text(encoding='utf-8')


def snapshot(radius, scaling, stage):
    cells = []
    for x_index in range(-20, 21):
        for y_index in range(-20, 21):
            x = x_index * 0.05
            y = y_index * 0.05
            distance = (x * x + y * y) ** 0.5
            cost = 0
            if stage == 'marked':
                if distance < 0.075:
                    cost = 99
                elif distance <= radius:
                    cost = max(1, int(80 * (2.71828 ** (-scaling * distance))))
            cells.append([x, y, cost])
    return {
        'outcome': 'PASS',
        'effective_parameters': {
            'inflation_layer.inflation_radius': radius,
            'inflation_layer.cost_scaling_factor': scaling,
        },
        'roi': {
            'frame_id': 'odom',
            'center': {'x': 0.0, 'y': 0.0},
            'radius_m': 1.0,
            'resolution': 0.05,
            'cells': cells,
        },
    }


def test_experiment_evaluator_proves_radius_and_scaling_effects_without_rviz():
    with PROFILE_FILE.open(encoding='utf-8') as stream:
        manifest = yaml.safe_load(stream)
    snapshots = {}
    for name, profile in manifest['profiles'].items():
        radius = profile['inflation_radius']
        scaling = profile['cost_scaling_factor']
        snapshots[name] = {
            'before': snapshot(radius, scaling, 'before'),
            'marked': snapshot(radius, scaling, 'marked'),
            'cleared': snapshot(radius, scaling, 'cleared'),
        }
    result = evaluate(manifest['profiles'], manifest['acceptance'], snapshots)
    assert result['outcome'] == 'PASS'
    assert result['comparisons']['wider_extent_increase_m'] >= 0.15
    assert result['comparisons']['steeper_annulus_median_cost_change'] < 0
    assert result['comparisons']['steeper_inflated_cost_sum_change'] < 0


def test_published_cost_contract_does_not_hide_dynamic_99_cells():
    distribution = cost_distribution((-1, 0, 1, 98, 99, 100))
    assert LETHAL_COST_THRESHOLD == 99
    assert distribution == {
        'unknown': 1,
        'free': 1,
        'inflated': 2,
        'lethal_like': 2,
        'cost_99': 1,
        'cost_100': 1,
    }
    assert 'wider_min_extent_increase_m' in EVALUATOR.read_text(encoding='utf-8')


def test_rolling_summary_requires_window_translation_to_follow_robot_motion():
    records = [
        {
            'costmap_stamp_sec': 1.0,
            'origin_x': -3.0,
            'origin_y': -3.0,
            'window_center_x': 0.0,
            'window_center_y': 0.0,
            'robot_x': 0.0,
            'robot_y': 0.0,
        },
        {
            'costmap_stamp_sec': 2.0,
            'origin_x': -2.6,
            'origin_y': -3.0,
            'window_center_x': 0.4,
            'window_center_y': 0.0,
            'robot_x': 0.4,
            'robot_y': 0.0,
        },
    ]
    summary = rolling_summary(records, min_motion_m=0.25, max_error_m=0.075)
    assert summary['robot_delta_m']['norm'] == 0.4
    assert summary['origin_delta_error_m'] == pytest.approx(0.0)
