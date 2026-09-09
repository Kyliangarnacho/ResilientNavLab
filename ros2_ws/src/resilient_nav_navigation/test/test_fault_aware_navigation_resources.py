"""Static contracts for the thin resilience/navigation integration."""

import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_runtime_launch_has_one_adaptive_tf_owner_and_faulted_scan_path():
    """The runtime chain owns odom TF once and routes scan consistently."""
    source = (
        ROOT / 'launch' / 'fault_aware_navigation.launch.py'
    ).read_text(encoding='utf-8')
    ast.parse(source)

    assert "'start_odom_tf_broadcaster': 'false'" in source
    assert "'publish_tf': 'true'" in source
    assert "'adaptive_output_topic': '/odometry/filtered'" in source
    assert "'navigation_scan_topic': '/faulted/scan'" in source
    assert "'costmap_update_timeout': '0.3'" in source
    assert "'replay_tf_confirmation_count': 2" in source
    assert 'replay_settle_wall_sec' not in source
    assert "'start_localization': 'false'" in source
    assert 'faulted_ekf_filter_node' not in source


def test_runtime_supervision_has_no_truth_input_or_direct_motion_output():
    """Supervisor and goal gate use runtime evidence, never benchmark truth."""
    supervisor = (ROOT / 'resilience_supervisor_node.py').read_text(
        encoding='utf-8'
    ).lower()
    gate = (ROOT / 'resilient_goal_gate.py').read_text(
        encoding='utf-8'
    ).lower()

    for topic in (
        "f'/health/{sensor}'", '/fusion/status',
        '/localization/quality', '/resilience/status',
    ):
        assert topic in supervisor
    for forbidden in (
        'faultstatus', '/fault_injection/', 'scenario_id', 'scenario_seed',
        '/evaluation/', 'ground_truth', '/cmd_vel',
    ):
        assert forbidden not in supervisor
    assert '/resilience/status' in gate
    assert '/faulted/scan' in gate
    assert 'can_transform' in gate
    assert 'replay_settle_wall_sec' not in gate
    assert '/cmd_vel' not in gate


def test_fault_aware_amcl_consumes_sanitized_scan():
    """AMCL and Nav2 costmaps share the injected scan branch."""
    config = yaml.safe_load(
        (ROOT / 'config' / 'nav2_fault_aware_localization.yaml').read_text(
            encoding='utf-8'
        )
    )
    assert config['amcl']['ros__parameters']['scan_topic'] == '/faulted/scan'


def test_benchmark_targets_gate_and_records_decision_evidence():
    """The healthy trial sends goals above Nav2 and records every decision."""
    source = (
        ROOT / 'launch' / 'fault_aware_navigation_benchmark.launch.py'
    ).read_text(encoding='utf-8')
    ast.parse(source)

    assert "'--action-name', '/resilient_navigate_to_pose'" in source
    assert "'--observe-path-safety-only'" in source
    for topic in (
        '/health/wheel', '/fusion/reliability', '/fusion/status',
        '/localization/quality', '/resilience/status', '/odometry/filtered',
    ):
        assert topic in source
    assert "'physical_scenario': LaunchConfiguration(" in source
    assert "'sensor_scenario_file': LaunchConfiguration(" in source
    assert "'/fault_injection/status'" in source


def test_rviz_shows_faulted_scan_goal_paths_and_costmaps():
    """Fault runs expose the minimum operator-facing navigation picture."""
    rviz = (ROOT / 'rviz' / 'phase10_bt_navigation.rviz').read_text(
        encoding='utf-8'
    )
    gate = (ROOT / 'resilient_goal_gate.py').read_text(encoding='utf-8')
    launch = (
        ROOT / 'launch' / 'phase10_bt_navigation_smoke.launch.py'
    ).read_text(encoding='utf-8')

    for required in (
        'Active Navigation Goal', '/resilience/active_goal', 'Planned Path',
        'Global Costmap', 'Local Costmap',
    ):
        assert required in rviz
    assert "remappings=[('/scan', navigation_scan_topic)]" in launch
    assert "'/resilience/active_goal'" in gate


def test_fault_aware_costmap_timeout_matches_legacy_default():
    """The supervised chain keeps the established 0.3-second timeout."""
    controller = (
        ROOT / 'launch' / 'phase10_controller_smoke.launch.py'
    ).read_text(encoding='utf-8')
    bt = (
        ROOT / 'launch' / 'phase10_bt_navigation_smoke.launch.py'
    ).read_text(encoding='utf-8')

    declaration = (
        "DeclareLaunchArgument('costmap_update_timeout', "
        "default_value='0.3')"
    )
    assert declaration in controller
    assert declaration in bt
    assert "'costmap_update_timeout': costmap_update_timeout" in bt
