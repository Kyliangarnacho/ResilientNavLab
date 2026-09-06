"""Static isolation contracts for the BRNE ROS shadow wrapper."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_shadow_node.py'
SETUP_PATH = PACKAGE_ROOT / 'setup.py'
PROFILE_PATH = PACKAGE_ROOT / 'config' / 'brne_v1_runtime.yaml'


def test_shadow_node_declares_only_the_isolated_brne_io_topics():
    """The wrapper must not directly subscribe or publish through formal Nav2 topics."""
    source = NODE_PATH.read_text(encoding='utf-8')

    for topic in ('/brne/odom', '/brne/goal_pose', '/brne/pedestrians'):
        assert topic in source
    for topic in ('/brne/cmd_vel_raw', '/brne/optimal_path'):
        assert topic in source
    assert "'/brne/ready'" in source
    assert "create_publisher(Twist, '/cmd_vel'" not in source
    assert "create_subscription(Path, '/plan'" not in source
    assert '0.040' not in source
    assert 'Unitree' not in source


def test_shadow_node_keeps_a_fail_safe_and_installs_both_tools():
    """Missing inputs must publish zero and the node/smoke remain invokable."""
    source = NODE_PATH.read_text(encoding='utf-8')
    setup_source = SETUP_PATH.read_text(encoding='utf-8')

    assert 'snapshot = self._fresh_input_snapshot()' in source
    assert 'if not self._snapshot_still_fresh(versions):' in source
    assert 'self._publish_stop(' in source
    assert 'if rclpy.ok():' in source
    assert 'self.planner.warm_up()' in source
    assert 'self._pedestrians = []' in source
    assert 'fresh empty array as observed no-agent' in source
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in source
    assert "brne_shadow_node = resilient_nav_brne.brne_shadow_node:main" in setup_source
    assert "brne_shadow_smoke = resilient_nav_brne.brne_shadow_smoke:main" in setup_source


def test_shadow_node_reports_the_effective_mask_threshold_and_stop_reason():
    source = NODE_PATH.read_text(encoding='utf-8')
    assert 'BRNE time-aligned safety mask configured:' in source
    assert 'close_stop_threshold={self.planner.config.close_stop_threshold:.3f} m' in source
    assert 'safety mask rejected all robot candidates' in source
    assert 'required input missing or stale' in source
    assert 'BRNE weighted control is zero with safe candidates remaining' in source
    assert 'BRNE interaction policy:' in source
    assert 'core_preferred_weight_share=' in source
    assert 'command_angular=' in source


def test_shadow_node_defaults_match_the_single_frozen_runtime_profile():
    source = NODE_PATH.read_text(encoding='utf-8')
    profile = PROFILE_PATH.read_text(encoding='utf-8')

    for declaration in (
        "self.declare_parameter('maximum_agents', 5)",
        "self.declare_parameter('num_samples', 196)",
        "self.declare_parameter('dt', 0.1)",
        "self.declare_parameter('plan_steps', 25)",
        "self.declare_parameter('kernel_a1', 0.2)",
        "self.declare_parameter('kernel_a2', 0.2)",
        "self.declare_parameter('cost_a1', 15.0)",
        "self.declare_parameter('cost_a2', 3.0)",
        "self.declare_parameter('cost_a3', 20.0)",
        "self.declare_parameter('ped_sample_scale', 0.1)",
        "self.declare_parameter('proposal_protection_window_outputs', 5)",
        "self.declare_parameter('proposal_opposite_scale', 0.25)",
        "self.declare_parameter('proposal_opposite_threshold', 0.35)",
        "self.declare_parameter('interaction_separation_margin', 0.20)",
        "self.declare_parameter('crossing_lateral_speed_threshold', 0.08)",
        "self.declare_parameter('crossing_minimum_lateral_alignment', 0.50)",
        "self.declare_parameter('crossing_time_max', 4.0)",
        "self.declare_parameter('crossing_forward_min', 0.20)",
        "self.declare_parameter('crossing_forward_max', 2.00)",
        "self.declare_parameter('crossing_side_bias_multiplier', 10.00)",
        "self.declare_parameter('crossing_preferred_safety_weight', 0.10)",
        "self.declare_parameter('interaction_entry_distance', 3.20)",
        "self.declare_parameter('crossing_initial_direction_window_outputs', 5)",
        "self.declare_parameter('head_on_minimum_approach_speed', 0.12)",
        "self.declare_parameter('head_on_maximum_direction_angle_rad', 1.05)",
        "self.declare_parameter('head_on_lateral_direction_deadband_rad', 0.17)",
        "self.declare_parameter('head_on_release_path_clearance', 0.58)",
    ):
        assert declaration in source
    for parameter in (
        'kernel_a1', 'kernel_a2', 'cost_a1', 'cost_a2', 'cost_a3',
        'ped_sample_scale', 'proposal_opposite_scale',
        'proposal_opposite_threshold', 'interaction_separation_margin',
        'interaction_entry_distance',
        'crossing_lateral_speed_threshold', 'crossing_minimum_lateral_alignment',
        'crossing_time_max',
        'crossing_forward_min', 'crossing_forward_max',
        'crossing_side_bias_multiplier', 'crossing_preferred_safety_weight',
        'head_on_minimum_approach_speed', 'head_on_maximum_direction_angle_rad',
        'head_on_lateral_direction_deadband_rad', 'head_on_release_path_clearance',
    ):
        assert f"self.get_parameter('{parameter}').value" in source
    assert 'brne_v1_runtime.yaml' not in source
    assert 'num_samples: 196' in profile
    assert 'close_stop_threshold: 0.20' in profile
    assert 'crossing_side_bias_multiplier: 10.0' in profile
    assert 'head_on_release_path_clearance: 0.58' in profile
    assert "glob('config/*.yaml')" in SETUP_PATH.read_text(encoding='utf-8')
    assert 'commitment' not in source
