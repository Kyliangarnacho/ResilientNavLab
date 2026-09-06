"""Regression contract for the frozen BRNE V1 Scene 1--3 profile."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROFILE = PACKAGE_ROOT / 'config' / 'brne_v1_runtime.yaml'
SENSOR_SCENES = (
    PACKAGE_ROOT / 'launch' / 'brne_sensor_scene1_demo.launch.py',
    PACKAGE_ROOT / 'launch' / 'brne_scene2_demo.launch.py',
    PACKAGE_ROOT / 'launch' / 'brne_scene3_head_on_demo.launch.py',
)


def test_frozen_profile_records_core_interaction_event_and_tracker_values():
    source = PROFILE.read_text(encoding='utf-8')
    expected_lines = (
        'maximum_agents: 5',
        'num_samples: 196',
        'plan_steps: 25',
        'close_stop_threshold: 0.20',
        'interaction_entry_distance: 3.20',
        'interaction_separation_margin: 0.20',
        'interaction_distance_history_outputs: 4',
        'crossing_lateral_speed_threshold: 0.08',
        'crossing_time_max: 4.0',
        'crossing_forward_max: 2.00',
        'crossing_side_bias_multiplier: 10.0',
        'crossing_preferred_safety_weight: 0.10',
        'crossing_initial_direction_window_outputs: 5',
        'head_on_minimum_approach_speed: 0.12',
        'head_on_maximum_direction_angle_rad: 1.05',
        'head_on_lateral_direction_deadband_rad: 0.17',
        'head_on_release_path_clearance: 0.58',
        'proposal_opposite_scale: 0.25',
        'proposal_opposite_threshold: 0.35',
        'proposal_protection_window_outputs: 5',
        'maximum_dynamic_agent_range: 4.0',
        'history_size: 6',
        'minimum_confirmations: 6',
        'velocity_ema_alpha: 0.25',
        'minimum_direction_step: 0.005',
        'velocity_ema_stability_window: 3',
        'velocity_ema_max_direction_change_rad: 0.35',
        'velocity_ema_outlier_direction_change_rad: 0.70',
    )
    for line in expected_lines:
        assert f'    {line}' in source


def test_all_sensor_scenes_load_one_profile_for_tracker_and_shadow():
    for launch_path in SENSOR_SCENES:
        source = launch_path.read_text(encoding='utf-8')
        assert "'config' / 'brne_v1_runtime.yaml'" in source
        assert source.count(
            "parameters=[str(runtime_profile), {'use_sim_time': True}]"
        ) == 2
        assert "DeclareLaunchArgument('crossing_" not in source
        assert "DeclareLaunchArgument('head_on_" not in source
        assert "DeclareLaunchArgument('proposal_" not in source
        assert 'commitment' not in source
