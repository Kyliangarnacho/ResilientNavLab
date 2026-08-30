"""Static isolation and frozen-resource contracts for Phase 10 Task 4."""

from __future__ import annotations

import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_benchmark_config_freezes_three_runs_gt_transform_and_thresholds():
    config = yaml.safe_load((ROOT / 'config' / 'healthy_navigation_benchmark.yaml').read_text())['benchmark']
    assert config['repetitions'] == 3
    assert config['readiness_timeout_wall_sec'] == 120.0
    assert config['infrastructure_start_attempts'] == 1
    assert config['scenario_order'] == ['simple_reachable', 'static_obstacle_detour', 'multi_turn_healthy']
    assert config['map_from_gt_odom'] == {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    assert config['max_alignment_delta_sec'] == 0.05
    assert config['minimum_alignment_coverage'] == 0.95
    assert config['gt_distance_minimum_sample_period_sec'] == 0.05
    assert config['maximum_invalid_gt_samples'] == 0
    assert config['final_gt_position_error_m'] == 0.25
    assert config['final_gt_yaw_error_rad'] == 0.30


def test_multi_turn_is_one_frozen_navigate_to_pose_goal_with_geometry_contract():
    scenario = yaml.safe_load((ROOT / 'config' / 'planner_smoke_scenarios.yaml').read_text())['scenarios']['multi_turn_healthy']
    assert scenario['goal'] == {'x': 4.7, 'y': 5.3, 'yaw': 1.57079632679}
    assert scenario['navigate_to_pose']['action_timeout_sec'] == 180.0
    assert scenario['benchmark'] == {'min_initial_path_length_m': 7.0, 'min_initial_path_turn_count': 2}
    assert 'controller' not in scenario


def test_runner_only_commands_navigate_to_pose_and_never_reads_ground_truth():
    source = (ROOT / 'navigation_benchmark_runner.py').read_text()
    ast.parse(source)
    for required in ('NavigateToPoseProbe', 'run_scenario', 'phase9_asset_hashes', 'wait_for_simulated_settle', 'POST_GOAL_SETTLE_SIM_SEC', 'full_footprint_path_sweep', 'initial_path_full_footprint_sweep'):
        assert required in source
    for forbidden in ('ground_truth', '/evaluation/', 'ComputePathToPose', 'FollowPath', 'create_publisher(Twist'):
        assert forbidden not in source


def test_gt_recorder_is_read_only_and_has_no_action_or_control_interface():
    source = (ROOT / 'navigation_benchmark_gt_recorder.py').read_text()
    ast.parse(source)
    assert "'/evaluation/ground_truth_pose'" in source
    for forbidden in ('ActionClient', 'NavigateToPose', 'create_publisher', '/cmd_vel', 'set_parameters'):
        assert forbidden not in source


def test_trial_launch_has_one_existing_navigation_chain_and_one_evaluation_overlay():
    source = (ROOT / 'launch' / 'phase10_navigation_benchmark_trial.launch.py').read_text()
    ast.parse(source)
    for required in ('phase10_bt_navigation_smoke.launch.py', 'phase8_ground_truth.launch.py', 'phase10_navigation_benchmark_runner', 'phase10_navigation_benchmark_gt_recorder', 'record_diagnostics', 'initial_pose_result', 'GZ_PARTITION', 'gz_partition', 'ROS_DOMAIN_ID', 'ros_domain_id', 'readiness_timeout_sec', "default_value='120.0'"):
        assert required in source
    for forbidden in ('nav2_costmap_2d', 'nav2_behaviors', 'behavior_server', 'ComputePathToPose', 'FollowPath'):
        assert forbidden not in source.lower()
    assert "DeclareLaunchArgument('record_diagnostics', default_value='false')" in source


def test_setup_installs_task4_modules_and_console_scripts():
    source = (ROOT / 'setup.py').read_text()
    for item in ('navigation_benchmark_metrics', 'navigation_benchmark_runner', 'navigation_benchmark_gt_recorder', 'navigation_benchmark_evaluator', 'navigation_benchmark_batch', 'phase10_navigation_benchmark_batch', 'navigation_robustness_trial', 'phase10_navigation_robustness_trial', 'localization_lifecycle_probe', 'phase10_localization_lifecycle_probe'):
        assert item in source


def test_task5_borrows_task4_trial_and_official_gazebo_entity_factory_once():
    launch_source = (ROOT / 'launch' / 'phase10_navigation_robustness_trial.launch.py').read_text()
    injector_source = (ROOT / 'navigation_obstacle_event_injector.py').read_text()
    scenario = yaml.safe_load((ROOT / 'config' / 'navigation_robustness_scenarios.yaml').read_text())['scenarios']
    ast.parse(launch_source)
    ast.parse(injector_source)
    assert 'phase10_navigation_benchmark_trial.launch.py' in launch_source
    assert 'ros_gz_bridge' in launch_source
    assert 'SpawnEntity' in launch_source
    assert "'/world/resilient_lab/create'" in injector_source
    assert 'ground_truth' not in injector_source
    for name, expected in (('dynamic_obstacle_detour', 'success'), ('dynamic_fully_blocked', 'safe_failure')):
        assert scenario[name]['expected'] == expected
        assert scenario[name]['base_scenario'] == 'static_obstacle_detour'
        assert scenario[name]['task5']['trigger']['minimum_odom_travel_m'] == 0.20


def test_batch_assigns_each_fresh_trial_a_unique_gazebo_partition_and_dds_domain():
    source = (ROOT / 'navigation_benchmark_batch.py').read_text()
    ast.parse(source)
    assert "partition = f'{partition_prefix}_{run_dir.name}'" in source
    assert "f'gz_partition:={partition}'" in source
    assert 'ros_domain_id=100 + physical_attempt_index' in source
    assert 'additional_evidence_paths' in source
    assert 'process_group_processes' in source
    assert 'sync_tree_if_present' in source
    assert 'TRIAL_TEARDOWN_TIMEOUT_SEC' in source
    assert "launch_environment['ROS_LOG_DIR']" in source
    assert "result['process_cleanup'] = attempt['process_cleanup']" in source
    assert "'--scenarios', nargs='+'" in source
    assert 'ordered prefix of the frozen scenario order' in source
    assert 'derived_trial_timeout' in source
    assert 'launch_failure_kind' in source
    assert "'stopped_early'" in source
    assert 'inter_trial_settle_wall_sec' not in source
    assert 'max_infrastructure_start_attempts' not in source


def test_task5_single_trial_supervisor_reuses_task4_barrier_without_retry():
    source = (ROOT / 'navigation_robustness_trial.py').read_text()
    launch_source = (ROOT / 'launch' / 'phase10_navigation_robustness_trial.launch.py').read_text()
    ast.parse(source)
    ast.parse(launch_source)
    for required in (
        'run_trial', 'phase10_navigation_robustness_trial.launch.py',
        'additional_evidence_paths=', 'scenario_requires_obstacle_event',
        "partition_prefix='resilient_nav_phase10_task5'", 'write_json_durable',
        'evaluate_files', 'infrastructure_start_attempts', '--evaluation-mode', '--use-rviz',
        "f\"use_rviz:={'true' if arguments.use_rviz else 'false'}\"",
        'scenario_use_recovery', "use_recovery:={'true' if use_recovery else 'false'}",
    ):
        assert required in source
    assert "'use_rviz': use_rviz" in launch_source
    assert 'enable_obstacle_event' in launch_source
    assert 'IfCondition(enable_obstacle_event)' in launch_source


def test_task54_goal_cancel_uses_one_native_action_client_without_entity_injection():
    runner_source = (ROOT / 'navigation_benchmark_runner.py').read_text()
    probe_source = (ROOT / 'navigate_to_pose_probe.py').read_text()
    supervisor_source = (ROOT / 'navigation_robustness_trial.py').read_text()
    scenario = yaml.safe_load(
        (ROOT / 'config' / 'navigation_robustness_scenarios.yaml').read_text()
    )['scenarios']['goal_cancel']
    assert scenario['expected'] == 'canceled'
    assert scenario['task5']['kind'] == 'goal_cancel'
    assert scenario['task5']['cancel']['minimum_odom_travel_m'] == 0.20
    assert "'canceled'" in runner_source
    assert 'cancel_goal_async' in probe_source
    assert probe_source.count('ActionClient(') == 1
    assert 'ground_truth' not in probe_source
    assert 'scenario_requires_obstacle_event' in supervisor_source


def test_task5_fully_blocked_freezes_observed_branch_without_terminal_window_gate():
    scenario = yaml.safe_load(
        (ROOT / 'config' / 'navigation_robustness_scenarios.yaml').read_text()
    )['scenarios']['dynamic_fully_blocked']['task5']['acceptance']
    assert scenario['frozen_error_code'] == 208
    assert scenario['post_detection_observation_warning_sec'] == 12.0
    assert 'post_detection_observation_sim_sec' not in scenario


def test_task53_borrows_official_entity_delete_and_keeps_trigger_independent_of_recovery():
    launch_source = (ROOT / 'launch' / 'phase10_navigation_robustness_trial.launch.py').read_text()
    injector_source = (ROOT / 'navigation_obstacle_event_injector.py').read_text()
    scenarios = yaml.safe_load(
        (ROOT / 'config' / 'navigation_robustness_scenarios.yaml').read_text()
    )['scenarios']
    scenario = scenarios['dynamic_temporary_blocked_recovery']
    assert 'DeleteEntity' in launch_source
    assert "'/world/resilient_lab/remove'" in injector_source
    assert 'obstacle_lifetime_sim_sec' in injector_source
    assert 'recovery_count' not in injector_source
    assert scenario['navigation_profile'] == 'recovery'
    assert scenario['task5']['event_mode'] == 'spawn_then_delete'
    assert scenario['task5']['trigger']['obstacle_lifetime_sim_sec'] == 45.0


def test_task53_discovery_is_measured_not_formal_acceptance():
    source = (ROOT / 'navigation_benchmark_evaluator.py').read_text()
    assert "task5_temporary_obstacle_recovery_discovery" in source
    assert "status = 'MEASURED'" in source


def test_runner_records_minimal_ordered_readiness_without_gt_access():
    source = (ROOT / 'navigation_benchmark_runner.py').read_text()
    assert 'wait_for_readiness_stage' in source
    assert "'clock'" in source
    assert "'localization_tf'" in source
    assert "'nav2_lifecycle'" in source
    assert "'navigate_to_pose'" in source
    assert "'costmaps'" in source
    assert source.index("'clock'") < source.index("'nav2_lifecycle'")
    assert 'navigation_manager_active' in source
    assert 'LIFECYCLE_SERVICES' not in source


def test_safe_failure_preserves_terminal_tf_feedback_divergence_as_evidence():
    source = (ROOT / 'navigation_benchmark_runner.py').read_text()
    probe_source = (ROOT / 'navigate_to_pose_probe.py').read_text()
    assert "terminal_tf_feedback_divergence_is_error=expected not in ('safe_failure', 'canceled')" in source
    assert 'final_tf_feedback_cross_check' in probe_source
    assert "outcome = 'ERROR' if divergence_is_error else 'WARNING'" in probe_source
    assert 'final TF/feedback cross-check diverged' in probe_source


def test_minimal_localization_reproducer_uses_official_manager_without_gazebo_overlay():
    launch_source = (ROOT / 'launch' / 'phase10_localization_lifecycle_reproducer.launch.py').read_text()
    probe_source = (ROOT / 'localization_lifecycle_probe.py').read_text()
    ast.parse(launch_source)
    ast.parse(probe_source)
    assert 'phase10_localization.launch.py' in launch_source
    assert "'use_sim_time': 'false'" in launch_source
    assert 'phase8_ground_truth.launch.py' not in launch_source
    assert '/lifecycle_manager_localization/is_active' in probe_source
    assert '/map_server/get_state' in probe_source
    assert '/amcl/get_state' in probe_source
