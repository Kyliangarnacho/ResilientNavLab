"""Unit contracts for evaluator-only Task 4 metrics."""

import json
import math
from pathlib import Path

import pytest

from navigation_benchmark_metrics import (
    apply_se2,
    evaluate_canceled_navigation_run,
    evaluate_navigation_run,
    heading_turn_count,
    localization_metrics,
    summarize_runs,
)
from navigation_benchmark_batch import infer_infrastructure_failure, write_json_durable
from navigation_benchmark_evaluator import (
    classify_evaluation_error,
    evaluate_files,
    inherited_healthy_baseline_warnings,
)
from navigate_to_pose_probe import final_tf_feedback_cross_check
from navigation_robustness_trial import parse_arguments as parse_robustness_trial_arguments


def pose(stamp, x, y, yaw=0.0):
    return {'stamp_sec': stamp, 'x': x, 'y': y, 'yaw': yaw}


def navigation_evidence():
    return {
        'status': 'PASS',
        'goal': {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
        'timing': {'goal_accepted_sim_sec': 0.0, 'action_result_sim_sec': 1.0, 'final_stop_sim_sec': 2.0},
        'feedback_samples': [pose(0.0, 0.0, 0.0), pose(1.0, 0.9, 0.0)],
        'plans': [{'path_length_m': 1.0, 'points': [pose(0.0, 0.0, 0.0), pose(0.0, 1.0, 0.0)]}],
        'scenario_result': {'feedback': {'maximum_number_of_recoveries': 0, 'final': {'navigation_time_sec': 1.0}}},
        'final_stop': {'controller_zero_observed': True, 'odometry_settled': True},
    }


def contract():
    return {
        'map_from_gt_odom': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'max_alignment_delta_sec': 0.05,
        'gt_distance_minimum_sample_period_sec': 0.05,
        'maximum_invalid_gt_samples': 0,
        'final_stop_window_sim_sec': 1.0,
    }


def canceled_navigation_evidence():
    return {
        'status': 'PASS',
        'timing': {'goal_accepted_sim_sec': 0.0, 'action_result_sim_sec': 1.0, 'final_stop_sim_sec': 2.0},
        'plans': [{'path_length_m': 1.0, 'points': [pose(0.0, 0.0, 0.0), pose(0.0, 1.0, 0.0)]}],
        'initial_path_full_footprint_sweep': {'safe': True},
        'scenario_result': {
            'navigation_canceled': True,
            'action_status': 5,
            'feedback': {'maximum_number_of_recoveries': 0},
            'cancellation': {
                'initial_path_sim_time': 0.1,
                'motion_gate_sim_time': 0.5,
                'cancel_request_sim_time': 0.5,
                'cancel_response_sim_time': 0.5,
                'motion_gate_odom_travel_m': 0.21,
                'minimum_odom_travel_m': 0.20,
                'cancel_return_code': 0,
                'cancel_goals_canceling_count': 1,
                'last_nonzero_command_before_cancel_sim_time': 0.4,
                'first_zero_command_after_cancel_sim_time': 0.6,
            },
        },
        'final_stop': {'controller_zero_observed': True, 'odometry_settled': True},
    }


def test_apply_se2_is_fixed_not_a_trajectory_fit():
    transformed = apply_se2(pose(1.0, 1.0, 0.0, 0.0), {'x': 2.0, 'y': 3.0, 'yaw': math.pi / 2.0})
    assert transformed['x'] == pytest.approx(2.0)
    assert transformed['y'] == pytest.approx(4.0)
    assert transformed['yaw'] == pytest.approx(math.pi / 2.0)


def test_localization_metrics_pairs_only_in_declared_time_window():
    metrics = localization_metrics([pose(1.0, 1.1, 0.0)], [pose(1.02, 1.0, 0.0)], maximum_delta_sec=0.05)
    assert metrics['aligned_sample_count'] == 1
    assert metrics['position_error_m']['final'] == pytest.approx(0.1)
    with pytest.raises(ValueError, match='no feedback'):
        localization_metrics([pose(1.0, 1.1, 0.0)], [pose(1.2, 1.0, 0.0)], maximum_delta_sec=0.05)


def test_feedback_can_coalesce_equal_action_pose_stamps_but_not_time_regressions():
    metrics = localization_metrics(
        [pose(1.0, 0.9, 0.0), pose(1.0, 1.0, 0.0)], [pose(1.0, 1.0, 0.0)],
        maximum_delta_sec=0.05,
    )
    assert metrics['feedback_sample_count'] == 1
    with pytest.raises(ValueError, match='strictly increasing'):
        localization_metrics([pose(2.0, 0.0, 0.0), pose(1.0, 0.0, 0.0)], [pose(1.0, 0.0, 0.0)], maximum_delta_sec=1.1)


def test_complete_run_reports_gt_endpoint_travel_efficiency_and_replans():
    outcome = evaluate_navigation_run(navigation_evidence(), {'invalid_sample_count': 0, 'samples': [pose(0.0, 0.0, 0.0), pose(1.0, 0.9, 0.0), pose(2.0, 0.9, 0.0)]}, contract())
    assert outcome['navigation_success'] is True
    assert outcome['final_gt_position_error_m'] == pytest.approx(0.1)
    assert outcome['actual_gt_travel_distance_m'] == pytest.approx(0.9)
    assert outcome['route_execution_efficiency'] == pytest.approx(1.0 / 0.9)
    assert outcome['replanning_count'] == 0
    assert outcome['final_stop']['ground_truth_translation_m'] == pytest.approx(0.0)
    assert outcome['navigation_time_sec'] == pytest.approx(1.0)


def test_native_goal_cancel_requires_ordered_cancel_and_controller_stop_evidence():
    evidence = canceled_navigation_evidence()
    gt = {'invalid_sample_count': 0, 'samples': [
        pose(0.0, 0.0, 0.0), pose(0.5, 0.21, 0.0),
        pose(1.0, 0.21, 0.0), pose(2.0, 0.21, 0.0),
    ]}
    outcome = evaluate_canceled_navigation_run(evidence, gt, contract())
    assert outcome['navigation_canceled'] is True
    assert outcome['actual_gt_travel_before_cancel_m'] == pytest.approx(0.21)
    assert outcome['controller_stop_evidence']['first_zero_after_cancel_sim_time'] == pytest.approx(0.6)

    evidence['scenario_result']['cancellation']['first_zero_command_after_cancel_sim_time'] = 2.1
    with pytest.raises(ValueError, match='timestamps are invalid'):
        evaluate_canceled_navigation_run(evidence, gt, contract())


def test_goal_cancel_evaluator_requires_no_obstacle_event_evidence(tmp_path):
    navigation = canceled_navigation_evidence()
    navigation['scenario'] = 'goal_cancel'
    ground_truth = {'invalid_sample_count': 0, 'samples': [
        pose(0.0, 0.0, 0.0), pose(0.5, 0.21, 0.0),
        pose(1.0, 0.21, 0.0), pose(2.0, 0.21, 0.0),
    ]}
    navigation_path = tmp_path / 'navigation.json'
    ground_truth_path = tmp_path / 'ground_truth.json'
    navigation_path.write_text(json.dumps(navigation), encoding='utf-8')
    ground_truth_path.write_text(json.dumps(ground_truth), encoding='utf-8')
    outcome = evaluate_files(
        navigation_path, ground_truth_path,
        Path(__file__).resolve().parents[1] / 'config' / 'healthy_navigation_benchmark.yaml',
        Path(__file__).resolve().parents[1] / 'config' / 'navigation_robustness_scenarios.yaml',
    )
    assert outcome['status'] == 'PASS'
    assert outcome['acceptance_scope'] == 'task5_goal_cancel'
    assert 'obstacle_event_evidence_path' not in outcome


def test_turn_count_ignores_zero_length_segments_and_summary_separates_attempts():
    assert heading_turn_count([pose(0, 0, 0), pose(0, 0.2, 0), pose(0, 0.2, 0.2), pose(0, 0.4, 0.2)]) == 2
    summary = summarize_runs([{'status': 'PASS', 'metrics': {'navigation_time_sec': 2.0, 'final_gt_position_error_m': 0.1, 'final_gt_yaw_error_rad': 0.1, 'actual_gt_travel_distance_m': 1.0, 'route_execution_efficiency': 1.0, 'geometric_directness': 1.0, 'replanning_count': 2}}, {'status': 'FAIL'}])
    assert summary['attempt_count'] == 2
    assert summary['pass_count'] == 1


def test_dense_navfn_style_path_keeps_real_turns_after_spatial_resampling_and_rdp():
    dense = [{'x': index * 0.05, 'y': 0.0} for index in range(9)]
    dense += [{'x': 0.4, 'y': index * 0.05} for index in range(1, 9)]
    dense += [{'x': 0.4 + index * 0.05, 'y': 0.8} for index in range(1, 9)]
    assert heading_turn_count(dense) >= 2


def test_structured_readiness_failures_are_infrastructure_not_navigation_failure():
    assert classify_evaluation_error(ValueError(
        'navigation runner did not pass: readiness/clock: /clock is not yet a stable monotonic simulation clock'
    )) == 'infrastructure_gazebo_clock'
    assert classify_evaluation_error(ValueError(
        'navigation runner did not pass: readiness/localization_tf: filtered odometry has not arrived'
    )) == 'infrastructure_localization_tf'
    assert classify_evaluation_error(ValueError(
        'navigation runner did not pass: readiness/nav2_lifecycle: not active'
    )) == 'infrastructure_nav2_lifecycle_service'


def test_invalid_ground_truth_evidence_fails_closed():
    with pytest.raises(ValueError, match='quality failure'):
        evaluate_navigation_run(navigation_evidence(), {'invalid_sample_count': 1, 'samples': [pose(0.0, 0.0, 0.0), pose(1.0, 0.9, 0.0), pose(2.0, 0.9, 0.0)]}, contract())


def test_task5_can_retain_inherited_localization_limit_as_warning():
    metrics = {
        'final_gt_position_error_m': 0.309,
        'final_gt_yaw_error_rad': 0.10,
        'localization_error': {'alignment_coverage': 1.0},
        'feedback_navigation_time_delta_sec': 0.01,
        'frozen_gt_origin_check': {'position_error_m': 0.0, 'yaw_error_rad': 0.0},
    }
    benchmark = {
        'final_gt_position_error_m': 0.25,
        'final_gt_yaw_error_rad': 0.30,
        'minimum_alignment_coverage': 0.95,
        'max_feedback_navigation_time_delta_sec': 0.10,
        'initial_gt_position_tolerance_m': 0.10,
        'initial_gt_yaw_tolerance_rad': 0.10,
    }
    warnings = inherited_healthy_baseline_warnings(metrics, benchmark)
    assert warnings == [{
        'id': 'final_gt_position_error_m',
        'value': 0.309,
        'threshold': 0.25,
        'severity': 'warning',
        'scope': 'inherited_healthy_baseline_localization',
    }]


def test_launch_evidence_prefers_lifecycle_service_cause_over_downstream_tf(tmp_path):
    launch_log = tmp_path / 'launch.log'
    initial_pose = tmp_path / 'initial_pose.json'
    launch_log.write_text(
        'map_server.rclcpp: failed to send response to /map_server/change_state\n'
        'Timed out waiting for transform from base_footprint to map\n'
    )
    initial_pose.write_text('{"outcome": "FAIL", "error": "no map-frame /amcl_pose observed"}\n')
    assert infer_infrastructure_failure(launch_log, initial_pose) == 'infrastructure_nav2_lifecycle_service'


def test_write_json_durable_converts_nested_path_values_to_strings_only(tmp_path):
    output = tmp_path / 'nested.json'
    nested_path = tmp_path / 'evidence' / 'navigation.json'
    write_json_durable(
        output,
        {'top_level_path': nested_path, 'nested': [{'launch_log_path': nested_path}]},
    )
    assert json.loads(output.read_text(encoding='utf-8')) == {
        'nested': [{'launch_log_path': str(nested_path)}],
        'top_level_path': str(nested_path),
    }
    with pytest.raises(TypeError, match='only pathlib.Path'):
        write_json_durable(output, {'unsupported': object()})


def test_safe_failure_records_final_tf_feedback_divergence_as_warning_only():
    strict = final_tf_feedback_cross_check(
        (1.0, 0.0, 0.0), (1.2, 0.0, 0.0), divergence_is_error=True,
    )
    safe_failure = final_tf_feedback_cross_check(
        (1.0, 0.0, 0.0), (1.2, 0.0, 0.0), divergence_is_error=False,
    )
    assert strict['diverged'] is True
    assert strict['outcome'] == 'ERROR'
    assert safe_failure['translation_m'] == pytest.approx(0.2)
    assert safe_failure['outcome'] == 'WARNING'


def test_task5_trial_cli_keeps_rviz_disabled_unless_manually_requested():
    base = ['--scenario', 'dynamic_fully_blocked', '--results-dir', '/tmp/task5-rviz-test']
    assert parse_robustness_trial_arguments(base).use_rviz is False
    assert parse_robustness_trial_arguments([*base, '--use-rviz']).use_rviz is True
