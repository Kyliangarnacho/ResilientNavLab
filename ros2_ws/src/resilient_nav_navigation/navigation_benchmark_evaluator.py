"""Offline-only merger for completed Task 4 navigation and GT evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from navigation_benchmark_metrics import evaluate_canceled_navigation_run, evaluate_navigation_run
from navigation_robustness_metrics import evaluate_dynamic_obstacle_event


def classify_evaluation_error(error: Exception) -> str:
    """Keep evaluator/infrastructure evidence failures distinct from navigation."""
    message = str(error)
    if message.startswith('Ground Truth evidence') or message.startswith('Ground Truth lacks'):
        return 'infrastructure_gt_evidence'
    if message.startswith('navigation runner did not pass: readiness/clock'):
        return 'infrastructure_gazebo_clock'
    if message.startswith('navigation runner did not pass: readiness/localization_tf'):
        return 'infrastructure_localization_tf'
    if message.startswith('navigation runner did not pass: readiness/nav2_lifecycle'):
        return 'infrastructure_nav2_lifecycle_service'
    if message.startswith('navigation runner did not pass: readiness/navigate_to_pose'):
        return 'infrastructure_nav2_action'
    if message.startswith('navigation runner did not pass: readiness/costmaps'):
        return 'infrastructure_nav2_costmaps'
    if message.startswith('navigation runner did not pass'):
        return 'navigation_runtime'
    return 'evaluation_contract'


def load_yaml(path: Path) -> dict[str, object]:
    value = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'{path} must be a mapping')
    return value


def inherited_healthy_baseline_warnings(
    metrics: dict[str, object], benchmark: dict[str, object],
) -> list[dict[str, object]]:
    """Retain inherited localization observations without redefining Task 5.

    Task 4 validates healthy end-to-end navigation.  Task 5 validates the
    causal response to a newly spawned obstacle.  Endpoint/localization
    thresholds remain visible in Task 5 evidence, but are warnings rather
    than blockers when the Task 5-specific dynamic contract is complete.
    """
    warnings: list[dict[str, object]] = []

    def add_if_exceeds(identifier: str, value: float, threshold: float) -> None:
        if value > threshold:
            warnings.append({
                'id': identifier,
                'value': value,
                'threshold': threshold,
                'severity': 'warning',
                'scope': 'inherited_healthy_baseline_localization',
            })

    add_if_exceeds(
        'final_gt_position_error_m', float(metrics['final_gt_position_error_m']),
        float(benchmark['final_gt_position_error_m']),
    )
    add_if_exceeds(
        'final_gt_yaw_error_rad', float(metrics['final_gt_yaw_error_rad']),
        float(benchmark['final_gt_yaw_error_rad']),
    )
    localization = metrics['localization_error']
    if not isinstance(localization, dict):
        raise ValueError('healthy localization metrics are malformed')
    coverage = float(localization['alignment_coverage'])
    minimum_coverage = float(benchmark['minimum_alignment_coverage'])
    if coverage < minimum_coverage:
        warnings.append({
            'id': 'localization_alignment_coverage',
            'value': coverage,
            'threshold': minimum_coverage,
            'comparison': 'minimum',
            'severity': 'warning',
            'scope': 'inherited_healthy_baseline_localization',
        })
    add_if_exceeds(
        'feedback_navigation_time_cross_check',
        float(metrics['feedback_navigation_time_delta_sec']),
        float(benchmark['max_feedback_navigation_time_delta_sec']),
    )
    origin = metrics['frozen_gt_origin_check']
    if not isinstance(origin, dict):
        raise ValueError('healthy frozen GT origin metrics are malformed')
    add_if_exceeds(
        'frozen_gt_origin_position', float(origin['position_error_m']),
        float(benchmark['initial_gt_position_tolerance_m']),
    )
    add_if_exceeds(
        'frozen_gt_origin_yaw', float(origin['yaw_error_rad']),
        float(benchmark['initial_gt_yaw_tolerance_rad']),
    )
    return warnings


def evaluate_files(
    navigation_path: Path, ground_truth_path: Path, benchmark_path: Path, scenarios_path: Path,
    event_path: Path | None = None, *, task5_mode: str = 'acceptance',
) -> dict[str, object]:
    if task5_mode not in ('acceptance', 'discovery'):
        raise ValueError(f'unsupported Task 5 evaluation mode: {task5_mode!r}')
    navigation = json.loads(navigation_path.read_text(encoding='utf-8'))
    ground_truth = json.loads(ground_truth_path.read_text(encoding='utf-8'))
    benchmark = load_yaml(benchmark_path)['benchmark']
    scenarios = load_yaml(scenarios_path)['scenarios']
    if not isinstance(benchmark, dict) or not isinstance(scenarios, dict):
        raise ValueError('benchmark configuration is malformed')
    name = navigation.get('scenario')
    scenario = scenarios.get(name)
    if not isinstance(name, str) or not isinstance(scenario, dict):
        raise ValueError('navigation evidence names no frozen scenario')
    task5 = scenario.get('task5')
    task5_kind = task5.get('kind') if isinstance(task5, dict) else None
    if scenario.get('expected') == 'canceled':
        if task5_kind != 'goal_cancel':
            raise ValueError('canceled terminal behavior is only supported for Task 5 goal_cancel')
        metrics = evaluate_canceled_navigation_run(navigation, ground_truth, benchmark)
        failures = []
        final_stop = metrics['final_stop']
        if not isinstance(final_stop, dict):
            raise ValueError('goal-cancel final-stop metrics are malformed')
        if final_stop['ground_truth_translation_m'] > float(benchmark['final_stop_translation_m']):
            failures.append('final_stop_ground_truth_translation')
        if final_stop['ground_truth_yaw_rad'] > float(benchmark['final_stop_yaw_rad']):
            failures.append('final_stop_ground_truth_yaw')
        return {
            'status': 'PASS' if not failures else 'FAIL',
            'acceptance_scope': 'task5_goal_cancel',
            'benchmark': benchmark['name'],
            'scenario': name,
            'navigation_evidence_path': str(navigation_path),
            'ground_truth_evidence_path': str(ground_truth_path),
            'failures': failures,
            'metrics': metrics,
        }
    event = None
    if isinstance(task5, dict):
        if event_path is None:
            raise ValueError('Task 5 evaluation requires obstacle-event evidence')
        event = json.loads(event_path.read_text(encoding='utf-8'))
        if not isinstance(event, dict):
            raise ValueError('Task 5 obstacle-event evidence must be a mapping')
    if scenario.get('expected') == 'safe_failure':
        if not isinstance(task5, dict) or event is None:
            raise ValueError('safe_failure is only supported for a Task 5 scenario')
        metrics = evaluate_dynamic_obstacle_event(
            navigation, ground_truth, benchmark, scenario, event,
            healthy_metrics=None, evaluation_mode=task5_mode,
        )
        return {
            'status': 'MEASURED' if task5_mode == 'discovery' else 'PASS',
            'acceptance_scope': 'task5_fully_blocked_discovery' if task5_mode == 'discovery' else 'task5_fully_blocked_safe_failure',
            'benchmark': benchmark['name'],
            'scenario': name,
            'navigation_evidence_path': str(navigation_path),
            'ground_truth_evidence_path': str(ground_truth_path),
            'obstacle_event_evidence_path': str(event_path),
            'failures': [],
            'metrics': metrics,
        }
    metrics = evaluate_navigation_run(navigation, ground_truth, benchmark)
    failures = []
    inherited_warnings = inherited_healthy_baseline_warnings(metrics, benchmark)
    if not isinstance(task5, dict):
        failures.extend(warning['id'] for warning in inherited_warnings)
    if task5_kind != 'temporary_blocked_recovery' and metrics['recovery_count'] != 0:
        failures.append('recovery_count')
    footprint_sweep = navigation.get('initial_path_full_footprint_sweep')
    if not isinstance(footprint_sweep, dict) or footprint_sweep.get('safe') is not True:
        failures.append('initial_path_full_footprint_safety')
    if not all(bool(value) for value in metrics['final_stop'].values() if isinstance(value, bool)):
        failures.append('final_stop')
    if metrics['final_stop']['ground_truth_translation_m'] > float(benchmark['final_stop_translation_m']):
        failures.append('final_stop_ground_truth_translation')
    if metrics['final_stop']['ground_truth_yaw_rad'] > float(benchmark['final_stop_yaw_rad']):
        failures.append('final_stop_ground_truth_yaw')
    multi = scenario.get('benchmark')
    if isinstance(multi, dict):
        if metrics['initial_global_path_length_m'] < float(multi['min_initial_path_length_m']):
            failures.append('multi_turn_path_length')
        if metrics['initial_path_turn_count'] < int(multi['min_initial_path_turn_count']):
            failures.append('multi_turn_heading_changes')
    if isinstance(task5, dict):
        assert event is not None
        robustness_metrics = evaluate_dynamic_obstacle_event(
            navigation, ground_truth, benchmark, scenario, event,
            healthy_metrics=metrics, evaluation_mode='acceptance',
        )
        robustness_metrics['inherited_healthy_baseline_warnings'] = inherited_warnings
        metrics = robustness_metrics
    scope = None
    status = 'PASS' if not failures else 'FAIL'
    if isinstance(task5, dict):
        if task5_kind == 'temporary_blocked_recovery':
            if task5_mode == 'discovery' and not failures:
                # The first host run measures/finalizes the frozen timeline;
                # it never grants the later formal acceptance by itself.
                status = 'MEASURED'
                scope = 'task5_temporary_obstacle_recovery_discovery'
            else:
                scope = 'task5_temporary_obstacle_recovery'
        else:
            scope = 'task5_dynamic_obstacle_engineering'
    return {
        'status': status,
        **({'acceptance_scope': scope} if scope is not None else {}),
        'benchmark': benchmark['name'],
        'scenario': name,
        'navigation_evidence_path': str(navigation_path),
        'ground_truth_evidence_path': str(ground_truth_path),
        **({'obstacle_event_evidence_path': str(event_path)} if event_path is not None else {}),
        'failures': failures,
        **({'warnings': inherited_warnings} if isinstance(task5, dict) else {}),
        'metrics': metrics,
    }


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigation-path', required=True)
    parser.add_argument('--ground-truth-path', required=True)
    parser.add_argument('--output-path', required=True)
    parser.add_argument('--benchmark-config', required=True)
    parser.add_argument('--scenarios-file', required=True)
    parser.add_argument('--event-path')
    parser.add_argument('--task5-mode', choices=['acceptance', 'discovery'], default='acceptance')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    result = {'status': 'FAIL'}
    try:
        result = evaluate_files(
            Path(arguments.navigation_path), Path(arguments.ground_truth_path),
            Path(arguments.benchmark_config), Path(arguments.scenarios_file),
            Path(arguments.event_path) if arguments.event_path else None,
            task5_mode=arguments.task5_mode,
        )
    except Exception as error:
        result['failure_kind'] = classify_evaluation_error(error)
        result['error'] = str(error)
    Path(arguments.output_path).parent.mkdir(parents=True, exist_ok=True)
    Path(arguments.output_path).write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['status'] == 'PASS' else 1
