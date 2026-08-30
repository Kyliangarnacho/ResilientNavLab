"""Run exactly one bounded, fresh-process Phase 10 Task 5 trial.

This is a thin Task 5 adapter over the Task 4 trial supervisor.  It owns a
single launch session, waits for its condition-based teardown barrier, then
evaluates fully flushed navigation, Ground Truth, and scenario-specific event
evidence when the scenario has an environment event. It deliberately provides
no retry or timing workaround.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from navigation_benchmark_batch import (
    derived_trial_timeout,
    json_path_default,
    load_config,
    run_trial,
    scenario_action_timeout,
    write_json_durable,
)
from navigation_benchmark_evaluator import classify_evaluation_error, evaluate_files


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    package = Path(get_package_share_directory('resilient_nav_navigation'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--results-dir', required=True)
    parser.add_argument('--ros-domain-id', type=int, default=201)
    parser.add_argument(
        '--evaluation-mode', choices=['acceptance', 'discovery'], default='acceptance',
        help='Task 5 discovery records a measured branch; it never grants acceptance.',
    )
    parser.add_argument('--trial-timeout-wall-sec', type=float, default=None)
    parser.add_argument('--record-diagnostics', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        '--use-rviz', action=argparse.BooleanOptionalAction, default=False,
        help='Start the existing Phase 10 navigation RViz view for one manual trial.',
    )
    parser.add_argument('--benchmark-config', default=str(package / 'config' / 'healthy_navigation_benchmark.yaml'))
    parser.add_argument('--scenarios-file', default=str(package / 'config' / 'navigation_robustness_scenarios.yaml'))
    return parser.parse_args(argv)


def _result_from_attempt(
    attempt: dict[str, object], *, benchmark_path: Path, scenarios_path: Path, evaluation_mode: str,
    event_path: Path | None,
) -> dict[str, object]:
    if attempt['status'] != 'MEASURED':
        return dict(attempt)
    try:
        result = evaluate_files(
            Path(attempt['navigation_path']), Path(attempt['ground_truth_path']),
            benchmark_path, scenarios_path, event_path,
            task5_mode=evaluation_mode,
        )
    except Exception as error:
        result = {
            'status': 'FAIL',
            'scenario': attempt['scenario'],
            'failure_kind': classify_evaluation_error(error),
            'error': str(error),
        }
    result.update({
        'process_cleanup': attempt['process_cleanup'],
        'trial_manifest_path': attempt['trial_manifest_path'],
        'launch_log_path': attempt['launch_log_path'],
    })
    if event_path is not None:
        result['obstacle_event_path'] = event_path
    return result


def scenario_use_recovery(scenarios_path: Path, scenario_name: str) -> bool:
    """Read the frozen scenario profile before creating the fresh launch."""
    loaded = yaml.safe_load(scenarios_path.read_text(encoding='utf-8'))
    scenarios = loaded.get('scenarios') if isinstance(loaded, dict) else None
    scenario = scenarios.get(scenario_name) if isinstance(scenarios, dict) else None
    if not isinstance(scenario, dict):
        raise ValueError(f'{scenario_name} is not a frozen Task 5 scenario')
    profile = scenario.get('navigation_profile', 'baseline')
    if profile not in ('baseline', 'recovery'):
        raise ValueError(f'{scenario_name} has unsupported navigation_profile {profile!r}')
    return profile == 'recovery'


def scenario_requires_obstacle_event(scenarios_path: Path, scenario_name: str) -> bool:
    """Keep Goal Cancel on the existing trial without starting Gazebo entity services."""
    loaded = yaml.safe_load(scenarios_path.read_text(encoding='utf-8'))
    scenarios = loaded.get('scenarios') if isinstance(loaded, dict) else None
    scenario = scenarios.get(scenario_name) if isinstance(scenarios, dict) else None
    task5 = scenario.get('task5') if isinstance(scenario, dict) else None
    if not isinstance(task5, dict) or not isinstance(task5.get('kind'), str):
        raise ValueError(f'{scenario_name} lacks a Task 5 kind')
    return task5['kind'] != 'goal_cancel'


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    if arguments.ros_domain_id < 1:
        raise ValueError('--ros-domain-id must be positive')
    if arguments.trial_timeout_wall_sec is not None and arguments.trial_timeout_wall_sec <= 0.0:
        raise ValueError('--trial-timeout-wall-sec must be positive')
    benchmark_path = Path(arguments.benchmark_config)
    scenarios_path = Path(arguments.scenarios_file)
    benchmark = load_config(benchmark_path)
    readiness_timeout_sec = float(benchmark['readiness_timeout_wall_sec'])
    if readiness_timeout_sec <= 0.0:
        raise ValueError('readiness_timeout_wall_sec must be positive')
    action_timeout_sec = scenario_action_timeout(scenarios_path, arguments.scenario)
    use_recovery = scenario_use_recovery(scenarios_path, arguments.scenario)
    requires_obstacle_event = scenario_requires_obstacle_event(scenarios_path, arguments.scenario)
    timeout_sec = (
        float(arguments.trial_timeout_wall_sec)
        if arguments.trial_timeout_wall_sec is not None
        else derived_trial_timeout(readiness_timeout_sec, action_timeout_sec)
    )
    run_dir = Path(arguments.results_dir)
    run_dir.mkdir(parents=True, exist_ok=False)
    event_path = run_dir / 'obstacle_event.json' if requires_obstacle_event else None
    supervisor_manifest = {
        'benchmark': 'phase10_task5_dynamic_environment',
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'scenario': arguments.scenario,
        'supervisor_pid': os.getpid(),
        'ros_domain_id': arguments.ros_domain_id,
        'readiness_timeout_wall_sec': readiness_timeout_sec,
        'action_timeout_sec': action_timeout_sec,
        'trial_timeout_wall_sec': timeout_sec,
        'record_diagnostics': arguments.record_diagnostics,
        'use_rviz': arguments.use_rviz,
        'evaluation_mode': arguments.evaluation_mode,
        'navigation_profile': 'recovery' if use_recovery else 'baseline',
        'obstacle_event_enabled': requires_obstacle_event,
        'infrastructure_start_attempts': 1,
        'status': 'RUNNING',
    }
    write_json_durable(run_dir / 'manifest.json', supervisor_manifest)
    attempt = run_trial(
        scenario=arguments.scenario,
        run_dir=run_dir,
        timeout_sec=timeout_sec,
        record_diagnostics=arguments.record_diagnostics,
        ros_domain_id=arguments.ros_domain_id,
        readiness_timeout_sec=readiness_timeout_sec,
        launch_file='phase10_navigation_robustness_trial.launch.py',
        extra_launch_arguments=(
            f'event_output:={event_path if event_path is not None else run_dir / "unused_obstacle_event.json"}',
            f'scenarios_file:={scenarios_path}',
            f"use_recovery:={'true' if use_recovery else 'false'}",
            f"enable_obstacle_event:={'true' if requires_obstacle_event else 'false'}",
            f"use_rviz:={'true' if arguments.use_rviz else 'false'}",
        ),
        additional_evidence_paths=(
            {'obstacle_event': event_path} if event_path is not None else {}
        ),
        partition_prefix='resilient_nav_phase10_task5',
    )
    result = _result_from_attempt(
        attempt, benchmark_path=benchmark_path, scenarios_path=scenarios_path,
        evaluation_mode=arguments.evaluation_mode, event_path=event_path,
    )
    write_json_durable(run_dir / 'result.json', result)
    supervisor_manifest.update({
        'status': result['status'],
        'result_path': run_dir / 'result.json',
        'trial_manifest_path': run_dir / 'trial_manifest.json',
        'launch_log_path': run_dir / 'launch.log',
        'process_cleanup': result.get('process_cleanup'),
    })
    if event_path is not None:
        supervisor_manifest['obstacle_event_path'] = event_path
    write_json_durable(run_dir / 'manifest.json', supervisor_manifest)
    print(json.dumps(result, default=json_path_default, indent=2, sort_keys=True))
    return 0 if result['status'] in ('PASS', 'MEASURED') else 1


if __name__ == '__main__':
    raise SystemExit(main())
