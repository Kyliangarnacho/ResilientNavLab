"""Fresh-process orchestration for the frozen Phase 10 Task 4 benchmark."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone

from ament_index_python.packages import get_package_share_directory
import yaml

from navigation_benchmark_evaluator import classify_evaluation_error, evaluate_files
from navigation_benchmark_metrics import summarize_runs


POST_GOAL_SETTLE_ALLOWANCE_WALL_SEC = 8.0
TRIAL_RESULT_GRACE_WALL_SEC = 15.0
TRIAL_TEARDOWN_TIMEOUT_SEC = 30.0
TRIAL_TEARDOWN_GRACE_SEC = 12.0


def json_path_default(value: object) -> str:
    """Convert only filesystem Path values at the JSON evidence boundary."""
    if isinstance(value, Path):
        return str(value)
    raise TypeError(
        f'Object of type {type(value).__name__} is not JSON serializable; '
        'only pathlib.Path is accepted as a JSON boundary conversion'
    )


def write_json_durable(path: Path, value: object) -> None:
    """Persist trial evidence before a later fresh process can begin."""
    path.write_text(
        json.dumps(value, default=json_path_default, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    with path.open('rb') as stream:
        os.fsync(stream.fileno())


def sync_file_if_present(path: Path) -> None:
    if path.is_file():
        with path.open('rb') as stream:
            os.fsync(stream.fileno())


def sync_tree_if_present(path: Path) -> None:
    if not path.is_dir():
        return
    for child in path.rglob('*'):
        sync_file_if_present(child)


def process_group_processes(process_group_id: int) -> list[dict[str, object]]:
    """List unreaped descendants of this trial's dedicated launch session."""
    matches: list[dict[str, object]] = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = entry.joinpath('stat').read_text(encoding='utf-8')
            command = entry.joinpath('cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            fields = stat[stat.rfind(')') + 2:].split()
            # /proc/<pid>/stat: state, ppid, pgrp after the final ')'.
            if int(fields[2]) != process_group_id:
                continue
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError, IndexError):
            continue
        matches.append({'pid': int(entry.name), 'command': command})
    return sorted(matches, key=lambda value: int(value['pid']))


def gz_partition_processes(partition: str) -> list[dict[str, object]]:
    """Return every process explicitly carrying this trial's GZ partition."""
    matches: list[dict[str, object]] = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            environment = entry.joinpath('environ').read_bytes().split(b'\0')
            command = entry.joinpath('cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if f'GZ_PARTITION={partition}'.encode() in environment:
            matches.append({'pid': int(entry.name), 'command': command})
    return sorted(matches, key=lambda value: int(value['pid']))


def _signal_processes(processes: list[dict[str, object]], signal_value: int) -> list[int]:
    signaled: list[int] = []
    for process in processes:
        pid = int(process['pid'])
        try:
            os.kill(pid, signal_value)
            signaled.append(pid)
        except ProcessLookupError:
            pass
    return signaled


def teardown_barrier(
    process_group_id: int, partition: str, run_dir: Path,
    additional_evidence_paths: tuple[Path, ...] = (),
) -> dict[str, object]:
    """Wait for one trial's process tree and evidence to become self-contained.

    This is deliberately a condition barrier, not an inter-trial delay: the
    next launch is blocked until no process in the dedicated launch session or
    Gazebo partition remains and all evidence files have been fsync'd.
    """
    initial_group = process_group_processes(process_group_id)
    initial_partition = gz_partition_processes(partition)
    signaled_group = []
    if initial_group:
        try:
            os.killpg(process_group_id, signal.SIGINT)
            signaled_group = [int(item['pid']) for item in initial_group]
        except ProcessLookupError:
            pass
    signaled_partition = _signal_processes(initial_partition, signal.SIGINT)
    started = time.monotonic()
    deadline = started + TRIAL_TEARDOWN_TIMEOUT_SEC
    graceful_deadline = started + TRIAL_TEARDOWN_GRACE_SEC
    term_sent = False
    while time.monotonic() < deadline:
        remaining_group = process_group_processes(process_group_id)
        remaining_partition = gz_partition_processes(partition)
        if not remaining_group and not remaining_partition:
            sync_file_if_present(run_dir / 'trial_manifest.json')
            sync_file_if_present(run_dir / 'launch.log')
            sync_file_if_present(run_dir / 'navigation.json')
            sync_file_if_present(run_dir / 'ground_truth.json')
            sync_file_if_present(run_dir / 'initial_pose.json')
            for path in additional_evidence_paths:
                sync_file_if_present(path)
            sync_tree_if_present(run_dir / 'diagnostics')
            sync_tree_if_present(run_dir / 'ros_launch_logs')
            return {
                'complete': True,
                'process_group_id': process_group_id,
                'gz_partition': partition,
                'signaled_group_pids': signaled_group,
                'signaled_partition_pids': signaled_partition,
                'remaining_group_processes': [],
                'remaining_partition_processes': [],
                'evidence_flushed': True,
            }
        if not term_sent and time.monotonic() >= graceful_deadline:
            if remaining_group:
                try:
                    os.killpg(process_group_id, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            _signal_processes(remaining_partition, signal.SIGTERM)
            term_sent = True
        time.sleep(0.2)
    return {
        'complete': False,
        'process_group_id': process_group_id,
        'gz_partition': partition,
        'signaled_group_pids': signaled_group,
        'signaled_partition_pids': signaled_partition,
        'remaining_group_processes': process_group_processes(process_group_id),
        'remaining_partition_processes': gz_partition_processes(partition),
        'evidence_flushed': False,
    }


def load_config(path: Path) -> dict[str, object]:
    value = yaml.safe_load(path.read_text(encoding='utf-8'))
    benchmark = value.get('benchmark') if isinstance(value, dict) else None
    if not isinstance(benchmark, dict):
        raise ValueError('benchmark config has no benchmark mapping')
    return benchmark


def stop_child(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10.0)


def scenario_action_timeout(path: Path, scenario: str) -> float:
    loaded = yaml.safe_load(path.read_text(encoding='utf-8'))
    scenarios = loaded.get('scenarios') if isinstance(loaded, dict) else None
    value = scenarios.get(scenario) if isinstance(scenarios, dict) else None
    action = value.get('navigate_to_pose') if isinstance(value, dict) else None
    timeout = action.get('action_timeout_sec') if isinstance(action, dict) else None
    if not isinstance(timeout, (int, float)) or isinstance(timeout, bool) or timeout <= 0.0:
        raise ValueError(f'{scenario} has no positive NavigateToPose action timeout')
    return float(timeout)


def derived_trial_timeout(readiness_timeout_sec: float, action_timeout_sec: float) -> float:
    """Bound a trial from its declared startup/action contracts, not a magic cap."""
    return readiness_timeout_sec + action_timeout_sec + POST_GOAL_SETTLE_ALLOWANCE_WALL_SEC + TRIAL_RESULT_GRACE_WALL_SEC


def infer_infrastructure_failure(launch_log: Path, initial_pose: Path) -> str:
    """Classify retained launch evidence with the earliest missing dependency first."""
    text = launch_log.read_text(encoding='utf-8', errors='replace') if launch_log.exists() else ''
    initial = initial_pose.read_text(encoding='utf-8', errors='replace') if initial_pose.exists() else ''
    if (
        'Waiting for clock to start' in text
        or '/world/resilient_lab/create' in text
        or 'Gazebo world' in text and 'service' in text
    ):
        return 'infrastructure_gazebo_clock'
    if 'failed to send response to /map_server/change_state' in text or 'async_send_request failed' in text:
        return 'infrastructure_nav2_lifecycle_service'
    if 'no map-frame /amcl_pose observed' in initial or 'Timed out waiting for transform' in text:
        return 'infrastructure_localization_tf'
    return 'infrastructure_process'


def run_trial(
    *, scenario: str, run_dir: Path, timeout_sec: float, record_diagnostics: bool,
    ros_domain_id: int, readiness_timeout_sec: float,
    launch_file: str = 'phase10_navigation_benchmark_trial.launch.py',
    extra_launch_arguments: tuple[str, ...] = (),
    additional_evidence_paths: dict[str, Path] | None = None,
    partition_prefix: str = 'resilient_nav_phase10_task4',
) -> dict[str, object]:
    navigation = run_dir / 'navigation.json'
    ground_truth = run_dir / 'ground_truth.json'
    diagnostics = run_dir / 'diagnostics'
    initial_pose = run_dir / 'initial_pose.json'
    launch_log = run_dir / 'launch.log'
    ros_launch_log_dir = run_dir / 'ros_launch_logs'
    partition = f'{partition_prefix}_{run_dir.name}'
    additional_evidence_paths = dict(additional_evidence_paths or {})
    command = [
        'ros2', 'launch', 'resilient_nav_navigation', launch_file,
        f'scenario:={scenario}', f'navigation_output:={navigation}',
        f'ground_truth_output:={ground_truth}', f'diagnostics_output:={diagnostics}',
        f'initial_pose_result:={initial_pose}',
        f'record_diagnostics:={str(record_diagnostics).lower()}',
        'use_rviz:=false',
        # A terminated `gz sim` client can leave its server alive.  Each
        # trial therefore owns a Transport partition and DDS domain.
        f'gz_partition:={partition}',
        f'ros_domain_id:={ros_domain_id}',
        f'readiness_timeout_sec:={readiness_timeout_sec}',
        *extra_launch_arguments,
    ]
    trial_context = {
        'scenario': scenario,
        'gz_partition': partition,
        'ros_domain_id': ros_domain_id,
        'readiness_timeout_wall_sec': readiness_timeout_sec,
        'trial_timeout_wall_sec': timeout_sec,
        'record_diagnostics': record_diagnostics,
        'navigation_path': str(navigation),
        'ground_truth_path': str(ground_truth),
        'initial_pose_path': str(initial_pose),
        'launch_log_path': str(launch_log),
        'ros_launch_log_dir': str(ros_launch_log_dir),
        'additional_evidence_paths': {
            name: str(path) for name, path in additional_evidence_paths.items()
        },
        'command': command,
    }
    write_json_durable(run_dir / 'trial_manifest.json', trial_context)
    timed_out = False
    with launch_log.open('w', encoding='utf-8') as stream:
        launch_environment = os.environ.copy()
        # Keep launch's own ROS logs inside this trial's durable evidence
        # directory.  This avoids dependence on a writable user home and
        # makes the process/session boundary self-contained.
        launch_environment['ROS_LOG_DIR'] = str(ros_launch_log_dir)
        process = subprocess.Popen(
            command, stdout=stream, stderr=subprocess.STDOUT, text=True,
            start_new_session=True, env=launch_environment,
        )
        try:
            return_code = process.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            timed_out = True
            stop_child(process)
            return_code = None
        except BaseException:
            # An interrupted batch must not leave a Gazebo/Nav2 process group
            # behind to contaminate a later fresh-process trial.
            stop_child(process)
            teardown_barrier(
                process.pid, partition, run_dir,
                tuple(additional_evidence_paths.values()),
            )
            raise
    # The launch parent has exited and its log stream is closed before this
    # barrier runs.  Do not begin another fresh process while any descendant,
    # partition process, or unflushed evidence remains.
    cleanup = teardown_barrier(
        process.pid, partition, run_dir, tuple(additional_evidence_paths.values())
    )
    if not cleanup['complete']:
        return {
            'status': 'FAIL', 'scenario': scenario,
            'failure_kind': 'infrastructure_teardown',
            'error': 'trial teardown barrier did not complete',
            'return_code': return_code, 'process_cleanup': cleanup,
            'trial_manifest_path': str(run_dir / 'trial_manifest.json'),
        }
    if timed_out:
        return {
            'status': 'FAIL', 'scenario': scenario,
            'failure_kind': 'infrastructure_process',
            'error': 'trial launch wall timeout', 'return_code': None,
            'process_cleanup': cleanup, 'trial_manifest_path': str(run_dir / 'trial_manifest.json'),
        }
    required_evidence = {
        'navigation': navigation,
        'ground_truth': ground_truth,
        **additional_evidence_paths,
    }
    missing_evidence = [name for name, path in required_evidence.items() if not path.exists()]
    if missing_evidence:
        return {
            'status': 'FAIL', 'scenario': scenario,
            'failure_kind': infer_infrastructure_failure(launch_log, initial_pose),
            'error': f"trial did not produce required evidence: {', '.join(missing_evidence)}",
            'return_code': return_code, 'process_cleanup': cleanup,
            'trial_manifest_path': str(run_dir / 'trial_manifest.json'),
        }
    return {
        'status': 'MEASURED', 'scenario': scenario, 'return_code': return_code,
        'navigation_path': navigation, 'ground_truth_path': ground_truth,
        'initial_pose_path': initial_pose, 'launch_log_path': launch_log,
        'trial_manifest_path': str(run_dir / 'trial_manifest.json'), 'process_cleanup': cleanup,
        **{f'{name}_path': path for name, path in additional_evidence_paths.items()},
    }


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    package = Path(get_package_share_directory('resilient_nav_navigation'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-root', required=True)
    parser.add_argument('--repetitions', type=int, default=None)
    parser.add_argument(
        '--scenarios', nargs='+', default=None,
        help='Ordered prefix of the frozen scenario order (for bounded diagnosis only).',
    )
    parser.add_argument('--trial-timeout-wall-sec', type=float, default=None)
    parser.add_argument('--record-diagnostics', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument('--benchmark-config', default=str(package / 'config' / 'healthy_navigation_benchmark.yaml'))
    parser.add_argument('--scenarios-file', default=str(package / 'config' / 'planner_smoke_scenarios.yaml'))
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    benchmark_path = Path(arguments.benchmark_config)
    scenarios_path = Path(arguments.scenarios_file)
    config = load_config(benchmark_path)
    repetitions = arguments.repetitions if arguments.repetitions is not None else int(config['repetitions'])
    if repetitions < 1:
        raise ValueError('repetitions must be positive')
    frozen_scenarios = list(config['scenario_order'])
    selected_scenarios = arguments.scenarios if arguments.scenarios is not None else frozen_scenarios
    if (
        not selected_scenarios
        or selected_scenarios != frozen_scenarios[:len(selected_scenarios)]
    ):
        raise ValueError('--scenarios must be a non-empty ordered prefix of the frozen scenario order')
    readiness_timeout_sec = float(config['readiness_timeout_wall_sec'])
    if readiness_timeout_sec <= 0.0:
        raise ValueError('readiness_timeout_wall_sec must be positive')
    if int(config['infrastructure_start_attempts']) != 1:
        raise ValueError('formal Task 4 batches require exactly one infrastructure attempt')
    if arguments.trial_timeout_wall_sec is not None and arguments.trial_timeout_wall_sec <= 0.0:
        raise ValueError('--trial-timeout-wall-sec must be positive')
    root = Path(arguments.results_root)
    root.mkdir(parents=True, exist_ok=True)
    manifest = {
        'benchmark': config['name'],
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'repetitions': repetitions,
        'scenario_order': selected_scenarios,
        'frozen_scenario_order': frozen_scenarios,
        'record_diagnostics': arguments.record_diagnostics,
        'readiness_timeout_wall_sec': readiness_timeout_sec,
        'infrastructure_start_attempts': 1,
        'trial_timeout_override_wall_sec': arguments.trial_timeout_wall_sec,
        'attempts': [],
        'stopped_early': None,
    }
    results_by_scenario: dict[str, list[dict[str, object]]] = {name: [] for name in selected_scenarios}
    physical_attempt_index = 0
    stopped_early = False
    for repetition in range(1, repetitions + 1):
        for scenario in selected_scenarios:
            logical_run_id = f'{scenario}-r{repetition:02d}'
            physical_attempt_index += 1
            run_dir = root / 'runs' / logical_run_id
            run_dir.mkdir(parents=True, exist_ok=False)
            action_timeout_sec = scenario_action_timeout(scenarios_path, scenario)
            trial_timeout_sec = (
                arguments.trial_timeout_wall_sec
                if arguments.trial_timeout_wall_sec is not None
                else derived_trial_timeout(readiness_timeout_sec, action_timeout_sec)
            )
            attempt = run_trial(
                scenario=scenario, run_dir=run_dir,
                timeout_sec=trial_timeout_sec,
                record_diagnostics=arguments.record_diagnostics,
                ros_domain_id=100 + physical_attempt_index,
                readiness_timeout_sec=readiness_timeout_sec,
            )
            if attempt['status'] == 'MEASURED':
                try:
                    result = evaluate_files(
                        Path(attempt['navigation_path']), Path(attempt['ground_truth_path']),
                        benchmark_path, scenarios_path,
                    )
                except Exception as error:
                    result = {
                        'status': 'FAIL',
                        'scenario': scenario,
                        'failure_kind': classify_evaluation_error(error),
                        'error': str(error),
                    }
                # Evaluation owns navigation/GT verdicts, while run_trial
                # owns the fresh-process teardown fact.  Preserve both in
                # durable per-run and batch evidence.
                result['process_cleanup'] = attempt['process_cleanup']
                result['trial_manifest_path'] = attempt['trial_manifest_path']
                result['launch_log_path'] = attempt['launch_log_path']
            else:
                result = attempt
            launch_failure_kind = infer_infrastructure_failure(
                run_dir / 'launch.log', run_dir / 'initial_pose.json'
            )
            if (
                result.get('failure_kind') == 'infrastructure_localization_tf'
                and launch_failure_kind == 'infrastructure_nav2_lifecycle_service'
            ):
                result['readiness_failure_kind'] = result['failure_kind']
                result['failure_kind'] = launch_failure_kind
            if result['status'] != 'PASS' and 'failure_kind' not in result:
                result['failure_kind'] = 'navigation_evaluation'
            record = {
                'logical_run_id': logical_run_id,
                'infrastructure_attempt': 1,
                'run_id': logical_run_id,
                'action_timeout_sec': action_timeout_sec,
                'trial_timeout_wall_sec': trial_timeout_sec,
                **result,
            }
            write_json_durable(run_dir / 'result.json', record)
            manifest['attempts'].append(record)
            results_by_scenario[scenario].append(result)
            if result['status'] != 'PASS':
                manifest['stopped_early'] = record
                stopped_early = True
            write_json_durable(root / 'manifest.json', manifest)
            if stopped_early:
                break
        if stopped_early:
            break
    summaries = {
        scenario: summarize_runs(values) if values else {
            'attempt_count': 0, 'pass_count': 0, 'success_rate': 0.0,
        }
        for scenario, values in results_by_scenario.items()
    }
    benchmark_summary = {
        'benchmark': config['name'],
        'status': 'PASS' if not stopped_early and all(
            summary['pass_count'] == repetitions for summary in summaries.values()
        ) else 'FAIL',
        'scenario_summaries': summaries,
        'manifest_path': str(root / 'manifest.json'),
        'stopped_early': manifest['stopped_early'],
    }
    write_json_durable(root / 'benchmark_summary.json', benchmark_summary)
    print(json.dumps(
        benchmark_summary, default=json_path_default, indent=2, sort_keys=True
    ))
    return 0 if benchmark_summary['status'] == 'PASS' else 1
