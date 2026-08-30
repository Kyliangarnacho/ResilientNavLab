"""One bounded, GT-free NavigateToPose run for the Task 4 benchmark."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy

from navigate_to_pose_probe import (
    NavigateToPoseProbe,
    run_scenario,
)
from planner_probe import path_length
from phase9_assets import verify_frozen_assets
from path_safety import RawCostmapGrid, full_footprint_path_sweep


POST_GOAL_SETTLE_SIM_SEC = 2.0
BASELINE_BT_XML = 'navigate_w_replanning_time.xml'
RECOVERY_BT_XML = 'navigate_to_pose_w_replanning_and_recovery.xml'


def wait_for_readiness_stage(
    probe: NavigateToPoseProbe,
    records: dict[str, object],
    deadline: float,
    name: str,
    predicate,
    *,
    poll_interval_sec: float = 1.0,
) -> dict[str, object]:
    """Observe one prerequisite without becoming a second lifecycle manager."""
    readiness = records['readiness']
    if not isinstance(readiness, dict):
        raise ValueError('benchmark readiness record is malformed')
    stages = readiness['stages']
    if not isinstance(stages, list):
        raise ValueError('benchmark readiness stages record is malformed')
    started = time.monotonic()
    next_attempt = started
    last_error = 'not yet observed'
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
        now = time.monotonic()
        if now < next_attempt:
            continue
        try:
            detail = predicate()
            if not isinstance(detail, dict):
                raise ValueError('readiness predicate did not return a mapping')
            stages.append({
                'name': name,
                'status': 'PASS',
                'elapsed_wall_sec': now - started,
                'detail': detail,
            })
            return detail
        except Exception as error:
            last_error = str(error)
            next_attempt = now + poll_interval_sec
    stages.append({
        'name': name,
        'status': 'TIMEOUT',
        'elapsed_wall_sec': time.monotonic() - started,
        'last_error': last_error,
    })
    raise TimeoutError(f'readiness/{name}: {last_error}')


def localization_readiness(probe: NavigateToPoseProbe) -> dict[str, object]:
    if probe.odom is None:
        raise ValueError('filtered odometry has not arrived')
    x, y, yaw = probe.map_pose()
    return {'map_to_base_footprint': {'x': x, 'y': y, 'yaw': yaw}}


def nav2_lifecycle_readiness(probe: NavigateToPoseProbe) -> dict[str, object]:
    manager = probe.navigation_manager_active()
    if not manager.get('active'):
        raise ValueError(f'Navigation Lifecycle Manager is not active: {manager}')
    return {'lifecycle_manager_navigation': manager}


def costmap_readiness(probe: NavigateToPoseProbe) -> dict[str, object]:
    missing = [
        name for name, value in (
            ('global_costmap_raw', probe.global_raw),
            ('local_costmap_raw', probe.local_raw),
        ) if value is None
    ]
    if missing:
        raise ValueError(f'no raw Costmap received: {missing}')
    return {
        'global_size': [probe.global_raw.metadata.size_x, probe.global_raw.metadata.size_y],
        'local_size': [probe.local_raw.metadata.size_x, probe.local_raw.metadata.size_y],
    }


def action_readiness(probe: NavigateToPoseProbe) -> dict[str, object]:
    if not probe.action_client.wait_for_server(timeout_sec=0.1):
        raise ValueError('NavigateToPose action server is unavailable')
    return {'action': 'navigate_to_pose'}

def load_scenario(path: Path, name: str) -> dict[str, object]:
    loaded = yaml.safe_load(path.read_text(encoding='utf-8'))
    scenarios = loaded.get('scenarios') if isinstance(loaded, dict) else None
    scenario = scenarios.get(name) if isinstance(scenarios, dict) else None
    if not isinstance(scenario, dict) or scenario.get('expected') not in ('success', 'safe_failure', 'canceled'):
        raise ValueError(f'{name} is not a frozen Task 5 benchmark scenario')
    if not isinstance(scenario.get('goal'), dict) or not isinstance(scenario.get('navigate_to_pose'), dict):
        raise ValueError(f'{name} lacks goal or NavigateToPose contract')
    return scenario


def navigation_profile(scenario: dict[str, object]) -> str:
    """Return the frozen server-side BT profile selected by a scenario."""
    profile = scenario.get('navigation_profile', 'baseline')
    if profile not in ('baseline', 'recovery'):
        raise ValueError(f'unsupported frozen navigation profile: {profile!r}')
    return str(profile)


def _plan_record(record: dict[str, object]) -> dict[str, object]:
    path = record['message']
    points = [
        {'x': pose.pose.position.x, 'y': pose.pose.position.y}
        for pose in path.poses
    ]
    return {
        'stamp_sec': path.header.stamp.sec + path.header.stamp.nanosec / 1e9,
        'frame_id': path.header.frame_id,
        'path_pose_count': len(path.poses),
        'path_length_m': path_length(path),
        'points': points,
    }


def _feedback_records(records: list[dict[str, object]]) -> list[dict[str, object]]:
    required = {'stamp_sec', 'x', 'y', 'yaw'}
    return [dict(record) for record in records if required.issubset(record)]


def wait_for_simulated_settle(probe: NavigateToPoseProbe, duration_sec: float) -> None:
    """Observe a fixed post-result interval; never use GT to decide it."""
    if not probe.clock_samples:
        raise ValueError('cannot observe post-goal stop without /clock')
    start = probe.clock_samples[-1]
    deadline = time.monotonic() + max(8.0, duration_sec * 4.0)
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
        if probe.clock_regressed:
            raise ValueError('/clock regressed during post-goal stop observation')
        if probe.clock_samples and probe.clock_samples[-1] - start >= duration_sec:
            return
    raise TimeoutError('simulation time did not advance through post-goal stop observation')


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--output-path', required=True)
    parser.add_argument('--timeout-sec', type=float, default=120.0)
    parser.add_argument(
        '--scenarios-file',
        default=str(Path(get_package_share_directory('resilient_nav_navigation')) / 'config' / 'planner_smoke_scenarios.yaml'),
    )
    # launch_ros appends ROS arguments after these application arguments.
    return parser.parse_known_args(argv)[0]


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    if not math.isfinite(arguments.timeout_sec) or arguments.timeout_sec <= 0.0:
        raise SystemExit('--timeout-sec must be finite and positive')
    output = Path(arguments.output_path)
    records: dict[str, object] = {
        'status': 'FAIL',
        'benchmark': 'phase10_healthy_navigation',
        'scenario': arguments.scenario,
        'zero_stop_messages': 0,
        'readiness': {'timeout_wall_sec': arguments.timeout_sec, 'stages': []},
    }
    rclpy.init(args=None)
    probe = NavigateToPoseProbe()
    try:
        scenario = load_scenario(Path(arguments.scenarios_file), arguments.scenario)
        deadline = time.monotonic() + arguments.timeout_sec
        # Infrastructure facts come first so a dead world/clock is not
        # misleadingly reported as a late Planner lifecycle transition.
        wait_for_readiness_stage(probe, records, deadline, 'clock', probe.clock_contract)
        localization = wait_for_readiness_stage(
            probe, records, deadline, 'localization_tf',
            lambda: localization_readiness(probe),
        )
        wait_for_readiness_stage(
            probe, records, deadline, 'nav2_lifecycle',
            lambda: nav2_lifecycle_readiness(probe),
        )
        wait_for_readiness_stage(probe, records, deadline, 'navigate_to_pose', lambda: action_readiness(probe))
        wait_for_readiness_stage(probe, records, deadline, 'costmaps', lambda: costmap_readiness(probe))
        if probe.node_names('global_costmap', '/global_costmap') != ['global_costmap']:
            raise ValueError('expected exactly one Planner-owned global_costmap')
        if probe.node_names('local_costmap', '/local_costmap') != ['local_costmap']:
            raise ValueError('expected exactly one Controller-owned local_costmap')
        forbidden = {'waypoint_follower', 'velocity_smoother'}
        found = sorted(name for name, _ in probe.get_node_names_and_namespaces() if name in forbidden)
        if found:
            raise ValueError(f'benchmark found forbidden nodes: {found}')
        records['clock'] = probe.clock_contract()
        records['phase9_asset_hashes'] = verify_frozen_assets(
            Path(get_package_share_directory('resilient_nav_slam')) / 'maps' / 'phase9'
        )
        records['goal'] = dict(scenario['goal'])
        records['initial_pose_map'] = dict(localization['map_to_base_footprint'])
        expected = scenario['expected']
        profile = navigation_profile(scenario)
        expected_xml = RECOVERY_BT_XML if profile == 'recovery' else BASELINE_BT_XML
        records['navigation_profile'] = profile
        records['expected_default_nav_to_pose_bt_xml'] = expected_xml
        behavior_servers = sorted(
            name for name, _ in probe.get_node_names_and_namespaces()
            if name == 'behavior_server'
        )
        if profile == 'recovery':
            if behavior_servers != ['behavior_server']:
                raise ValueError('recovery profile requires exactly one behavior_server')
        elif behavior_servers:
            raise ValueError(f'baseline profile found unexpected behavior_server: {behavior_servers}')
        records['bt_navigator_parameters'] = probe.bt_parameters(expected_xml)
        scenario_result = run_scenario(
            probe, arguments.scenario, scenario,
            allow_failure=expected == 'safe_failure',
            cancellation_contract=(
                scenario.get('task5', {}).get('cancel')
                if expected == 'canceled' and isinstance(scenario.get('task5'), dict)
                else None
            ),
            # Task 4 keeps the original strict live sweep.  In Task 5 the
            # pre-event route is historical evidence; the offline contract
            # requires a safe *post-detection* replan and must not reject it
            # for a transient scan/costmap observation before the event.
            strict_path_safety='task5' not in scenario,
            # Preserve Task 4 and Task 5.1 successful-navigation endpoint
            # validation. For Task 5.2's expected safe failure, retain a
            # terminal TF/feedback divergence as evidence: the last feedback
            # can precede the natural BT abort/cancel.
            terminal_tf_feedback_divergence_is_error=expected not in ('safe_failure', 'canceled'),
            recovery_contract='required' if profile == 'recovery' else 'forbidden',
        )
        wait_for_simulated_settle(probe, POST_GOAL_SETTLE_SIM_SEC)
        records['timing'] = {
            'goal_accepted_sim_sec': scenario_result['goal_accepted_sim_sec'],
            'action_result_sim_sec': scenario_result['action_result_sim_sec'],
            # run_scenario observes the required controller completion stop
            # before returning, so this timestamp bounds the final stop.
            'final_stop_sim_sec': probe.clock_samples[-1],
        }
        plans = [_plan_record(record) for record in probe.plan_messages]
        if not plans:
            raise ValueError('benchmark recorded no initial global Path for full-footprint evidence')
        observed_sweeps = scenario_result.get('full_footprint_path_evidence')
        if not isinstance(observed_sweeps, list) or not observed_sweeps:
            raise ValueError('benchmark recorded no time-associated Path safety evidence')
        initial_sweep = observed_sweeps[0].get('sweep')
        if not isinstance(initial_sweep, dict):
            raise ValueError('initial Path safety evidence is malformed')
        if 'task5' not in scenario and not initial_sweep['safe']:
            raise ValueError(
                'initial Nav2 Path fails full-footprint safety sweep: '
                f"{initial_sweep['first_violation']}"
            )
        records['scenario_result'] = scenario_result
        records['expected_terminal_behavior'] = expected
        records['feedback_samples'] = _feedback_records(probe.feedback_records)
        records['plans'] = plans
        records['received_controller_paths'] = [
            _plan_record(record) for record in probe.received_plan_messages
        ]
        records['initial_path_full_footprint_sweep'] = initial_sweep
        records['command_samples'] = list(probe.command_records)
        records['bt_events'] = list(probe.bt_events)
        records['final_stop'] = {
            'post_goal_settle_sim_sec': POST_GOAL_SETTLE_SIM_SEC,
            # Keep this separate from the Runner's later teardown-only zero
            # publishes.  run_scenario observed the Controller stream before
            # this record is assembled.
            'controller_zero_observed': scenario_result['controller_zero_observed'],
            'odometry_settled': True,
            'latest_odom_linear_x': probe.odom.twist.twist.linear.x,
            'latest_odom_angular_z': probe.odom.twist.twist.angular.z,
        }
        records['status'] = 'PASS'
    except Exception as error:
        records['error'] = str(error)
    finally:
        records['zero_stop_messages'] = probe.publish_zero_stop()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(records, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        probe.destroy_node()
        rclpy.shutdown()
    if records['status'] != 'PASS':
        print(json.dumps(records, indent=2, sort_keys=True), file=sys.stderr)
        return 1
    print(f'PASS: wrote frozen benchmark navigation evidence to {output}')
    return 0
