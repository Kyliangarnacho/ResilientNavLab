"""Pure, fail-closed metrics for the Phase 10 healthy navigation benchmark.

This module deliberately has no ROS imports.  Ground Truth is combined with
already-finished navigation evidence only; it is never available to Nav2 while
the robot is running.
"""

from __future__ import annotations

import math
from statistics import mean, median
from typing import Iterable, Mapping


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def finite_pose(sample: Mapping[str, object]) -> bool:
    try:
        return all(math.isfinite(float(sample[key])) for key in ('stamp_sec', 'x', 'y', 'yaw'))
    except (KeyError, TypeError, ValueError):
        return False


def validate_trajectory(
    samples: Iterable[Mapping[str, object]], name: str, *, collapse_equal_stamps: bool = False,
) -> list[dict[str, float]]:
    result = []
    previous = None
    for sample in samples:
        if not finite_pose(sample):
            raise ValueError(f'{name} contains an invalid pose sample')
        value = {key: float(sample[key]) for key in ('stamp_sec', 'x', 'y', 'yaw')}
        if previous is not None and value['stamp_sec'] < previous:
            raise ValueError(f'{name} timestamps are not strictly increasing')
        if previous is not None and value['stamp_sec'] == previous:
            if not collapse_equal_stamps:
                raise ValueError(f'{name} timestamps are not strictly increasing')
            # Action feedback can repeat a current_pose stamp while changing
            # distance estimates. Keep the latest complete record, but never
            # reorder time or coalesce Ground Truth samples.
            result[-1] = value
            continue
        previous = value['stamp_sec']
        result.append(value)
    if not result:
        raise ValueError(f'{name} is empty')
    return result


def apply_se2(sample: Mapping[str, float], transform: Mapping[str, object]) -> dict[str, float]:
    """Map an exact GT odom pose into the frozen map frame without fitting."""
    try:
        tx, ty, tyaw = (float(transform[key]) for key in ('x', 'y', 'yaw'))
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('invalid frozen map_from_gt_odom transform') from error
    if not all(math.isfinite(value) for value in (tx, ty, tyaw)):
        raise ValueError('non-finite frozen map_from_gt_odom transform')
    cosine, sine = math.cos(tyaw), math.sin(tyaw)
    return {
        'stamp_sec': float(sample['stamp_sec']),
        'x': tx + cosine * float(sample['x']) - sine * float(sample['y']),
        'y': ty + sine * float(sample['x']) + cosine * float(sample['y']),
        'yaw': normalize_angle(tyaw + float(sample['yaw'])),
    }


def nearest_sample(samples: list[dict[str, float]], stamp_sec: float, maximum_delta_sec: float) -> tuple[dict[str, float], float] | None:
    if not math.isfinite(stamp_sec) or not math.isfinite(maximum_delta_sec) or maximum_delta_sec < 0.0:
        raise ValueError('invalid timestamp alignment contract')
    nearest = min(samples, key=lambda item: abs(item['stamp_sec'] - stamp_sec))
    delta = abs(nearest['stamp_sec'] - stamp_sec)
    return (nearest, delta) if delta <= maximum_delta_sec else None


def polyline_length(points: Iterable[Mapping[str, object]]) -> float:
    values = [(float(item['x']), float(item['y'])) for item in points]
    if len(values) < 2:
        return 0.0
    return sum(math.dist(first, second) for first, second in zip(values, values[1:]))


def subsample_by_minimum_period(
    samples: list[dict[str, float]], minimum_period_sec: float,
) -> list[dict[str, float]]:
    """Keep observed samples at least a fixed time apart; no interpolation."""
    if not math.isfinite(minimum_period_sec) or minimum_period_sec <= 0.0:
        raise ValueError('invalid Ground Truth distance sample period')
    result = [samples[0]]
    last_stamp = samples[0]['stamp_sec']
    for sample in samples[1:-1]:
        if sample['stamp_sec'] - last_stamp >= minimum_period_sec:
            result.append(sample)
            last_stamp = sample['stamp_sec']
    if len(samples) > 1 and result[-1] is not samples[-1]:
        result.append(samples[-1])
    return result


def spatial_resample(points: Iterable[Mapping[str, object]], spacing_m: float = 0.10) -> list[tuple[float, float]]:
    """Interpolate a polyline onto a fixed spatial grid, preserving its end."""
    if not math.isfinite(spacing_m) or spacing_m <= 0.0:
        raise ValueError('invalid path spatial sample spacing')
    raw = [(float(item['x']), float(item['y'])) for item in points]
    if len(raw) < 2:
        return raw
    result, carry = [raw[0]], 0.0
    for previous, target in zip(raw, raw[1:]):
        cursor = previous
        segment = math.dist(cursor, target)
        while carry + segment >= spacing_m and segment > 1e-12:
            fraction = (spacing_m - carry) / segment
            cursor = (cursor[0] + fraction * (target[0] - cursor[0]), cursor[1] + fraction * (target[1] - cursor[1]))
            result.append(cursor)
            segment, carry = math.dist(cursor, target), 0.0
        carry += segment
    if math.dist(result[-1], raw[-1]) > 1e-9:
        result.append(raw[-1])
    return result


def _perpendicular_distance(point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]) -> float:
    length = math.dist(start, end)
    if length <= 1e-12:
        return math.dist(point, start)
    return abs((end[0] - start[0]) * (start[1] - point[1]) - (start[0] - point[0]) * (end[1] - start[1])) / length


def rdp_simplify(points: list[tuple[float, float]], epsilon_m: float = 0.05) -> list[tuple[float, float]]:
    """Fixed-parameter Ramer-Douglas-Peucker simplification."""
    if len(points) < 3:
        return points
    index, distance = max(((index, _perpendicular_distance(point, points[0], points[-1])) for index, point in enumerate(points[1:-1], 1)), key=lambda item: item[1])
    if distance <= epsilon_m:
        return [points[0], points[-1]]
    return rdp_simplify(points[:index + 1], epsilon_m)[:-1] + rdp_simplify(points[index:], epsilon_m)


def path_turn_geometry(points: Iterable[Mapping[str, object]]) -> dict[str, object]:
    sampled = spatial_resample(points, 0.10)
    simplified = rdp_simplify(sampled, 0.05)
    headings = [math.atan2(second[1] - first[1], second[0] - first[0]) for first, second in zip(simplified, simplified[1:]) if math.dist(first, second) > 1e-9]
    turns = sum(abs(normalize_angle(second - first)) >= math.radians(20.0) for first, second in zip(headings, headings[1:]))
    return {'spatial_sample_spacing_m': 0.10, 'rdp_epsilon_m': 0.05, 'sampled_point_count': len(sampled), 'simplified_point_count': len(simplified), 'turn_count': turns}


def heading_turn_count(points: Iterable[Mapping[str, object]]) -> int:
    """Count >=20 degree turns after frozen resampling and RDP simplification."""
    return int(path_turn_geometry(points)['turn_count'])


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        raise ValueError('cannot calculate a percentile of no values')
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower, upper = math.floor(index), math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def localization_metrics(
    feedback: Iterable[Mapping[str, object]], gt_map: list[dict[str, float]], *, maximum_delta_sec: float,
) -> dict[str, object]:
    valid_feedback = validate_trajectory(feedback, 'feedback', collapse_equal_stamps=True)
    position_errors: list[float] = []
    yaw_errors: list[float] = []
    deltas: list[float] = []
    aligned = []
    for estimate in valid_feedback:
        matched = nearest_sample(gt_map, estimate['stamp_sec'], maximum_delta_sec)
        if matched is None:
            continue
        truth, delta = matched
        position_errors.append(math.dist((estimate['x'], estimate['y']), (truth['x'], truth['y'])))
        yaw_errors.append(abs(normalize_angle(estimate['yaw'] - truth['yaw'])))
        deltas.append(delta)
        aligned.append({'stamp_sec': estimate['stamp_sec'], 'gt_stamp_sec': truth['stamp_sec'], 'position_error_m': position_errors[-1], 'yaw_error_rad': yaw_errors[-1], 'alignment_delta_sec': delta})
    coverage = len(aligned) / len(valid_feedback)
    if not aligned:
        raise ValueError('no feedback samples have Ground Truth within the frozen alignment window')
    return {
        'feedback_sample_count': len(valid_feedback),
        'aligned_sample_count': len(aligned),
        'alignment_coverage': coverage,
        'alignment_delta_sec': {'mean': mean(deltas), 'max': max(deltas)},
        'position_error_m': {'rmse': math.sqrt(mean([value * value for value in position_errors])), 'mean': mean(position_errors), 'median': median(position_errors), 'p95': percentile(position_errors, 0.95), 'max': max(position_errors), 'final': position_errors[-1]},
        'yaw_error_rad': {'rmse': math.sqrt(mean([value * value for value in yaw_errors])), 'mean': mean(yaw_errors), 'median': median(yaw_errors), 'p95': percentile(yaw_errors, 0.95), 'max': max(yaw_errors), 'final': yaw_errors[-1]},
        'aligned_samples': aligned,
    }


def evaluate_navigation_run(
    navigation: Mapping[str, object], ground_truth: Mapping[str, object], contract: Mapping[str, object],
) -> dict[str, object]:
    """Evaluate one completed run; missing evidence fails closed instead of guessing."""
    if navigation.get('status') != 'PASS':
        raise ValueError(f"navigation runner did not pass: {navigation.get('error', 'unknown error')}")
    invalid_gt = ground_truth.get('invalid_sample_count')
    if not isinstance(invalid_gt, int) or invalid_gt < 0:
        raise ValueError('Ground Truth evidence has an invalid invalid_sample_count')
    if invalid_gt > int(contract['maximum_invalid_gt_samples']):
        raise ValueError(f'Ground Truth evidence quality failure: invalid_sample_count={invalid_gt}')
    gt_raw = validate_trajectory(ground_truth.get('samples', []), 'ground_truth')
    gt_map = [apply_se2(sample, contract['map_from_gt_odom']) for sample in gt_raw]
    goal = navigation['goal']
    goal_x, goal_y, goal_yaw = (float(goal[key]) for key in ('x', 'y', 'yaw'))
    timing = navigation['timing']
    start_stamp, result_stamp, stop_stamp = (float(timing[key]) for key in ('goal_accepted_sim_sec', 'action_result_sim_sec', 'final_stop_sim_sec'))
    if not start_stamp <= result_stamp <= stop_stamp:
        raise ValueError('navigation timing order is invalid')
    maximum_delta = float(contract['max_alignment_delta_sec'])
    final_match = nearest_sample(gt_map, result_stamp, maximum_delta)
    start_match = nearest_sample(gt_map, start_stamp, maximum_delta)
    if final_match is None or start_match is None:
        raise ValueError('Ground Truth lacks start or final sample in the frozen alignment window')
    final_gt, final_gt_delta = final_match
    start_gt, start_gt_delta = start_match
    initial_origin_error = math.hypot(gt_map[0]['x'], gt_map[0]['y'])
    initial_yaw_error = abs(normalize_angle(gt_map[0]['yaw']))
    travel = [sample for sample in gt_map if start_stamp <= sample['stamp_sec'] <= result_stamp]
    if len(travel) < 2:
        raise ValueError('Ground Truth lacks a complete navigation interval')
    final_position_error = math.dist((final_gt['x'], final_gt['y']), (goal_x, goal_y))
    final_yaw_error = abs(normalize_angle(final_gt['yaw'] - goal_yaw))
    feedback_metrics = localization_metrics(navigation['feedback_samples'], gt_map, maximum_delta_sec=maximum_delta)
    plans = navigation['plans']
    if not plans:
        raise ValueError('navigation runner recorded no Nav2 plan')
    initial_length = float(plans[0]['path_length_m'])
    gt_travel = polyline_length(
        subsample_by_minimum_period(travel, float(contract['gt_distance_minimum_sample_period_sec']))
    )
    if gt_travel <= 0.0:
        raise ValueError('actual Ground Truth travel distance is not positive')
    run_result = navigation['scenario_result']
    recovery_count = int(run_result['feedback']['maximum_number_of_recoveries'])
    settle_start = stop_stamp - float(contract['final_stop_window_sim_sec'])
    settle_match = nearest_sample(gt_map, settle_start, maximum_delta)
    if settle_match is None:
        raise ValueError('Ground Truth lacks a sample for final-stop evaluation')
    settled_gt, settle_delta = settle_match
    return {
        'navigation_success': True,
        'navigation_time_sec': result_stamp - start_stamp,
        'nav2_feedback_navigation_time_sec': float(run_result['feedback']['final']['navigation_time_sec']),
        'feedback_navigation_time_delta_sec': abs((result_stamp - start_stamp) - float(run_result['feedback']['final']['navigation_time_sec'])),
        'final_gt_pose_map': final_gt,
        'final_gt_position_error_m': final_position_error,
        'final_gt_yaw_error_rad': final_yaw_error,
        'endpoint_alignment_delta_sec': {'start': start_gt_delta, 'final': final_gt_delta},
        'frozen_gt_origin_check': {
            'position_error_m': initial_origin_error,
            'yaw_error_rad': initial_yaw_error,
        },
        'localization_error': feedback_metrics,
        'actual_gt_travel_distance_m': gt_travel,
        'initial_global_path_length_m': initial_length,
        'final_global_path_length_m': float(plans[-1]['path_length_m']),
        'path_count': len(plans),
        'replanning_count': max(0, len(plans) - 1),
        'initial_path_geometry': path_turn_geometry(plans[0]['points']),
        'initial_path_turn_count': heading_turn_count(plans[0]['points']),
        'route_execution_efficiency': initial_length / gt_travel,
        'geometric_directness': math.dist((start_gt['x'], start_gt['y']), (goal_x, goal_y)) / gt_travel,
        'recovery_count': recovery_count,
        'final_stop': {
            **navigation['final_stop'],
            'ground_truth_translation_m': math.dist(
                (settled_gt['x'], settled_gt['y']), (final_gt['x'], final_gt['y'])
            ),
            'ground_truth_yaw_rad': abs(normalize_angle(settled_gt['yaw'] - final_gt['yaw'])),
            'ground_truth_alignment_delta_sec': settle_delta,
        },
    }


def evaluate_canceled_navigation_run(
    navigation: Mapping[str, object], ground_truth: Mapping[str, object], contract: Mapping[str, object],
) -> dict[str, object]:
    """Evaluate native NavigateToPose cancellation without treating it as a goal failure.

    This is deliberately narrower than healthy-goal evaluation: the robot is
    expected to stop before the goal, so endpoint error is an observation with
    no goal-tolerance meaning.  Missing cancellation, controller-stop, or GT
    evidence fails closed.
    """
    if navigation.get('status') != 'PASS':
        raise ValueError(f"navigation runner did not pass: {navigation.get('error', 'unknown error')}")
    invalid_gt = ground_truth.get('invalid_sample_count')
    if not isinstance(invalid_gt, int) or invalid_gt < 0:
        raise ValueError('Ground Truth evidence has an invalid invalid_sample_count')
    if invalid_gt > int(contract['maximum_invalid_gt_samples']):
        raise ValueError(f'Ground Truth evidence quality failure: invalid_sample_count={invalid_gt}')
    gt_raw = validate_trajectory(ground_truth.get('samples', []), 'ground_truth')
    gt_map = [apply_se2(sample, contract['map_from_gt_odom']) for sample in gt_raw]
    timing = navigation.get('timing')
    result = navigation.get('scenario_result')
    final_stop = navigation.get('final_stop')
    if not isinstance(timing, Mapping) or not isinstance(result, Mapping) or not isinstance(final_stop, Mapping):
        raise ValueError('cancel navigation evidence is malformed')
    cancellation = result.get('cancellation')
    if not isinstance(cancellation, Mapping):
        raise ValueError('cancel navigation lacks native cancellation evidence')
    try:
        start_stamp = float(timing['goal_accepted_sim_sec'])
        result_stamp = float(timing['action_result_sim_sec'])
        stop_stamp = float(timing['final_stop_sim_sec'])
        initial_path_stamp = float(cancellation['initial_path_sim_time'])
        motion_gate_stamp = float(cancellation['motion_gate_sim_time'])
        cancel_request_stamp = float(cancellation['cancel_request_sim_time'])
        cancel_response_stamp = float(cancellation['cancel_response_sim_time'])
        gate_travel_m = float(cancellation['motion_gate_odom_travel_m'])
        minimum_gate_travel_m = float(cancellation['minimum_odom_travel_m'])
        last_nonzero_stamp = float(cancellation['last_nonzero_command_before_cancel_sim_time'])
        first_zero_stamp = float(cancellation['first_zero_command_after_cancel_sim_time'])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('cancel navigation evidence lacks ordered timestamps') from error
    ordered = (
        start_stamp <= initial_path_stamp <= motion_gate_stamp <= cancel_request_stamp
        <= cancel_response_stamp <= result_stamp <= stop_stamp
    )
    if not ordered:
        raise ValueError('cancel navigation timestamps are not ordered')
    if gate_travel_m < minimum_gate_travel_m:
        raise ValueError('cancel motion gate did not reach its frozen odometry distance')
    if int(cancellation.get('cancel_return_code', -1)) != 0 or int(cancellation.get('cancel_goals_canceling_count', 0)) < 1:
        raise ValueError('NavigateToPose native cancel was not acknowledged')
    if result.get('navigation_canceled') is not True or int(result.get('action_status', -1)) != 5:
        raise ValueError('NavigateToPose did not reach the required CANCELED terminal state')
    feedback = result.get('feedback')
    if not isinstance(feedback, Mapping) or int(feedback.get('maximum_number_of_recoveries', -1)) != 0:
        raise ValueError('goal-cancel baseline must prove recovery_count=0')
    if not (last_nonzero_stamp <= cancel_request_stamp <= first_zero_stamp <= stop_stamp):
        raise ValueError('Controller cancel-stop command timestamps are invalid')
    if final_stop.get('controller_zero_observed') is not True or final_stop.get('odometry_settled') is not True:
        raise ValueError('goal-cancel final stop is incomplete')
    sweep = navigation.get('initial_path_full_footprint_sweep')
    if not isinstance(sweep, Mapping) or sweep.get('safe') is not True:
        raise ValueError('goal-cancel initial Path fails full-footprint safety evidence')
    plans = navigation.get('plans')
    if not isinstance(plans, list) or not plans:
        raise ValueError('goal-cancel navigation recorded no initial global Path')
    maximum_delta = float(contract['max_alignment_delta_sec'])
    matches = {
        name: nearest_sample(gt_map, stamp, maximum_delta)
        for name, stamp in (
            ('start', start_stamp), ('cancel', cancel_request_stamp),
            ('result', result_stamp), ('stop', stop_stamp),
        )
    }
    if any(match is None for match in matches.values()):
        raise ValueError('Ground Truth lacks a cancel lifecycle sample in the frozen alignment window')
    start_gt, start_delta = matches['start']  # type: ignore[misc]
    cancel_gt, cancel_delta = matches['cancel']  # type: ignore[misc]
    result_gt, result_delta = matches['result']  # type: ignore[misc]
    stop_gt, stop_delta = matches['stop']  # type: ignore[misc]
    travel = [sample for sample in gt_map if start_stamp <= sample['stamp_sec'] <= cancel_request_stamp]
    if len(travel) < 2:
        raise ValueError('Ground Truth lacks the pre-cancel motion interval')
    gt_travel = polyline_length(
        subsample_by_minimum_period(travel, float(contract['gt_distance_minimum_sample_period_sec']))
    )
    if gt_travel <= 0.0:
        raise ValueError('Ground Truth did not show pre-cancel travel')
    settle_start = stop_stamp - float(contract['final_stop_window_sim_sec'])
    settle_match = nearest_sample(gt_map, settle_start, maximum_delta)
    if settle_match is None:
        raise ValueError('Ground Truth lacks a sample for cancel final-stop evaluation')
    settled_gt, settle_delta = settle_match
    return {
        'navigation_canceled': True,
        'navigation_time_to_cancel_sec': cancel_request_stamp - start_stamp,
        'cancel_ack_latency_sec': cancel_response_stamp - cancel_request_stamp,
        'cancel_to_terminal_sec': result_stamp - cancel_request_stamp,
        'motion_gate_odom_travel_m': gate_travel_m,
        'actual_gt_travel_before_cancel_m': gt_travel,
        'initial_global_path_length_m': float(plans[0]['path_length_m']),
        'path_count': len(plans),
        'recovery_count': 0,
        'controller_stop_evidence': {
            'last_nonzero_before_cancel_sim_time': last_nonzero_stamp,
            'first_zero_after_cancel_sim_time': first_zero_stamp,
        },
        'gt_event_alignment_delta_sec': {
            'goal_accepted': start_delta,
            'cancel_requested': cancel_delta,
            'action_result': result_delta,
            'final_stop': stop_delta,
        },
        'final_stop': {
            **final_stop,
            'ground_truth_translation_m': math.dist(
                (settled_gt['x'], settled_gt['y']), (stop_gt['x'], stop_gt['y'])
            ),
            'ground_truth_yaw_rad': abs(normalize_angle(settled_gt['yaw'] - stop_gt['yaw'])),
            'ground_truth_alignment_delta_sec': settle_delta,
            'action_result_to_final_stop_gt_translation_m': math.dist(
                (result_gt['x'], result_gt['y']), (stop_gt['x'], stop_gt['y'])
            ),
        },
        'final_gt_pose_map': stop_gt,
        'cancel_gt_pose_map': cancel_gt,
    }


def summarize_runs(results: Iterable[Mapping[str, object]]) -> dict[str, object]:
    completed = list(results)
    if not completed:
        raise ValueError('cannot summarize zero benchmark runs')
    successes = [item for item in completed if item.get('status') == 'PASS']
    metrics = [item['metrics'] for item in successes if isinstance(item.get('metrics'), Mapping)]
    summary: dict[str, object] = {
        'attempt_count': len(completed),
        'pass_count': len(successes),
        'success_rate': len(successes) / len(completed),
    }
    if metrics:
        for field in ('navigation_time_sec', 'final_gt_position_error_m', 'final_gt_yaw_error_rad', 'actual_gt_travel_distance_m', 'route_execution_efficiency', 'geometric_directness', 'replanning_count'):
            values = [float(metric[field]) for metric in metrics]
            summary[field] = {'mean': mean(values), 'median': median(values), 'min': min(values), 'max': max(values)}
    return summary
