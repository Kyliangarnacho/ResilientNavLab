"""Offline-only Task 5 evidence contracts over Task 4 trial records."""

from __future__ import annotations

import math
from typing import Mapping

from costmap_contract import padded_footprint
from navigation_benchmark_metrics import apply_se2, nearest_sample, normalize_angle, validate_trajectory


ACTION_STATUS_ABORTED = 6


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f'{name} is malformed')
    return value


def _finite(value: object, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{name} is not numeric') from error
    if not math.isfinite(number):
        raise ValueError(f'{name} is non-finite')
    return number


def _event_times(event: Mapping[str, object]) -> dict[str, float]:
    if event.get('status') != 'PASS':
        raise ValueError(f"Task 5 obstacle event did not pass: {event.get('status')}")
    keys = (
        'initial_path_sim_time', 'motion_gate_sim_time', 'spawn_request_sim_time',
        'spawn_ack_sim_time', 'first_global_costmap_detection_sim_time',
        'first_local_costmap_detection_sim_time',
    )
    values = {key: _finite(event.get(key), f'event/{key}') for key in keys}
    if not (
        values['initial_path_sim_time'] <= values['motion_gate_sim_time']
        <= values['spawn_request_sim_time'] <= values['spawn_ack_sim_time']
    ):
        raise ValueError('Task 5 event timing order is invalid before Gazebo spawn')
    if min(values['first_global_costmap_detection_sim_time'], values['first_local_costmap_detection_sim_time']) < values['spawn_ack_sim_time']:
        raise ValueError('Task 5 Costmap detection predates the acknowledged Gazebo spawn')
    if event.get('event_mode') == 'spawn_then_delete':
        removal_keys = (
            'obstacle_remove_due_sim_time', 'obstacle_remove_request_sim_time',
            'obstacle_remove_ack_sim_time', 'first_global_costmap_clear_sim_time',
        )
        values.update({key: _finite(event.get(key), f'event/{key}') for key in removal_keys})
        if event.get('obstacle_remove_success') is not True:
            raise ValueError('temporary-obstacle DeleteEntity did not succeed')
        if not (
            values['spawn_ack_sim_time'] <= values['obstacle_remove_due_sim_time']
            <= values['obstacle_remove_request_sim_time']
            <= values['obstacle_remove_ack_sim_time']
            <= values['first_global_costmap_clear_sim_time']
        ):
            raise ValueError('temporary-obstacle removal/Costmap-clear timing is invalid')
    return values


def _validate_event_identity(
    navigation: Mapping[str, object], scenario: Mapping[str, object], event: Mapping[str, object],
) -> dict[str, object]:
    """Reject evidence that cannot be tied to the frozen selected scenario."""
    task5 = _mapping(scenario.get('task5'), 'scenario/task5')
    obstacle = _mapping(task5.get('obstacle'), 'scenario/task5/obstacle')
    map_pose = _mapping(obstacle.get('map_pose'), 'scenario/task5/obstacle/map_pose')
    size = _mapping(obstacle.get('size_m'), 'scenario/task5/obstacle/size_m')
    expected_name = navigation.get('scenario')
    if not isinstance(expected_name, str) or event.get('scenario') != expected_name:
        raise ValueError('Task 5 event scenario identity does not match navigation evidence')
    if event.get('task5_kind') != task5.get('kind'):
        raise ValueError('Task 5 event kind does not match frozen scenario')
    if event.get('entity_name') != obstacle.get('entity_name'):
        raise ValueError('Task 5 event entity does not match frozen scenario')
    digest = event.get('scenario_sha256')
    if not isinstance(digest, str) or len(digest) != 64:
        raise ValueError('Task 5 event lacks a valid frozen scenario digest')
    actual = _mapping(event.get('map_box'), 'event/map_box')
    expected = {
        'center_x': _finite(map_pose.get('x'), 'scenario/map_pose.x'),
        'center_y': _finite(map_pose.get('y'), 'scenario/map_pose.y'),
        'size_x': _finite(size.get('x'), 'scenario/size_m.x'),
        'size_y': _finite(size.get('y'), 'scenario/size_m.y'),
        'yaw': _finite(map_pose.get('yaw'), 'scenario/map_pose.yaw'),
    }
    for key, value in expected.items():
        if abs(_finite(actual.get(key), f'event/map_box.{key}') - value) > 1e-9:
            raise ValueError(f'Task 5 event map_box.{key} differs from frozen scenario')
    return {'scenario': expected_name, 'task5_kind': task5['kind'], 'entity_name': obstacle['entity_name'], 'scenario_sha256': digest}


def _transform_polygon(pose: Mapping[str, float]) -> tuple[tuple[float, float], ...]:
    yaw = float(pose['yaw'])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return tuple(
        (
            float(pose['x']) + cosine * x - sine * y,
            float(pose['y']) + sine * x + cosine * y,
        )
        for x, y in padded_footprint()
    )


def _box_polygon(box: Mapping[str, object]) -> tuple[tuple[float, float], ...]:
    x = _finite(box.get('center_x'), 'event/map_box.center_x')
    y = _finite(box.get('center_y'), 'event/map_box.center_y')
    sx = _finite(box.get('size_x'), 'event/map_box.size_x')
    sy = _finite(box.get('size_y'), 'event/map_box.size_y')
    yaw = _finite(box.get('yaw'), 'event/map_box.yaw')
    if sx <= 0.0 or sy <= 0.0:
        raise ValueError('event/map_box size must be positive')
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return tuple(
        (x + cosine * px - sine * py, y + sine * px + cosine * py)
        for px, py in ((-sx / 2.0, -sy / 2.0), (sx / 2.0, -sy / 2.0), (sx / 2.0, sy / 2.0), (-sx / 2.0, sy / 2.0))
    )


def _project(points: tuple[tuple[float, float], ...], axis: tuple[float, float]) -> tuple[float, float]:
    values = [x * axis[0] + y * axis[1] for x, y in points]
    return min(values), max(values)


def polygons_intersect(first: tuple[tuple[float, float], ...], second: tuple[tuple[float, float], ...]) -> bool:
    """Convex SAT used only by the evaluator, never by Nav2 runtime."""
    for polygon in (first, second):
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            ex, ey = end[0] - start[0], end[1] - start[1]
            length = math.hypot(ex, ey)
            if length <= 1e-12:
                continue
            first_range, second_range = _project(first, (-ey / length, ex / length)), _project(second, (-ey / length, ex / length))
            if first_range[1] < second_range[0] or second_range[1] < first_range[0]:
                return False
    return True


def _gt_collision_evidence(
    ground_truth: Mapping[str, object], contract: Mapping[str, object], event: Mapping[str, object],
    start_sec: float, end_sec: float,
) -> dict[str, object]:
    raw = validate_trajectory(ground_truth.get('samples', []), 'ground_truth')
    map_samples = [apply_se2(sample, _mapping(contract.get('map_from_gt_odom'), 'map_from_gt_odom')) for sample in raw]
    samples = [sample for sample in map_samples if start_sec <= sample['stamp_sec'] <= end_sec]
    if not samples:
        raise ValueError('Ground Truth lacks samples during the dynamic-obstacle interval')
    box = _box_polygon(_mapping(event.get('map_box'), 'event/map_box'))
    contacts = [sample for sample in samples if polygons_intersect(_transform_polygon(sample), box)]
    return {
        'sample_count': len(samples),
        'full_footprint_collision_with_spawned_obstacle': bool(contacts),
        'first_collision_sample': contacts[0] if contacts else None,
    }


def _failure_ground_truth_evidence(
    navigation: Mapping[str, object], ground_truth: Mapping[str, object], contract: Mapping[str, object],
) -> tuple[list[dict[str, float]], dict[str, object]]:
    """Keep failure runs subject to GT quality and terminal-settle contracts."""
    invalid_count = ground_truth.get('invalid_sample_count')
    if not isinstance(invalid_count, int) or invalid_count < 0:
        raise ValueError('Ground Truth evidence has an invalid invalid_sample_count')
    if invalid_count > int(contract['maximum_invalid_gt_samples']):
        raise ValueError(f'Ground Truth evidence quality failure: invalid_sample_count={invalid_count}')
    timing = _mapping(navigation.get('timing'), 'navigation/timing')
    accepted = _finite(timing.get('goal_accepted_sim_sec'), 'navigation/goal_accepted_sim_sec')
    result = _finite(timing.get('action_result_sim_sec'), 'navigation/action_result_sim_sec')
    stop = _finite(timing.get('final_stop_sim_sec'), 'navigation/final_stop_sim_sec')
    if not accepted <= result <= stop:
        raise ValueError('navigation timing order is invalid')
    samples = [
        apply_se2(sample, _mapping(contract.get('map_from_gt_odom'), 'map_from_gt_odom'))
        for sample in validate_trajectory(ground_truth.get('samples', []), 'ground_truth')
    ]
    maximum_delta = _finite(contract.get('max_alignment_delta_sec'), 'benchmark/max_alignment_delta_sec')
    result_match = nearest_sample(samples, result, maximum_delta)
    stop_match = nearest_sample(samples, stop, maximum_delta)
    settle_start = stop - _finite(contract.get('final_stop_window_sim_sec'), 'benchmark/final_stop_window_sim_sec')
    settle_match = nearest_sample(samples, settle_start, maximum_delta)
    if result_match is None or stop_match is None or settle_match is None:
        raise ValueError('Ground Truth lacks terminal samples in the frozen alignment window')
    result_pose, result_delta = result_match
    stop_pose, stop_delta = stop_match
    settle_pose, settle_delta = settle_match
    translation = math.dist((settle_pose['x'], settle_pose['y']), (stop_pose['x'], stop_pose['y']))
    yaw = abs(normalize_angle(settle_pose['yaw'] - stop_pose['yaw']))
    if translation > _finite(contract.get('final_stop_translation_m'), 'benchmark/final_stop_translation_m'):
        raise ValueError('Ground Truth did not settle after fully-blocked terminal action')
    if yaw > _finite(contract.get('final_stop_yaw_rad'), 'benchmark/final_stop_yaw_rad'):
        raise ValueError('Ground Truth yaw did not settle after fully-blocked terminal action')
    return samples, {
        'invalid_sample_count': invalid_count,
        'result_alignment_delta_sec': result_delta,
        'stop_alignment_delta_sec': stop_delta,
        'settle_alignment_delta_sec': settle_delta,
        'ground_truth_translation_m': translation,
        'ground_truth_yaw_rad': yaw,
        'action_result_pose_map': result_pose,
        'final_stop_pose_map': stop_pose,
    }


def _point_in_rotated_box(x: float, y: float, box: Mapping[str, object]) -> tuple[float, float, float, float]:
    center_x, center_y = _finite(box.get('center_x'), 'event/map_box.center_x'), _finite(box.get('center_y'), 'event/map_box.center_y')
    size_x, size_y, yaw = _finite(box.get('size_x'), 'event/map_box.size_x'), _finite(box.get('size_y'), 'event/map_box.size_y'), _finite(box.get('yaw'), 'event/map_box.yaw')
    cosine, sine = math.cos(yaw), math.sin(yaw)
    dx, dy = x - center_x, y - center_y
    return cosine * dx + sine * dy, -sine * dx + cosine * dy, size_x, size_y


def _segment_intersects_rotated_box(
    first: Mapping[str, object], second: Mapping[str, object], box: Mapping[str, object],
) -> bool:
    x0, y0, size_x, size_y = _point_in_rotated_box(_finite(first.get('x'), 'path/x'), _finite(first.get('y'), 'path/y'), box)
    x1, y1, _, _ = _point_in_rotated_box(_finite(second.get('x'), 'path/x'), _finite(second.get('y'), 'path/y'), box)
    lower_x, upper_x, lower_y, upper_y = -size_x / 2.0, size_x / 2.0, -size_y / 2.0, size_y / 2.0
    delta_x, delta_y = x1 - x0, y1 - y0
    enter, leave = 0.0, 1.0
    for coordinate, delta, lower, upper in ((x0, delta_x, lower_x, upper_x), (y0, delta_y, lower_y, upper_y)):
        if abs(delta) < 1e-12:
            if coordinate < lower or coordinate > upper:
                return False
            continue
        first_t, second_t = (lower - coordinate) / delta, (upper - coordinate) / delta
        if first_t > second_t:
            first_t, second_t = second_t, first_t
        enter, leave = max(enter, first_t), min(leave, second_t)
        if enter > leave:
            return False
    return True


def _initial_path_impact(navigation: Mapping[str, object], event: Mapping[str, object]) -> dict[str, object]:
    plans = navigation.get('plans')
    if not isinstance(plans, list) or not plans:
        raise ValueError('Task 5 navigation evidence lacks the initial global Path')
    initial = _mapping(plans[0], 'initial global Path')
    points = initial.get('points')
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError('initial global Path is malformed')
    box = _mapping(event.get('map_box'), 'event/map_box')
    intersects = any(
        _segment_intersects_rotated_box(_mapping(first, 'initial path point'), _mapping(second, 'initial path point'), box)
        for first, second in zip(points, points[1:])
    )
    if not intersects:
        raise ValueError('spawned obstacle does not intersect the initial global Path corridor')
    return {'initial_path_intersects_spawned_obstacle': True, 'initial_path_length_m': _finite(initial.get('path_length_m'), 'initial/path_length_m')}


def _barrier_crossing_evidence(
    samples: list[dict[str, float]], event: Mapping[str, object], start_sec: float, end_sec: float,
) -> dict[str, object]:
    """Verify every padded GT footprint remains on its pre-barrier side."""
    relevant = [sample for sample in samples if start_sec <= sample['stamp_sec'] <= end_sec]
    before = [sample for sample in samples if sample['stamp_sec'] <= start_sec]
    if not relevant or not before:
        raise ValueError('Ground Truth lacks samples needed for barrier-side evidence')
    box = _mapping(event.get('map_box'), 'event/map_box')
    reference = before[-1]
    local_x, local_y, size_x, size_y = _point_in_rotated_box(reference['x'], reference['y'], box)
    use_y_axis = size_x >= size_y
    coordinate, half_extent = (local_y, size_y / 2.0) if use_y_axis else (local_x, size_x / 2.0)
    if abs(coordinate) <= half_extent:
        raise ValueError('robot was not on a valid side of the barrier at spawn time')
    start_side = 1.0 if coordinate > 0.0 else -1.0
    crossing = None
    minimum_clearance = math.inf
    for sample in relevant:
        footprint = _transform_polygon(sample)
        coordinates = [
            (_point_in_rotated_box(x, y, box)[1] if use_y_axis else _point_in_rotated_box(x, y, box)[0])
            for x, y in footprint
        ]
        clearance = min(start_side * value - half_extent for value in coordinates)
        minimum_clearance = min(minimum_clearance, clearance)
        if clearance < -1e-9 and crossing is None:
            crossing = sample
    if crossing is not None:
        raise ValueError('GT padded footprint crossed the fully-blocked barrier')
    return {
        'barrier_axis': 'local_y' if use_y_axis else 'local_x',
        'initial_barrier_side': 'positive' if start_side > 0.0 else 'negative',
        'minimum_padded_footprint_clearance_m': minimum_clearance,
        'full_footprint_barrier_crossing': False,
        'sample_count': len(relevant),
    }


def _controller_zero_before_terminal(
    navigation: Mapping[str, object], after_sec: float, final_stop_sec: float,
) -> dict[str, object]:
    """Require a Nav2 stop before Runner teardown can publish its own zero.

    NavigateToPose can report ABORTED immediately after FollowPath cancellation
    while Controller publishes its final zero on the next control turn.  The
    evidence record is assembled before the Runner's finally-block safety
    zeroes, so a zero between obstacle detection and recorded final-stop is
    Controller-originated without incorrectly demanding it precede the action
    response timestamp.
    """
    commands = navigation.get('command_samples')
    if not isinstance(commands, list):
        raise ValueError('fully-blocked navigation evidence lacks Controller command samples')
    zeroes = []
    for raw in commands:
        command = _mapping(raw, 'command sample')
        stamp = command.get('sim_time_sec')
        if stamp is None:
            continue
        sim_time = _finite(stamp, 'command/sim_time_sec')
        if after_sec <= sim_time <= final_stop_sec and abs(_finite(command.get('linear_x'), 'command/linear_x')) < 1e-9 and abs(_finite(command.get('angular_z'), 'command/angular_z')) < 1e-9:
            zeroes.append({'sim_time_sec': sim_time, 'linear_x': float(command['linear_x']), 'angular_z': float(command['angular_z'])})
    if not zeroes:
        raise ValueError('fully-blocked run lacks a Controller-originated zero cmd_vel before final stop')
    return {'controller_zero_before_final_stop': True, 'first_controller_zero_sim_time': zeroes[0]['sim_time_sec'], 'zero_sample_count': len(zeroes)}


def _fully_blocked_failure_branch(navigation: Mapping[str, object]) -> dict[str, object]:
    """Classify the observed BT failure ordering without deciding acceptance."""
    events = navigation.get('bt_events')
    if not isinstance(events, list):
        raise ValueError('fully-blocked navigation evidence lacks BT event records')

    def first_failure(name: str) -> float | None:
        for raw in events:
            event = _mapping(raw, 'BT event')
            if event.get('node_name') != name or event.get('current_status') != 'FAILURE':
                continue
            stamp = event.get('sim_time_sec')
            if stamp is not None:
                return _finite(stamp, f'BT event {name}/sim_time_sec')
        return None

    planner = first_failure('ComputePathToPose')
    controller = first_failure('FollowPath')
    if planner is not None and (controller is None or planner <= controller):
        branch = 'planner_first'
        source = 'bt_event_order'
    elif controller is not None:
        branch = 'controller_first'
        source = 'bt_event_order'
    elif _mapping(navigation.get('scenario_result'), 'scenario_result').get('result_error_code') == 208:
        # r03's retained BT log did not emit a terminal FAILURE transition,
        # but Navfn's terminal NO_VALID_PATH code is direct Planner evidence.
        branch = 'planner_first'
        source = 'terminal_navfn_no_valid_path'
    else:
        branch = 'unclassified'
        source = 'insufficient_bt_event_evidence'
    return {
        'classification': branch,
        'source': source,
        'first_compute_path_failure_sim_time': planner,
        'first_follow_path_failure_sim_time': controller,
    }


def _safe_paths_after_detection(navigation: Mapping[str, object], detection_sec: float) -> list[Mapping[str, object]]:
    run = _mapping(navigation.get('scenario_result'), 'scenario_result')
    evidence = run.get('full_footprint_path_evidence')
    if not isinstance(evidence, list):
        raise ValueError('scenario_result full-footprint evidence is malformed')
    return [
        _mapping(item, 'post-obstacle path evidence')
        for item in evidence
        if _finite(_mapping(item, 'path evidence').get('path_stamp_sec'), 'path_stamp_sec') >= detection_sec
    ]


def _recovery_evidence_before_delete(
    navigation: Mapping[str, object], delete_sec: float,
) -> dict[str, object]:
    """Derive minimal official-BT recovery evidence without a custom tracer."""
    branch = _fully_blocked_failure_branch(navigation)
    failures = [
        stamp for stamp in (
            branch['first_compute_path_failure_sim_time'],
            branch['first_follow_path_failure_sim_time'],
        ) if stamp is not None
    ]
    if not failures:
        raise ValueError('temporary-recovery run lacks Planner/Controller BT failure evidence')
    first_failure = min(failures)
    if first_failure >= delete_sec:
        raise ValueError('temporary-recovery failure began only after obstacle deletion')
    events = navigation.get('bt_events')
    if not isinstance(events, list):
        raise ValueError('temporary-recovery navigation evidence lacks BT events')
    clears: list[dict[str, object]] = []
    behavior_actions: list[dict[str, object]] = []
    for raw in events:
        event = _mapping(raw, 'BT event')
        name = event.get('node_name')
        status = event.get('current_status')
        stamp = event.get('sim_time_sec')
        if not isinstance(name, str) or not isinstance(status, str) or stamp is None:
            continue
        sim_time = _finite(stamp, f'BT event {name}/sim_time_sec')
        if not first_failure <= sim_time < delete_sec:
            continue
        record = {'node_name': name, 'current_status': status, 'sim_time_sec': sim_time}
        if 'Clear' in name and status in ('RUNNING', 'SUCCESS'):
            clears.append(record)
        if name in ('Spin', 'Wait', 'BackUp') and status == 'RUNNING':
            behavior_actions.append(record)
    if not clears:
        raise ValueError('temporary-recovery run lacks official Costmap-clear evidence before deletion')
    if not behavior_actions:
        raise ValueError('temporary-recovery run lacks Behavior Server action evidence before deletion')
    return {
        'failure_branch_observation': branch,
        'first_failure_sim_time': first_failure,
        'first_clear_sim_time': clears[0]['sim_time_sec'],
        'clear_events': clears,
        'first_behavior_action_sim_time': behavior_actions[0]['sim_time_sec'],
        'behavior_actions': behavior_actions,
    }


def _controller_resumption_after_clear(
    navigation: Mapping[str, object], clear_sec: float, action_result_sec: float,
) -> dict[str, object]:
    """Tie resumed velocity to a post-clear Controller-received Path."""
    received_paths = navigation.get('received_controller_paths')
    if not isinstance(received_paths, list):
        raise ValueError('temporary-recovery run lacks Controller Path history')
    post_clear_paths = [
        _mapping(raw, 'received controller path') for raw in received_paths
        if _finite(_mapping(raw, 'received controller path').get('stamp_sec'), 'received path stamp')
        >= clear_sec
    ]
    if not post_clear_paths:
        raise ValueError('temporary-recovery run lacks Controller Path after obstacle clearance')
    first_path_sec = _finite(post_clear_paths[0].get('stamp_sec'), 'received path stamp')
    commands = navigation.get('command_samples')
    if not isinstance(commands, list):
        raise ValueError('temporary-recovery run lacks command evidence')
    resumed = []
    for raw in commands:
        command = _mapping(raw, 'command sample')
        stamp = command.get('sim_time_sec')
        if stamp is None:
            continue
        sim_time = _finite(stamp, 'command/sim_time_sec')
        if first_path_sec <= sim_time <= action_result_sec and (
            abs(_finite(command.get('linear_x'), 'command/linear_x')) > 0.01
            or abs(_finite(command.get('angular_z'), 'command/angular_z')) > 0.01
        ):
            resumed.append({'sim_time_sec': sim_time, 'linear_x': command['linear_x'], 'angular_z': command['angular_z']})
    if not resumed:
        raise ValueError('temporary-recovery run lacks motion after Controller received its post-clear Path')
    return {
        'controller_path_count_after_clear': len(post_clear_paths),
        'first_received_controller_path_after_clear_sim_time': first_path_sec,
        'first_motion_after_controller_path_sim_time': resumed[0]['sim_time_sec'],
        'motion_sample_count_after_controller_path': len(resumed),
    }


def evaluate_dynamic_obstacle_event(
    navigation: Mapping[str, object], ground_truth: Mapping[str, object], benchmark: Mapping[str, object],
    scenario: Mapping[str, object], event: Mapping[str, object], *, healthy_metrics: Mapping[str, object] | None,
    evaluation_mode: str = 'acceptance',
) -> dict[str, object]:
    """Evaluate one Task 5 scenario without supplying GT to the trial."""
    if evaluation_mode not in ('acceptance', 'discovery'):
        raise ValueError(f'unsupported Task 5 evaluation mode: {evaluation_mode!r}')
    if navigation.get('status') != 'PASS':
        raise ValueError(f"navigation runner did not pass: {navigation.get('error', 'unknown error')}")
    task5 = _mapping(scenario.get('task5'), 'scenario/task5')
    acceptance = _mapping(task5.get('acceptance'), 'scenario/task5/acceptance')
    event_identity = _validate_event_identity(navigation, scenario, event)
    event_times = _event_times(event)
    timing = _mapping(navigation.get('timing'), 'navigation/timing')
    action_result = _finite(timing.get('action_result_sim_sec'), 'navigation/action_result_sim_sec')
    if action_result < event_times['first_global_costmap_detection_sim_time']:
        raise ValueError('NavigateToPose ended before the global Costmap observed the obstacle')
    collision_end = event_times.get('obstacle_remove_ack_sim_time', action_result)
    collision = _gt_collision_evidence(
        ground_truth, benchmark, event, event_times['spawn_ack_sim_time'], collision_end,
    )
    if collision['full_footprint_collision_with_spawned_obstacle']:
        raise ValueError('GT full footprint intersected the spawned obstacle')
    initial_path_impact = _initial_path_impact(navigation, event)
    run = _mapping(navigation.get('scenario_result'), 'scenario_result')
    feedback = _mapping(run.get('feedback'), 'scenario_result/feedback')
    recovery_count = int(feedback.get('maximum_number_of_recoveries', -1))
    common = {
        'event_times_sim_sec': event_times,
        'global_detection_latency_sec': event_times['first_global_costmap_detection_sim_time'] - event_times['spawn_ack_sim_time'],
        'local_detection_latency_sec': event_times['first_local_costmap_detection_sim_time'] - event_times['spawn_ack_sim_time'],
        'collision_evidence': collision,
        'event_identity': event_identity,
        'initial_path_impact': initial_path_impact,
        'maximum_number_of_recoveries': recovery_count,
    }
    kind = task5.get('kind')
    if kind == 'unmapped_obstacle':
        if recovery_count != 0:
            raise ValueError('unmapped-obstacle baseline unexpectedly reported a recovery')
        if run.get('navigation_success') is not True:
            raise ValueError('unmapped-obstacle scenario did not reach NavigateToPose success')
        post_paths = _safe_paths_after_detection(navigation, event_times['first_global_costmap_detection_sim_time'])
        if not post_paths:
            raise ValueError('no new global Path was observed after global Costmap detection')
        if not all(_mapping(item.get('sweep'), 'post-obstacle path sweep').get('safe') is True for item in post_paths):
            raise ValueError('a post-obstacle global Path failed full-footprint safety')
        if healthy_metrics is None:
            raise ValueError('unmapped-obstacle success requires healthy Task 4 metrics')
        return {
            'navigation_success': True,
            'no_recovery': True,
            'post_obstacle_replan_count': len(post_paths),
            'first_replan_after_global_detection_latency_sec': _finite(post_paths[0].get('path_stamp_sec'), 'path_stamp_sec') - event_times['first_global_costmap_detection_sim_time'],
            'healthy_navigation_metrics': dict(healthy_metrics),
            **common,
        }
    if kind == 'fully_blocked':
        if recovery_count != 0:
            raise ValueError('fully-blocked baseline unexpectedly reported a recovery')
        if run.get('navigation_success') is not False:
            raise ValueError('fully-blocked baseline unexpectedly reported NavigateToPose success')
        if int(run.get('action_status', -1)) != ACTION_STATUS_ABORTED:
            raise ValueError(f"fully-blocked terminal action status is {run.get('action_status')}, not ABORTED")
        result_code = int(run.get('result_error_code', -1))
        if evaluation_mode == 'acceptance':
            frozen_code = acceptance.get('frozen_error_code')
            if not isinstance(frozen_code, int) or isinstance(frozen_code, bool):
                raise ValueError('fully-blocked acceptance requires one frozen_error_code after discovery')
            if result_code != frozen_code:
                raise ValueError('fully-blocked result error code differs from the frozen discovery branch')
        final_stop = _mapping(navigation.get('final_stop'), 'final_stop')
        if final_stop.get('controller_zero_observed') is not True or final_stop.get('odometry_settled') is not True:
            raise ValueError('fully-blocked run lacks a safe final stop')
        terminal_latency = action_result - event_times['first_global_costmap_detection_sim_time']
        gt_samples, gt_terminal = _failure_ground_truth_evidence(
            navigation, ground_truth, benchmark,
        )
        timing = _mapping(navigation.get('timing'), 'navigation/timing')
        final_stop_sec = _finite(timing.get('final_stop_sim_sec'), 'navigation/final_stop_sim_sec')
        barrier_crossing = _barrier_crossing_evidence(
            gt_samples, event, event_times['spawn_ack_sim_time'], final_stop_sec,
        )
        controller_zero = _controller_zero_before_terminal(
            navigation, event_times['first_global_costmap_detection_sim_time'], final_stop_sec,
        )
        branch = _fully_blocked_failure_branch(navigation)
        return {
            'navigation_success': False,
            'no_recovery': True,
            'terminal_action_status': int(run['action_status']),
            'terminal_error_code': result_code,
            'terminal_error_message': run.get('result_error_message'),
            'terminal_after_global_detection_sec': terminal_latency,
            'post_detection_observation_warning_sec': acceptance.get(
                'post_detection_observation_warning_sec'
            ),
            'safe_stop': True,
            'ground_truth_terminal_evidence': gt_terminal,
            'barrier_crossing_evidence': barrier_crossing,
            'controller_zero_evidence': controller_zero,
            'failure_branch_observation': branch,
            'evaluation_mode': evaluation_mode,
            **common,
        }
    if kind == 'temporary_blocked_recovery':
        if run.get('navigation_success') is not True:
            raise ValueError('temporary-recovery scenario did not reach NavigateToPose success')
        if int(run.get('action_status', -1)) != 4 or int(run.get('result_error_code', -1)) != 0:
            raise ValueError('temporary-recovery terminal result is not NavigateToPose success')
        if recovery_count <= 0:
            raise ValueError('temporary-recovery feedback did not prove an official recovery')
        if healthy_metrics is None:
            raise ValueError('temporary-recovery success requires healthy Task 4 metrics')
        delete_sec = event_times['obstacle_remove_ack_sim_time']
        recovery = _recovery_evidence_before_delete(navigation, delete_sec)
        gt_samples = [
            apply_se2(sample, _mapping(benchmark.get('map_from_gt_odom'), 'map_from_gt_odom'))
            for sample in validate_trajectory(ground_truth.get('samples', []), 'ground_truth')
        ]
        barrier_crossing = _barrier_crossing_evidence(
            gt_samples, event, event_times['spawn_ack_sim_time'], delete_sec,
        )
        clear_sec = event_times['first_global_costmap_clear_sim_time']
        post_paths = _safe_paths_after_detection(navigation, clear_sec)
        if not post_paths:
            raise ValueError('temporary-recovery run lacks a global Path after obstacle clearance')
        if not all(_mapping(item.get('sweep'), 'post-clear path sweep').get('safe') is True for item in post_paths):
            raise ValueError('a post-clear global Path failed full-footprint safety')
        resumption = _controller_resumption_after_clear(navigation, clear_sec, action_result)
        final_stop = _mapping(navigation.get('final_stop'), 'final_stop')
        if final_stop.get('controller_zero_observed') is not True or final_stop.get('odometry_settled') is not True:
            raise ValueError('temporary-recovery run lacks a safe final stop')
        return {
            'navigation_success': True,
            'terminal_action_status': int(run['action_status']),
            'terminal_error_code': int(run['result_error_code']),
            'recovery_evidence': recovery,
            'barrier_crossing_evidence_before_delete': barrier_crossing,
            'post_delete_replan_count': len(post_paths),
            'first_replan_after_costmap_clear_latency_sec': (
                _finite(post_paths[0].get('path_stamp_sec'), 'path_stamp_sec') - clear_sec
            ),
            'controller_resumption_evidence': resumption,
            'healthy_navigation_metrics': dict(healthy_metrics),
            'safe_stop': True,
            'no_recovery': False,
            **common,
        }
    raise ValueError(f'Task 5 scenario kind is unsupported: {kind!r}')
