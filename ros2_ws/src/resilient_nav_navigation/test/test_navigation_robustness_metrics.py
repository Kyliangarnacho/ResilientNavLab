"""Pure evidence-contract tests for the Phase 10 Task 5 evaluator."""

from navigation_robustness_metrics import evaluate_dynamic_obstacle_event


def _event():
    return {
        'status': 'PASS',
        'scenario': 'dynamic_obstacle_detour',
        'task5_kind': 'unmapped_obstacle',
        'entity_name': 'phase10_task5_unmapped_box',
        'scenario_sha256': '0' * 64,
        'initial_path_sim_time': 1.0,
        'motion_gate_sim_time': 2.0,
        'spawn_request_sim_time': 2.0,
        'spawn_ack_sim_time': 2.1,
        'first_global_costmap_detection_sim_time': 2.4,
        'first_local_costmap_detection_sim_time': 2.5,
        'map_box': {'center_x': 1.0, 'center_y': 0.8, 'size_x': 0.5, 'size_y': 0.5, 'yaw': 0.0},
    }


def _ground_truth():
    return {
        'invalid_sample_count': 0,
        'samples': [
            {'stamp_sec': 2.1, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            {'stamp_sec': 3.0, 'x': 0.3, 'y': 0.0, 'yaw': 0.0},
            {'stamp_sec': 5.0, 'x': 0.4, 'y': 0.0, 'yaw': 0.0},
            {'stamp_sec': 6.0, 'x': 0.4, 'y': 0.0, 'yaw': 0.0},
        ]
    }


def _base_navigation(success):
    return {
        'status': 'PASS',
        'scenario': 'dynamic_obstacle_detour',
        'goal': {'x': 2.0, 'y': 0.8, 'yaw': 0.0},
        'timing': {
            'goal_accepted_sim_sec': 1.0,
            'action_result_sim_sec': 5.0,
            'final_stop_sim_sec': 6.0,
        },
        'plans': [{
            'path_length_m': 2.0,
            'points': [{'x': 0.0, 'y': 0.8}, {'x': 2.0, 'y': 0.8}],
        }],
        'scenario_result': {
            'navigation_success': success,
            'action_status': 4 if success else 6,
            'result_error_code': 0 if success else 208,
            'result_error_message': 'no valid path' if not success else '',
            'feedback': {'maximum_number_of_recoveries': 0},
            'full_footprint_path_evidence': [
                {'path_stamp_sec': 2.6, 'sweep': {'safe': True}},
            ],
        },
        'final_stop': {'controller_zero_observed': True, 'odometry_settled': True},
        'command_samples': [
            {'sim_time_sec': 2.5, 'linear_x': 0.0, 'angular_z': 0.0},
        ],
        'bt_events': [
            {'node_name': 'ComputePathToPose', 'current_status': 'FAILURE', 'sim_time_sec': 3.0},
        ],
    }


def _benchmark():
    return {
        'map_from_gt_odom': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'maximum_invalid_gt_samples': 0,
        'max_alignment_delta_sec': 1.1,
        'final_stop_window_sim_sec': 1.0,
        'final_stop_translation_m': 0.05,
        'final_stop_yaw_rad': 0.05,
    }


def test_unmapped_obstacle_requires_post_detection_safe_replan_without_gt_control():
    scenario = {
        'task5': {
            'kind': 'unmapped_obstacle',
            'obstacle': {
                'entity_name': 'phase10_task5_unmapped_box',
                'map_pose': {'x': 1.0, 'y': 0.8, 'yaw': 0.0},
                'size_m': {'x': 0.5, 'y': 0.5},
            },
            'acceptance': {},
        }
    }
    result = evaluate_dynamic_obstacle_event(
        _base_navigation(True), _ground_truth(), _benchmark(), scenario, _event(),
        healthy_metrics={'navigation_time_sec': 5.0},
    )
    assert result['navigation_success'] is True
    assert result['post_obstacle_replan_count'] == 1
    assert result['collision_evidence']['full_footprint_collision_with_spawned_obstacle'] is False


def test_fully_blocked_discovery_records_branch_without_preaccepting_an_error_code():
    scenario = {
        'task5': {
            'kind': 'fully_blocked',
            'acceptance': {
                'discovery_candidate_error_codes': [208, 104, 105],
                'post_detection_observation_sim_sec': 12.0,
            },
        }
    }
    navigation = _base_navigation(False)
    navigation['scenario'] = 'dynamic_fully_blocked'
    event = _event()
    event.update({
        'scenario': 'dynamic_fully_blocked',
        'task5_kind': 'fully_blocked',
        'entity_name': 'phase10_task5_blocking_wall',
        'map_box': {'center_x': 1.0, 'center_y': 0.8, 'size_x': 2.0, 'size_y': 0.2, 'yaw': 0.0},
    })
    scenario['task5']['obstacle'] = {
        'entity_name': 'phase10_task5_blocking_wall',
        'map_pose': {'x': 1.0, 'y': 0.8, 'yaw': 0.0},
        'size_m': {'x': 2.0, 'y': 0.2},
    }
    result = evaluate_dynamic_obstacle_event(
        navigation, _ground_truth(), _benchmark(), scenario, event,
        healthy_metrics=None, evaluation_mode='discovery',
    )
    assert result['navigation_success'] is False
    assert result['terminal_action_status'] == 6
    assert result['safe_stop'] is True
    assert result['evaluation_mode'] == 'discovery'
    assert result['failure_branch_observation']['classification'] == 'planner_first'


def test_fully_blocked_acceptance_requires_one_discovery_frozen_error_code():
    scenario = {
        'task5': {
            'kind': 'fully_blocked',
            'obstacle': {
                'entity_name': 'phase10_task5_blocking_wall',
                'map_pose': {'x': 1.0, 'y': 0.8, 'yaw': 0.0},
                'size_m': {'x': 2.0, 'y': 0.2},
            },
            'acceptance': {
                'frozen_error_code': 208,
                'post_detection_observation_sim_sec': 12.0,
            },
        }
    }
    navigation = _base_navigation(False)
    navigation['scenario'] = 'dynamic_fully_blocked'
    event = _event()
    event.update({
        'scenario': 'dynamic_fully_blocked',
        'task5_kind': 'fully_blocked',
        'entity_name': 'phase10_task5_blocking_wall',
        'map_box': {'center_x': 1.0, 'center_y': 0.8, 'size_x': 2.0, 'size_y': 0.2, 'yaw': 0.0},
    })
    result = evaluate_dynamic_obstacle_event(
        navigation, _ground_truth(), _benchmark(), scenario, event,
        healthy_metrics=None,
    )
    assert result['terminal_error_code'] == 208


def test_fully_blocked_uses_navfn_terminal_code_when_bt_log_lacks_failure_transition():
    scenario = {
        'task5': {
            'kind': 'fully_blocked',
            'obstacle': {
                'entity_name': 'phase10_task5_blocking_wall',
                'map_pose': {'x': 1.0, 'y': 0.8, 'yaw': 0.0},
                'size_m': {'x': 2.0, 'y': 0.2},
            },
            'acceptance': {'frozen_error_code': 208},
        }
    }
    navigation = _base_navigation(False)
    navigation['scenario'] = 'dynamic_fully_blocked'
    navigation['bt_events'] = []
    event = _event()
    event.update({
        'scenario': 'dynamic_fully_blocked',
        'task5_kind': 'fully_blocked',
        'entity_name': 'phase10_task5_blocking_wall',
        'map_box': {'center_x': 1.0, 'center_y': 0.8, 'size_x': 2.0, 'size_y': 0.2, 'yaw': 0.0},
    })
    result = evaluate_dynamic_obstacle_event(
        navigation, _ground_truth(), _benchmark(), scenario, event,
        healthy_metrics=None,
    )
    branch = result['failure_branch_observation']
    assert branch['classification'] == 'planner_first'
    assert branch['source'] == 'terminal_navfn_no_valid_path'


def test_fully_blocked_rejects_runner_teardown_zero_and_barrier_crossing():
    scenario = {
        'task5': {
            'kind': 'fully_blocked',
            'obstacle': {
                'entity_name': 'phase10_task5_blocking_wall',
                'map_pose': {'x': 1.0, 'y': 0.8, 'yaw': 0.0},
                'size_m': {'x': 2.0, 'y': 0.2},
            },
            'acceptance': {
                'frozen_error_code': 208,
                'post_detection_observation_sim_sec': 12.0,
            },
        }
    }
    navigation = _base_navigation(False)
    navigation['scenario'] = 'dynamic_fully_blocked'
    navigation['command_samples'] = [{'sim_time_sec': 6.1, 'linear_x': 0.0, 'angular_z': 0.0}]
    event = _event()
    event.update({
        'scenario': 'dynamic_fully_blocked',
        'task5_kind': 'fully_blocked',
        'entity_name': 'phase10_task5_blocking_wall',
        'map_box': {'center_x': 1.0, 'center_y': 0.8, 'size_x': 2.0, 'size_y': 0.2, 'yaw': 0.0},
    })
    try:
        evaluate_dynamic_obstacle_event(
            navigation, _ground_truth(), _benchmark(), scenario, event,
            healthy_metrics=None,
        )
    except ValueError as error:
        assert 'Controller-originated zero' in str(error)
    else:
        raise AssertionError('teardown-only zero command incorrectly passed')


def test_temporary_recovery_requires_official_recovery_before_independent_delete():
    scenario = {
        'task5': {
            'kind': 'temporary_blocked_recovery',
            'event_mode': 'spawn_then_delete',
            'obstacle': {
                'entity_name': 'phase10_task5_temporary_blocking_wall',
                'map_pose': {'x': 1.0, 'y': 0.8, 'yaw': 0.0},
                'size_m': {'x': 2.0, 'y': 0.2},
            },
            'acceptance': {},
        }
    }
    navigation = _base_navigation(True)
    navigation['scenario'] = 'dynamic_temporary_blocked_recovery'
    navigation['timing'] = {
        'goal_accepted_sim_sec': 1.0,
        'action_result_sim_sec': 8.0,
        'final_stop_sim_sec': 9.0,
    }
    navigation['scenario_result']['feedback']['maximum_number_of_recoveries'] = 1
    navigation['scenario_result']['full_footprint_path_evidence'] = [
        {'path_stamp_sec': 2.6, 'sweep': {'safe': True}},
        {'path_stamp_sec': 6.4, 'sweep': {'safe': True}},
    ]
    navigation['bt_events'] = [
        {'node_name': 'ComputePathToPose', 'current_status': 'FAILURE', 'sim_time_sec': 3.0},
        {'node_name': 'ClearGlobalCostmap-Context', 'current_status': 'SUCCESS', 'sim_time_sec': 3.1},
        {'node_name': 'Spin', 'current_status': 'RUNNING', 'sim_time_sec': 3.2},
    ]
    navigation['received_controller_paths'] = [
        {'stamp_sec': 6.5, 'path_pose_count': 2, 'points': []},
    ]
    navigation['command_samples'] = [
        {'sim_time_sec': 6.8, 'linear_x': 0.10, 'angular_z': 0.0},
        {'sim_time_sec': 7.9, 'linear_x': 0.0, 'angular_z': 0.0},
    ]
    event = _event()
    event.update({
        'scenario': 'dynamic_temporary_blocked_recovery',
        'task5_kind': 'temporary_blocked_recovery',
        'entity_name': 'phase10_task5_temporary_blocking_wall',
        'event_mode': 'spawn_then_delete',
        'map_box': {'center_x': 1.0, 'center_y': 0.8, 'size_x': 2.0, 'size_y': 0.2, 'yaw': 0.0},
        'obstacle_remove_due_sim_time': 5.0,
        'obstacle_remove_request_sim_time': 5.0,
        'obstacle_remove_ack_sim_time': 5.1,
        'obstacle_remove_success': True,
        'first_global_costmap_clear_sim_time': 6.0,
    })
    result = evaluate_dynamic_obstacle_event(
        navigation, _ground_truth(), _benchmark(), scenario, event,
        healthy_metrics={'navigation_time_sec': 8.0},
    )
    assert result['navigation_success'] is True
    assert result['no_recovery'] is False
    assert result['recovery_evidence']['behavior_actions'][0]['node_name'] == 'Spin'
    assert result['controller_resumption_evidence']['controller_path_count_after_clear'] == 1


def test_temporary_recovery_rejects_delete_before_recovery_evidence():
    scenario = {
        'task5': {
            'kind': 'temporary_blocked_recovery',
            'event_mode': 'spawn_then_delete',
            'obstacle': {
                'entity_name': 'phase10_task5_temporary_blocking_wall',
                'map_pose': {'x': 1.0, 'y': 0.8, 'yaw': 0.0},
                'size_m': {'x': 2.0, 'y': 0.2},
            },
            'acceptance': {},
        }
    }
    navigation = _base_navigation(True)
    navigation['scenario'] = 'dynamic_temporary_blocked_recovery'
    navigation['scenario_result']['feedback']['maximum_number_of_recoveries'] = 1
    navigation['bt_events'] = [
        {'node_name': 'ComputePathToPose', 'current_status': 'FAILURE', 'sim_time_sec': 5.2},
    ]
    event = _event()
    event.update({
        'scenario': 'dynamic_temporary_blocked_recovery',
        'task5_kind': 'temporary_blocked_recovery',
        'entity_name': 'phase10_task5_temporary_blocking_wall',
        'event_mode': 'spawn_then_delete',
        'map_box': {'center_x': 1.0, 'center_y': 0.8, 'size_x': 2.0, 'size_y': 0.2, 'yaw': 0.0},
        'obstacle_remove_due_sim_time': 5.0,
        'obstacle_remove_request_sim_time': 5.0,
        'obstacle_remove_ack_sim_time': 5.1,
        'obstacle_remove_success': True,
        'first_global_costmap_clear_sim_time': 6.0,
    })
    try:
        evaluate_dynamic_obstacle_event(
            navigation, _ground_truth(), _benchmark(), scenario, event,
            healthy_metrics={'navigation_time_sec': 5.0},
        )
    except ValueError as error:
        assert 'only after obstacle deletion' in str(error)
    else:
        raise AssertionError('recovery after deletion incorrectly passed')
