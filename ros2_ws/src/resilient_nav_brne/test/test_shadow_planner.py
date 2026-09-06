"""Numerical and fail-safe tests for the ROS-free shadow planner."""

import numpy as np

from resilient_nav_brne import brne
from resilient_nav_brne.shadow_planner import (
    BrneShadowPlanner,
    ShadowPlannerConfig,
    _interaction_velocity_classification,
)


def test_default_profile_matches_the_pinned_official_ros_algorithm_values():
    config = ShadowPlannerConfig()

    assert config.maximum_agents == 5
    assert config.num_samples == 196
    assert config.dt == 0.1
    assert config.plan_steps == 25
    assert config.kernel_a1 == 0.2
    assert config.kernel_a2 == 0.2
    assert config.cost_a1 == 15.0
    assert config.cost_a2 == 3.0
    assert config.cost_a3 == 20.0
    assert config.pedestrian_sample_scale == 0.1
    assert config.close_stop_threshold == 0.20
    assert config.interaction_separation_margin == 0.20
    assert config.crossing_lateral_speed_threshold == 0.08
    assert config.crossing_minimum_lateral_alignment == 0.50
    assert config.head_on_minimum_approach_speed == 0.12
    assert config.head_on_maximum_direction_angle_rad == 1.05
    assert config.crossing_time_max == 4.0
    assert config.crossing_forward_min == 0.20
    assert config.crossing_forward_max == 2.00
    assert config.crossing_side_bias_multiplier == 10.00
    assert config.crossing_preferred_safety_weight == 0.10


def test_shadow_planner_runs_brne_and_clamps_the_raw_command():
    """A fixed robot, goal, and moving pedestrian produce finite bounded output."""
    config = ShadowPlannerConfig(
        num_samples=16,
        plan_steps=12,
        max_linear_velocity=0.30,
        max_angular_velocity=0.80,
        nominal_linear_velocity=0.20,
    )
    result = BrneShadowPlanner(config).plan(
        robot_pose=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0],
        pedestrians=[[1.20, 0.50, 0.0, -0.10]],
    )

    assert result is not None
    assert result.trajectory.shape == (12, 3)
    assert np.isfinite(result.trajectory).all()
    assert 0.0 <= result.linear_velocity <= config.max_linear_velocity
    assert abs(result.angular_velocity) <= config.max_angular_velocity


def test_shadow_planner_follows_the_path_deterministically_with_no_agents():
    result = BrneShadowPlanner().plan(
        robot_pose=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0],
        pedestrians=[],
    )

    assert result is not None
    assert result.linear_velocity == 0.20
    assert result.angular_velocity == 0.0
    assert result.trajectory.shape == (25, 3)
    assert result.trajectory[-1, 0] > result.trajectory[0, 0]


def test_no_agent_path_fallback_slows_before_a_large_heading_correction():
    result = BrneShadowPlanner().plan(
        robot_pose=[0.0, 0.0, 0.0],
        goal=[0.0, 2.0],
        pedestrians=[],
    )

    assert result is not None
    assert result.linear_velocity == 0.0
    assert result.angular_velocity == 0.80


def test_no_agent_path_fallback_stops_inside_goal_tolerance():
    result = BrneShadowPlanner().plan(
        robot_pose=[0.0, 0.0, 0.0],
        goal=[0.1, 0.0],
        pedestrians=[],
    )

    assert result is not None
    assert result.linear_velocity == 0.0
    assert result.angular_velocity == 0.0


def test_shadow_planner_still_fails_closed_when_pedestrian_input_is_missing():
    assert BrneShadowPlanner().plan(
        robot_pose=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0],
        pedestrians=None,
    ) is None


def test_shadow_planner_rejects_duplicate_pedestrian_ids():
    assert BrneShadowPlanner().plan(
        robot_pose=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0],
        pedestrians=[
            (4, [1.0, 0.2, 0.0, 0.0]),
            (4, [1.2, -0.2, 0.0, 0.0]),
        ],
    ) is None


def test_shadow_planner_returns_a_stationary_finite_plan_at_goal():
    """A close goal is explicitly stationary rather than numerically uncontrolled."""
    result = BrneShadowPlanner().plan(
        robot_pose=[0.0, 0.0, 0.0],
        goal=[0.1, 0.0],
        pedestrians=[[1.20, 0.50, 0.0, -0.10]],
    )

    assert result is not None
    assert result.linear_velocity == 0.0
    assert result.angular_velocity == 0.0
    assert np.isfinite(result.trajectory).all()


def test_warm_up_returns_a_finite_runtime_plan():
    planner = BrneShadowPlanner()
    result = planner.warm_up()

    assert result is not None
    assert result.trajectory.shape == (25, 3)
    assert np.isfinite(result.trajectory).all()


def test_close_stop_mask_uses_time_aligned_cv_prediction():
    planner = BrneShadowPlanner()
    robot_ensemble = np.zeros((planner.config.plan_steps, 3, planner.config.num_samples))
    stationary = planner._robot_safety_mask(
        robot_ensemble, [np.array([0.19, 0.0, 0.0, 0.0])]
    )
    moving_away = planner._robot_safety_mask(
        robot_ensemble, [np.array([0.19, 0.0, 0.40, 0.0])]
    )

    assert planner.config.close_stop_threshold == 0.20
    assert np.all(stationary == 0.0)
    assert np.all(moving_away == 1.0)


def test_close_stop_pedestrian_trajectory_is_unperturbed_cv_mean():
    planner = BrneShadowPlanner(
        ShadowPlannerConfig(num_samples=4, plan_steps=3, dt=0.1)
    )

    trajectory = planner._pedestrian_mean(
        np.array([1.0, 2.0, 0.30, -0.40])
    )

    assert np.allclose(trajectory, [
        [1.03, 1.96],
        [1.06, 1.92],
        [1.09, 1.88],
    ])


def test_close_stop_mask_checks_corresponding_times_not_spatial_path_crossing():
    planner = BrneShadowPlanner(
        ShadowPlannerConfig(num_samples=4, plan_steps=3, dt=0.1)
    )
    asynchronous = np.zeros((3, 3, 4))
    asynchronous[:, 0, :] = np.array([0.0, 1.0, 2.0])[:, np.newaxis]
    aligned = np.zeros((3, 3, 4))
    aligned[:, 0, :] = np.array([1.0, 2.0, 3.0])[:, np.newaxis]
    pedestrian = [np.array([0.0, 0.0, 10.0, 0.0])]

    asynchronous_mask = planner._robot_safety_mask(asynchronous, pedestrian)
    aligned_mask = planner._robot_safety_mask(aligned, pedestrian)

    assert np.all(asynchronous_mask == 1.0)
    assert np.all(aligned_mask == 0.0)


def test_separating_preferred_crossing_candidate_gets_soft_safety_weight():
    planner = BrneShadowPlanner(ShadowPlannerConfig(
        num_samples=4,
        plan_steps=4,
        dt=0.1,
        close_stop_threshold=0.5,
    ))
    ensemble = np.zeros((4, 3, 4))
    ensemble[:, 0, :] = np.array([
        [0.6, 0.6, 0.7, 0.7],
        [0.4, 0.4, 0.6, 0.7],
        [0.5, 0.5, 0.5, 0.7],
        [0.7, 0.7, 0.4, 0.7],
    ])
    preferred = np.array([True, False, True, True])

    factors = planner._robot_safety_mask(
        ensemble,
        [np.array([0.0, 0.0, 0.0, 0.0])],
        pedestrian_ids=[8],
        softened_pedestrian_id=8,
        softened_candidates=preferred,
    )

    # Only candidate 0 is preferred, breaches the outer threshold, and ends
    # more than the existing 0.20 m separation margin beyond its closest point.
    assert np.allclose(factors, [0.1, 0.0, 0.0, 1.0])


def test_crossing_soft_safety_does_not_override_a_second_pedestrian_mask():
    planner = BrneShadowPlanner(ShadowPlannerConfig(
        num_samples=4,
        plan_steps=4,
        dt=0.1,
        close_stop_threshold=0.5,
    ))
    ensemble = np.zeros((4, 3, 4))
    ensemble[:, 0, :] = np.array([
        [0.6, 0.6, 0.7, 0.7],
        [0.4, 0.4, 0.6, 0.7],
        [0.5, 0.5, 0.5, 0.7],
        [0.7, 0.7, 0.4, 0.7],
    ])
    pedestrian = np.array([0.0, 0.0, 0.0, 0.0])

    factors = planner._robot_safety_mask(
        ensemble,
        [pedestrian, pedestrian],
        pedestrian_ids=[8, 9],
        softened_pedestrian_id=8,
        softened_candidates=np.ones(4, dtype=bool),
    )

    assert np.allclose(factors, [0.0, 0.0, 0.0, 1.0])


def _crossing_bias_fixture(v_lateral):
    config = ShadowPlannerConfig(
        num_samples=4,
        plan_steps=3,
        dt=0.5,
    )
    planner = BrneShadowPlanner(config)
    robot = np.array([0.0, 0.0, np.pi / 2.0])
    forward_axis = np.array([0.0, 1.0])
    left_axis = np.array([-1.0, 0.0])
    p_lateral = -np.sign(v_lateral) * 0.5
    pedestrian = np.concatenate((
        0.8 * forward_axis + p_lateral * left_axis,
        v_lateral * left_axis,
    ))
    ensemble = np.zeros((3, 3, 4))
    local_lateral = np.array([-0.30, 0.0, 0.30, -0.20])
    ensemble[1, :2, :] = (
        local_lateral[:, np.newaxis] * left_axis
    ).T
    return planner, robot, ensemble, pedestrian


def test_crossing_bias_uses_current_robot_frame_and_rewards_passing_behind():
    planner, robot, ensemble, pedestrian = _crossing_bias_fixture(0.5)

    bias = planner._crossing_side_bias(robot, ensemble, [pedestrian])

    # The robot faces world +Y, so robot-right is world +X. A pedestrian
    # moving robot-left (world -X) rewards the two robot-right candidates.
    assert np.allclose(bias, [10.0, 1.0, 1.0, 10.0])


def test_crossing_bias_is_left_right_symmetric():
    planner, robot, ensemble, pedestrian = _crossing_bias_fixture(-0.5)

    bias = planner._crossing_side_bias(robot, ensemble, [pedestrian])

    assert np.allclose(bias, [1.0, 1.0, 10.0, 1.0])


def test_crossing_bias_stays_neutral_outside_imminent_crossing_gates():
    planner, robot, ensemble, pedestrian = _crossing_bias_fixture(0.5)
    cases = (
        np.array([*pedestrian[:2], *(-0.07 * np.array([-1.0, 0.0]))]),
        np.array([1.0, 0.8, -0.20, 0.0]),
        np.array([2.1, 0.8, -0.50, 0.0]),
        np.array([0.5, 2.1, -0.50, 0.0]),
    )

    for non_crossing in cases:
        bias = planner._crossing_side_bias(robot, ensemble, [non_crossing])
        assert np.allclose(bias, np.ones(4))


def test_head_on_velocity_noise_cannot_trigger_crossing_bias():
    planner = BrneShadowPlanner(ShadowPlannerConfig(
        num_samples=4,
        plan_steps=3,
    ))
    robot = np.array([0.0, 0.0, 0.0])
    ensemble = np.zeros((3, 3, 4))
    head_on = np.array([1.0, -0.1, -0.25, 0.09])

    bias = planner._crossing_side_bias(robot, ensemble, [head_on])

    assert np.allclose(bias, np.ones(4))
    assert planner.last_crossing_bias_records[0]['gate'] == 'head_on'


def test_full_velocity_classifier_prefers_head_on_at_45_degrees():
    result = _interaction_velocity_classification(
        -0.2,
        0.2,
        crossing_lateral_speed_threshold=0.08,
        crossing_minimum_lateral_alignment=0.50,
        head_on_minimum_approach_speed=0.12,
        head_on_maximum_direction_angle_rad=1.05,
    )

    assert result == 'head_on'


def test_full_velocity_classifier_keeps_near_lateral_motion_as_crossing():
    result = _interaction_velocity_classification(
        -0.086,
        0.235,
        crossing_lateral_speed_threshold=0.08,
        crossing_minimum_lateral_alignment=0.50,
        head_on_minimum_approach_speed=0.12,
        head_on_maximum_direction_angle_rad=1.05,
    )

    assert result == 'crossing'


def test_full_velocity_classifier_is_exclusive_for_fast_diagonal_motion():
    result = _interaction_velocity_classification(
        -10.0,
        1.0,
        crossing_lateral_speed_threshold=0.08,
        crossing_minimum_lateral_alignment=0.50,
        head_on_minimum_approach_speed=0.12,
        head_on_maximum_direction_angle_rad=1.05,
    )

    assert result == 'head_on'


def test_crossing_bias_accepts_crossing_between_point_25_and_four_seconds():
    planner, robot, ensemble, _ = _crossing_bias_fixture(0.5)
    ensemble[2] = ensemble[1]
    pedestrian = np.array([1.5, 0.8, -0.5, 0.0])

    bias = planner._crossing_side_bias(
        robot, ensemble, [pedestrian], pedestrian_ids=[22]
    )

    assert np.any(bias > 1.0)
    assert planner.last_crossing_bias_records[0]['t_cross'] == 3.0
    assert planner.last_crossing_bias_records[0]['gate'] == 'eligible'
    assert planner.last_crossing_bias_summary['selected_pedestrian_id'] == 22


def test_crossing_bias_multiplies_original_brne_weights_before_mixing(monkeypatch):
    config = ShadowPlannerConfig(
        num_samples=4,
        plan_steps=3,
        dt=0.5,
        max_linear_velocity=1.0,
        max_angular_velocity=1.0,
        close_stop_threshold=0.01,
    )
    planner = BrneShadowPlanner(config)
    controls = np.tile(
        [[0.20, -0.50], [0.20, -0.20], [0.20, 0.20], [0.20, 0.50]],
        (config.plan_steps, 1, 1),
    )
    weights = np.ones(4)
    monkeypatch.setattr(brne, 'get_ulist_essemble', lambda *_: controls)
    monkeypatch.setattr(brne, 'brne_nav', lambda *_: np.array([weights, weights]))

    result = planner.plan(
        robot_pose=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0],
        pedestrians=[[0.8, -0.5, 0.0, 0.5]],
        crossing_interaction_pedestrian_id=1,
    )

    assert result is not None
    assert result.angular_velocity < 0.0
    assert (
        planner.last_crossing_bias_summary['final_preferred_weight_share']
        > planner.last_crossing_bias_summary['core_preferred_weight_share']
    )


def test_initial_crossing_window_zeros_same_direction_candidates(monkeypatch):
    config = ShadowPlannerConfig(
        num_samples=4,
        plan_steps=3,
        max_linear_velocity=1.0,
        max_angular_velocity=1.0,
        close_stop_threshold=0.01,
    )
    planner = BrneShadowPlanner(config)
    controls = np.tile(
        [[0.2, -0.5], [0.2, -0.2], [0.2, 0.2], [0.2, 0.5]],
        (config.plan_steps, 1, 1),
    )
    weights = np.ones(4)
    monkeypatch.setattr(brne, 'get_ulist_essemble', lambda *_: controls)
    monkeypatch.setattr(brne, 'brne_nav', lambda *_: np.array([weights, weights]))

    result = planner.plan(
        robot_pose=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0],
        pedestrians=[[1.0, -0.5, 0.0, 0.2]],
        crossing_interaction_pedestrian_id=1,
        initial_crossing_preferred_side=-1,
    )

    assert result is not None
    assert result.angular_velocity < 0.0
    assert planner.last_crossing_bias_summary[
        'initial_direction_safe_candidates'
    ] == 2


def test_head_on_side_mask_zeros_the_opposite_immediate_direction(monkeypatch):
    config = ShadowPlannerConfig(
        num_samples=4,
        plan_steps=3,
        max_linear_velocity=1.0,
        max_angular_velocity=1.0,
        close_stop_threshold=0.01,
    )
    planner = BrneShadowPlanner(config)
    controls = np.tile(
        [[0.2, -0.5], [0.2, -0.2], [0.2, 0.2], [0.2, 0.5]],
        (config.plan_steps, 1, 1),
    )
    weights = np.ones(4)
    monkeypatch.setattr(brne, 'get_ulist_essemble', lambda *_: controls)
    monkeypatch.setattr(brne, 'brne_nav', lambda *_: np.array([weights, weights]))

    result = planner.plan(
        robot_pose=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0],
        pedestrians=[[2.0, 0.0, -0.2, 0.0]],
        head_on_preferred_side=1,
    )

    assert result is not None
    assert result.angular_velocity > 0.0
    assert planner.last_crossing_bias_summary['preferred_candidates'] == 0
    assert planner.last_crossing_bias_summary['head_on_direction_side'] == 1


def test_weighted_control_sequence_aggregates_only_the_sample_axis():
    controls = np.array(
        [
            [[0.10, -0.30], [0.20, -0.10], [0.40, 0.20], [0.80, 0.50]],
            [[0.15, -0.20], [0.25, 0.00], [0.45, 0.30], [0.85, 0.60]],
            [[0.20, -0.10], [0.30, 0.10], [0.50, 0.40], [0.90, 0.70]],
        ]
    )
    weights = np.array([1.0, 2.0, 3.0, 4.0])

    weighted_controls = BrneShadowPlanner._weighted_control_sequence(
        controls,
        weights,
        float(np.sum(weights)),
    )

    expected = np.sum(controls * weights[np.newaxis, :, np.newaxis], axis=1) / 10.0
    assert weighted_controls.shape == (3, 2)
    assert np.allclose(weighted_controls, expected)


def test_weighted_control_sequence_already_normalizes_biased_masked_weights():
    controls = np.array([
        [[0.10, -0.30], [0.20, 0.10], [0.30, 0.40]],
        [[0.15, -0.20], [0.25, 0.20], [0.35, 0.50]],
    ])
    biased_masked_weights = np.array([2.0, 0.0, 1.0])

    mixed = BrneShadowPlanner._weighted_control_sequence(
        controls,
        biased_masked_weights,
        float(np.sum(biased_masked_weights)),
    )
    rescaled = BrneShadowPlanner._weighted_control_sequence(
        controls,
        7.0 * biased_masked_weights,
        float(np.sum(7.0 * biased_masked_weights)),
    )

    assert np.allclose(mixed, rescaled)


def test_weighted_control_first_step_is_independent_of_plan_length():
    weights = np.array([1.0, 2.0, 3.0, 4.0])
    short_controls = np.array(
        [
            [[0.10, -0.30], [0.20, -0.10], [0.40, 0.20], [0.80, 0.50]],
            [[0.15, -0.20], [0.25, 0.00], [0.45, 0.30], [0.85, 0.60]],
        ]
    )
    long_controls = np.vstack((short_controls, short_controls, short_controls))

    short_weighted = BrneShadowPlanner._weighted_control_sequence(
        short_controls,
        weights,
        float(np.sum(weights)),
    )
    long_weighted = BrneShadowPlanner._weighted_control_sequence(
        long_controls,
        weights,
        float(np.sum(weights)),
    )

    assert np.allclose(long_weighted[0], short_weighted[0])


def test_plan_uses_first_mixed_control_and_visualizes_its_trajectory(monkeypatch):
    config = ShadowPlannerConfig(
        num_samples=4,
        plan_steps=3,
        max_linear_velocity=1.0,
        max_angular_velocity=1.0,
        nominal_linear_velocity=0.20,
    )
    planner = BrneShadowPlanner(config)
    controls = np.array(
        [
            [[0.10, -0.30], [0.20, -0.10], [0.40, 0.20], [0.80, 0.50]],
            [[0.15, -0.20], [0.25, 0.00], [0.45, 0.30], [0.85, 0.60]],
            [[0.20, -0.10], [0.30, 0.10], [0.50, 0.40], [0.90, 0.70]],
        ]
    )
    weights = np.array([1.0, 2.0, 3.0, 4.0])
    monkeypatch.setattr(brne, 'get_ulist_essemble', lambda *_: controls)
    monkeypatch.setattr(brne, 'brne_nav', lambda *_: np.array([weights, weights]))

    result = planner.plan(
        robot_pose=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0],
        pedestrians=[[10.0, 10.0, 0.0, 0.0]],
    )

    expected_controls = BrneShadowPlanner._weighted_control_sequence(
        controls,
        weights,
        float(np.sum(weights)),
    )
    expected_trajectory = brne.traj_sim_essemble(
        np.array([[0.0], [0.0], [0.0]]),
        expected_controls[:, np.newaxis, :],
        config.dt,
    )[:, :, 0]
    argmax_trajectory = brne.traj_sim_essemble(
        np.zeros((3, config.num_samples)),
        controls,
        config.dt,
    )[:, :, int(np.argmax(weights))]

    assert result is not None
    assert np.allclose(
        [result.linear_velocity, result.angular_velocity], expected_controls[0]
    )
    assert np.allclose(result.trajectory, expected_trajectory)
    assert not np.allclose(result.trajectory, argmax_trajectory)


def _angular_support(planner, nominal_angular):
    controls = np.tile(
        [planner.config.nominal_linear_velocity, nominal_angular],
        (planner.config.plan_steps, 1),
    )
    ensemble = brne.get_ulist_essemble(
        controls,
        planner.config.max_linear_velocity,
        planner.config.max_angular_velocity,
        planner.config.num_samples,
    )
    return float(np.min(ensemble[:, :, 1])), float(np.max(ensemble[:, :, 1]))


def test_pinned_boundary_nominal_reproduces_one_sided_support():
    planner = BrneShadowPlanner()

    assert _angular_support(planner, -0.8) == (-0.8, -0.8)
    assert _angular_support(planner, 0.8) == (0.8, 0.8)


def test_proposal_protection_scale_restores_two_sided_sampler_support():
    planner = BrneShadowPlanner()

    angular_min, angular_max = _angular_support(planner, -0.8 * 0.25)

    assert angular_min < 0.0 < angular_max


def test_proposal_override_sets_sampler_nominal_without_forcing_command(monkeypatch):
    config = ShadowPlannerConfig(
        num_samples=4,
        plan_steps=3,
        max_linear_velocity=1.0,
        max_angular_velocity=0.8,
    )
    planner = BrneShadowPlanner(config)
    controls = np.tile(
        [[0.20, -0.40], [0.20, 0.10], [0.20, 0.30], [0.20, 0.40]],
        (config.plan_steps, 1, 1),
    )
    captured_nominals = []

    def capture_controls(nominal_controls, *_):
        captured_nominals.append(nominal_controls.copy())
        return controls

    weights = np.array([1.0, 1.0, 4.0, 4.0])
    monkeypatch.setattr(brne, 'get_ulist_essemble', capture_controls)
    monkeypatch.setattr(brne, 'brne_nav', lambda *_: np.array([weights, weights]))

    result = planner.plan(
        robot_pose=[0.0, 0.0, 0.0],
        goal=[0.0, -2.0],
        pedestrians=[[2.0, 0.0, 0.0, 0.0]],
        proposal_nominal_angular_override=0.0,
    )

    assert result is not None
    assert result.diagnostics.path_nominal_angular == -0.8
    assert result.diagnostics.proposal_nominal_angular == 0.0
    assert np.all(captured_nominals[0][:, 1] == 0.0)
    expected = planner._weighted_control_sequence(controls, weights, 10.0)
    assert result.angular_velocity == expected[0, 1]
    assert result.angular_velocity > 0.0


def test_proposal_nominal_override_rejects_nonfinite_and_out_of_bounds_values():
    planner = BrneShadowPlanner(ShadowPlannerConfig(num_samples=4, plan_steps=3))
    arguments = ([0.0, 0.0, 0.0], [2.0, 0.0], [[1.0, 0.0, 0.0, 0.0]])

    assert planner.plan(
        *arguments, proposal_nominal_angular_override=float('nan')
    ) is None
    assert planner.plan(
        *arguments, proposal_nominal_angular_override=0.81
    ) is None
