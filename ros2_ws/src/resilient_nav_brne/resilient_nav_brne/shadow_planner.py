"""ROS-free adaptation of the pinned BRNE core for shadow planning."""

from dataclasses import dataclass
from math import atan2, pi
from typing import Sequence

import numpy as np

from . import brne


@dataclass(frozen=True)
class ShadowPlannerConfig:
    """Explicit numerical limits for the isolated BRNE shadow planner."""

    maximum_agents: int = 5
    num_samples: int = 196
    dt: float = 0.1
    plan_steps: int = 25
    max_linear_velocity: float = 0.30
    max_angular_velocity: float = 0.80
    nominal_linear_velocity: float = 0.20
    kernel_a1: float = 0.2
    kernel_a2: float = 0.2
    cost_a1: float = 15.0
    cost_a2: float = 3.0
    cost_a3: float = 20.0
    pedestrian_sample_scale: float = 0.1
    corridor_y_min: float = -2.0
    corridor_y_max: float = 2.0
    close_stop_threshold: float = 0.20
    goal_tolerance: float = 0.20
    interaction_separation_margin: float = 0.20
    crossing_lateral_speed_threshold: float = 0.08
    crossing_minimum_lateral_alignment: float = 0.50
    head_on_minimum_approach_speed: float = 0.12
    head_on_maximum_direction_angle_rad: float = 1.05
    crossing_time_max: float = 4.0
    crossing_forward_min: float = 0.20
    crossing_forward_max: float = 2.00
    crossing_side_bias_multiplier: float = 10.00
    crossing_preferred_safety_weight: float = 0.10

    def __post_init__(self):
        """Reject configurations that cannot satisfy the upstream sampler contract."""
        sample_side = int(np.sqrt(self.num_samples))
        if self.maximum_agents < 2:
            raise ValueError('maximum_agents must be at least 2')
        if self.num_samples < 4 or sample_side * sample_side != self.num_samples:
            raise ValueError('num_samples must be a square integer of at least 4')
        if self.dt <= 0.0 or self.plan_steps < 2:
            raise ValueError('dt must be positive and plan_steps must be at least 2')
        if not 0.0 < self.nominal_linear_velocity <= self.max_linear_velocity:
            raise ValueError('nominal velocity must be in (0, max_linear_velocity]')
        if self.max_angular_velocity <= 0.0:
            raise ValueError('max_angular_velocity must be positive')
        if self.corridor_y_min >= self.corridor_y_max:
            raise ValueError('corridor_y_min must be below corridor_y_max')
        if self.interaction_separation_margin <= 0.0:
            raise ValueError('interaction_separation_margin must be positive')
        if (
            self.crossing_lateral_speed_threshold <= 0.0
            or not 0.0 < self.crossing_minimum_lateral_alignment <= 1.0
            or self.head_on_minimum_approach_speed <= 0.0
            or not 0.0 < self.head_on_maximum_direction_angle_rad < pi / 2.0
            or self.crossing_time_max <= 0.0
            or self.crossing_forward_min < 0.0
            or self.crossing_forward_max <= self.crossing_forward_min
            or self.crossing_side_bias_multiplier < 1.0
            or not 0.0 < self.crossing_preferred_safety_weight <= 1.0
        ):
            raise ValueError('crossing side-bias parameters are invalid')


@dataclass(frozen=True)
class ShadowPlanDiagnostics:
    """Per-plan evidence for proposal support and safety selection."""

    path_nominal_angular: float
    proposal_nominal_angular: float
    angular_candidate_min: float | None
    angular_candidate_max: float | None
    safe_candidate_count: int


@dataclass(frozen=True)
class ShadowPlan:
    """Finite, bounded raw command and trajectory produced by BRNE."""

    linear_velocity: float
    angular_velocity: float
    trajectory: np.ndarray
    compute_ms: float
    diagnostics: ShadowPlanDiagnostics | None = None


@dataclass(frozen=True)
class _PedestrianObservation:
    """Identity plus the pinned BRNE [x, y, vx, vy] numeric state."""

    pedestrian_id: int
    state: np.ndarray


class BrneShadowPlanner:
    """Use the upstream BRNE routines without ROS or hardware policy."""

    def __init__(self, config: ShadowPlannerConfig | None = None):
        """Precompute the covariance factor used by pedestrian trajectory samples."""
        self.config = config or ShadowPlannerConfig()
        time_points = np.arange(self.config.plan_steps, dtype=float) * self.config.dt
        self._covariance_factor, _ = brne.get_Lmat_nb(
            np.array([time_points[0]]),
            time_points,
            np.array([1e-4]),
            self.config.kernel_a1,
            self.config.kernel_a2,
        )
        self.last_crossing_bias_records: tuple[dict, ...] = ()
        self.last_crossing_bias_summary: dict = {}

    def plan(
        self,
        robot_pose: Sequence[float],
        goal: Sequence[float],
        pedestrians: Sequence[Sequence[float]],
        *,
        proposal_nominal_angular_override: float | None = None,
        crossing_interaction_pedestrian_id: int | None = None,
        initial_crossing_preferred_side: int = 0,
        head_on_preferred_side: int = 0,
    ) -> ShadowPlan | None:
        """Compute one bounded plan, or fail closed when inputs are unsuitable."""
        robot = _finite_vector(robot_pose, 3)
        goal_xy = _finite_vector(goal, 2)
        if robot is None or goal_xy is None or pedestrians is None:
            return None
        pedestrian_data = [
            _pedestrian_observation(pedestrian, fallback_id=index + 1)
            for index, pedestrian in enumerate(pedestrians)
        ]
        if not pedestrian_data:
            return self._no_agent_path_following_plan(robot, goal_xy)
        if any(pedestrian is None for pedestrian in pedestrian_data):
            return None
        if len({pedestrian.pedestrian_id for pedestrian in pedestrian_data}) != len(pedestrian_data):
            return None
        if (
            initial_crossing_preferred_side not in (-1, 0, 1)
            or head_on_preferred_side not in (-1, 0, 1)
        ):
            return None

        delta = goal_xy - robot[:2]
        goal_distance = float(np.linalg.norm(delta))
        if goal_distance <= self.config.goal_tolerance:
            return self._stopped_plan(robot, diagnostics=self._diagnostics())

        ordered_observations = sorted(
            pedestrian_data,
            key=lambda observation: (
                float(np.linalg.norm(robot[:2] - observation.state[:2])),
                observation.pedestrian_id,
            ),
        )
        selected_observations = ordered_observations[: self.config.maximum_agents - 1]
        ordered_pedestrians = [observation.state for observation in selected_observations]
        agent_count = len(ordered_pedestrians) + 1
        if agent_count < 2:
            return None

        heading_error = _normalize_angle(atan2(delta[1], delta[0]) - robot[2])
        path_nominal_angular = float(np.clip(
            2.0 * heading_error,
            -self.config.max_angular_velocity,
            self.config.max_angular_velocity,
        ))
        if proposal_nominal_angular_override is None:
            proposal_nominal_angular = path_nominal_angular
        else:
            if (
                not np.isfinite(proposal_nominal_angular_override)
                or abs(proposal_nominal_angular_override)
                > self.config.max_angular_velocity
            ):
                return None
            proposal_nominal_angular = float(proposal_nominal_angular_override)
        nominal_controls = np.tile(
            [self.config.nominal_linear_velocity, proposal_nominal_angular],
            (self.config.plan_steps, 1),
        )
        control_ensemble = brne.get_ulist_essemble(
            nominal_controls,
            self.config.max_linear_velocity,
            self.config.max_angular_velocity,
            self.config.num_samples,
        )
        robot_ensemble = brne.traj_sim_essemble(
            np.tile(robot, (self.config.num_samples, 1)).T,
            control_ensemble,
            self.config.dt,
        )
        x_samples, y_samples = self._assemble_samples(
            robot_ensemble,
            ordered_pedestrians,
            agent_count,
        )
        weights = brne.brne_nav(
            x_samples,
            y_samples,
            agent_count,
            self.config.plan_steps,
            self.config.num_samples,
            self.config.cost_a1,
            self.config.cost_a2,
            self.config.cost_a3,
            self.config.pedestrian_sample_scale,
            self.config.corridor_y_min,
            self.config.corridor_y_max,
        )
        if weights is None or not np.isfinite(weights).all():
            return None

        crossing_observations = [
            observation for observation in selected_observations
            if observation.pedestrian_id == crossing_interaction_pedestrian_id
        ]
        crossing_bias = self._crossing_side_bias(
            robot,
            robot_ensemble,
            [observation.state for observation in crossing_observations],
            pedestrian_ids=[
                observation.pedestrian_id for observation in crossing_observations
            ],
        )
        preferred_candidates = crossing_bias > 1.0
        safety_mask = self._robot_safety_mask(
            robot_ensemble,
            ordered_pedestrians,
            pedestrian_ids=[
                observation.pedestrian_id for observation in selected_observations
            ],
            softened_pedestrian_id=crossing_interaction_pedestrian_id,
            softened_candidates=preferred_candidates,
        )
        initial_direction_mask = self._preferred_direction_mask(
            control_ensemble,
            initial_crossing_preferred_side,
        )
        head_on_direction_mask = self._preferred_direction_mask(
            control_ensemble,
            head_on_preferred_side,
        )
        self.last_crossing_bias_summary['safe_preferred_candidates'] = int(
            np.count_nonzero(preferred_candidates & safety_mask.astype(bool))
        )
        self.last_crossing_bias_summary['initial_direction_side'] = int(
            initial_crossing_preferred_side
        )
        self.last_crossing_bias_summary['initial_direction_safe_candidates'] = int(
            np.count_nonzero(
                initial_direction_mask.astype(bool) & safety_mask.astype(bool)
            )
        )
        self.last_crossing_bias_summary['head_on_direction_side'] = int(
            head_on_preferred_side
        )
        core_safe_weights = weights[0] * safety_mask
        core_safe_weight_sum = float(np.sum(core_safe_weights))
        self.last_crossing_bias_summary['core_preferred_weight_share'] = (
            0.0
            if core_safe_weight_sum <= 0.0 or not np.isfinite(core_safe_weight_sum)
            else float(
                np.sum(core_safe_weights[preferred_candidates])
                / core_safe_weight_sum
            )
        )
        angular_candidate_min = float(np.min(control_ensemble[:, :, 1]))
        angular_candidate_max = float(np.max(control_ensemble[:, :, 1]))
        safe_candidate_count = int(np.count_nonzero(safety_mask))
        if not np.any(safety_mask):
            return self._stopped_plan(
                robot,
                diagnostics=self._diagnostics(
                    path_nominal_angular=path_nominal_angular,
                    proposal_nominal_angular=proposal_nominal_angular,
                    angular_candidate_min=angular_candidate_min,
                    angular_candidate_max=angular_candidate_max,
                    safe_candidate_count=safe_candidate_count,
                ),
            )
        robot_weights = (
            weights[0]
            * crossing_bias
            * safety_mask
            * initial_direction_mask
            * head_on_direction_mask
        )
        weight_sum = float(np.sum(robot_weights))
        self.last_crossing_bias_summary['final_preferred_weight_share'] = (
            0.0
            if weight_sum <= 0.0 or not np.isfinite(weight_sum)
            else float(np.sum(robot_weights[preferred_candidates]) / weight_sum)
        )
        if not np.isfinite(weight_sum) or weight_sum <= 0.0:
            return self._stopped_plan(
                robot,
                diagnostics=self._diagnostics(
                    path_nominal_angular=path_nominal_angular,
                    proposal_nominal_angular=proposal_nominal_angular,
                    angular_candidate_min=angular_candidate_min,
                    angular_candidate_max=angular_candidate_max,
                    safe_candidate_count=safe_candidate_count,
                ),
            )

        weighted_controls = self._weighted_control_sequence(
            control_ensemble,
            robot_weights,
            weight_sum,
        )
        linear_velocity = float(weighted_controls[0, 0])
        angular_velocity = float(weighted_controls[0, 1])
        linear_velocity = float(np.clip(linear_velocity, 0.0, self.config.max_linear_velocity))
        angular_velocity = float(np.clip(
            angular_velocity,
            -self.config.max_angular_velocity,
            self.config.max_angular_velocity,
        ))
        trajectory = brne.traj_sim_essemble(
            robot.reshape(3, 1),
            weighted_controls[:, np.newaxis, :],
            self.config.dt,
        )[:, :, 0].copy()
        if not np.isfinite(trajectory).all():
            return None
        return ShadowPlan(
            linear_velocity,
            angular_velocity,
            trajectory,
            0.0,
            self._diagnostics(
                path_nominal_angular=path_nominal_angular,
                proposal_nominal_angular=proposal_nominal_angular,
                angular_candidate_min=angular_candidate_min,
                angular_candidate_max=angular_candidate_max,
                safe_candidate_count=safe_candidate_count,
            ),
        )

    def warm_up(self) -> ShadowPlan | None:
        """Compile every runtime Numba path before the demo grants pedestrian motion."""
        return self.plan(
            robot_pose=(0.0, 0.0, 0.0),
            goal=(0.8, 0.0),
            pedestrians=((0.5, -1.0, 0.0, 0.25),),
        )

    def _no_agent_path_following_plan(self, robot, goal_xy):
        """Follow the fresh Navfn waypoint deterministically when no agent exists."""
        delta = goal_xy - robot[:2]
        goal_distance = float(np.linalg.norm(delta))
        if goal_distance <= self.config.goal_tolerance:
            return self._stopped_plan(robot, diagnostics=self._diagnostics())

        heading_error = _normalize_angle(atan2(delta[1], delta[0]) - robot[2])
        angular_velocity = float(np.clip(
            2.0 * heading_error,
            -self.config.max_angular_velocity,
            self.config.max_angular_velocity,
        ))
        heading_scale = (
            0.0
            if abs(heading_error) >= pi / 2.0
            else max(0.0, float(np.cos(heading_error))) ** 2
        )
        slowdown_distance = max(
            2.0 * self.config.goal_tolerance,
            self.config.nominal_linear_velocity,
        )
        distance_scale = float(np.clip(
            (goal_distance - self.config.goal_tolerance)
            / (slowdown_distance - self.config.goal_tolerance),
            0.0,
            1.0,
        ))
        linear_velocity = (
            self.config.nominal_linear_velocity
            * heading_scale
            * distance_scale
        )
        controls = np.tile(
            [linear_velocity, angular_velocity],
            (self.config.plan_steps, 1),
        )
        trajectory = brne.traj_sim_essemble(
            robot.reshape(3, 1),
            controls[:, np.newaxis, :],
            self.config.dt,
        )[:, :, 0].copy()
        if not np.isfinite(trajectory).all():
            return None
        return ShadowPlan(
            linear_velocity,
            angular_velocity,
            trajectory,
            0.0,
            self._diagnostics(
                path_nominal_angular=angular_velocity,
                proposal_nominal_angular=angular_velocity,
            ),
        )

    def _assemble_samples(self, robot_ensemble, pedestrians, agent_count):
        """Borrow upstream pedestrian sampling while retaining only numeric inputs."""
        shape = (agent_count * self.config.num_samples, self.config.plan_steps)
        x_samples = np.zeros(shape)
        y_samples = np.zeros(shape)
        x_samples[: self.config.num_samples] = robot_ensemble[:, 0, :].T
        y_samples[: self.config.num_samples] = robot_ensemble[:, 1, :].T
        sample_count = len(pedestrians) * self.config.num_samples
        x_noise = brne.mvn_sample_normal(sample_count, self.config.plan_steps, self._covariance_factor)
        y_noise = brne.mvn_sample_normal(sample_count, self.config.plan_steps, self._covariance_factor)
        horizon = np.arange(self.config.plan_steps, dtype=float) * self.config.dt
        for index, pedestrian in enumerate(pedestrians):
            start = (index + 1) * self.config.num_samples
            end = start + self.config.num_samples
            speed = float(np.linalg.norm(pedestrian[2:]))
            x_mean = pedestrian[0] + horizon * pedestrian[2]
            y_mean = pedestrian[1] + horizon * pedestrian[3]
            x_samples[start:end] = x_noise[index * self.config.num_samples:(index + 1) * self.config.num_samples] * speed + x_mean
            y_samples[start:end] = y_noise[index * self.config.num_samples:(index + 1) * self.config.num_samples] * speed + y_mean
        return x_samples, y_samples

    def _pedestrian_mean(self, pedestrian):
        """Time-align a CV mean with post-integration robot trajectory samples."""
        horizon = (np.arange(self.config.plan_steps, dtype=float) + 1.0) * self.config.dt
        return pedestrian[:2] + horizon[:, np.newaxis] * pedestrian[2:]

    def _diagnostics(
        self,
        *,
        path_nominal_angular=0.0,
        proposal_nominal_angular=0.0,
        angular_candidate_min=None,
        angular_candidate_max=None,
        safe_candidate_count=0,
    ):
        """Snapshot explicit numerical evidence without affecting planner output."""
        return ShadowPlanDiagnostics(
            path_nominal_angular=float(path_nominal_angular),
            proposal_nominal_angular=float(proposal_nominal_angular),
            angular_candidate_min=angular_candidate_min,
            angular_candidate_max=angular_candidate_max,
            safe_candidate_count=int(safe_candidate_count),
        )

    @staticmethod
    def _weighted_control_sequence(control_ensemble, robot_weights, weight_sum):
        """Preserve time while mixing each control step over robot samples."""
        return np.sum(
            control_ensemble * robot_weights[np.newaxis, :, np.newaxis],
            axis=1,
        ) / weight_sum

    def _robot_safety_mask(
        self,
        robot_ensemble,
        pedestrians,
        *,
        pedestrian_ids=None,
        softened_pedestrian_id=None,
        softened_candidates=None,
    ):
        """Weight time-aligned CV proximity, softening only a separating crosser."""
        ids = (
            list(range(1, len(pedestrians) + 1))
            if pedestrian_ids is None else list(pedestrian_ids)
        )
        factors = np.ones(self.config.num_samples, dtype=float)
        softened_count = 0
        for pedestrian_id, pedestrian in zip(ids, pedestrians):
            pedestrian_mean = self._pedestrian_mean(pedestrian)
            dx = robot_ensemble[:, 0, :] - pedestrian_mean[:, 0, np.newaxis]
            dy = robot_ensemble[:, 1, :] - pedestrian_mean[:, 1, np.newaxis]
            distances = np.hypot(dx, dy)
            minimum_distances = np.min(distances, axis=0)
            pedestrian_mask = minimum_distances > self.config.close_stop_threshold
            pedestrian_factors = pedestrian_mask.astype(float)
            if (
                pedestrian_id == softened_pedestrian_id
                and softened_candidates is not None
            ):
                candidates = np.asarray(softened_candidates, dtype=bool)
                separating = (
                    distances[-1]
                    > minimum_distances + self.config.interaction_separation_margin
                )
                softened = ~pedestrian_mask & candidates & separating
                pedestrian_factors[softened] = (
                    self.config.crossing_preferred_safety_weight
                )
                softened_count += int(np.count_nonzero(softened))
            factors *= pedestrian_factors
        self.last_crossing_bias_summary['softened_preferred_candidates'] = (
            softened_count
        )
        return factors

    def _crossing_side_bias(
        self, robot, robot_ensemble, pedestrians, *, pedestrian_ids=None
    ):
        """Lightly reward candidates passing behind one imminent crosser."""
        bias = np.ones(self.config.num_samples, dtype=float)
        self.last_crossing_bias_records = ()
        self.last_crossing_bias_summary = {
            'selected_pedestrian_id': None,
            'preferred_side': 0,
            'preferred_candidates': 0,
            'safe_preferred_candidates': 0,
            'core_preferred_weight_share': 0.0,
            'final_preferred_weight_share': 0.0,
            'initial_direction_side': 0,
            'initial_direction_safe_candidates': 0,
            'head_on_direction_side': 0,
            'softened_preferred_candidates': 0,
        }
        forward_axis = np.array([np.cos(robot[2]), np.sin(robot[2])])
        left_axis = np.array([-forward_axis[1], forward_axis[0]])
        events = []
        records = []
        ids = (
            list(range(1, len(pedestrians) + 1))
            if pedestrian_ids is None else list(pedestrian_ids)
        )
        for pedestrian_id, pedestrian in zip(ids, pedestrians):
            relative_position = pedestrian[:2] - robot[:2]
            p_forward = float(relative_position @ forward_axis)
            p_lateral = float(relative_position @ left_axis)
            v_forward = float(pedestrian[2:] @ forward_axis)
            v_lateral = float(pedestrian[2:] @ left_axis)
            t_cross = None
            x_cross = None
            gate = 'eligible'
            motion_class = _interaction_velocity_classification(
                v_forward,
                v_lateral,
                crossing_lateral_speed_threshold=(
                    self.config.crossing_lateral_speed_threshold
                ),
                crossing_minimum_lateral_alignment=(
                    self.config.crossing_minimum_lateral_alignment
                ),
                head_on_minimum_approach_speed=(
                    self.config.head_on_minimum_approach_speed
                ),
                head_on_maximum_direction_angle_rad=(
                    self.config.head_on_maximum_direction_angle_rad
                ),
            )
            if motion_class != 'crossing':
                gate = motion_class or 'motion_direction'
            else:
                t_cross = -p_lateral / v_lateral
                x_cross = p_forward + v_forward * t_cross
                if t_cross <= 0.0 or t_cross > self.config.crossing_time_max:
                    gate = 'crossing_time'
                elif (
                    x_cross < self.config.crossing_forward_min
                    or x_cross > self.config.crossing_forward_max
                ):
                    gate = 'forward_position'
                else:
                    events.append((t_cross, x_cross, v_lateral, pedestrian_id))
            records.append({
                'pedestrian_id': int(pedestrian_id),
                'p_forward': p_forward,
                'p_lateral': p_lateral,
                'v_forward': v_forward,
                'v_lateral': v_lateral,
                't_cross': t_cross,
                'x_cross': x_cross,
                'gate': gate,
            })
        self.last_crossing_bias_records = tuple(records)

        if not events:
            return bias
        t_cross, _, v_lateral, pedestrian_id = min(events)
        crossing_step = int(np.clip(
            np.floor(t_cross / self.config.dt + 0.5) - 1,
            0,
            self.config.plan_steps - 1,
        ))
        candidate_displacements = (
            robot_ensemble[crossing_step, :2, :].T - robot[:2]
        )
        candidate_lateral = candidate_displacements @ left_axis
        preferred_side = -np.sign(v_lateral)
        preferred_candidates = preferred_side * candidate_lateral > 1e-6
        bias[preferred_candidates] = self.config.crossing_side_bias_multiplier
        self.last_crossing_bias_summary.update({
            'selected_pedestrian_id': int(pedestrian_id),
            'preferred_side': int(preferred_side),
            'preferred_candidates': int(np.count_nonzero(preferred_candidates)),
            'crossing_step': crossing_step,
        })
        return bias

    @staticmethod
    def _preferred_direction_mask(control_ensemble, preferred_side):
        """Keep only immediate angular candidates on an event-selected side."""
        if preferred_side == 0:
            return np.ones(control_ensemble.shape[1], dtype=float)
        angular_candidates = control_ensemble[0, :, 1]
        return (preferred_side * angular_candidates >= -1e-9).astype(float)

    def _stopped_plan(self, robot, diagnostics=None):
        """Return a finite stationary trajectory for close-goal or safety stops."""
        return ShadowPlan(
            0.0,
            0.0,
            np.tile(robot, (self.config.plan_steps, 1)),
            0.0,
            diagnostics,
        )


def _finite_vector(values, expected_size):
    """Coerce one fixed-size numeric vector or reject it without guessing."""
    array = np.asarray(values, dtype=float)
    if array.shape != (expected_size,) or not np.isfinite(array).all():
        return None
    return array


def _interaction_velocity_classification(
    v_forward,
    v_lateral,
    *,
    crossing_lateral_speed_threshold,
    crossing_minimum_lateral_alignment,
    head_on_minimum_approach_speed,
    head_on_maximum_direction_angle_rad,
):
    """Classify one velocity vector into exactly one interaction event."""
    speed = float(np.hypot(v_forward, v_lateral))
    if not np.isfinite(speed) or speed <= 1e-9:
        return None
    approach_speed = -float(v_forward)
    approach_alignment = approach_speed / speed
    if (
        approach_speed >= head_on_minimum_approach_speed
        and approach_alignment
        >= np.cos(head_on_maximum_direction_angle_rad)
    ):
        return 'head_on'
    lateral_alignment = abs(float(v_lateral)) / speed
    if (
        abs(float(v_lateral)) >= crossing_lateral_speed_threshold
        and lateral_alignment >= crossing_minimum_lateral_alignment
    ):
        return 'crossing'
    return None


def _pedestrian_observation(values, fallback_id):
    """Accept legacy state vectors and explicit (id, state) observations."""
    pedestrian_id = fallback_id
    state_values = values
    if isinstance(values, _PedestrianObservation):
        return values
    if isinstance(values, tuple) and len(values) == 2:
        pedestrian_id, state_values = values
    try:
        pedestrian_id = int(pedestrian_id)
    except (TypeError, ValueError, OverflowError):
        return None
    if pedestrian_id < 0:
        return None
    state = _finite_vector(state_values, 4)
    if state is None:
        return None
    return _PedestrianObservation(pedestrian_id, state)


def _normalize_angle(angle):
    """Normalize an angle to [-pi, pi] for bounded nominal steering."""
    return (angle + pi) % (2.0 * pi) - pi
