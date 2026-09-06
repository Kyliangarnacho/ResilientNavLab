"""Isolated ROS 2 shadow wrapper for the BRNE Python/Numba core."""

from math import atan2
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from resilient_nav_interfaces.msg import PedestrianArray

from .interaction_state import InteractionState
from .shadow_planner import (
    BrneShadowPlanner,
    ShadowPlannerConfig,
    _interaction_velocity_classification,
)


READY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class BrneShadowNode(Node):
    """Plan only on /brne/* topics and publish bounded, non-authoritative output."""

    def __init__(self):
        """Create the isolated I/O graph and conservative numerical configuration."""
        super().__init__('brne_shadow_node')
        self.declare_parameter('expected_frame', 'odom')
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('replan_frequency_hz', 5.0)
        self.declare_parameter('maximum_agents', 5)
        self.declare_parameter('num_samples', 196)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('plan_steps', 25)
        self.declare_parameter('max_linear_velocity', 0.30)
        self.declare_parameter('max_angular_velocity', 0.80)
        self.declare_parameter('nominal_linear_velocity', 0.20)
        self.declare_parameter('kernel_a1', 0.2)
        self.declare_parameter('kernel_a2', 0.2)
        self.declare_parameter('cost_a1', 15.0)
        self.declare_parameter('cost_a2', 3.0)
        self.declare_parameter('cost_a3', 20.0)
        self.declare_parameter('ped_sample_scale', 0.1)
        self.declare_parameter('corridor_y_min', -2.0)
        self.declare_parameter('corridor_y_max', 2.0)
        self.declare_parameter('close_stop_threshold', 0.20)
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('interaction_entry_distance', 3.20)
        self.declare_parameter('interaction_separation_margin', 0.20)
        self.declare_parameter('interaction_distance_history_outputs', 4)
        self.declare_parameter('crossing_lateral_speed_threshold', 0.08)
        self.declare_parameter('crossing_minimum_lateral_alignment', 0.50)
        self.declare_parameter('crossing_time_max', 4.0)
        self.declare_parameter('crossing_forward_min', 0.20)
        self.declare_parameter('crossing_forward_max', 2.00)
        self.declare_parameter('crossing_side_bias_multiplier', 10.00)
        self.declare_parameter('crossing_preferred_safety_weight', 0.10)
        self.declare_parameter('crossing_initial_direction_window_outputs', 5)
        self.declare_parameter('head_on_minimum_approach_speed', 0.12)
        self.declare_parameter('head_on_maximum_direction_angle_rad', 1.05)
        self.declare_parameter('head_on_lateral_direction_deadband_rad', 0.17)
        self.declare_parameter('head_on_release_path_clearance', 0.58)
        self.declare_parameter('proposal_opposite_scale', 0.25)
        self.declare_parameter('proposal_opposite_threshold', 0.35)
        self.declare_parameter('proposal_protection_window_outputs', 5)
        self.declare_parameter('proposal_protection_output_deadband', 0.05)

        self.expected_frame = self.get_parameter('expected_frame').value
        self.input_timeout_sec = float(self.get_parameter('input_timeout_sec').value)
        frequency = float(self.get_parameter('replan_frequency_hz').value)
        if self.input_timeout_sec <= 0.0 or frequency <= 0.0:
            raise ValueError('input_timeout_sec and replan_frequency_hz must be positive')
        self.planner = BrneShadowPlanner(ShadowPlannerConfig(
            maximum_agents=int(self.get_parameter('maximum_agents').value),
            num_samples=int(self.get_parameter('num_samples').value),
            dt=float(self.get_parameter('dt').value),
            plan_steps=int(self.get_parameter('plan_steps').value),
            max_linear_velocity=float(self.get_parameter('max_linear_velocity').value),
            max_angular_velocity=float(self.get_parameter('max_angular_velocity').value),
            nominal_linear_velocity=float(self.get_parameter('nominal_linear_velocity').value),
            kernel_a1=float(self.get_parameter('kernel_a1').value),
            kernel_a2=float(self.get_parameter('kernel_a2').value),
            cost_a1=float(self.get_parameter('cost_a1').value),
            cost_a2=float(self.get_parameter('cost_a2').value),
            cost_a3=float(self.get_parameter('cost_a3').value),
            pedestrian_sample_scale=float(
                self.get_parameter('ped_sample_scale').value
            ),
            corridor_y_min=float(self.get_parameter('corridor_y_min').value),
            corridor_y_max=float(self.get_parameter('corridor_y_max').value),
            close_stop_threshold=float(self.get_parameter('close_stop_threshold').value),
            goal_tolerance=float(self.get_parameter('goal_tolerance').value),
            interaction_separation_margin=float(
                self.get_parameter('interaction_separation_margin').value
            ),
            crossing_lateral_speed_threshold=float(
                self.get_parameter('crossing_lateral_speed_threshold').value
            ),
            crossing_minimum_lateral_alignment=float(
                self.get_parameter('crossing_minimum_lateral_alignment').value
            ),
            head_on_minimum_approach_speed=float(
                self.get_parameter('head_on_minimum_approach_speed').value
            ),
            head_on_maximum_direction_angle_rad=float(
                self.get_parameter('head_on_maximum_direction_angle_rad').value
            ),
            crossing_time_max=float(
                self.get_parameter('crossing_time_max').value
            ),
            crossing_forward_min=float(
                self.get_parameter('crossing_forward_min').value
            ),
            crossing_forward_max=float(
                self.get_parameter('crossing_forward_max').value
            ),
            crossing_side_bias_multiplier=float(
                self.get_parameter('crossing_side_bias_multiplier').value
            ),
            crossing_preferred_safety_weight=float(
                self.get_parameter('crossing_preferred_safety_weight').value
            ),
        ))
        self._last_stop_reason = None
        self.get_logger().info(
            'BRNE time-aligned safety mask configured: '
            f'close_stop_threshold={self.planner.config.close_stop_threshold:.3f} m'
        )
        self.proposal_opposite_scale = float(
            self.get_parameter('proposal_opposite_scale').value
        )
        self.proposal_opposite_threshold = float(
            self.get_parameter('proposal_opposite_threshold').value
        )
        self.proposal_protection_window_outputs = int(
            self.get_parameter('proposal_protection_window_outputs').value
        )
        self.proposal_protection_output_deadband = float(
            self.get_parameter('proposal_protection_output_deadband').value
        )
        self.interaction_entry_distance = float(
            self.get_parameter('interaction_entry_distance').value
        )
        self.crossing_initial_direction_window_outputs = int(
            self.get_parameter(
                'crossing_initial_direction_window_outputs'
            ).value
        )
        self.head_on_lateral_direction_deadband_rad = float(
            self.get_parameter('head_on_lateral_direction_deadband_rad').value
        )
        self.head_on_release_path_clearance = float(
            self.get_parameter('head_on_release_path_clearance').value
        )
        if (
            self.proposal_protection_window_outputs < 2
            or self.proposal_protection_output_deadband <= 0.0
            or not 0.0 < self.proposal_opposite_scale <= 1.0
            or self.proposal_opposite_threshold <= 0.0
            or self.interaction_entry_distance <= 0.0
            or self.crossing_initial_direction_window_outputs < 1
            or not 0.0 < self.head_on_lateral_direction_deadband_rad
            < self.planner.config.head_on_maximum_direction_angle_rad
            or self.head_on_release_path_clearance <= 0.0
        ):
            raise ValueError('proposal temporal protection parameters are invalid')
        self._interaction = InteractionState(
            separation_margin=float(
                self.get_parameter('interaction_separation_margin').value
            ),
            history_size=int(
                self.get_parameter('interaction_distance_history_outputs').value
            ),
            entry_distance=self.interaction_entry_distance,
        )
        self._passing_side = 0
        self._passing_side_output_count = 0
        self._crossing_event_side = 0
        self._crossing_event_complete = False
        self._head_on_event_active = False
        self._head_on_event_complete = False
        self._head_on_path_origin = None
        self._head_on_path_direction = None
        self._head_on_path_clearance = None
        self.cmd_publisher = self.create_publisher(Twist, '/brne/cmd_vel_raw', 10)
        self.path_publisher = self.create_publisher(Path, '/brne/optimal_path', 10)
        self.ready_publisher = self.create_publisher(Bool, '/brne/ready', READY_QOS)
        self._warmup_complete = False
        self.ready_publisher.publish(Bool(data=False))
        warmup_start = time.perf_counter_ns()
        try:
            if self.planner.warm_up() is None:
                raise RuntimeError('warm-up returned no finite BRNE plan')
        except Exception as error:
            self.get_logger().error(
                f'BRNE JIT warm-up failed; demo remains not ready: {type(error).__name__}'
            )
        else:
            warmup_ms = (time.perf_counter_ns() - warmup_start) / 1_000_000.0
            self._warmup_complete = True
            self.ready_publisher.publish(Bool(data=True))
            self.get_logger().info(f'BRNE JIT warm-up complete: warmup_ms={warmup_ms:.3f}')
        self.create_subscription(Odometry, '/brne/odom', self._odom_callback, 10)
        self.create_subscription(PoseStamped, '/brne/goal_pose', self._goal_callback, 10)
        self.create_subscription(PedestrianArray, '/brne/pedestrians', self._pedestrian_callback, 10)
        self._robot_pose = None
        self._goal = None
        self._pedestrians = None
        self._received_at = {}
        self._input_versions = {'odom': 0, 'goal': 0, 'pedestrians': 0}
        self._input_lock = threading.Lock()
        self.last_compute_ms = None
        self.compute_latencies_ms = []
        self.compute_count = 0
        self.create_timer(1.0 / frequency, self._plan_timer)

    def _odom_callback(self, message):
        """Accept one finite odometry pose only in the isolated odom frame."""
        if message.header.frame_id != self.expected_frame:
            with self._input_lock:
                self._robot_pose = None
            return
        orientation = message.pose.pose.orientation
        yaw = atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        values = np.array([
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw,
        ])
        if not np.isfinite(values).all():
            with self._input_lock:
                self._robot_pose = None
            return
        with self._input_lock:
            self._robot_pose = values
            self._received_at['odom'] = time.monotonic()
            self._input_versions['odom'] += 1

    def _goal_callback(self, message):
        """Accept one finite local goal only in the isolated odom frame."""
        if message.header.frame_id != self.expected_frame:
            with self._input_lock:
                self._goal = None
            return
        values = np.array([message.pose.position.x, message.pose.position.y])
        if not np.isfinite(values).all():
            with self._input_lock:
                self._goal = None
            return
        with self._input_lock:
            self._goal = values
            self._received_at['goal'] = time.monotonic()
            self._input_versions['goal'] += 1

    def _pedestrian_callback(self, message):
        """Accept a fresh empty array as observed no-agent, not missing input."""
        if message.header.frame_id != self.expected_frame:
            with self._input_lock:
                self._pedestrians = None
            self._reset_interaction_state()
            return
        if not message.pedestrians:
            with self._input_lock:
                self._pedestrians = []
                self._received_at['pedestrians'] = time.monotonic()
                self._input_versions['pedestrians'] += 1
            self._reset_interaction_state()
            return
        pedestrians = []
        pedestrian_ids = set()
        for pedestrian in message.pedestrians:
            pedestrian_id = int(pedestrian.id)
            if pedestrian_id < 0 or pedestrian_id in pedestrian_ids:
                with self._input_lock:
                    self._pedestrians = None
                self._reset_interaction_state()
                return
            values = np.array([
                pedestrian.pose.position.x,
                pedestrian.pose.position.y,
                pedestrian.velocity.linear.x,
                pedestrian.velocity.linear.y,
            ])
            if not np.isfinite(values).all():
                with self._input_lock:
                    self._pedestrians = None
                self._reset_interaction_state()
                return
            pedestrian_ids.add(pedestrian_id)
            pedestrians.append((pedestrian_id, values))
        with self._input_lock:
            self._pedestrians = pedestrians
            self._received_at['pedestrians'] = time.monotonic()
            self._input_versions['pedestrians'] += 1

    def _plan_timer(self):
        """Publish a plan and raw command only while all required inputs stay fresh."""
        if not self._warmup_complete:
            self._publish_stop('JIT warm-up incomplete')
            return
        snapshot = self._fresh_input_snapshot()
        if snapshot is None:
            self._reset_interaction_state()
            self._publish_stop('required input missing or stale')
            return
        robot_pose, goal, pedestrians, versions = snapshot
        nominal_override = self._proposal_nominal_override(
            robot_pose, goal, pedestrians
        )
        start = time.perf_counter_ns()
        if nominal_override is None:
            plan = self.planner.plan(
                robot_pose,
                goal,
                pedestrians,
                crossing_interaction_pedestrian_id=(
                    self._interaction.pedestrian_id
                ),
                initial_crossing_preferred_side=(
                    self._initial_crossing_preferred_side()
                ),
                head_on_preferred_side=self._head_on_preferred_side(),
            )
        else:
            plan = self.planner.plan(
                robot_pose,
                goal,
                pedestrians,
                proposal_nominal_angular_override=nominal_override,
                crossing_interaction_pedestrian_id=(
                    self._interaction.pedestrian_id
                ),
                initial_crossing_preferred_side=(
                    self._initial_crossing_preferred_side()
                ),
                head_on_preferred_side=self._head_on_preferred_side(),
            )
        self.last_compute_ms = (time.perf_counter_ns() - start) / 1_000_000.0
        # Numba's initial compilation can last much longer than the input
        # watchdog.  Never issue a non-zero command from the pre-JIT snapshot;
        # in a multi-threaded executor also reject a replaced input set.
        if not self._snapshot_still_fresh(versions):
            self._publish_stop('input changed or became stale during planning')
            return
        if plan is None:
            self._reset_interaction_state()
            self._publish_stop('planner rejected input or numerical result')
            return
        self._record_passing_side_direction(plan.angular_velocity)
        self.compute_count += 1
        self.compute_latencies_ms.append(self.last_compute_ms)
        self.get_logger().info(
            f'BRNE shadow plan {self.compute_count} '
            f'compute_ms={self.last_compute_ms:.3f}'
        )
        if (
            plan.diagnostics is not None
        ):
            records = '; '.join(
                'id={pedestrian_id} pf={p_forward:.2f} pl={p_lateral:.2f} '
                'vf={v_forward:.2f} vl={v_lateral:.2f} tc={t_cross} '
                'xc={x_cross} gate={gate}'.format(**{
                    **record,
                    't_cross': _optional_number(record['t_cross']),
                    'x_cross': _optional_number(record['x_cross']),
                })
                for record in self.planner.last_crossing_bias_records
            )
            summary = self.planner.last_crossing_bias_summary
            self.get_logger().info(
                'BRNE interaction policy: '
                f'[{records}] selected={summary.get("selected_pedestrian_id")}, '
                f'preferred_side={summary.get("preferred_side")}, '
                f'preferred_candidates={summary.get("preferred_candidates")}, '
                f'safe_preferred={summary.get("safe_preferred_candidates")}, '
                f'initial_direction_side='
                f'{summary.get("initial_direction_side")}, '
                f'initial_direction_safe='
                f'{summary.get("initial_direction_safe_candidates")}, '
                f'softened_preferred='
                f'{summary.get("softened_preferred_candidates")}, '
                f'interaction_owner='
                f'{self._interaction.pedestrian_id}, '
                f'event_side={self._crossing_event_side}, '
                f'event_complete={self._crossing_event_complete}, '
                f'head_on_active={self._head_on_event_active}, '
                f'head_on_complete={self._head_on_event_complete}, '
                f'head_on_side={self._head_on_preferred_side()}, '
                f'head_on_path_clearance={self._head_on_path_clearance}, '
                f'protected_output_count={self._passing_side_output_count}, '
                f'core_preferred_weight_share='
                f'{summary.get("core_preferred_weight_share", 0.0):.3f}, '
                f'preferred_weight_share='
                f'{summary.get("final_preferred_weight_share", 0.0):.3f}, '
                f'multiplier={self.planner.config.crossing_side_bias_multiplier:.1f}, '
                f'path_nominal={plan.diagnostics.path_nominal_angular:.3f}, '
                f'proposal_nominal={plan.diagnostics.proposal_nominal_angular:.3f}, '
                f'angular_support=[{plan.diagnostics.angular_candidate_min}, '
                f'{plan.diagnostics.angular_candidate_max}], '
                f'command_angular={plan.angular_velocity:.3f}'
            )
        if plan.diagnostics is not None:
            diagnostics = plan.diagnostics
            proposal_protected = not np.isclose(
                diagnostics.path_nominal_angular,
                diagnostics.proposal_nominal_angular,
            )
            self.get_logger().debug(
                'BRNE proposal diagnostics: '
                f'path_nominal={diagnostics.path_nominal_angular:.3f}, '
                f'proposal_nominal={diagnostics.proposal_nominal_angular:.3f}, '
                f'proposal_protected={proposal_protected}, '
                f'passing_side={self._passing_side}, '
                f'passing_side_output_count={self._passing_side_output_count}, '
                f'command_angular={plan.angular_velocity:.3f}, '
                f'angular_support=[{diagnostics.angular_candidate_min}, '
                f'{diagnostics.angular_candidate_max}], '
                f'safe_candidates={diagnostics.safe_candidate_count}'
            )
        command = Twist()
        command.linear.x = plan.linear_velocity
        command.angular.z = plan.angular_velocity
        self._report_plan_command_state(plan)
        self.cmd_publisher.publish(command)
        self.path_publisher.publish(self._path_message(plan.trajectory))

    def _inputs_ready(self):
        """Require all three independent inputs to be present and recently received."""
        return self._fresh_input_snapshot() is not None

    def _fresh_input_snapshot(self):
        """Copy one coherent, fresh input set for a potentially slow calculation."""
        with self._input_lock:
            if self._robot_pose is None or self._goal is None or self._pedestrians is None:
                return None
            now = time.monotonic()
            names = ('odom', 'goal', 'pedestrians')
            if not all(
                now - self._received_at.get(name, float('-inf')) <= self.input_timeout_sec
                for name in names
            ):
                return None
            return (
                self._robot_pose.copy(),
                self._goal.copy(),
                self._pedestrian_snapshot(),
                {name: self._input_versions[name] for name in names},
            )

    def _pedestrian_snapshot(self):
        """Copy explicit ID observations; retain legacy numeric test inputs."""
        snapshot = []
        for index, entry in enumerate(self._pedestrians):
            if isinstance(entry, tuple):
                pedestrian_id, pedestrian = entry
            else:
                pedestrian_id, pedestrian = index + 1, entry
            snapshot.append((pedestrian_id, pedestrian.copy()))
        return snapshot

    def _snapshot_still_fresh(self, versions):
        """Require that the calculation's exact input generation remains fresh."""
        with self._input_lock:
            names = ('odom', 'goal', 'pedestrians')
            if any(self._input_versions[name] != versions[name] for name in names):
                return False
            now = time.monotonic()
            return all(
                now - self._received_at.get(name, float('-inf')) <= self.input_timeout_sec
                for name in names
            )

    def _publish_stop(self, reason='unspecified fail-closed condition'):
        """Fail safe with an explicit zero raw command; never publish to /cmd_vel."""
        self._report_stop_reason(reason)
        self.cmd_publisher.publish(Twist())

    def _report_plan_command_state(self, plan):
        """Report why a completed BRNE plan emitted an exact zero command."""
        if not np.isclose(plan.linear_velocity, 0.0) or not np.isclose(
            plan.angular_velocity, 0.0
        ):
            self._last_stop_reason = None
            return
        diagnostics = plan.diagnostics
        if diagnostics is None:
            reason = 'completed planner returned zero command without diagnostics'
        elif (
            diagnostics.angular_candidate_min is not None
            and diagnostics.safe_candidate_count == 0
        ):
            reason = (
                'safety mask rejected all robot candidates '
                f'(threshold={self.planner.config.close_stop_threshold:.3f} m)'
            )
        elif diagnostics.angular_candidate_min is None:
            reason = 'local goal is within goal tolerance'
        else:
            reason = 'BRNE weighted control is zero with safe candidates remaining'
        self._report_stop_reason(reason)

    def _report_stop_reason(self, reason):
        """Log only stop-reason transitions so the 5 Hz loop stays readable."""
        if reason == self._last_stop_reason:
            return
        self._last_stop_reason = reason
        self.get_logger().warning(f'BRNE raw command stopped: {reason}')

    def _reset_interaction_state(self):
        """Discard the active interaction and event state after input loss."""
        self._interaction.reset()
        self._passing_side = 0
        self._passing_side_output_count = 0
        self._crossing_event_side = 0
        self._crossing_event_complete = False
        self._reset_head_on_event()
        self._head_on_event_complete = False

    def _proposal_nominal_override(self, robot_pose, goal, pedestrians):
        """Preserve passing-side proposal support during one interaction."""
        pedestrian_positions = {
            int(pedestrian_id): (float(state[0]), float(state[1]))
            for pedestrian_id, state in pedestrians
        }
        transition = self._interaction.update(
            (float(robot_pose[0]), float(robot_pose[1])),
            pedestrian_positions,
        )
        if transition.entered_pedestrian_id is not None:
            self._passing_side = 0
            self._passing_side_output_count = 0
            self._crossing_event_side = 0
            self._crossing_event_complete = False
            self._reset_head_on_event()
            self._head_on_event_complete = False
        if transition.released_pedestrian_id is not None:
            self._passing_side = 0
            self._passing_side_output_count = 0
            self._crossing_event_side = 0
            self._crossing_event_complete = False
            self._reset_head_on_event()
            self._head_on_event_complete = False
            return None
        self._update_interaction_event(robot_pose, pedestrians)
        head_on_protection = (
            self._head_on_event_active and self._passing_side != 0
        )
        if (
            not self._interaction.active
            or self._passing_side == 0
            or (
                not head_on_protection
                and self._passing_side_output_count
                >= self.proposal_protection_window_outputs
            )
        ):
            return None
        path_nominal = _path_nominal_angular(
            robot_pose,
            goal,
            self.planner.config.max_angular_velocity,
        )
        if self._passing_side * path_nominal < -self.proposal_opposite_threshold:
            return self.proposal_opposite_scale * path_nominal
        return None

    def _update_interaction_event(self, robot_pose, pedestrians):
        """Select one mutually exclusive crossing or head-on interaction event."""
        owner_id = self._interaction.pedestrian_id
        if owner_id is None:
            return
        owner = next(
            (state for pedestrian_id, state in pedestrians
             if int(pedestrian_id) == owner_id),
            None,
        )
        if owner is None:
            return
        motion_class = self._pedestrian_velocity_event_class(robot_pose, owner)
        if motion_class != 'crossing':
            self._crossing_event_complete = False
        if motion_class != 'head_on':
            self._head_on_event_complete = False
        if self._head_on_event_active:
            self._update_head_on_release(robot_pose)
            return
        p_lateral, preferred_side = self._crossing_event_geometry(
            robot_pose, owner
        )
        if self._crossing_event_side != 0:
            if self._crossing_event_side * p_lateral <= 0.0:
                self._crossing_event_side = 0
                self._crossing_event_complete = True
                self.get_logger().info(
                    f'crossing event {owner_id} cleared robot centerline'
                )
            return
        if preferred_side != 0:
            if self._crossing_event_complete:
                return
            self._crossing_event_side = preferred_side
            self._passing_side = preferred_side
            self._passing_side_output_count = 0
            self.get_logger().info(
                f'crossing event {owner_id} entered: '
                f'preferred_robot_side={preferred_side}'
            )
            return
        if motion_class == 'head_on':
            if not self._head_on_event_complete and self._head_on_event_geometry(
                robot_pose, owner
            ):
                velocity = np.asarray(owner[2:], dtype=float)
                v_forward, v_lateral = self._pedestrian_velocity_in_robot_frame(
                    robot_pose, owner
                )
                direction_angle = atan2(v_lateral, -v_forward)
                preferred_side = 0
                if (
                    abs(direction_angle)
                    >= self.head_on_lateral_direction_deadband_rad
                ):
                    preferred_side = int(-np.sign(v_lateral))
                self._head_on_event_active = True
                self._head_on_path_origin = np.asarray(
                    owner[:2], dtype=float
                ).copy()
                self._head_on_path_direction = velocity / np.linalg.norm(velocity)
                self._head_on_path_clearance = 0.0
                self._passing_side = preferred_side
                self._passing_side_output_count = 0
                if preferred_side == 0:
                    self.get_logger().info(
                        f'head-on event {owner_id} entered inside '
                        'lateral-direction deadband; awaiting first BRNE side'
                    )
                else:
                    self.get_logger().info(
                        f'head-on event {owner_id} entered: '
                        f'velocity_angle={direction_angle:.3f} rad, '
                        f'preferred_robot_side={preferred_side}'
                    )
            return

    def _pedestrian_velocity_event_class(self, robot_pose, pedestrian):
        """Classify one observed velocity in the current robot frame."""
        v_forward, v_lateral = self._pedestrian_velocity_in_robot_frame(
            robot_pose, pedestrian
        )
        return self._velocity_event_class(v_forward, v_lateral)

    @staticmethod
    def _pedestrian_velocity_in_robot_frame(robot_pose, pedestrian):
        """Return one pedestrian velocity as robot forward/lateral components."""
        forward = np.array([
            np.cos(float(robot_pose[2])), np.sin(float(robot_pose[2]))
        ])
        left = np.array([-forward[1], forward[0]])
        velocity = np.asarray(pedestrian[2:], dtype=float)
        return (
            float(velocity @ forward),
            float(velocity @ left),
        )

    def _crossing_event_geometry(self, robot_pose, pedestrian):
        forward = np.array([
            np.cos(float(robot_pose[2])), np.sin(float(robot_pose[2]))
        ])
        left = np.array([-forward[1], forward[0]])
        relative_position = np.asarray(pedestrian[:2]) - np.asarray(robot_pose[:2])
        p_forward = float(relative_position @ forward)
        p_lateral = float(relative_position @ left)
        v_forward = float(np.asarray(pedestrian[2:]) @ forward)
        v_lateral = float(np.asarray(pedestrian[2:]) @ left)
        if self._velocity_event_class(v_forward, v_lateral) != 'crossing':
            return p_lateral, 0
        t_cross = -p_lateral / v_lateral
        x_cross = p_forward + v_forward * t_cross
        if (
            t_cross <= 0.0
            or t_cross > self.planner.config.crossing_time_max
            or x_cross < self.planner.config.crossing_forward_min
            or x_cross > self.planner.config.crossing_forward_max
        ):
            return p_lateral, 0
        return p_lateral, int(-np.sign(v_lateral))

    def _head_on_event_geometry(self, robot_pose, pedestrian):
        """Recognize an approaching longitudinal agent without choosing a side."""
        forward = np.array([
            np.cos(float(robot_pose[2])), np.sin(float(robot_pose[2]))
        ])
        left = np.array([-forward[1], forward[0]])
        relative_position = np.asarray(pedestrian[:2]) - np.asarray(robot_pose[:2])
        p_forward = float(relative_position @ forward)
        velocity = np.asarray(pedestrian[2:], dtype=float)
        v_forward = float(velocity @ forward)
        v_lateral = float(velocity @ left)
        return (
            p_forward >= self.planner.config.crossing_forward_min
            and self._velocity_event_class(v_forward, v_lateral) == 'head_on'
        )

    def _velocity_event_class(self, v_forward, v_lateral):
        """Apply the one-of crossing/head-on full-vector direction rule."""
        config = self.planner.config
        return _interaction_velocity_classification(
            v_forward,
            v_lateral,
            crossing_lateral_speed_threshold=(
                config.crossing_lateral_speed_threshold
            ),
            crossing_minimum_lateral_alignment=(
                config.crossing_minimum_lateral_alignment
            ),
            head_on_minimum_approach_speed=(
                config.head_on_minimum_approach_speed
            ),
            head_on_maximum_direction_angle_rad=(
                config.head_on_maximum_direction_angle_rad
            ),
        )

    def _update_head_on_release(self, robot_pose):
        """Release after the robot clears the frozen unperturbed CV path."""
        if (
            self._head_on_path_origin is None
            or self._head_on_path_direction is None
        ):
            self._reset_head_on_event()
            return
        offset = np.asarray(robot_pose[:2]) - self._head_on_path_origin
        direction = self._head_on_path_direction
        self._head_on_path_clearance = abs(
            direction[0] * offset[1] - direction[1] * offset[0]
        )
        if self._head_on_path_clearance < self.head_on_release_path_clearance:
            return
        self._reset_head_on_event()
        self._head_on_event_complete = True
        self._passing_side = 0
        self._passing_side_output_count = self.proposal_protection_window_outputs
        self.get_logger().info('head-on event cleared frozen pedestrian CV path')

    def _reset_head_on_event(self):
        self._head_on_event_active = False
        self._head_on_path_origin = None
        self._head_on_path_direction = None
        self._head_on_path_clearance = None

    def _initial_crossing_preferred_side(self):
        if (
            self._crossing_event_side != 0
            and self._passing_side_output_count
            < self.crossing_initial_direction_window_outputs
        ):
            return self._crossing_event_side
        return 0

    def _head_on_preferred_side(self):
        if self._head_on_event_active:
            return self._passing_side
        return 0

    def _record_passing_side_direction(self, angular_velocity):
        """Anchor once, then count the configured finite output window."""
        if (
            not self._interaction.active
            or self._head_on_event_complete
        ):
            return
        if self._passing_side == 0:
            if abs(angular_velocity) < self.proposal_protection_output_deadband:
                return
            self._passing_side = 1 if angular_velocity > 0.0 else -1
            self._passing_side_output_count = 1
            return
        if self._passing_side_output_count < self.proposal_protection_window_outputs:
            self._passing_side_output_count += 1

    def _path_message(self, trajectory):
        """Convert the finite numeric trajectory into an odom-frame Path."""
        message = Path()
        message.header.frame_id = self.expected_frame
        message.header.stamp = self.get_clock().now().to_msg()
        for x_position, y_position, yaw in trajectory:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(x_position)
            pose.pose.position.y = float(y_position)
            pose.pose.orientation.z = float(np.sin(yaw / 2.0))
            pose.pose.orientation.w = float(np.cos(yaw / 2.0))
            message.poses.append(pose)
        return message


def main(args=None):
    """Run the standalone shadow node without a controller or selector."""
    rclpy.init(args=args)
    node = BrneShadowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _path_nominal_angular(robot_pose, goal, maximum_angular_velocity):
    """Match ShadowPlanner's path heading controller without running BRNE."""
    delta = np.asarray(goal, dtype=float) - np.asarray(robot_pose[:2], dtype=float)
    heading_error = atan2(delta[1], delta[0]) - float(robot_pose[2])
    heading_error = (heading_error + np.pi) % (2.0 * np.pi) - np.pi
    return float(np.clip(
        2.0 * heading_error,
        -maximum_angular_velocity,
        maximum_angular_velocity,
    ))


def _optional_number(value):
    return 'none' if value is None else f'{value:.2f}'
