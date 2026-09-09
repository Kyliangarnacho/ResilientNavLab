"""Bounded Task 3.3 NavigateToPose smoke probe for healthy static scenes."""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import time
from pathlib import Path

import yaml
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog, Costmap
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from costmap_contract import yaw_from_quaternion
from global_costmap_probe import MAP_QOS, VOLATILE_RELIABLE_QOS
from path_safety import RawCostmapGrid, full_footprint_path_sweep


NAVIGATE_TO_POSE_ACTION = '/navigate_to_pose'
MAX_LINEAR_COMMAND_MPS = 0.22
MAX_ANGULAR_COMMAND_RADPS = 0.75
PATH_FEEDBACK_ASSOCIATION_MAX_SEC = 0.50
PATH_EVIDENCE_WAIT_SEC = 1.00
FINAL_TF_FEEDBACK_MAX_TRANSLATION_M = 0.10
FINAL_TF_FEEDBACK_MAX_YAW_RAD = 0.10
LIFECYCLE_SERVICES = {
    'planner': '/planner_server/get_state',
    'controller': '/controller_server/get_state',
    'bt_navigator': '/bt_navigator/get_state',
}
EXPECTED_BT_PARAMETERS = {
    'global_frame': 'map',
    'robot_base_frame': 'base_footprint',
    'odom_topic': '/odometry/filtered',
    'bt_loop_duration': 10,
    'navigators': ['navigate_to_pose'],
    'navigate_to_pose.plugin': 'nav2_bt_navigator::NavigateToPoseNavigator',
}


def normalize_angle(angle: float) -> float:
    """Normalize a planar angle into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def final_tf_feedback_cross_check(
    tf_pose: tuple[float, float, float], feedback_pose: tuple[float, float, float], *,
    divergence_is_error: bool,
) -> dict[str, object]:
    """Record the terminal TF/feedback comparison without hiding its outcome.

    A successful NavigateToPose remains subject to the historical strict
    endpoint check. A Task 5 safe-failure terminal state, however, can have
    a final action feedback pose that predates BT abort/cancel processing.
    In that case the numerical divergence remains evidence, but it cannot
    suppress the failure-chain evidence that the offline evaluator must see.
    """
    translation_m = math.dist(tf_pose[:2], feedback_pose[:2])
    yaw_rad = abs(normalize_angle(tf_pose[2] - feedback_pose[2]))
    diverged = (
        translation_m > FINAL_TF_FEEDBACK_MAX_TRANSLATION_M
        or yaw_rad > FINAL_TF_FEEDBACK_MAX_YAW_RAD
    )
    if diverged:
        outcome = 'ERROR' if divergence_is_error else 'WARNING'
    else:
        outcome = 'PASS'
    return {
        'translation_m': translation_m,
        'yaw_rad': yaw_rad,
        'max_translation_m': FINAL_TF_FEEDBACK_MAX_TRANSLATION_M,
        'max_yaw_rad': FINAL_TF_FEEDBACK_MAX_YAW_RAD,
        'diverged': diverged,
        'outcome': outcome,
    }


def pose_stamped(x: float, y: float, yaw: float) -> PoseStamped:
    """Construct one explicit map-frame navigation goal."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def parameter_value(value) -> object:
    """Convert the small parameter subset used by this runtime contract."""
    if value.type == 1:
        return value.bool_value
    if value.type == 2:
        return value.integer_value
    if value.type == 3:
        return value.double_value
    if value.type == 4:
        return value.string_value
    if value.type == 9:
        return list(value.string_array_value)
    raise ValueError(f'unsupported BT parameter type {value.type}')


def load_scenarios(path: Path) -> dict[str, dict[str, object]]:
    """Load only the frozen healthy NavigateToPose scenario contracts."""
    loaded = yaml.safe_load(path.read_text(encoding='utf-8'))
    scenarios = loaded.get('scenarios') if isinstance(loaded, dict) else None
    if not isinstance(scenarios, dict):
        raise ValueError(f'{path} has no scenarios mapping')
    selected = {}
    for name in ('simple_reachable', 'static_obstacle_detour'):
        specification = scenarios.get(name)
        if not isinstance(specification, dict) or specification.get('expected') != 'success':
            raise ValueError(f'{name} is not a successful frozen scenario')
        if not isinstance(specification.get('navigate_to_pose'), dict):
            raise ValueError(f'{name} has no NavigateToPose contract')
        selected[name] = specification
    return selected


def stamp_seconds(stamp) -> float:
    """Return one finite ROS timestamp in seconds."""
    value = stamp.sec + stamp.nanosec / 1e9
    if not math.isfinite(value):
        raise ValueError('received a non-finite ROS timestamp')
    return value


def feedback_pose_record(pose: PoseStamped) -> dict[str, float | str]:
    """Validate and normalize BT Navigator's map-frame feedback pose."""
    if pose.header.frame_id != 'map':
        raise ValueError(f'NavigateToPose feedback frame is {pose.header.frame_id!r}, not map')
    values = (
        pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.x,
        pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('NavigateToPose feedback pose contains a non-finite value')
    yaw = yaw_from_quaternion(pose.pose.orientation)
    if not math.isfinite(yaw):
        raise ValueError('NavigateToPose feedback yaw is non-finite')
    return {
        'frame_id': pose.header.frame_id,
        'stamp_sec': stamp_seconds(pose.header.stamp),
        'x': pose.pose.position.x,
        'y': pose.pose.position.y,
        'yaw': yaw,
    }


class NavigateToPoseProbe(Node):
    """Observe Nav2 and issue one outer navigation action only."""

    def __init__(self, action_name: str = NAVIGATE_TO_POSE_ACTION) -> None:
        super().__init__('phase10_navigate_to_pose_probe')
        if not action_name:
            raise ValueError('NavigateToPose action name must not be empty')
        self.action_name = action_name
        self.global_raw: Costmap | None = None
        self.local_raw: Costmap | None = None
        self.odom: Odometry | None = None
        self.received_plan: NavPath | None = None
        self.received_plan_messages: list[dict[str, object]] = []
        self.plan_messages: list[dict[str, object]] = []
        self.command_records: list[dict[str, float | None]] = []
        self.feedback_records: list[dict[str, float | int | str]] = []
        self.bt_events: list[dict[str, str | int]] = []
        self.clock_samples: list[float] = []
        self.clock_regressed = False
        # Gazebo can advance simulation time much faster than wall time during
        # smoke runs.  Keep enough ROS-time history for the post-action TF /
        # feedback cross-check; runtime footprint evidence uses feedback.
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lifecycle_clients = {
            name: self.create_client(GetState, service)
            for name, service in LIFECYCLE_SERVICES.items()
        }
        # Nav2 Lifecycle Manager owns configure/activate ordering.  Task 4
        # only needs its aggregate readiness fact and must not recreate a
        # per-node lifecycle manager through repeated GetState requests.
        self.navigation_manager_active_client = self.create_client(
            Trigger, '/lifecycle_manager_navigation/is_active'
        )
        self.parameter_client = self.create_client(GetParameters, '/bt_navigator/get_parameters')
        self.action_client = ActionClient(self, NavigateToPose, action_name)
        self.zero_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Costmap, '/global_costmap/costmap_raw', self._on_global_raw, MAP_QOS)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw', self._on_local_raw, MAP_QOS)
        self.create_subscription(NavPath, '/plan', self._on_plan, VOLATILE_RELIABLE_QOS)
        self.create_subscription(NavPath, '/received_global_plan', self._on_received_plan, VOLATILE_RELIABLE_QOS)
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self._on_bt_log, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self._on_command, 10)
        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)

    def _on_global_raw(self, message: Costmap) -> None:
        self.global_raw = message

    def _on_local_raw(self, message: Costmap) -> None:
        self.local_raw = message

    def _on_plan(self, message: NavPath) -> None:
        self.plan_messages.append({
            'message': copy.deepcopy(message),
            'received_monotonic': time.monotonic(),
            # Preserve the last planner Costmap visible when Nav2 published
            # this Path.  Task 5 later mutates the world, so re-reading the
            # current grid would retrospectively judge the pre-event route.
            'global_raw_snapshot': copy.deepcopy(self.global_raw),
        })

    def _on_received_plan(self, message: NavPath) -> None:
        self.received_plan = copy.deepcopy(message)
        self.received_plan_messages.append({
            'message': copy.deepcopy(message),
            'received_monotonic': time.monotonic(),
        })

    def _on_bt_log(self, message: BehaviorTreeLog) -> None:
        for event in message.event_log:
            self.bt_events.append({
                'node_name': event.node_name,
                'uid': int(event.uid),
                'previous_status': event.previous_status,
                'current_status': event.current_status,
                'sim_time_sec': self.clock_samples[-1] if self.clock_samples else None,
            })

    def _on_odom(self, message: Odometry) -> None:
        self.odom = message

    def _on_command(self, message: Twist) -> None:
        self.command_records.append({
            'linear_x': message.linear.x,
            'angular_z': message.angular.z,
            'time_monotonic': time.monotonic(),
            'sim_time_sec': self.clock_samples[-1] if self.clock_samples else None,
        })

    def _on_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        record = {
            'distance_remaining': feedback.distance_remaining,
            'number_of_recoveries': int(feedback.number_of_recoveries),
            'navigation_time_sec': feedback.navigation_time.sec + feedback.navigation_time.nanosec / 1e9,
            'estimated_time_remaining_sec': (
                feedback.estimated_time_remaining.sec
                + feedback.estimated_time_remaining.nanosec / 1e9
            ),
            'received_monotonic': time.monotonic(),
        }
        try:
            record.update(feedback_pose_record(feedback.current_pose))
        except ValueError as error:
            record['pose_error'] = str(error)
        self.feedback_records.append(record)

    def _on_clock(self, message: Clock) -> None:
        current = stamp_seconds(message.clock)
        if self.clock_samples and current < self.clock_samples[-1]:
            self.clock_regressed = True
        self.clock_samples.append(current)

    def lifecycle_active(self, name: str) -> bool:
        return self.lifecycle_state(name).get('state_id') == State.PRIMARY_STATE_ACTIVE

    def lifecycle_state(self, name: str) -> dict[str, object]:
        """Return one read-only lifecycle observation for application readiness."""
        client = self.lifecycle_clients[name]
        if not client.service_is_ready():
            return {'service_ready': False, 'state_id': None}
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if not future.done() or future.result() is None:
            return {'service_ready': True, 'state_id': None}
        state = future.result().current_state
        return {
            'service_ready': True,
            'state_id': int(state.id),
            'state_label': state.label,
        }

    def navigation_manager_active(self) -> dict[str, object]:
        """Observe Nav2's official aggregate lifecycle state once."""
        client = self.navigation_manager_active_client
        if not client.service_is_ready():
            return {'service_ready': False, 'active': False}
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if not future.done() or future.result() is None:
            return {
                'service_ready': True,
                'active': False,
                'response_received': False,
            }
        response = future.result()
        return {
            'service_ready': True,
            'active': bool(response.success),
            'response_received': True,
            'message': response.message,
        }

    def map_pose(self) -> tuple[float, float, float]:
        # Gazebo /clock and TF can arrive in separate executor turns.  Request
        # the latest transform and tolerate that bounded hand-off, rather than
        # rejecting a healthy navigation result on one past-extrapolation read.
        last_error = None
        for _ in range(5):
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', Time(), timeout=Duration(seconds=0.3)
                ).transform
                return (
                    transform.translation.x,
                    transform.translation.y,
                    yaw_from_quaternion(transform.rotation),
                )
            except Exception as error:
                last_error = error
                rclpy.spin_once(self, timeout_sec=0.05)
        raise last_error

    def clock_contract(self) -> dict[str, object]:
        """Require one monotonic ROS clock before commanding the robot."""
        publishers = self.get_publishers_info_by_topic('/clock')
        if len(publishers) != 1:
            raise ValueError(f'/clock has {len(publishers)} publishers, expected one')
        if len(self.clock_samples) < 3 or self.clock_regressed:
            raise ValueError('/clock is not yet a stable monotonic simulation clock')
        return {
            'publisher_node': publishers[0].node_name,
            'publisher_namespace': publishers[0].node_namespace,
            'sample_count': len(self.clock_samples),
            'first_sec': self.clock_samples[0],
            'last_sec': self.clock_samples[-1],
        }

    def bt_parameters(
        self, expected_xml_basename: str = 'navigate_w_replanning_time.xml',
    ) -> dict[str, object]:
        if not self.parameter_client.wait_for_service(timeout_sec=0.5):
            raise ValueError('BT Navigator parameter service is unavailable')
        names = list(EXPECTED_BT_PARAMETERS) + ['default_nav_to_pose_bt_xml']
        request = GetParameters.Request(names=names)
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            raise TimeoutError('timed out reading BT Navigator parameters')
        values = {
            name: parameter_value(value)
            for name, value in zip(names, future.result().values)
        }
        actual = {name: values[name] for name in EXPECTED_BT_PARAMETERS}
        if actual != EXPECTED_BT_PARAMETERS:
            raise ValueError(f'BT Navigator parameters differ from contract: {actual}')
        xml_path = values['default_nav_to_pose_bt_xml']
        if not isinstance(xml_path, str) or not xml_path.endswith(expected_xml_basename):
            raise ValueError(f'BT Navigator selected unexpected XML: {xml_path!r}')
        return values

    def node_names(self, name: str, namespace: str) -> list[str]:
        return sorted(
            found
            for found, found_namespace in self.get_node_names_and_namespaces()
            if found == name and found_namespace == namespace
        )

    def publish_zero_stop(self) -> int:
        """Publish only all-zero safety stop messages during probe shutdown."""
        for _ in range(5):
            self.zero_publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        return 5


def wait_for(probe: NavigateToPoseProbe, predicate, deadline: float, description: str) -> None:
    """Spin until a readiness predicate holds or report the missing fact."""
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if predicate():
            return
    raise TimeoutError(f'timed out waiting for {description}')


def wait_for_future(probe: NavigateToPoseProbe, future, timeout_sec: float, description: str):
    rclpy.spin_until_future_complete(probe, future, timeout_sec=timeout_sec)
    if not future.done() or future.result() is None:
        raise TimeoutError(f'timed out waiting for {description}')
    return future.result()


def associated_feedback(
    feedback: list[dict[str, object]], plan_record: dict[str, object]
) -> dict[str, object] | None:
    """Return the closest valid feedback pose in the frozen association window."""
    path = plan_record['message']
    if not isinstance(path, NavPath):
        raise ValueError('probe stored an invalid Path record')
    path_stamp = stamp_seconds(path.header.stamp)
    candidates = [item for item in feedback if 'stamp_sec' in item]
    if not candidates:
        return None
    stamped = min(candidates, key=lambda item: abs(float(item['stamp_sec']) - path_stamp))
    stamp_delta = abs(float(stamped['stamp_sec']) - path_stamp)
    if path_stamp > 0.0 and float(stamped['stamp_sec']) > 0.0 and stamp_delta <= PATH_FEEDBACK_ASSOCIATION_MAX_SEC:
        return {**stamped, 'association_mode': 'ros_stamp', 'association_delta_sec': stamp_delta}
    received = min(
        candidates,
        key=lambda item: abs(float(item['received_monotonic']) - float(plan_record['received_monotonic'])),
    )
    received_delta = abs(float(received['received_monotonic']) - float(plan_record['received_monotonic']))
    if received_delta > PATH_FEEDBACK_ASSOCIATION_MAX_SEC:
        return None
    return {**received, 'association_mode': 'received_monotonic', 'association_delta_sec': received_delta}


def sweep_observed_plan(
    probe: NavigateToPoseProbe, plan_record: dict[str, object], feedback: dict[str, object]
) -> dict[str, object]:
    """Keep one Path's full-footprint check external to the BT runtime tree."""
    snapshot = plan_record.get('global_raw_snapshot')
    if not isinstance(snapshot, Costmap) and probe.global_raw is None:
        raise ValueError('Planner Global Costmap raw snapshot is unavailable')
    path = plan_record['message']
    if not isinstance(path, NavPath) or path.header.frame_id != 'map' or not path.poses:
        raise ValueError('BT replanning published an empty or non-map Path')
    points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
    path_stamp = stamp_seconds(path.header.stamp)
    delta_sec = float(feedback['association_delta_sec'])
    start_error = math.dist(points[0], (float(feedback['x']), float(feedback['y'])))
    sweep = full_footprint_path_sweep(
        RawCostmapGrid.from_message(
            snapshot if isinstance(snapshot, Costmap) else probe.global_raw
        ), points,
        float(feedback['yaw']), yaw_from_quaternion(path.poses[-1].pose.orientation),
    )
    return {
        'path_stamp_sec': path_stamp,
        'feedback_stamp_sec': float(feedback['stamp_sec']),
        'association_mode': feedback['association_mode'],
        'association_delta_sec': delta_sec,
        'path_start_error_m': start_error,
        'path_pose_count': len(path.poses),
        'sweep': sweep,
    }


def run_scenario(
    probe: NavigateToPoseProbe, name: str, scenario: dict[str, object], *, allow_failure: bool = False,
    strict_path_safety: bool = True, terminal_tf_feedback_divergence_is_error: bool = True,
    recovery_contract: str = 'forbidden', cancellation_contract: dict[str, object] | None = None,
) -> dict[str, object]:
    """Send one NavigateToPose goal and validate BT-owned lower-level work."""
    if recovery_contract not in ('forbidden', 'required'):
        raise ValueError(f'unsupported recovery contract: {recovery_contract!r}')
    if cancellation_contract is not None:
        try:
            minimum_odom_travel_m = float(cancellation_contract['minimum_odom_travel_m'])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'{name} cancellation contract lacks minimum_odom_travel_m') from error
        if not math.isfinite(minimum_odom_travel_m) or minimum_odom_travel_m <= 0.0:
            raise ValueError(f'{name} cancellation minimum_odom_travel_m must be finite and positive')
    else:
        minimum_odom_travel_m = None
    contract = scenario['navigate_to_pose']
    goal_specification = scenario['goal']
    request = NavigateToPose.Goal()
    request.pose = pose_stamped(
        goal_specification['x'], goal_specification['y'], goal_specification['yaw']
    )
    # Empty selects the server-side frozen upstream XML, rather than allowing
    # this probe to substitute a client-specific behavior tree.
    request.behavior_tree = ''
    plan_start = len(probe.plan_messages)
    command_start = len(probe.command_records)
    feedback_start = len(probe.feedback_records)
    event_start = len(probe.bt_events)
    received_plan_start = len(probe.received_plan_messages)
    goal_handle = wait_for_future(
        probe,
        probe.action_client.send_goal_async(request, feedback_callback=probe._on_feedback),
        4.0,
        f'{name} NavigateToPose acceptance',
    )
    if not goal_handle.accepted:
        raise ValueError(f'{name} NavigateToPose goal was rejected')
    # Keep an action-acceptance timestamp for offline evaluators.  This is
    # evidence only; it does not alter the navigation action or BT runtime.
    goal_accepted_sim_sec = probe.clock_samples[-1]
    result_future = goal_handle.get_result_async()
    deadline = time.monotonic() + contract['action_timeout_sec']
    violation = None
    cancellation: dict[str, object] | None = None
    cancellation_motion_origin: tuple[float, float] | None = None
    safety_evidence: list[dict[str, object]] = []
    checked_plan_count = plan_start
    while not result_future.done() and time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if probe.clock_regressed:
            violation = '/clock regressed while NavigateToPose was running'
        while len(probe.plan_messages) > checked_plan_count:
            plan_record = probe.plan_messages[checked_plan_count]
            path = plan_record['message']
            if not isinstance(path, NavPath):
                violation = 'probe stored an invalid Path record'
                break
            feedback = associated_feedback(
                probe.feedback_records[feedback_start:], plan_record
            )
            if feedback is None:
                age = time.monotonic() - float(plan_record['received_monotonic'])
                if age > PATH_EVIDENCE_WAIT_SEC:
                    violation = 'Path lacked time-associated NavigateToPose feedback evidence'
                break
            try:
                observed_sweep = sweep_observed_plan(probe, plan_record, feedback)
                if strict_path_safety and observed_sweep['sweep']['safe'] is not True:
                    raise ValueError(
                        'full-footprint sweep rejected BT Path: '
                        f"{observed_sweep['sweep']['first_violation']}"
                    )
                safety_evidence.append(observed_sweep)
                checked_plan_count += 1
            except Exception as error:
                violation = f'BT runtime full-footprint evidence failed: {error}'
                break
        if cancellation_contract is not None and (
            cancellation is None or cancellation.get('status') == 'WAITING_FOR_MOTION'
        ):
            # The trigger is deliberately navigation-only: an accepted goal,
            # one BT global Path, then real filtered-odometry motion.  GT is
            # never observed by this online action client.
            if cancellation_motion_origin is None and len(probe.plan_messages) > plan_start:
                if probe.odom is None:
                    violation = 'cancel trigger lacks filtered odometry after initial Path'
                else:
                    cancellation_motion_origin = (
                        probe.odom.pose.pose.position.x,
                        probe.odom.pose.pose.position.y,
                    )
                    cancellation = {
                        'initial_path_sim_time': probe.clock_samples[-1],
                        'motion_origin_odom': {
                            'x': cancellation_motion_origin[0],
                            'y': cancellation_motion_origin[1],
                        },
                        'minimum_odom_travel_m': minimum_odom_travel_m,
                        'status': 'WAITING_FOR_MOTION',
                    }
            if cancellation_motion_origin is not None and probe.odom is not None:
                travel_m = math.dist(
                    cancellation_motion_origin,
                    (probe.odom.pose.pose.position.x, probe.odom.pose.pose.position.y),
                )
                if travel_m >= minimum_odom_travel_m:
                    cancellation['motion_gate_sim_time'] = probe.clock_samples[-1]
                    cancellation['motion_gate_odom_travel_m'] = travel_m
                    cancellation['cancel_request_sim_time'] = probe.clock_samples[-1]
                    response = wait_for_future(
                        probe, goal_handle.cancel_goal_async(), 3.0,
                        f'{name} native NavigateToPose cancellation acknowledgement',
                    )
                    cancellation['cancel_response_sim_time'] = probe.clock_samples[-1]
                    cancellation['cancel_return_code'] = int(response.return_code)
                    cancellation['cancel_goals_canceling_count'] = len(response.goals_canceling)
                    if response.return_code != CancelGoal.Response.ERROR_NONE or not response.goals_canceling:
                        violation = (
                            f'{name} native cancellation was not accepted: '
                            f'return_code={response.return_code}, goals={len(response.goals_canceling)}'
                        )
                    else:
                        cancellation['status'] = 'ACKNOWLEDGED'
        commands = probe.command_records[command_start:]
        if any(
            abs(command['linear_x']) > MAX_LINEAR_COMMAND_MPS
            or abs(command['angular_z']) > MAX_ANGULAR_COMMAND_RADPS
            for command in commands
        ):
            violation = 'BT navigation exceeded the fixed Task 3 smoke command bound'
        if violation is not None:
            wait_for_future(probe, goal_handle.cancel_goal_async(), 3.0, f'{name} cancellation')
            break
    if not result_future.done():
        wait_for_future(probe, goal_handle.cancel_goal_async(), 3.0, f'{name} timeout cancellation')
        rclpy.spin_until_future_complete(probe, result_future, timeout_sec=5.0)
        if violation is None:
            violation = f'{name} NavigateToPose exceeded its predeclared timeout'
    if violation is not None:
        raise ValueError(violation)
    result = wait_for_future(probe, result_future, 5.0, f'{name} NavigateToPose result')
    action_result_sim_sec = probe.clock_samples[-1]
    action_succeeded = (
        result.status == GoalStatus.STATUS_SUCCEEDED
        and result.result.error_code == NavigateToPose.Result.NONE
    )
    action_canceled = result.status == GoalStatus.STATUS_CANCELED
    if cancellation_contract is not None and not action_canceled:
        raise ValueError(
            f'{name} NavigateToPose cancel did not reach CANCELED terminal state: '
            f'status={result.status}, error={result.result.error_code}, message={result.result.error_msg}'
        )
    if cancellation_contract is not None and cancellation is None:
        raise ValueError(f'{name} canceled before the frozen initial-Path plus motion trigger')
    if not action_succeeded and not action_canceled and not allow_failure:
        raise ValueError(
            f'{name} NavigateToPose failed: status={result.status}, '
            f'error={result.result.error_code}, message={result.result.error_msg}'
        )
    plans = probe.plan_messages[plan_start:]
    while checked_plan_count < len(probe.plan_messages):
        plan_record = probe.plan_messages[checked_plan_count]
        path = plan_record['message']
        if not isinstance(path, NavPath):
            raise ValueError('probe stored an invalid terminal Path record')
        feedback = associated_feedback(
            probe.feedback_records[feedback_start:], plan_record
        )
        if feedback is None:
            raise ValueError('terminal Path lacked time-associated feedback safety evidence')
        observed_sweep = sweep_observed_plan(probe, plan_record, feedback)
        if strict_path_safety and observed_sweep['sweep']['safe'] is not True:
            raise ValueError(
                'full-footprint sweep rejected terminal BT Path: '
                f"{observed_sweep['sweep']['first_violation']}"
            )
        safety_evidence.append(observed_sweep)
        checked_plan_count += 1
    if len(plans) < contract['min_plan_updates']:
        raise ValueError(f'{name} observed only {len(plans)} replans')
    events = probe.bt_events[event_start:]
    compute_successes = sum(
        event['node_name'] == 'ComputePathToPose' and event['current_status'] == 'SUCCESS'
        for event in events
    )
    if action_succeeded and compute_successes < 2:
        raise ValueError(f'{name} lacks repeated ComputePathToPose BT evidence')
    feedback = probe.feedback_records[feedback_start:]
    if not feedback:
        raise ValueError(f'{name} has no NavigateToPose feedback evidence')
    maximum_recoveries = max(item['number_of_recoveries'] for item in feedback)
    if recovery_contract == 'forbidden' and maximum_recoveries != 0:
        raise ValueError(f'{name} feedback does not prove the no-recovery BT contract')
    if recovery_contract == 'required' and maximum_recoveries <= 0:
        raise ValueError(f'{name} feedback does not prove the Recovery BT contract')
    feedback_summary = {
        'count': len(feedback),
        'initial': feedback[0],
        'final': feedback[-1],
        'minimum_distance_remaining': min(item['distance_remaining'] for item in feedback),
        'maximum_number_of_recoveries': maximum_recoveries,
    }
    commands = probe.command_records[command_start:]
    if not any(abs(item['linear_x']) > 0.01 or abs(item['angular_z']) > 0.01 for item in commands):
        raise ValueError(f'{name} NavigateToPose produced no motion command')
    controller_paths = probe.received_plan_messages[received_plan_start:]
    if not controller_paths or not isinstance(controller_paths[-1]['message'], NavPath) or not controller_paths[-1]['message'].poses:
        raise ValueError(f'{name} Controller did not receive a BT Path')
    valid_feedback = [item for item in feedback if 'stamp_sec' in item]
    if not valid_feedback:
        raise ValueError(f'{name} has no valid final NavigateToPose feedback pose')
    final_feedback = valid_feedback[-1]
    final_pose = (float(final_feedback['x']), float(final_feedback['y']), float(final_feedback['yaw']))
    xy_error = math.dist(final_pose[:2], (goal_specification['x'], goal_specification['y']))
    yaw_error = abs(normalize_angle(final_pose[2] - goal_specification['yaw']))
    if action_succeeded and (
        xy_error > contract['max_final_xy_error_m'] or yaw_error > contract['max_final_yaw_error_rad']
    ):
        raise ValueError(f'{name} final error exceeds frozen goal contract')
    if len(safety_evidence) != len(plans):
        raise ValueError(f'{name} swept {len(safety_evidence)} of {len(plans)} observed Paths')
    stop_deadline = time.monotonic() + 1.0
    while time.monotonic() < stop_deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
    latest_commands = probe.command_records[-10:]
    controller_zero_observed = bool(latest_commands) and any(
        abs(item['linear_x']) < 1e-9 and abs(item['angular_z']) < 1e-9
        for item in latest_commands
    )
    if not controller_zero_observed:
        raise ValueError(f'{name} did not observe Controller completion stop')
    if probe.odom is None or abs(probe.odom.twist.twist.linear.x) > 0.01 or abs(probe.odom.twist.twist.angular.z) > 0.01:
        raise ValueError(f'{name} odometry did not settle after NavigateToPose')
    tf_pose = probe.map_pose()
    tf_feedback_cross_check = final_tf_feedback_cross_check(
        tf_pose,
        final_pose,
        divergence_is_error=terminal_tf_feedback_divergence_is_error,
    )
    if tf_feedback_cross_check['outcome'] == 'ERROR':
        raise ValueError(f'{name} final TF/feedback cross-check diverged')
    if cancellation is not None:
        cancel_request_sim_sec = float(cancellation['cancel_request_sim_time'])
        before_cancel_nonzero = [
            item for item in commands
            if item['sim_time_sec'] is not None
            and float(item['sim_time_sec']) <= cancel_request_sim_sec
            and (abs(item['linear_x']) > 0.01 or abs(item['angular_z']) > 0.01)
        ]
        after_cancel_zero = [
            item for item in probe.command_records[command_start:]
            if item['sim_time_sec'] is not None
            and float(item['sim_time_sec']) >= cancel_request_sim_sec
            and abs(item['linear_x']) < 1e-9 and abs(item['angular_z']) < 1e-9
        ]
        if not before_cancel_nonzero:
            raise ValueError(f'{name} lacks a pre-cancel Controller motion command')
        if not after_cancel_zero:
            raise ValueError(f'{name} lacks a Controller-originated zero command after cancellation')
        cancellation['last_nonzero_command_before_cancel_sim_time'] = before_cancel_nonzero[-1]['sim_time_sec']
        cancellation['first_zero_command_after_cancel_sim_time'] = after_cancel_zero[0]['sim_time_sec']
        cancellation['action_result_sim_time'] = action_result_sim_sec
        cancellation['status'] = 'CANCELED'
    return {
        'goal_accepted_sim_sec': goal_accepted_sim_sec,
        'action_result_sim_sec': action_result_sim_sec,
        'action_status': int(result.status),
        'result_error_code': int(result.result.error_code),
        'result_error_message': result.result.error_msg,
        'navigation_success': action_succeeded,
        'navigation_canceled': action_canceled,
        'feedback': feedback_summary,
        'plan_update_count': len(plans),
        'compute_path_success_events': compute_successes,
        'received_controller_path_pose_count': len(controller_paths[-1]['message'].poses),
        'received_controller_path_count': len(controller_paths),
        'full_footprint_path_evidence': safety_evidence,
        'command_count': len(commands),
        'max_linear_command_mps': max(abs(item['linear_x']) for item in commands),
        'max_angular_command_radps': max(abs(item['angular_z']) for item in commands),
        'final_pose_map': {'x': final_pose[0], 'y': final_pose[1], 'yaw': final_pose[2]},
        'final_xy_error_m': xy_error,
        'final_yaw_error_rad': yaw_error,
        'final_tf_feedback_translation_m': tf_feedback_cross_check['translation_m'],
        'final_tf_feedback_yaw_rad': tf_feedback_cross_check['yaw_rad'],
        'final_tf_feedback_cross_check': tf_feedback_cross_check,
        'controller_zero_observed': controller_zero_observed,
        **({'cancellation': cancellation} if cancellation is not None else {}),
    }


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=90.0)
    parser.add_argument('--result-path', required=True)
    parser.add_argument('--scenario', choices=['simple_reachable', 'static_obstacle_detour'], required=True)
    parser.add_argument(
        '--scenarios-file',
        default=str(
            Path(get_package_share_directory('resilient_nav_navigation'))
            / 'config' / 'planner_smoke_scenarios.yaml'
        ),
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    records: dict[str, object] = {'status': 'FAIL', 'scenario': arguments.scenario, 'zero_stop_messages': 0}
    rclpy.init(args=None)
    probe = NavigateToPoseProbe()
    try:
        scenario = load_scenarios(Path(arguments.scenarios_file))[arguments.scenario]
        deadline = time.monotonic() + arguments.timeout_sec
        for name in LIFECYCLE_SERVICES:
            wait_for(probe, lambda name=name: probe.lifecycle_active(name), deadline, f'{name} active lifecycle')
        wait_for(
            probe,
            lambda: probe.action_client.wait_for_server(timeout_sec=0.1),
            deadline,
            'NavigateToPose action server',
        )
        wait_for(
            probe,
            lambda: probe.global_raw is not None and probe.local_raw is not None and probe.odom is not None and bool(probe.map_pose()) and bool(probe.clock_contract()),
            deadline,
            'both Costmaps, odometry, map-to-base TF, and one monotonic /clock',
        )
        if probe.node_names('global_costmap', '/global_costmap') != ['global_costmap']:
            raise ValueError('expected exactly one Planner-owned global_costmap')
        if probe.node_names('local_costmap', '/local_costmap') != ['local_costmap']:
            raise ValueError('expected exactly one Controller-owned local_costmap')
        forbidden = {'behavior_server', 'waypoint_follower', 'velocity_smoother'}
        found = sorted(name for name, _ in probe.get_node_names_and_namespaces() if name in forbidden)
        if found:
            raise ValueError(f'BT smoke found forbidden nodes: {found}')
        records['bt_navigator_parameters'] = probe.bt_parameters()
        records['clock'] = probe.clock_contract()
        records['scenario_result'] = run_scenario(probe, arguments.scenario, scenario)
        records['status'] = 'PASS'
    except Exception as error:
        records['error'] = str(error)
    finally:
        records['zero_stop_messages'] = probe.publish_zero_stop()
        Path(arguments.result_path).write_text(json.dumps(records, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        probe.destroy_node()
        rclpy.shutdown()
    if records['status'] != 'PASS':
        print(json.dumps(records, indent=2, sort_keys=True), file=sys.stderr)
        return 1
    print(json.dumps(records, indent=2, sort_keys=True))
    return 0
