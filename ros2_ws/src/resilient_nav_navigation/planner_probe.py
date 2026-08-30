"""Read-only Task 3.1 ComputePathToPose readiness and path-safety probe."""

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
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PolygonStamped, PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import OccupancyGrid, Path as NavPath
import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from costmap_contract import LETHAL_COST_THRESHOLD, grid_cell_index, yaw_from_quaternion
from global_costmap_probe import (
    MAP_QOS,
    VOLATILE_RELIABLE_QOS,
    validate_costmap,
    validate_footprint,
)
from map_server_probe import validate_map
from path_safety import RawCostmapGrid, full_footprint_path_sweep


PLANNER_LIFECYCLE_SERVICE = '/planner_server/get_state'
PLANNER_PARAMETER_SERVICE = '/planner_server/get_parameters'
PATH_VALIDITY_SERVICE = '/is_path_valid'
COMPUTE_PATH_ACTION = '/compute_path_to_pose'

EXPECTED_PARAMETERS = {
    'planner_plugins': ['GridBased'],
    'GridBased.plugin': 'nav2_navfn_planner::NavfnPlanner',
    'GridBased.tolerance': 0.0,
    'GridBased.use_astar': False,
    'GridBased.allow_unknown': False,
}
EXPECTED_ERROR_CODES = {'GOAL_OCCUPIED': 206}
FORBIDDEN_NODE_NAMES = {
    'controller_server',
    'bt_navigator',
    'behavior_server',
    'waypoint_follower',
    'velocity_smoother',
}


def normalize_angle(angle: float) -> float:
    """Normalize a planar angle into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    """Return the z/w components of a planar unit quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def path_length(path: NavPath) -> float:
    """Return total 2D distance along a Nav2 path."""
    return sum(
        math.hypot(
            following.pose.position.x - preceding.pose.position.x,
            following.pose.position.y - preceding.pose.position.y,
        )
        for preceding, following in zip(path.poses, path.poses[1:])
    )


def sample_segment(
    start: tuple[float, float], end: tuple[float, float], spacing: float
) -> list[tuple[float, float]]:
    """Densify a 2D line segment so Costmap checks cannot skip lethal cells."""
    distance = math.dist(start, end)
    count = max(1, math.ceil(distance / spacing))
    return [
        (
            start[0] + (end[0] - start[0]) * index / count,
            start[1] + (end[1] - start[1]) * index / count,
        )
        for index in range(count + 1)
    ]


def path_cost_evidence(
    costmap: OccupancyGrid, points: list[tuple[float, float]], spacing: float = 0.025
) -> dict[str, int | float]:
    """Inspect a path or direct line against the published Costmap grid."""
    if not points:
        raise ValueError('cannot check an empty path')
    if not all(math.isfinite(value) for point in points for value in point):
        raise ValueError('path contains a non-finite coordinate')
    sampled = [points[0]]
    for start, end in zip(points, points[1:]):
        sampled.extend(sample_segment(start, end, spacing)[1:])
    origin = costmap.info.origin.position
    values = []
    for x, y in sampled:
        index = grid_cell_index(
            origin.x,
            origin.y,
            costmap.info.resolution,
            costmap.info.width,
            costmap.info.height,
            x,
            y,
        )
        values.append(costmap.data[index])
    return {
        'sample_count': len(values),
        'max_cost': max(values),
        'lethal_sample_count': sum(value >= LETHAL_COST_THRESHOLD for value in values),
    }


def load_scenarios(path: Path) -> dict[str, dict[str, object]]:
    """Load and validate the small, explicit Task 3.1 action contract."""
    loaded = yaml.safe_load(path.read_text(encoding='utf-8'))
    scenarios = loaded.get('scenarios') if isinstance(loaded, dict) else None
    if not isinstance(scenarios, dict) or not scenarios:
        raise ValueError(f'{path} has no scenarios mapping')
    validated = {}
    for name, specification in scenarios.items():
        if not isinstance(name, str) or not isinstance(specification, dict):
            raise ValueError('scenario names and definitions must be mappings')
        goal = specification.get('goal')
        expected = specification.get('expected')
        if not isinstance(goal, dict) or expected not in {'success', 'failure'}:
            raise ValueError(f'{name} has invalid goal or expected result')
        try:
            x, y, yaw = float(goal['x']), float(goal['y']), float(goal['yaw'])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'{name} has invalid goal coordinates') from error
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError(f'{name} has non-finite goal coordinates')
        if specification.get('expected_error_code') not in (None, 'GOAL_OCCUPIED'):
            raise ValueError(f'{name} uses an unsupported expected error code')
        controller = specification.get('controller')
        if controller is not None:
            if expected != 'success' or not isinstance(controller, dict):
                raise ValueError(f'{name} has an invalid controller contract')
            try:
                action_timeout = float(controller['action_timeout_sec'])
                max_cross_track_error = float(controller['max_cross_track_error_m'])
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f'{name} has invalid controller limits') from error
            if (
                not math.isfinite(action_timeout)
                or action_timeout <= 0.0
                or not math.isfinite(max_cross_track_error)
                or max_cross_track_error <= 0.0
            ):
                raise ValueError(f'{name} has non-positive controller limits')
        validated[name] = {
            'goal': {'x': x, 'y': y, 'yaw': yaw},
            'expected': expected,
            'expected_error_code': specification.get('expected_error_code'),
            'require_direct_line_lethal': bool(
                specification.get('require_direct_line_lethal', False)
            ),
            'controller': (
                {
                    'action_timeout_sec': action_timeout,
                    'max_cross_track_error_m': max_cross_track_error,
                }
                if controller is not None
                else None
            ),
        }
    return validated


def value_from_parameter(value) -> object:
    """Turn the narrow Planner Server parameter types into JSON values."""
    if value.type == ParameterType.PARAMETER_STRING:
        return value.string_value
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if value.type == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    if value.type == ParameterType.PARAMETER_STRING_ARRAY:
        return list(value.string_array_value)
    raise ValueError(f'unsupported parameter type {value.type}')


class PlannerProbe(Node):
    """Observe Planner Server and send only ComputePathToPose action requests."""

    def __init__(self) -> None:
        super().__init__('phase10_planner_probe')
        self.map_message: OccupancyGrid | None = None
        self.costmap_message: OccupancyGrid | None = None
        self.raw_costmap_message: Costmap | None = None
        self.scan_message: LaserScan | None = None
        self.footprint_message: PolygonStamped | None = None
        self.plan_message: NavPath | None = None
        self.plan_count = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lifecycle_client = self.create_client(GetState, PLANNER_LIFECYCLE_SERVICE)
        self.parameters_client = self.create_client(
            GetParameters, PLANNER_PARAMETER_SERVICE
        )
        self.path_validity_client = self.create_client(IsPathValid, PATH_VALIDITY_SERVICE)
        self.action_client = ActionClient(self, ComputePathToPose, COMPUTE_PATH_ACTION)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, MAP_QOS)
        self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self._on_costmap,
            VOLATILE_RELIABLE_QOS,
        )
        self.create_subscription(
            Costmap,
            '/global_costmap/costmap_raw',
            self._on_raw_costmap,
            VOLATILE_RELIABLE_QOS,
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            PolygonStamped,
            '/global_costmap/published_footprint',
            self._on_footprint,
            VOLATILE_RELIABLE_QOS,
        )
        self.create_subscription(NavPath, '/plan', self._on_plan, VOLATILE_RELIABLE_QOS)

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_message = message

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self.costmap_message = message

    def _on_raw_costmap(self, message: Costmap) -> None:
        self.raw_costmap_message = message

    def _on_scan(self, message: LaserScan) -> None:
        self.scan_message = message

    def _on_footprint(self, message: PolygonStamped) -> None:
        self.footprint_message = message

    def _on_plan(self, message: NavPath) -> None:
        self.plan_message = message
        self.plan_count += 1

    def lifecycle_active(self) -> bool:
        if not self.lifecycle_client.service_is_ready():
            return False
        future = self.lifecycle_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        return bool(
            future.done()
            and future.result() is not None
            and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        )

    def map_to_base_ready(self) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=0.1)
            )
        except Exception:
            return False
        return True

    def robot_pose(self) -> tuple[float, float, float]:
        transform = self.tf_buffer.lookup_transform(
            'map', 'base_footprint', Time(), timeout=Duration(seconds=0.5)
        ).transform
        return (
            transform.translation.x,
            transform.translation.y,
            yaw_from_quaternion(transform.rotation),
        )

    def planner_parameters(self) -> dict[str, object]:
        if not self.parameters_client.wait_for_service(timeout_sec=0.5):
            raise ValueError('planner parameter service is unavailable')
        request = GetParameters.Request()
        request.names = list(EXPECTED_PARAMETERS)
        future = self.parameters_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            raise ValueError('planner parameter request timed out')
        values = {
            name: value_from_parameter(parameter)
            for name, parameter in zip(request.names, future.result().values)
        }
        if values != EXPECTED_PARAMETERS:
            raise ValueError(f'planner parameters do not match Task 3.1 contract: {values}')
        return values

    def global_costmap_nodes(self) -> list[str]:
        return sorted(
            name
            for name, namespace in self.get_node_names_and_namespaces()
            if name == 'global_costmap' and namespace == '/global_costmap'
        )


def wait_for(probe: PlannerProbe, predicate, deadline: float, description: str) -> None:
    """Spin until a readiness predicate holds or report the exact missing fact."""
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if predicate():
            return
    raise TimeoutError(f'timed out waiting for {description}')


def wait_for_future(probe: PlannerProbe, future, timeout_sec: float, description: str):
    rclpy.spin_until_future_complete(probe, future, timeout_sec=timeout_sec)
    if not future.done() or future.result() is None:
        raise TimeoutError(f'timed out waiting for {description}')
    return future.result()


def pose_stamped(x: float, y: float, yaw: float) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = 'map'
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z, message.pose.orientation.w = quaternion_from_yaw(yaw)
    return message


def verify_path_headers(path: NavPath) -> None:
    if path.header.frame_id != 'map' or not path.poses:
        raise ValueError('successful action result has no map-frame path')
    for pose in path.poses:
        if pose.header.frame_id != 'map':
            raise ValueError('a path pose is not in the map frame')
        values = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('a path pose is non-finite')


def run_scenario(
    probe: PlannerProbe, name: str, scenario: dict[str, object]
) -> dict[str, object]:
    """Execute one non-motion ComputePathToPose request and validate its result."""
    if probe.costmap_message is None or probe.raw_costmap_message is None:
        raise ValueError('costmap vanished before action request')
    goal_specification = scenario['goal']
    goal = pose_stamped(
        goal_specification['x'], goal_specification['y'], goal_specification['yaw']
    )
    costmap_before = copy.deepcopy(probe.costmap_message)
    start = probe.robot_pose()
    plan_count_before = probe.plan_count
    request = ComputePathToPose.Goal()
    request.goal = goal
    request.planner_id = 'GridBased'
    request.use_start = False
    goal_handle = wait_for_future(
        probe,
        probe.action_client.send_goal_async(request),
        3.0,
        f'{name} action acceptance',
    )
    if not goal_handle.accepted:
        raise ValueError(f'{name} action goal was rejected')
    wrapped_result = wait_for_future(
        probe, goal_handle.get_result_async(), 10.0, f'{name} action result'
    )
    result = wrapped_result.result
    expected = scenario['expected']
    record = {
        'accepted': True,
        'goal': goal_specification,
        'status': int(wrapped_result.status),
        'error_code': int(result.error_code),
        'error_msg': result.error_msg,
        'planning_time_sec': (
            result.planning_time.sec + result.planning_time.nanosec / 1e9
        ),
    }
    if expected == 'failure':
        goal_cost = path_cost_evidence(
            costmap_before,
            [(goal_specification['x'], goal_specification['y'])],
        )['max_cost']
        expected_code = EXPECTED_ERROR_CODES[scenario['expected_error_code']]
        if (
            wrapped_result.status != GoalStatus.STATUS_ABORTED
            or result.error_code != expected_code
            or result.path.poses
            or goal_cost < LETHAL_COST_THRESHOLD
        ):
            raise ValueError(
                f'{name} expected occupied-goal abort, received status='
                f'{wrapped_result.status}, error={result.error_code}, path='
                f'{len(result.path.poses)}, goal_cost={goal_cost}'
            )
        record['goal_cost'] = goal_cost
        return record

    if (
        wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
        or result.error_code != ComputePathToPose.Result.NONE
    ):
        raise ValueError(
            f'{name} expected successful plan, received status={wrapped_result.status}, '
            f'error={result.error_code}: {result.error_msg}'
        )
    verify_path_headers(result.path)
    action_points = [
        (pose.pose.position.x, pose.pose.position.y) for pose in result.path.poses
    ]
    action_evidence = path_cost_evidence(costmap_before, action_points)
    if action_evidence['lethal_sample_count']:
        raise ValueError(f'{name} action path samples lethal Costmap cells')
    if math.dist(action_points[0], start[:2]) > 0.15:
        raise ValueError(f'{name} path does not start near the robot pose')
    if math.dist(action_points[-1], (goal_specification['x'], goal_specification['y'])) > 0.10:
        raise ValueError(f'{name} path does not end near its requested goal')
    full_footprint_evidence = full_footprint_path_sweep(
        RawCostmapGrid.from_message(probe.raw_costmap_message),
        action_points,
        initial_yaw=start[2],
        goal_yaw=goal_specification['yaw'],
    )
    if not full_footprint_evidence['safe']:
        raise ValueError(
            f"{name} full-footprint path sweep is unsafe: "
            f"{full_footprint_evidence['first_violation']}"
        )
    if scenario['require_direct_line_lethal']:
        direct_evidence = path_cost_evidence(
            costmap_before, [start[:2], (goal_specification['x'], goal_specification['y'])]
        )
        if not direct_evidence['lethal_sample_count']:
            raise ValueError(f'{name} direct line did not cross a lethal static obstacle')
        if path_length(result.path) <= math.dist(start[:2], action_points[-1]) + 0.05:
            raise ValueError(f'{name} path did not demonstrate a detour')
        record['direct_line_cost_evidence'] = direct_evidence

    wait_for(
        probe,
        lambda: probe.plan_count > plan_count_before,
        time.monotonic() + 2.0,
        f'{name} /plan publication',
    )
    if probe.plan_message is None:
        raise ValueError(f'{name} /plan was not received')
    verify_path_headers(probe.plan_message)
    validity = wait_for_future(
        probe,
        probe.path_validity_client.call_async(IsPathValid.Request(path=result.path)),
        3.0,
        f'{name} IsPathValid response',
    )
    if not validity.is_valid or validity.invalid_pose_indices:
        raise ValueError(f'{name} Nav2 IsPathValid rejected the returned path')
    record.update(
        path_pose_count=len(result.path.poses),
        path_length_m=path_length(result.path),
        path_cost_evidence=action_evidence,
        full_footprint_sweep=full_footprint_evidence,
        is_path_valid=True,
    )
    return record


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=45.0)
    parser.add_argument('--result-path', required=True)
    parser.add_argument('--scenario', default='all')
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
    records: dict[str, object] = {'status': 'FAIL', 'scenarios': {}}
    rclpy.init(args=None)
    probe = PlannerProbe()
    try:
        scenarios = load_scenarios(Path(arguments.scenarios_file))
        requested = list(scenarios) if arguments.scenario == 'all' else [arguments.scenario]
        if any(name not in scenarios for name in requested):
            raise ValueError(f'unknown requested scenario: {arguments.scenario}')
        deadline = time.monotonic() + arguments.timeout_sec
        wait_for(probe, probe.lifecycle_active, deadline, 'Planner Server active lifecycle')
        wait_for(
            probe,
            lambda: probe.action_client.wait_for_server(timeout_sec=0.1),
            deadline,
            'ComputePathToPose action server',
        )
        wait_for(
            probe,
            lambda: all(
                value is not None
                for value in (
                    probe.map_message,
                    probe.costmap_message,
                    probe.raw_costmap_message,
                    probe.scan_message,
                    probe.footprint_message,
                )
            ) and probe.map_to_base_ready(),
            deadline,
            'map, global Costmap/raw Costmap, scan, footprint, and map-to-base TF',
        )
        if (
            probe.map_message is None
            or probe.costmap_message is None
            or probe.raw_costmap_message is None
            or probe.footprint_message is None
        ):
            raise ValueError('planner readiness messages are incomplete')
        records['map'] = validate_map(probe.map_message)
        records['global_costmap'] = validate_costmap(
            probe.costmap_message, probe.map_message
        )
        records['footprint'] = validate_footprint(probe, probe.footprint_message)
        records['planner_parameters'] = probe.planner_parameters()
        global_nodes = probe.global_costmap_nodes()
        if global_nodes != ['global_costmap']:
            raise ValueError(f'expected exactly one Planner-owned global_costmap, got {global_nodes}')
        present_forbidden = sorted(
            name
            for name, _ in probe.get_node_names_and_namespaces()
            if name in FORBIDDEN_NODE_NAMES
        )
        if present_forbidden:
            raise ValueError(f'planner-only smoke found forbidden nodes: {present_forbidden}')
        initial_pose = probe.robot_pose()
        records['initial_robot_pose_map'] = {
            'x': initial_pose[0], 'y': initial_pose[1], 'yaw': initial_pose[2]
        }
        for name in requested:
            records['scenarios'][name] = run_scenario(probe, name, scenarios[name])
        final_pose = probe.robot_pose()
        translation = math.dist(initial_pose[:2], final_pose[:2])
        yaw_delta = abs(normalize_angle(final_pose[2] - initial_pose[2]))
        if translation > 0.03 or yaw_delta > 0.03:
            raise ValueError(
                f'planner-only action moved robot: translation={translation:.3f}, yaw={yaw_delta:.3f}'
            )
        records['final_robot_pose_map'] = {
            'x': final_pose[0], 'y': final_pose[1], 'yaw': final_pose[2]
        }
        records['robot_motion_delta'] = {'translation_m': translation, 'yaw_rad': yaw_delta}
        records['status'] = 'PASS'
    except Exception as error:
        records['error'] = str(error)
    finally:
        Path(arguments.result_path).write_text(
            json.dumps(records, indent=2, sort_keys=True) + '\n', encoding='utf-8'
        )
        probe.destroy_node()
        rclpy.shutdown()
    if records['status'] != 'PASS':
        print(json.dumps(records, indent=2, sort_keys=True), file=sys.stderr)
        return 1
    print(json.dumps(records, indent=2, sort_keys=True))
    return 0
