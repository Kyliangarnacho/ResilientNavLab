"""Bounded Task 3.2 FollowPath smoke probe for the healthy simulation only."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PolygonStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from global_costmap_probe import MAP_QOS, VOLATILE_RELIABLE_QOS
from path_safety import LETHAL_COST, RawCostmapGrid, UNKNOWN_COST, full_footprint_path_sweep
from planner_probe import (
    COMPUTE_PATH_ACTION,
    PLANNER_LIFECYCLE_SERVICE,
    load_scenarios,
    normalize_angle,
    path_length,
    pose_stamped,
    value_from_parameter,
    verify_path_headers,
    wait_for,
    wait_for_future,
)
from costmap_contract import grid_cell_index, yaw_from_quaternion


CONTROLLER_LIFECYCLE_SERVICE = '/controller_server/get_state'
CONTROLLER_PARAMETER_SERVICE = '/controller_server/get_parameters'
FOLLOW_PATH_ACTION = '/follow_path'
MAX_LINEAR_COMMAND_MPS = 0.22
MAX_ANGULAR_COMMAND_RADPS = 0.75
EXPECTED_CONTROLLER_PARAMETERS = {
    'controller_plugins': ['FollowPath'],
    'FollowPath.plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'odom_topic': '/odometry/filtered',
    'enable_stamped_cmd_vel': False,
    'publish_zero_velocity': True,
    'controller_frequency': 20.0,
}


def point_to_segment_distance(
    point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]
) -> float:
    """Return the Euclidean distance from a point to one finite segment."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.dist(point, start)
    fraction = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared))
    return math.dist(point, (start[0] + fraction * dx, start[1] + fraction * dy))


def distance_to_path(point: tuple[float, float], path: NavPath) -> float:
    """Return the closest 2-D distance to a non-empty Nav2 path."""
    points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
    if len(points) == 1:
        return math.dist(point, points[0])
    return min(point_to_segment_distance(point, start, end) for start, end in zip(points, points[1:]))


def raw_centerline_evidence(grid: RawCostmapGrid, path: NavPath) -> dict[str, int]:
    """Check RPP's published local collision arc against its Local Costmap."""
    if not path.poses:
        raise ValueError('RPP did not publish a lookahead collision arc')
    if path.header.frame_id != 'odom':
        raise ValueError(
            f'RPP collision arc is not in the Local Costmap odom frame: {path.header.frame_id!r}'
        )
    lethal = unknown = 0
    for pose in path.poses:
        try:
            index = grid_cell_index(
                grid.origin_x, grid.origin_y, grid.resolution, grid.width, grid.height,
                pose.pose.position.x, pose.pose.position.y,
            )
        except ValueError as error:
            raise ValueError('RPP collision arc leaves the Local Costmap') from error
        value = grid.data[index]
        lethal += value == LETHAL_COST
        unknown += value == UNKNOWN_COST
    if lethal or unknown:
        raise ValueError(f'RPP collision arc includes lethal={lethal}, unknown={unknown} cells')
    return {'sample_count': len(path.poses), 'lethal_sample_count': lethal, 'unknown_sample_count': unknown}


class ControllerProbe(Node):
    """Observe Nav2 and issue only one predeclared FollowPath request."""

    def __init__(self) -> None:
        super().__init__('phase10_controller_probe')
        self.map_message: OccupancyGrid | None = None
        self.global_raw: Costmap | None = None
        self.local_costmap: OccupancyGrid | None = None
        self.local_raw: Costmap | None = None
        self.local_footprint: PolygonStamped | None = None
        self.odom_message: Odometry | None = None
        self.plan_message: NavPath | None = None
        self.received_plan: NavPath | None = None
        self.collision_arc: NavPath | None = None
        self.command_records: list[dict[str, float]] = []
        self.feedback_records: list[dict[str, float]] = []
        self.map_trajectory: list[tuple[float, float, float]] = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.planner_lifecycle = self.create_client(GetState, PLANNER_LIFECYCLE_SERVICE)
        self.controller_lifecycle = self.create_client(GetState, CONTROLLER_LIFECYCLE_SERVICE)
        self.controller_parameters = self.create_client(GetParameters, CONTROLLER_PARAMETER_SERVICE)
        self.compute_client = ActionClient(self, ComputePathToPose, COMPUTE_PATH_ACTION)
        self.follow_client = ActionClient(self, FollowPath, FOLLOW_PATH_ACTION)
        # This publisher is used solely in finally for an all-zero emergency
        # stop; the probe never publishes a non-zero velocity.
        self.zero_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, MAP_QOS)
        # Costmaps and published footprint are state snapshots.  Request their
        # transient-local sample so a probe starting after lifecycle activation
        # still observes the current state rather than waiting for a change.
        self.create_subscription(Costmap, '/global_costmap/costmap_raw', self._on_global_raw, MAP_QOS)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._on_local_costmap, MAP_QOS)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw', self._on_local_raw, MAP_QOS)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint', self._on_local_footprint, MAP_QOS)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(NavPath, '/plan', self._on_plan, VOLATILE_RELIABLE_QOS)
        self.create_subscription(NavPath, '/received_global_plan', self._on_received_plan, VOLATILE_RELIABLE_QOS)
        self.create_subscription(NavPath, '/lookahead_collision_arc', self._on_collision_arc, VOLATILE_RELIABLE_QOS)
        self.create_subscription(Twist, '/cmd_vel', self._on_command, 10)

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_message = message

    def _on_global_raw(self, message: Costmap) -> None:
        self.global_raw = message

    def _on_local_costmap(self, message: OccupancyGrid) -> None:
        self.local_costmap = message

    def _on_local_raw(self, message: Costmap) -> None:
        self.local_raw = message

    def _on_local_footprint(self, message: PolygonStamped) -> None:
        self.local_footprint = message

    def _on_odom(self, message: Odometry) -> None:
        self.odom_message = message

    def _on_plan(self, message: NavPath) -> None:
        self.plan_message = message

    def _on_received_plan(self, message: NavPath) -> None:
        self.received_plan = message

    def _on_collision_arc(self, message: NavPath) -> None:
        self.collision_arc = message

    def _on_command(self, message: Twist) -> None:
        self.command_records.append({
            'linear_x': message.linear.x,
            'angular_z': message.angular.z,
            'time_monotonic': time.monotonic(),
        })

    def _on_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.feedback_records.append({
            'distance_to_goal': feedback.distance_to_goal,
            'speed': feedback.speed,
            'time_monotonic': time.monotonic(),
        })

    def lifecycle_active(self, client) -> bool:
        if not client.service_is_ready():
            return False
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        return bool(future.done() and future.result() is not None and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE)

    def map_pose(self) -> tuple[float, float, float]:
        transform = self.tf_buffer.lookup_transform(
            'map', 'base_footprint', Time(), timeout=Duration(seconds=0.3)
        ).transform
        return transform.translation.x, transform.translation.y, yaw_from_quaternion(transform.rotation)

    def map_to_base_ready(self) -> bool:
        try:
            self.map_pose()
        except Exception:
            return False
        return True

    def controller_parameter_values(self) -> dict[str, object]:
        if not self.controller_parameters.wait_for_service(timeout_sec=0.5):
            raise ValueError('Controller Server parameter service is unavailable')
        request = GetParameters.Request()
        request.names = list(EXPECTED_CONTROLLER_PARAMETERS)
        response = wait_for_future(self, self.controller_parameters.call_async(request), 2.0, 'Controller parameters')
        values = {name: value_from_parameter(value) for name, value in zip(request.names, response.values)}
        if values != EXPECTED_CONTROLLER_PARAMETERS:
            raise ValueError(f'Controller parameters differ from the Task 3.2 contract: {values}')
        return values

    def node_names(self, name: str, namespace: str) -> list[str]:
        return sorted(found for found, found_namespace in self.get_node_names_and_namespaces() if found == name and found_namespace == namespace)

    def publish_zero_stop(self) -> int:
        """Publish a short all-zero safety stop, and nothing else."""
        for _ in range(5):
            self.zero_publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        return 5


def compute_path(probe: ControllerProbe, scenario: dict[str, object]) -> NavPath:
    goal_specification = scenario['goal']
    request = ComputePathToPose.Goal()
    request.goal = pose_stamped(goal_specification['x'], goal_specification['y'], goal_specification['yaw'])
    request.planner_id = 'GridBased'
    request.use_start = False
    goal_handle = wait_for_future(probe, probe.compute_client.send_goal_async(request), 4.0, 'ComputePathToPose acceptance')
    if not goal_handle.accepted:
        raise ValueError('Planner rejected the controller smoke path request')
    result = wait_for_future(probe, goal_handle.get_result_async(), 12.0, 'ComputePathToPose result')
    if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != ComputePathToPose.Result.NONE:
        raise ValueError(f'Planner failed before FollowPath: {result.result.error_msg}')
    verify_path_headers(result.result.path)
    return result.result.path


def run_scenario(probe: ControllerProbe, name: str, scenario: dict[str, object]) -> dict[str, object]:
    """Plan and execute one fixed healthy FollowPath scenario."""
    contract = scenario['controller']
    if contract is None:
        raise ValueError(f'{name} has no controller contract')
    if probe.global_raw is None or probe.local_raw is None:
        raise ValueError('raw Costmap disappeared before FollowPath')
    start_pose = probe.map_pose()
    path = compute_path(probe, scenario)
    path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
    sweep = full_footprint_path_sweep(
        RawCostmapGrid.from_message(probe.global_raw), path_points, start_pose[2], scenario['goal']['yaw']
    )
    if not sweep['safe']:
        raise ValueError(f"{name} planned path fails full-footprint sweep: {sweep['first_violation']}")
    request = FollowPath.Goal()
    request.path = path
    request.controller_id = 'FollowPath'
    request.goal_checker_id = 'goal_checker'
    request.progress_checker_id = 'progress_checker'
    command_count_before = len(probe.command_records)
    feedback_count_before = len(probe.feedback_records)
    trajectory_start = len(probe.map_trajectory)
    goal_handle = wait_for_future(
        probe, probe.follow_client.send_goal_async(request, feedback_callback=probe._on_feedback), 4.0,
        f'{name} FollowPath acceptance',
    )
    if not goal_handle.accepted:
        raise ValueError(f'{name} FollowPath goal was rejected')
    result_future = goal_handle.get_result_async()
    deadline = time.monotonic() + contract['action_timeout_sec']
    runtime_sweeps = 0
    violation = None
    last_sweep = 0.0
    while not result_future.done() and time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
        try:
            current_pose = probe.map_pose()
            probe.map_trajectory.append(current_pose)
            if time.monotonic() - last_sweep >= 0.25 and probe.global_raw is not None:
                runtime = full_footprint_path_sweep(
                    RawCostmapGrid.from_message(probe.global_raw),
                    [(current_pose[0], current_pose[1])], current_pose[2], current_pose[2],
                )
                runtime_sweeps += 1
                last_sweep = time.monotonic()
                if not runtime['safe']:
                    violation = f"runtime footprint collision: {runtime['first_violation']}"
        except Exception as error:
            violation = f'map pose/runtime safety unavailable: {error}'
        new_commands = probe.command_records[command_count_before:]
        if any(command['linear_x'] > MAX_LINEAR_COMMAND_MPS or abs(command['angular_z']) > MAX_ANGULAR_COMMAND_RADPS for command in new_commands):
            violation = 'Controller command exceeded fixed smoke safety bound'
        if violation is not None:
            wait_for_future(probe, goal_handle.cancel_goal_async(), 3.0, f'{name} FollowPath cancellation')
            break
    if not result_future.done():
        wait_for_future(probe, goal_handle.cancel_goal_async(), 3.0, f'{name} FollowPath timeout cancellation')
        rclpy.spin_until_future_complete(probe, result_future, timeout_sec=5.0)
        if violation is None:
            violation = f'{name} FollowPath exceeded its predeclared timeout'
    if violation is not None:
        raise ValueError(violation)
    wrapped_result = wait_for_future(probe, result_future, 5.0, f'{name} FollowPath result')
    if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED or wrapped_result.result.error_code != FollowPath.Result.NONE:
        raise ValueError(
            f'{name} FollowPath failed: status={wrapped_result.status}, '
            f'error={wrapped_result.result.error_code}, message={wrapped_result.result.error_msg}'
        )
    final_pose = probe.map_pose()
    xy_error = math.dist(final_pose[:2], (scenario['goal']['x'], scenario['goal']['y']))
    yaw_error = abs(normalize_angle(final_pose[2] - scenario['goal']['yaw']))
    if xy_error > 0.12 or yaw_error > 0.17:
        raise ValueError(f'{name} final pose exceeds GoalChecker tolerance: xy={xy_error:.3f}, yaw={yaw_error:.3f}')
    trajectory = probe.map_trajectory[trajectory_start:]
    if not trajectory:
        raise ValueError(f'{name} recorded no map-frame controller trajectory')
    max_cross_track = max(distance_to_path(pose[:2], path) for pose in trajectory)
    if max_cross_track > contract['max_cross_track_error_m']:
        raise ValueError(f'{name} exceeds fixed cross-track limit: {max_cross_track:.3f} m')
    commands = probe.command_records[command_count_before:]
    feedback = probe.feedback_records[feedback_count_before:]
    if not commands or not any(abs(command['linear_x']) > 0.01 or abs(command['angular_z']) > 0.01 for command in commands):
        raise ValueError(f'{name} FollowPath produced no non-zero command')
    if not feedback:
        raise ValueError(f'{name} FollowPath produced no action feedback')
    if probe.received_plan is None or not probe.received_plan.poses:
        raise ValueError(f'{name} did not publish /received_global_plan')
    if probe.collision_arc is None:
        raise ValueError(f'{name} did not publish /lookahead_collision_arc')
    if probe.local_raw is None:
        raise ValueError('Local Costmap raw message disappeared after FollowPath')
    collision_arc = raw_centerline_evidence(RawCostmapGrid.from_message(probe.local_raw), probe.collision_arc)
    # Let Controller Server publish its configured completion stop before the
    # probe's independent zero-only failsafe runs in finally.
    stop_deadline = time.monotonic() + 1.0
    while time.monotonic() < stop_deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
    completion_commands = probe.command_records[len(probe.command_records) - 10:]
    if not completion_commands or not any(abs(command['linear_x']) < 1e-9 and abs(command['angular_z']) < 1e-9 for command in completion_commands):
        raise ValueError(f'{name} did not observe Controller Server zero-velocity completion output')
    return {
        'path_pose_count': len(path.poses),
        'path_length_m': path_length(path),
        'full_footprint_sweep': sweep,
        'feedback_count': len(feedback),
        'command_count': len(commands),
        'max_linear_command_mps': max(abs(command['linear_x']) for command in commands),
        'max_angular_command_radps': max(abs(command['angular_z']) for command in commands),
        'trajectory_sample_count': len(trajectory),
        'max_cross_track_error_m': max_cross_track,
        'final_pose_map': {'x': final_pose[0], 'y': final_pose[1], 'yaw': final_pose[2]},
        'final_xy_error_m': xy_error,
        'final_yaw_error_rad': yaw_error,
        'runtime_full_footprint_sweep_count': runtime_sweeps,
        'collision_arc': collision_arc,
        'result_error_code': int(wrapped_result.result.error_code),
    }


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=90.0)
    parser.add_argument('--result-path', required=True)
    parser.add_argument('--scenario', choices=['simple_reachable', 'static_obstacle_detour'], required=True)
    parser.add_argument('--scenarios-file', default=str(Path(get_package_share_directory('resilient_nav_navigation')) / 'config' / 'planner_smoke_scenarios.yaml'))
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    records: dict[str, object] = {'status': 'FAIL', 'scenario': arguments.scenario, 'zero_stop_messages': 0}
    rclpy.init(args=None)
    probe = ControllerProbe()
    try:
        scenario = load_scenarios(Path(arguments.scenarios_file))[arguments.scenario]
        deadline = time.monotonic() + arguments.timeout_sec
        wait_for(probe, lambda: probe.lifecycle_active(probe.planner_lifecycle), deadline, 'Planner Server active')
        wait_for(probe, lambda: probe.lifecycle_active(probe.controller_lifecycle), deadline, 'Controller Server active')
        wait_for(probe, lambda: probe.compute_client.wait_for_server(timeout_sec=0.1), deadline, 'ComputePathToPose action')
        wait_for(probe, lambda: probe.follow_client.wait_for_server(timeout_sec=0.1), deadline, 'FollowPath action')
        wait_for(probe, lambda: all(value is not None for value in (probe.map_message, probe.global_raw, probe.local_costmap, probe.local_raw, probe.local_footprint, probe.odom_message)) and probe.map_to_base_ready(), deadline, 'map, both Costmaps, local footprint, odometry, and map-to-base TF')
        if probe.node_names('global_costmap', '/global_costmap') != ['global_costmap']:
            raise ValueError('expected exactly one Planner-owned global_costmap')
        if probe.node_names('local_costmap', '/local_costmap') != ['local_costmap']:
            raise ValueError('expected exactly one Controller-owned local_costmap')
        records['controller_parameters'] = probe.controller_parameter_values()
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
