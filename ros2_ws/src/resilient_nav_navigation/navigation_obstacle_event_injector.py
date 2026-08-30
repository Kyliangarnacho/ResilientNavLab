"""Apply one frozen Task 5 Gazebo environment event after navigation begins.

This node is deliberately an environment event adapter, not a navigation
component.  It observes only the normal plan / odometry / Costmap streams,
uses official ros_gz SpawnEntity/DeleteEntity services, and never reads
evaluator Ground Truth, changes a Nav2 goal, calls lower-level actions, or
publishes velocity.  A temporary obstacle's deletion deadline is derived only
from its acknowledged spawn time and `/clock`; it never observes Recovery.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from global_costmap_probe import MAP_QOS, VOLATILE_RELIABLE_QOS


LETHAL_COST = 254
MAP_FRAME = 'map'
SPAWN_SERVICE = '/world/resilient_lab/create'
DELETE_SERVICE = '/world/resilient_lab/remove'
MODEL_ENTITY_TYPE = 2


def stamp_seconds(stamp) -> float:
    value = float(stamp.sec) + float(stamp.nanosec) / 1e9
    if not math.isfinite(value):
        raise ValueError('received non-finite ROS timestamp')
    return value


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def rotated_box_contains(
    x: float, y: float, *, center_x: float, center_y: float,
    size_x: float, size_y: float, yaw: float, margin: float,
) -> bool:
    """Return whether a point lies in a box, with explicit detection margin."""
    cosine, sine = math.cos(yaw), math.sin(yaw)
    relative_x, relative_y = x - center_x, y - center_y
    local_x = cosine * relative_x + sine * relative_y
    local_y = -sine * relative_x + cosine * relative_y
    return (
        abs(local_x) <= size_x / 2.0 + margin
        and abs(local_y) <= size_y / 2.0 + margin
    )


def costmap_detects_box(
    message: Costmap, box: dict[str, float], *, minimum_cost: int = LETHAL_COST,
) -> bool:
    """Detect a newly lethal cell spatially associated with the frozen box."""
    metadata = message.metadata
    resolution = float(metadata.resolution)
    width, height = int(metadata.size_x), int(metadata.size_y)
    if not math.isfinite(resolution) or resolution <= 0.0 or width <= 0 or height <= 0:
        return False
    if len(message.data) != width * height:
        return False
    origin_x, origin_y = float(metadata.origin.position.x), float(metadata.origin.position.y)
    margin = resolution / math.sqrt(2.0) + 0.01
    for row in range(height):
        y = origin_y + (row + 0.5) * resolution
        for column in range(width):
            if int(message.data[row * width + column]) < minimum_cost:
                continue
            x = origin_x + (column + 0.5) * resolution
            if rotated_box_contains(x, y, margin=margin, **box):
                return True
    return False


def load_scenario(path: Path, name: str) -> dict[str, object]:
    loaded = yaml.safe_load(path.read_text(encoding='utf-8'))
    scenarios = loaded.get('scenarios') if isinstance(loaded, dict) else None
    scenario = scenarios.get(name) if isinstance(scenarios, dict) else None
    if not isinstance(scenario, dict):
        raise ValueError(f'{name} is not a Task 5 scenario')
    task5 = scenario.get('task5')
    if not isinstance(task5, dict):
        raise ValueError(f'{name} has no task5 contract')
    obstacle = task5.get('obstacle')
    trigger = task5.get('trigger')
    if not isinstance(obstacle, dict) or not isinstance(trigger, dict):
        raise ValueError(f'{name} has malformed obstacle or trigger contract')
    required = ('entity_name', 'model_sdf', 'map_pose', 'gazebo_pose', 'size_m')
    if any(key not in obstacle for key in required):
        raise ValueError(f'{name} obstacle contract is incomplete')
    return scenario


def scenario_sha256(scenario: dict[str, object]) -> str:
    """Freeze the exact non-GT scenario contract recorded by this injector."""
    canonical = json.dumps(scenario, sort_keys=True, separators=(',', ':'))
    return hashlib.sha256(canonical.encode('utf-8')).hexdigest()


class ObstacleEventInjector(Node):
    """Observe a progress gate then execute one frozen Gazebo event schedule."""

    def __init__(
        self, scenario_name: str, scenario: dict[str, object], output_path: Path,
        scenarios_file: Path,
    ) -> None:
        super().__init__('phase10_navigation_obstacle_event_injector')
        task5 = scenario['task5']
        if not isinstance(task5, dict):
            raise ValueError('task5 contract is malformed')
        obstacle = task5['obstacle']
        trigger = task5['trigger']
        if not isinstance(obstacle, dict) or not isinstance(trigger, dict):
            raise ValueError('task5 obstacle/trigger is malformed')
        map_pose = obstacle['map_pose']
        gazebo_pose = obstacle['gazebo_pose']
        size = obstacle['size_m']
        if not all(isinstance(value, dict) for value in (map_pose, gazebo_pose, size)):
            raise ValueError('Task 5 obstacle pose/size is malformed')
        self.output_path = output_path
        self.model_sdf = Path(get_package_share_directory('resilient_nav_navigation')) / str(obstacle['model_sdf'])
        self.entity_name = str(obstacle['entity_name'])
        self.map_box = {
            'center_x': float(map_pose['x']),
            'center_y': float(map_pose['y']),
            'size_x': float(size['x']),
            'size_y': float(size['y']),
            'yaw': float(map_pose['yaw']),
        }
        self.gazebo_pose = {
            'x': float(gazebo_pose['x']), 'y': float(gazebo_pose['y']),
            'z': float(gazebo_pose['z']), 'yaw': float(gazebo_pose['yaw']),
        }
        self.minimum_odom_travel_m = float(trigger['minimum_odom_travel_m'])
        if not math.isfinite(self.minimum_odom_travel_m) or self.minimum_odom_travel_m <= 0.0:
            raise ValueError('minimum_odom_travel_m must be finite and positive')
        if not self.model_sdf.is_file():
            raise ValueError(f'configured obstacle SDF is missing: {self.model_sdf}')
        self.event_mode = str(task5.get('event_mode', 'spawn_only'))
        if self.event_mode not in ('spawn_only', 'spawn_then_delete'):
            raise ValueError(f'unsupported Task 5 event_mode: {self.event_mode!r}')
        self.obstacle_lifetime_sim_sec: float | None = None
        if self.event_mode == 'spawn_then_delete':
            self.obstacle_lifetime_sim_sec = float(trigger.get('obstacle_lifetime_sim_sec', 0.0))
            if (
                not math.isfinite(self.obstacle_lifetime_sim_sec)
                or self.obstacle_lifetime_sim_sec <= 0.0
            ):
                raise ValueError('spawn_then_delete requires finite positive obstacle_lifetime_sim_sec')
        self.initial_odom: tuple[float, float] | None = None
        self.latest_odom: Odometry | None = None
        self.initial_plan_stamp: float | None = None
        self.spawned = False
        self.complete = False
        self.spawn_timeout_timer = None
        self.delete_timeout_timer = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.spawn_client = self.create_client(SpawnEntity, SPAWN_SERVICE)
        self.delete_client = self.create_client(DeleteEntity, DELETE_SERVICE)
        self.record: dict[str, object] = {
            'status': 'PENDING',
            'benchmark': 'phase10_task5_dynamic_environment',
            'scenario': scenario_name,
            'task5_kind': task5.get('kind'),
            'base_scenario': scenario.get('base_scenario'),
            'scenario_sha256': scenario_sha256(scenario),
            'scenarios_file': str(scenarios_file),
            'entity_name': self.entity_name,
            'spawn_service': SPAWN_SERVICE,
            'delete_service': DELETE_SERVICE if self.event_mode == 'spawn_then_delete' else None,
            'event_mode': self.event_mode,
            'obstacle_lifetime_sim_sec': self.obstacle_lifetime_sim_sec,
            'model_sdf': str(self.model_sdf),
            'map_box': dict(self.map_box),
            'gazebo_pose': dict(self.gazebo_pose),
            'minimum_odom_travel_m': self.minimum_odom_travel_m,
            'initial_path_sim_time': None,
            'motion_gate_sim_time': None,
            'spawn_request_sim_time': None,
            'spawn_ack_sim_time': None,
            'first_global_costmap_detection_sim_time': None,
            'first_local_costmap_detection_sim_time': None,
            'obstacle_remove_due_sim_time': None,
            'obstacle_remove_request_sim_time': None,
            'obstacle_remove_ack_sim_time': None,
            'obstacle_remove_success': None,
            'deleted_entity_type': MODEL_ENTITY_TYPE if self.event_mode == 'spawn_then_delete' else None,
            'first_global_costmap_clear_sim_time': None,
            'first_local_costmap_clear_sim_time': None,
        }
        self.create_subscription(NavPath, '/plan', self._on_plan, VOLATILE_RELIABLE_QOS)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Costmap, '/global_costmap/costmap_raw', self._on_global_costmap, MAP_QOS)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw', self._on_local_costmap, MAP_QOS)
        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        self.latest_clock: float | None = None
        self.write_record()

    def write_record(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(json.dumps(self.record, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        with self.output_path.open('rb') as stream:
            import os
            os.fsync(stream.fileno())

    def _now_sim(self) -> float | None:
        return self.latest_clock

    def _on_clock(self, message: Clock) -> None:
        self.latest_clock = stamp_seconds(message.clock)
        self._maybe_remove()

    def _on_plan(self, message: NavPath) -> None:
        if self.initial_plan_stamp is not None or not message.poses:
            return
        self.initial_plan_stamp = stamp_seconds(message.header.stamp)
        self.record['initial_path_sim_time'] = self.initial_plan_stamp
        if self.latest_odom is not None:
            self.initial_odom = (
                float(self.latest_odom.pose.pose.position.x),
                float(self.latest_odom.pose.pose.position.y),
            )
        self.write_record()

    def _on_odom(self, message: Odometry) -> None:
        self.latest_odom = message
        if self.initial_plan_stamp is None:
            return
        if self.initial_odom is None:
            self.initial_odom = (float(message.pose.pose.position.x), float(message.pose.pose.position.y))
            return
        if self.spawned:
            return
        traveled = math.dist(
            self.initial_odom,
            (float(message.pose.pose.position.x), float(message.pose.pose.position.y)),
        )
        if traveled < self.minimum_odom_travel_m:
            return
        self.record['motion_gate_sim_time'] = self._now_sim()
        self.record['motion_gate_odom_travel_m'] = traveled
        self._spawn_once()

    def _spawn_once(self) -> None:
        if self.spawned:
            return
        self.spawned = True
        self.record['spawn_request_sim_time'] = self._now_sim()
        if not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.record.update({'status': 'FAIL', 'error': f'{SPAWN_SERVICE} is unavailable at event time'})
            self.write_record()
            return
        request = SpawnEntity.Request()
        request.entity_factory.name = self.entity_name
        request.entity_factory.allow_renaming = False
        request.entity_factory.sdf_filename = str(self.model_sdf)
        pose = Pose()
        pose.position.x = self.gazebo_pose['x']
        pose.position.y = self.gazebo_pose['y']
        pose.position.z = self.gazebo_pose['z']
        pose.orientation.z = math.sin(self.gazebo_pose['yaw'] / 2.0)
        pose.orientation.w = math.cos(self.gazebo_pose['yaw'] / 2.0)
        request.entity_factory.pose = pose
        request.entity_factory.relative_to = 'world'
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self._on_spawn_response)
        self.spawn_timeout_timer = self.create_timer(3.0, self._on_spawn_timeout)

    def _on_spawn_timeout(self) -> None:
        if self.spawn_timeout_timer is not None:
            self.spawn_timeout_timer.cancel()
            self.spawn_timeout_timer = None
        if self.record['spawn_ack_sim_time'] is None and self.record.get('status') != 'FAIL':
            self.record.update({'status': 'FAIL', 'error': 'SpawnEntity did not return a response'})
            self.write_record()

    def _on_spawn_response(self, future) -> None:
        if self.spawn_timeout_timer is not None:
            self.spawn_timeout_timer.cancel()
            self.spawn_timeout_timer = None
        try:
            response = future.result()
        except Exception as error:
            self.record.update({'status': 'FAIL', 'error': f'SpawnEntity call failed: {error}'})
            self.write_record()
            return
        if response is None:
            self.record.update({'status': 'FAIL', 'error': 'SpawnEntity returned no response'})
            self.write_record()
            return
        if not response.success:
            self.record.update({'status': 'FAIL', 'error': 'Gazebo rejected SpawnEntity request'})
            self.write_record()
            return
        acknowledged_sim_time = self._now_sim()
        if acknowledged_sim_time is None:
            self.record.update({
                'status': 'FAIL',
                'error': 'SpawnEntity acknowledged before a usable simulation clock sample',
            })
            self.write_record()
            return
        self.record['spawn_ack_sim_time'] = acknowledged_sim_time
        self.record['status'] = 'SPAWNED'
        if self.obstacle_lifetime_sim_sec is not None:
            self.record['obstacle_remove_due_sim_time'] = (
                acknowledged_sim_time + self.obstacle_lifetime_sim_sec
            )
        self.write_record()

    def _maybe_remove(self) -> None:
        """Request the predeclared deletion once its independent clock is due."""
        if self.event_mode != 'spawn_then_delete':
            return
        due = self.record.get('obstacle_remove_due_sim_time')
        if not isinstance(due, float) or self.record['obstacle_remove_request_sim_time'] is not None:
            return
        now = self._now_sim()
        if now is None or now < due:
            return
        self.record['obstacle_remove_request_sim_time'] = now
        self.record['status'] = 'REMOVE_REQUESTED'
        if not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.record.update({'status': 'FAIL', 'error': f'{DELETE_SERVICE} is unavailable at event time'})
            self.write_record()
            return
        request = DeleteEntity.Request()
        request.entity.name = self.entity_name
        request.entity.type = MODEL_ENTITY_TYPE
        future = self.delete_client.call_async(request)
        future.add_done_callback(self._on_delete_response)
        self.delete_timeout_timer = self.create_timer(3.0, self._on_delete_timeout)
        self.write_record()

    def _on_delete_timeout(self) -> None:
        if self.delete_timeout_timer is not None:
            self.delete_timeout_timer.cancel()
            self.delete_timeout_timer = None
        if self.record['obstacle_remove_ack_sim_time'] is None and self.record.get('status') != 'FAIL':
            self.record.update({'status': 'FAIL', 'error': 'DeleteEntity did not return a response'})
            self.write_record()

    def _on_delete_response(self, future) -> None:
        if self.delete_timeout_timer is not None:
            self.delete_timeout_timer.cancel()
            self.delete_timeout_timer = None
        try:
            response = future.result()
        except Exception as error:
            self.record.update({'status': 'FAIL', 'error': f'DeleteEntity call failed: {error}'})
            self.write_record()
            return
        if response is None or not response.success:
            self.record.update({
                'status': 'FAIL',
                'error': 'DeleteEntity returned no response' if response is None
                else 'Gazebo rejected DeleteEntity request',
            })
            self.write_record()
            return
        self.record['obstacle_remove_ack_sim_time'] = self._now_sim()
        self.record['obstacle_remove_success'] = True
        self.record['status'] = 'REMOVE_ACKED'
        self.write_record()

    def _on_global_costmap(self, message: Costmap) -> None:
        if not self.spawned or self.record['spawn_ack_sim_time'] is None:
            return
        detects_box = costmap_detects_box(message, self.map_box)
        if self.record['first_global_costmap_detection_sim_time'] is None and detects_box:
            self.record['first_global_costmap_detection_sim_time'] = stamp_seconds(message.header.stamp)
            self._maybe_complete()
        if (
            self.record['obstacle_remove_ack_sim_time'] is not None
            and self.record['first_global_costmap_clear_sim_time'] is None
            and not detects_box
        ):
            self.record['first_global_costmap_clear_sim_time'] = stamp_seconds(message.header.stamp)
            self._maybe_complete()

    def _map_box_in_costmap_frame(self, message: Costmap) -> dict[str, float] | None:
        target_frame = message.header.frame_id
        if target_frame == MAP_FRAME:
            return self.map_box
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, MAP_FRAME, Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = yaw_from_quaternion(rotation)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return {
            'center_x': float(translation.x) + cosine * self.map_box['center_x'] - sine * self.map_box['center_y'],
            'center_y': float(translation.y) + sine * self.map_box['center_x'] + cosine * self.map_box['center_y'],
            'size_x': self.map_box['size_x'],
            'size_y': self.map_box['size_y'],
            'yaw': yaw + self.map_box['yaw'],
        }

    def _on_local_costmap(self, message: Costmap) -> None:
        if not self.spawned or self.record['spawn_ack_sim_time'] is None:
            return
        box = self._map_box_in_costmap_frame(message)
        if box is None:
            return
        detects_box = costmap_detects_box(message, box)
        if self.record['first_local_costmap_detection_sim_time'] is None and detects_box:
            self.record['first_local_costmap_detection_sim_time'] = stamp_seconds(message.header.stamp)
            self._maybe_complete()
        if (
            self.record['obstacle_remove_ack_sim_time'] is not None
            and self.record['first_local_costmap_clear_sim_time'] is None
            and not detects_box
        ):
            self.record['first_local_costmap_clear_sim_time'] = stamp_seconds(message.header.stamp)
            self.write_record()

    def _maybe_complete(self) -> None:
        if self.complete:
            return
        if (
            self.record['first_global_costmap_detection_sim_time'] is None
            or self.record['first_local_costmap_detection_sim_time'] is None
        ):
            self.write_record()
            return
        if self.event_mode == 'spawn_only':
            self.complete = True
            self.record['status'] = 'PASS'
            self.write_record()
            return
        if (
            self.record['obstacle_remove_ack_sim_time'] is not None
            and self.record['first_global_costmap_clear_sim_time'] is not None
        ):
            self.complete = True
            self.record['status'] = 'PASS'
        else:
            self.record['status'] = 'DETECTED'
        self.write_record()


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    package = Path(get_package_share_directory('resilient_nav_navigation'))
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--output-path', required=True)
    parser.add_argument('--scenarios-file', default=str(package / 'config' / 'navigation_robustness_scenarios.yaml'))
    return parser.parse_known_args(argv)[0]


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    node: ObstacleEventInjector | None = None
    rclpy.init(args=None)
    try:
        scenarios_file = Path(arguments.scenarios_file)
        node = ObstacleEventInjector(
            arguments.scenario, load_scenario(scenarios_file, arguments.scenario),
            Path(arguments.output_path), scenarios_file,
        )
        while rclpy.ok() and not node.complete and node.record.get('status') != 'FAIL':
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0 if node.record.get('status') == 'PASS' else 1
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        if node is not None:
            node.record.update({'status': 'FAIL', 'error': str(error)})
            node.write_record()
        return 1
    finally:
        if node is not None:
            if node.record.get('status') not in ('PENDING', 'PASS', 'FAIL'):
                node.record['status'] = 'INCOMPLETE'
            node.write_record()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
