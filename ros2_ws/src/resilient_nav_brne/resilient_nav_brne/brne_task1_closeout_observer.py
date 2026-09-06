"""Bounded, read-only observer for the one Task 1 Phase 10 shadow run."""

from __future__ import annotations

import copy
import json
import math
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from resilient_nav_interfaces.msg import PedestrianArray

from .brne_shadow_input_adapter import PLAN_QOS
from .waypoint_adapter import select_local_waypoint, transform_points_se2, yaw_from_quaternion


def _stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class BrneTask1CloseoutObserver(Node):
    """Save only observed data-flow, bounded-output, and stale-stop facts."""

    def __init__(self):
        super().__init__('brne_task1_closeout_observer')
        self.declare_parameter('output_path', '/tmp/brne_task1_closeout.json')
        self.declare_parameter('deadline_sec', 75.0)
        self.output_path = Path(str(self.get_parameter('output_path').value))
        self.deadline = time.monotonic() + float(self.get_parameter('deadline_sec').value)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.source_odom: dict[int, Odometry] = {}
        self.brne_odom: dict[int, Odometry] = {}
        self.source_paths: deque[NavPath] = deque(maxlen=30)
        self.goal_records: deque[tuple[PoseStamped, float]] = deque(maxlen=30)
        self.facts = {
            'source_odom_count': 0, 'source_plan_count': 0, 'brne_odom_count': 0,
            'brne_goal_count': 0, 'pedestrian_count': 0, 'raw_count': 0,
            'active_raw_count': 0, 'optimal_path_count': 0,
            'raw_linear_min': None, 'raw_linear_max': None,
            'raw_angular_min': None, 'raw_angular_max': None,
            'invalid_raw_count': 0, 'invalid_path_count': 0,
            'source_frames': set(), 'brne_odom_frames': set(), 'goal_frames': set(),
            'pedestrian_frames': set(), 'optimal_path_frames': set(),
        }
        self.waypoint_evidence = None
        self.last_goal_received = None
        self.active_seen = False
        self.stale_stop = None
        self.done = False
        self.success = False

        self.create_subscription(Odometry, '/odometry/filtered', self._source_odom, qos_profile_sensor_data)
        self.create_subscription(NavPath, '/plan', self._source_path, PLAN_QOS)
        self.create_subscription(Odometry, '/brne/odom', self._brne_odom, 10)
        self.create_subscription(PoseStamped, '/brne/goal_pose', self._goal, 10)
        self.create_subscription(PedestrianArray, '/brne/pedestrians', self._pedestrian, 10)
        self.create_subscription(Twist, '/brne/cmd_vel_raw', self._raw, 10)
        self.create_subscription(NavPath, '/brne/optimal_path', self._optimal_path, 10)
        self.create_timer(0.05, self._tick)

    def _source_odom(self, message):
        message = copy.deepcopy(message)
        self.facts['source_odom_count'] += 1
        self.facts['source_frames'].add(message.header.frame_id)
        self.source_odom[_stamp_ns(message)] = message
        if len(self.source_odom) > 300:
            self.source_odom.pop(next(iter(self.source_odom)))

    def _source_path(self, message):
        self.facts['source_plan_count'] += 1
        self.source_paths.append(copy.deepcopy(message))

    def _brne_odom(self, message):
        message = copy.deepcopy(message)
        self.facts['brne_odom_count'] += 1
        self.facts['brne_odom_frames'].add(message.header.frame_id)
        self.brne_odom[_stamp_ns(message)] = message
        if len(self.brne_odom) > 300:
            self.brne_odom.pop(next(iter(self.brne_odom)))

    def _goal(self, message):
        self.facts['brne_goal_count'] += 1
        self.facts['goal_frames'].add(message.header.frame_id)
        self.last_goal_received = time.monotonic()
        self.goal_records.append((copy.deepcopy(message), self.last_goal_received))

    def _pedestrian(self, message):
        self.facts['pedestrian_count'] += 1
        self.facts['pedestrian_frames'].add(message.header.frame_id)

    def _raw(self, message):
        values = (float(message.linear.x), float(message.angular.z))
        self.facts['raw_count'] += 1
        if not all(math.isfinite(value) for value in values):
            self.facts['invalid_raw_count'] += 1
            return
        for key, value in (('raw_linear', values[0]), ('raw_angular', values[1])):
            minimum, maximum = f'{key}_min', f'{key}_max'
            self.facts[minimum] = value if self.facts[minimum] is None else min(self.facts[minimum], value)
            self.facts[maximum] = value if self.facts[maximum] is None else max(self.facts[maximum], value)
        if abs(values[0]) > 1e-6 or abs(values[1]) > 1e-6:
            self.active_seen = True
            self.facts['active_raw_count'] += 1
        elif self.last_goal_received is not None:
            age = time.monotonic() - self.last_goal_received
            if age >= 0.5 and self.stale_stop is None:
                self.stale_stop = {
                    'goal_silence_sec': round(age, 3),
                    'raw_command': list(values),
                    'active_raw_seen_before_stop': self.active_seen,
                }

    def _optimal_path(self, message):
        self.facts['optimal_path_count'] += 1
        self.facts['optimal_path_frames'].add(message.header.frame_id)
        if (
            message.header.frame_id != 'odom'
            or _stamp_ns(message) <= 0
            or not message.poses
            or not all(math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y) for p in message.poses)
        ):
            self.facts['invalid_path_count'] += 1

    def _tick(self):
        if self.done:
            return
        if self.waypoint_evidence is None:
            self.waypoint_evidence = self._find_waypoint_evidence()
        if self._complete():
            self.success = True
            self._write_result('PASS')
            self.done = True
        elif time.monotonic() >= self.deadline:
            self._write_result('FAIL', 'deadline before all closeout facts were observed')
            self.done = True

    def _find_waypoint_evidence(self):
        """Match one timestamped BRNE goal to its real map Path waypoint."""
        for goal, _received_at in self.goal_records:
            stamp_ns = _stamp_ns(goal)
            odom = self.brne_odom.get(stamp_ns)
            if odom is None:
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    'odom', 'map', Time.from_msg(goal.header.stamp),
                    timeout=Duration(seconds=0.01),
                ).transform
            except Exception:
                continue
            yaw = yaw_from_quaternion(transform.rotation)
            if yaw is None:
                continue
            for path in reversed(self.source_paths):
                if path.header.frame_id != 'map' or not path.poses:
                    continue
                points = transform_points_se2(
                    [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses],
                    transform.translation.x, transform.translation.y, yaw,
                )
                selected = select_local_waypoint(
                    points,
                    (odom.pose.pose.position.x, odom.pose.pose.position.y),
                    0.8, 0.75,
                )
                if selected is None:
                    continue
                index, waypoint, nearest_distance = selected
                residual = math.hypot(
                    float(goal.pose.position.x) - waypoint[0],
                    float(goal.pose.position.y) - waypoint[1],
                )
                if residual <= 1e-5:
                    source = self.source_odom.get(stamp_ns)
                    return {
                        'goal_odom_stamp_ns': stamp_ns,
                        'source_plan_frame': path.header.frame_id,
                        'tf_query_stamp_ns': stamp_ns,
                        'waypoint_index': index,
                        'nearest_distance_m': round(float(nearest_distance), 6),
                        'goal_waypoint_residual_m': round(residual, 9),
                        'brne_odom_matches_source_odom': bool(
                            source is not None
                            and source.pose.pose.position.x == odom.pose.pose.position.x
                            and source.pose.pose.position.y == odom.pose.pose.position.y
                        ),
                    }
        return None

    def _complete(self):
        frames_ok = (
            self.facts['source_frames'] == {'odom'}
            and self.facts['brne_odom_frames'] == {'odom'}
            and self.facts['goal_frames'] == {'odom'}
            and self.facts['pedestrian_frames'] == {'odom'}
            and self.facts['optimal_path_frames'] == {'odom'}
        )
        bounded = (
            self.facts['invalid_raw_count'] == 0
            and self.facts['raw_linear_min'] is not None
            and 0.0 <= self.facts['raw_linear_min']
            and self.facts['raw_linear_max'] <= 0.30
            and abs(self.facts['raw_angular_min']) <= 0.80
            and abs(self.facts['raw_angular_max']) <= 0.80
        )
        return bool(
            self.waypoint_evidence
            and self.waypoint_evidence['brne_odom_matches_source_odom']
            and self.facts['source_plan_count'] > 0
            and self.facts['pedestrian_count'] > 0
            and self.facts['raw_count'] > 0
            and self.facts['optimal_path_count'] > 0
            and self.facts['invalid_path_count'] == 0
            and frames_ok and bounded and self.stale_stop
        )

    def _write_result(self, status, error=None):
        publishers = {
            topic: sorted(
                f'{info.node_namespace}/{info.node_name}'
                for info in self.get_publishers_info_by_topic(topic)
            )
            for topic in ('/brne/cmd_vel_raw', '/cmd_vel')
        }
        record = {
            'status': status,
            'facts': {key: sorted(value) if isinstance(value, set) else value for key, value in self.facts.items()},
            'timestamped_tf_waypoint': self.waypoint_evidence,
            'stale_stop': self.stale_stop,
            'topic_isolation': {
                'raw_publishers': publishers['/brne/cmd_vel_raw'],
                'cmd_vel_publishers': publishers['/cmd_vel'],
                'brne_publishes_formal_cmd_vel': any('brne' in name for name in publishers['/cmd_vel']),
            },
            **({'error': error} if error else {}),
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(json.dumps(record, indent=2, sort_keys=True), encoding='utf-8')


def main(args=None):
    rclpy.init(args=args)
    observer = BrneTask1CloseoutObserver()
    try:
        while rclpy.ok() and not observer.done:
            rclpy.spin_once(observer, timeout_sec=0.1)
        if not observer.success:
            raise RuntimeError(f'closeout observer failed; see {observer.output_path}')
    finally:
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
