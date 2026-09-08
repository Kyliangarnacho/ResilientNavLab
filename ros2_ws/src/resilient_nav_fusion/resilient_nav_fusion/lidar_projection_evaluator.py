"""Evaluator-only comparison of exact-stamp EKF and Gazebo-GT LiDAR projection."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


@dataclass(frozen=True)
class Pose2D:
    stamp_ns: int
    x: float
    y: float
    yaw: float


def interpolate_pose(samples: list[Pose2D], stamp_ns: int) -> Pose2D | None:
    """Interpolate an exact timestamp between adjacent evaluator GT poses."""
    if not samples or stamp_ns < samples[0].stamp_ns or stamp_ns > samples[-1].stamp_ns:
        return None
    for first, second in zip(samples, samples[1:]):
        if stamp_ns > second.stamp_ns:
            continue
        if stamp_ns == first.stamp_ns or second.stamp_ns == first.stamp_ns:
            return Pose2D(stamp_ns, first.x, first.y, first.yaw)
        ratio = (stamp_ns - first.stamp_ns) / (second.stamp_ns - first.stamp_ns)
        yaw_delta = normalize_angle(second.yaw - first.yaw)
        return Pose2D(
            stamp_ns,
            first.x + ratio * (second.x - first.x),
            first.y + ratio * (second.y - first.y),
            normalize_angle(first.yaw + ratio * yaw_delta),
        )
    last = samples[-1]
    return Pose2D(stamp_ns, last.x, last.y, last.yaw)


def compose_pose(parent: Pose2D, child_x: float, child_y: float, child_yaw: float) -> Pose2D:
    """Compose odom<-base and base<-LiDAR planar transforms."""
    cosine = math.cos(parent.yaw)
    sine = math.sin(parent.yaw)
    return Pose2D(
        parent.stamp_ns,
        parent.x + cosine * child_x - sine * child_y,
        parent.y + sine * child_x + cosine * child_y,
        normalize_angle(parent.yaw + child_yaw),
    )


def project_scan_local(scan: LaserScan) -> np.ndarray:
    """Project valid LaserScan returns in its local frame at header stamp."""
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = float(scan.angle_min) + np.arange(len(ranges)) * float(
        scan.angle_increment
    )
    valid = (
        np.isfinite(ranges)
        & (ranges >= float(scan.range_min))
        & (ranges < float(scan.range_max))
    )
    return np.column_stack((
        ranges[valid] * np.cos(angles[valid]),
        ranges[valid] * np.sin(angles[valid]),
    ))


def transform_points(points: np.ndarray, pose: Pose2D) -> np.ndarray:
    """Apply one planar rigid transform to local scan points."""
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return points @ rotation.T + np.array([pose.x, pose.y])


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class LidarProjectionEvaluator(Node):
    """Publish and record read-only GT/EKF projection evidence in odom."""

    def __init__(self) -> None:
        super().__init__('lidar_projection_evaluator')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('ground_truth_topic', '/evaluation/ground_truth_pose')
        self.declare_parameter('ekf_cloud_topic', '/evaluation/lidar_points_ekf')
        self.declare_parameter('ground_truth_cloud_topic', '/evaluation/lidar_points_gt')
        self.declare_parameter(
            'output_path', '/tmp/brne_lidar_geometry_diagnostic.json'
        )
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('lidar_frame', 'lidar_link')

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.lidar_frame = str(self.get_parameter('lidar_frame').value)
        self.output_path = Path(str(self.get_parameter('output_path').value))
        self._gt_samples: deque[Pose2D] = deque(maxlen=5000)
        self._pending_scans: deque[LaserScan] = deque(maxlen=10)
        self._records: list[dict] = []
        self._counts = {
            'scans_received': 0,
            'scans_projected': 0,
            'scans_dropped': 0,
        }
        self._scan_contract: dict | None = None
        self._last_gt_projection: Pose2D | None = None
        self._base_to_lidar: tuple[float, float, float] | None = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ekf_publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter('ekf_cloud_topic').value), 10
        )
        self.gt_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('ground_truth_cloud_topic').value),
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('ground_truth_topic').value),
            self._on_ground_truth,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / 60.0, self._process)
        self.get_logger().info(
            'Evaluator-only LiDAR projection enabled; GT is not published to runtime topics'
        )

    def _on_scan(self, message: LaserScan) -> None:
        self._counts['scans_received'] += 1
        if message.header.frame_id != self.lidar_frame:
            self._counts['scans_dropped'] += 1
            return
        if len(self._pending_scans) == self._pending_scans.maxlen:
            self._counts['scans_dropped'] += 1
        self._pending_scans.append(message)
        if self._scan_contract is None:
            self._scan_contract = {
                'frame_id': message.header.frame_id,
                'beam_count': len(message.ranges),
                'time_increment_sec': float(message.time_increment),
                'scan_time_sec': float(message.scan_time),
                'header_stamp_semantics': 'first_ray',
                'projection_model': 'whole_scan_rigid_at_header_stamp',
                'per_beam_timing_available': float(message.time_increment) > 0.0,
            }

    def _on_ground_truth(self, message: PoseStamped) -> None:
        if message.header.frame_id != self.odom_frame:
            return
        stamp_ns = _stamp_ns(message.header.stamp)
        yaw = _yaw_from_quaternion(message.pose.orientation)
        if stamp_ns <= 0 or yaw is None:
            return
        sample = Pose2D(
            stamp_ns,
            float(message.pose.position.x),
            float(message.pose.position.y),
            yaw,
        )
        if self._gt_samples and stamp_ns <= self._gt_samples[-1].stamp_ns:
            return
        self._gt_samples.append(sample)

    def _process(self) -> None:
        if not self._pending_scans or len(self._gt_samples) < 2:
            return
        scan = self._pending_scans[0]
        stamp_ns = _stamp_ns(scan.header.stamp)
        gt_base = interpolate_pose(list(self._gt_samples), stamp_ns)
        if gt_base is None:
            if self._gt_samples[-1].stamp_ns > stamp_ns:
                self._pending_scans.popleft()
                self._counts['scans_dropped'] += 1
            return
        if self._base_to_lidar is None:
            try:
                static_tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.lidar_frame, Time()
                )
            except Exception:
                return
            static_yaw = _yaw_from_quaternion(static_tf.transform.rotation)
            if static_yaw is None:
                return
            translation = static_tf.transform.translation
            self._base_to_lidar = (
                float(translation.x), float(translation.y), static_yaw
            )
        try:
            ekf_tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.lidar_frame,
                Time.from_msg(scan.header.stamp),
            )
        except Exception:
            return
        ekf_yaw = _yaw_from_quaternion(ekf_tf.transform.rotation)
        if ekf_yaw is None:
            self._pending_scans.popleft()
            self._counts['scans_dropped'] += 1
            return

        self._pending_scans.popleft()
        gt_lidar = compose_pose(gt_base, *self._base_to_lidar)
        translation = ekf_tf.transform.translation
        ekf_lidar = Pose2D(
            stamp_ns, float(translation.x), float(translation.y), ekf_yaw
        )
        local_points = project_scan_local(scan)
        if not len(local_points):
            self._counts['scans_dropped'] += 1
            return
        gt_points = transform_points(local_points, gt_lidar)
        ekf_points = transform_points(local_points, ekf_lidar)
        self.gt_publisher.publish(_cloud_message(scan, self.odom_frame, gt_points))
        self.ekf_publisher.publish(_cloud_message(scan, self.odom_frame, ekf_points))
        self._record_projection(gt_lidar, ekf_lidar, gt_points, ekf_points)
        self._counts['scans_projected'] += 1
        if self._counts['scans_projected'] % 15 == 0:
            self._write_report()

    def _record_projection(
        self,
        gt_pose: Pose2D,
        ekf_pose: Pose2D,
        gt_points: np.ndarray,
        ekf_points: np.ndarray,
    ) -> None:
        endpoint_errors = np.linalg.norm(ekf_points - gt_points, axis=1)
        angular_velocity = 0.0
        if self._last_gt_projection is not None:
            elapsed = (gt_pose.stamp_ns - self._last_gt_projection.stamp_ns) / 1e9
            if elapsed > 0.0:
                angular_velocity = normalize_angle(
                    gt_pose.yaw - self._last_gt_projection.yaw
                ) / elapsed
        self._last_gt_projection = gt_pose
        direction = 'stationary'
        if angular_velocity > 0.02:
            direction = 'ccw'
        elif angular_velocity < -0.02:
            direction = 'cw'
        self._records.append({
            'scan_stamp_sec': gt_pose.stamp_ns / 1e9,
            'rotation_direction': direction,
            'gt_angular_velocity_rad_s': angular_velocity,
            'point_count': len(gt_points),
            'gt_lidar_pose_odom': _pose_dict(gt_pose),
            'ekf_lidar_pose_odom': _pose_dict(ekf_pose),
            'ekf_minus_gt': {
                'x_m': ekf_pose.x - gt_pose.x,
                'y_m': ekf_pose.y - gt_pose.y,
                'yaw_rad': normalize_angle(ekf_pose.yaw - gt_pose.yaw),
            },
            'endpoint_error_m': {
                'median': float(np.median(endpoint_errors)),
                'p95': float(np.percentile(endpoint_errors, 95)),
                'maximum': float(np.max(endpoint_errors)),
            },
        })

    def _write_report(self) -> None:
        report = {
            'channel': 'evaluator_only',
            'runtime_topics_written': [],
            'comparison_frame': self.odom_frame,
            'counts': self._counts,
            'scan_contract': self._scan_contract,
            'records': self._records,
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_path.with_suffix(self.output_path.suffix + '.tmp')
        temporary.write_text(json.dumps(report, indent=2), encoding='utf-8')
        temporary.replace(self.output_path)

    def destroy_node(self):
        self._write_report()
        return super().destroy_node()


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw_from_quaternion(quaternion) -> float | None:
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        return None
    x_value, y_value, z_value, w_value = (value / norm for value in values)
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


def _cloud_message(scan: LaserScan, frame_id: str, points: np.ndarray) -> PointCloud2:
    header = Header(stamp=scan.header.stamp, frame_id=frame_id)
    xyz = np.column_stack((points, np.zeros(len(points), dtype=float)))
    return point_cloud2.create_cloud_xyz32(header, xyz.tolist())


def _pose_dict(pose: Pose2D) -> dict[str, float]:
    return {'x_m': pose.x, 'y_m': pose.y, 'yaw_rad': pose.yaw}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarProjectionEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
