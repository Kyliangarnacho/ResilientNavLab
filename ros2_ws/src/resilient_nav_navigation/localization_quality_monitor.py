"""ROS adapter publishing GT-free map-frame localization quality."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseWithCovarianceStamped

from localization_quality import (
    LocalizationEvidence,
    LocalizationQualityConfig,
    LocalizationQualityEvaluator,
    valid_runtime_stamp,
)

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from resilient_nav_interfaces.msg import LocalizationQuality, SensorHealth

from tf2_msgs.msg import TFMessage


AMCL_POSE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def seconds_from_stamp(stamp) -> float:
    """Convert a ROS time message to floating-point seconds."""
    return stamp.sec + stamp.nanosec / 1_000_000_000.0


def yaw_from_quaternion(quaternion) -> float:
    """Return planar yaw and reject malformed orientations."""
    norm = sum(
        value * value
        for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    )
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError('invalid quaternion')
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        ),
    )


def angular_distance(first: float, second: float) -> float:
    """Return the absolute shortest angular distance."""
    return abs((second - first + math.pi) % (2.0 * math.pi) - math.pi)


class LocalizationQualityMonitor(Node):
    """Turn AMCL, map-to-odom TF, and scan health into one runtime signal."""

    def __init__(self) -> None:
        """Create the subscriptions and periodic quality publisher."""
        super().__init__('localization_quality_monitor')
        self._declare_parameters()
        self._evaluator = LocalizationQualityEvaluator(self._read_config())
        self._start_sec = self._now_sec()
        self._pose_received_sec = None
        self._pose_stamp_sec = None
        self._pose_valid = True
        self._position_variance = None
        self._yaw_variance = None
        self._previous_pose = None
        self._position_jump_m = 0.0
        self._yaw_jump_rad = 0.0
        self._tf_received_sec = None
        self._tf_stamp_sec = None
        self._tf_valid = True
        self._tf_update_sequence = 0
        self._scan_health_received_sec = None
        self._scan_health_state = None
        self._publisher = self.create_publisher(
            LocalizationQuality, '/localization/quality', 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._on_amcl_pose,
            AMCL_POSE_QOS,
        )
        self.create_subscription(
            TFMessage, '/tf', self._on_tf, qos_profile_sensor_data
        )
        self.create_subscription(
            SensorHealth, '/health/scan', self._on_scan_health, 10
        )
        publish_rate_hz = self._float_parameter('publish_rate_hz')
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.create_timer(1.0 / publish_rate_hz, self._publish)

    def _declare_parameters(self) -> None:
        defaults = LocalizationQualityConfig()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('pose_jump_max_interval_sec', 1.0)

    def _read_config(self) -> LocalizationQualityConfig:
        names = LocalizationQualityConfig.__dataclass_fields__
        values = {name: self.get_parameter(name).value for name in names}
        return LocalizationQualityConfig(**values)

    def _float_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        return value

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        received_sec = self._now_sec()
        stamp_sec = seconds_from_stamp(message.header.stamp)
        pose = message.pose.pose
        covariance = tuple(message.pose.covariance)
        try:
            yaw = yaw_from_quaternion(pose.orientation)
            values = (
                stamp_sec,
                pose.position.x,
                pose.position.y,
                yaw,
                covariance[0],
                covariance[7],
                covariance[35],
            )
            valid = (
                message.header.frame_id == 'map'
                and len(covariance) == 36
                and all(math.isfinite(value) for value in values)
                and min(covariance[0], covariance[7], covariance[35]) >= 0.0
                and valid_runtime_stamp(stamp_sec)
            )
        except (IndexError, ValueError):
            valid = False
            yaw = 0.0
        self._pose_received_sec = received_sec
        self._pose_stamp_sec = stamp_sec
        self._pose_valid = valid
        if not valid:
            self._position_variance = None
            self._yaw_variance = None
            return
        self._position_variance = max(covariance[0], covariance[7])
        self._yaw_variance = covariance[35]
        current_pose = (stamp_sec, pose.position.x, pose.position.y, yaw)
        self._update_pose_jump(current_pose)
        self._previous_pose = current_pose

    def _update_pose_jump(self, current_pose) -> None:
        self._position_jump_m = 0.0
        self._yaw_jump_rad = 0.0
        if self._previous_pose is None:
            return
        previous_stamp, previous_x, previous_y, previous_yaw = (
            self._previous_pose
        )
        stamp, x, y, yaw = current_pose
        delta_sec = stamp - previous_stamp
        if not 0.0 < delta_sec <= self._float_parameter(
            'pose_jump_max_interval_sec'
        ):
            return
        self._position_jump_m = math.hypot(x - previous_x, y - previous_y)
        self._yaw_jump_rad = angular_distance(previous_yaw, yaw)

    def _on_tf(self, message: TFMessage) -> None:
        received_sec = self._now_sec()
        for transform in message.transforms:
            if (
                transform.header.frame_id != 'map'
                or transform.child_frame_id != 'odom'
            ):
                continue
            stamp_sec = seconds_from_stamp(transform.header.stamp)
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            try:
                yaw_from_quaternion(rotation)
                values = (
                    stamp_sec,
                    translation.x,
                    translation.y,
                    translation.z,
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                )
                valid = (
                    all(math.isfinite(value) for value in values)
                    and valid_runtime_stamp(stamp_sec)
                )
            except ValueError:
                valid = False
            self._tf_received_sec = received_sec
            self._tf_stamp_sec = stamp_sec
            self._tf_valid = valid
            self._tf_update_sequence += 1

    def _on_scan_health(self, message: SensorHealth) -> None:
        self._scan_health_received_sec = self._now_sec()
        self._scan_health_state = int(message.state)

    @staticmethod
    def _effective_age(now_sec, received_sec, stamp_sec=None):
        if received_sec is None:
            return None
        ages = [now_sec - received_sec]
        if stamp_sec is not None:
            ages.append(now_sec - stamp_sec)
        return max(0.0, *ages)

    def _evidence(self, now_sec: float) -> LocalizationEvidence:
        return LocalizationEvidence(
            pose_age_sec=self._effective_age(
                now_sec, self._pose_received_sec, self._pose_stamp_sec
            ),
            map_to_odom_age_sec=self._effective_age(
                now_sec, self._tf_received_sec, self._tf_stamp_sec
            ),
            scan_health_age_sec=self._effective_age(
                now_sec, self._scan_health_received_sec
            ),
            position_variance=self._position_variance,
            yaw_variance=self._yaw_variance,
            position_jump_m=self._position_jump_m,
            yaw_jump_rad=self._yaw_jump_rad,
            scan_health_state=self._scan_health_state,
            pose_valid=self._pose_valid,
            map_to_odom_valid=self._tf_valid,
            update_sequence=self._tf_update_sequence,
        )

    def _publish(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1_000_000_000.0
        evidence = self._evidence(now_sec)
        decision = self._evaluator.evaluate(
            evidence, max(now_sec - self._start_sec, 0.0)
        )
        message = LocalizationQuality()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'map'
        message.state = int(decision.state)
        message.localization_usable = decision.localization_usable
        message.confidence = decision.confidence
        message.pose_age_sec = self._value_or_unknown(evidence.pose_age_sec)
        message.map_to_odom_age_sec = self._value_or_unknown(
            evidence.map_to_odom_age_sec
        )
        message.scan_health_age_sec = self._value_or_unknown(
            evidence.scan_health_age_sec
        )
        message.position_variance = self._value_or_unknown(
            evidence.position_variance
        )
        message.yaw_variance = self._value_or_unknown(evidence.yaw_variance)
        message.position_jump_m = evidence.position_jump_m
        message.yaw_jump_rad = evidence.yaw_jump_rad
        message.reasons = list(decision.reasons)
        self._publisher.publish(message)

    @staticmethod
    def _value_or_unknown(value) -> float:
        return -1.0 if value is None else float(value)


def main(args=None) -> None:
    """Run the localization-quality monitor."""
    rclpy.init(args=args)
    node = LocalizationQualityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
