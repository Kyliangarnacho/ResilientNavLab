"""Assess IMU and wheel-odometry delivery and wheel-freeze health."""

from collections import deque
from dataclasses import dataclass
from math import atan2, hypot, pi

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from resilient_nav_interfaces.msg import SensorHealth
from sensor_msgs.msg import Imu


DEFAULT_PUBLISH_RATE_HZ = 5.0
DEFAULT_IMU_SOURCE_TOPIC = '/faulted/imu/data'
DEFAULT_WHEEL_SOURCE_TOPIC = '/faulted/wheel/odometry'
DEFAULT_SCAN_SOURCE_TOPIC = '/faulted/scan'


@dataclass(frozen=True)
class Observation:
    """A received sensor sample expressed in seconds and selected fields."""

    received_sec: float
    stamp_sec: float
    pose_x: float = 0.0
    pose_y: float = 0.0
    yaw_rad: float = 0.0
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class HealthDecision:
    """The transport and freeze assessment before conversion to a ROS message."""

    state: int
    health_score: float
    confidence: float
    detected_fault: str
    reasons: list
    metric_names: list
    metric_values: list
    window_start_sec: float
    window_end_sec: float
    sample_count: int


class SensorWindow:
    """Keep observations that remain inside a ROS-time sliding window."""

    def __init__(self, duration_sec):
        self._duration_sec = duration_sec
        self._samples = deque()

    def add(self, observation):
        """Append one observation."""
        self._samples.append(observation)

    def samples(self, now_sec):
        """Return the active observations after removing expired samples."""
        cutoff_sec = now_sec - self._duration_sec
        while self._samples and self._samples[0].received_sec < cutoff_sec:
            self._samples.popleft()
        return list(self._samples)


class HealthEvaluator:
    """Evaluate windowed timing conditions and commanded wheel freezes."""

    def __init__(
        self,
        window_duration_sec,
        min_samples,
        stale_timeout_sec,
        delay_warning_sec,
        delay_fault_sec,
        delay_confirmation_cycles,
        command_linear_threshold_mps,
        command_angular_threshold_rad_s,
        freeze_duration_sec,
        wheel_pose_span_threshold_m,
        wheel_linear_span_threshold_mps,
        wheel_angular_span_threshold_rad_s,
    ):
        self._min_samples = min_samples
        self._stale_timeout_sec = stale_timeout_sec
        self._delay_warning_sec = delay_warning_sec
        self._delay_fault_sec = delay_fault_sec
        if delay_confirmation_cycles < 1:
            raise ValueError('delay_confirmation_cycles must be at least 1')
        self._delay_confirmation_cycles = delay_confirmation_cycles
        self._delay_warning_counts = {'imu': 0, 'wheel': 0}
        self._delay_fault_counts = {'imu': 0, 'wheel': 0}
        self._command_linear_threshold_mps = command_linear_threshold_mps
        self._command_angular_threshold_rad_s = command_angular_threshold_rad_s
        self._freeze_duration_sec = freeze_duration_sec
        self._wheel_pose_span_threshold_m = wheel_pose_span_threshold_m
        self._wheel_linear_span_threshold_mps = wheel_linear_span_threshold_mps
        self._wheel_angular_span_threshold_rad_s = (
            wheel_angular_span_threshold_rad_s
        )
        self.imu_window = SensorWindow(window_duration_sec)
        self.wheel_window = SensorWindow(window_duration_sec)
        self._command_linear_abs_mps = 0.0
        self._command_angular_abs_rad_s = 0.0
        self._motion_command_started_sec = None

    def add_imu(self, received_sec, stamp_sec):
        """Record a received IMU sample."""
        self.imu_window.add(Observation(received_sec, stamp_sec))

    def add_wheel(
        self,
        received_sec,
        stamp_sec,
        pose_x,
        pose_y,
        yaw_rad,
        linear_x,
        angular_z,
    ):
        """Record a received wheel odometry sample."""
        self.wheel_window.add(
            Observation(
                received_sec,
                stamp_sec,
                pose_x,
                pose_y,
                yaw_rad,
                linear_x,
                angular_z,
            )
        )

    def set_command(self, received_sec, linear_x, angular_z):
        """Update the most recent command and its continuous active interval."""
        self._command_linear_abs_mps = abs(linear_x)
        self._command_angular_abs_rad_s = abs(angular_z)
        if self._is_motion_commanded():
            if self._motion_command_started_sec is None:
                self._motion_command_started_sec = received_sec
        else:
            self._motion_command_started_sec = None

    def evaluate_imu(self, now_sec):
        """Return the IMU timing decision at ``now_sec``."""
        return self._timing_decision(
            'imu', self.imu_window.samples(now_sec), now_sec
        )

    def evaluate_wheel(self, now_sec):
        """Return wheel timing decision, augmented by an active freeze check."""
        samples = self.wheel_window.samples(now_sec)
        decision = self._timing_decision('wheel', samples, now_sec)
        names = list(decision.metric_names)
        values = list(decision.metric_values)
        names.extend([
            'commanded_linear_abs_mps',
            'commanded_angular_abs_rad_s',
        ])
        values.extend([
            self._command_linear_abs_mps,
            self._command_angular_abs_rad_s,
        ])

        if samples:
            (
                pose_span_m,
                yaw_span_rad,
                linear_span_mps,
                angular_span_rad_s,
            ) = (
                self._wheel_spans(samples)
            )
            names.extend([
                'wheel_pose_span_m',
                'wheel_yaw_span_rad',
                'wheel_linear_span_mps',
                'wheel_angular_span_rad_s',
            ])
            values.extend([
                pose_span_m,
                yaw_span_rad,
                linear_span_mps,
                angular_span_rad_s,
            ])
        else:
            pose_span_m = yaw_span_rad = linear_span_mps = angular_span_rad_s = 0.0

        if decision.state in (SensorHealth.UNKNOWN, SensorHealth.FAULT):
            return self._with_metrics(decision, names, values)

        if self._wheel_is_frozen(
            now_sec,
            pose_span_m,
            yaw_span_rad,
            samples,
        ):
            return HealthDecision(
                SensorHealth.FAULT,
                0.0,
                decision.confidence,
                'freeze',
                ['motion_command_persisted_while_wheel_state_was_static'],
                names,
                values,
                decision.window_start_sec,
                decision.window_end_sec,
                decision.sample_count,
            )

        return self._with_metrics(decision, names, values)

    def _timing_decision(self, sensor, samples, now_sec):
        if len(samples) < self._min_samples:
            self._reset_delay_counts(sensor)
            return self._unknown_decision(samples)

        last = samples[-1]
        message_age_sec = max(now_sec - last.received_sec, 0.0)
        stamp_age_sec = max(now_sec - last.stamp_sec, 0.0)
        interarrival_sec = last.received_sec - samples[-2].received_sec
        names = [
            'message_age_sec',
            'stamp_age_sec',
            'interarrival_sec',
        ]
        values = [message_age_sec, stamp_age_sec, interarrival_sec]
        confidence = min(len(samples) / self._min_samples, 1.0)
        window_start_sec = samples[0].received_sec
        window_end_sec = last.received_sec

        if message_age_sec > self._stale_timeout_sec:
            self._reset_delay_counts(sensor)
            return HealthDecision(
                SensorHealth.FAULT,
                0.0,
                confidence,
                'stale',
                ['latest_message_exceeded_stale_timeout'],
                names,
                values,
                window_start_sec,
                window_end_sec,
                len(samples),
            )
        self._update_delay_counts(sensor, stamp_age_sec)
        if (
            self._delay_fault_counts[sensor]
            >= self._delay_confirmation_cycles
        ):
            return HealthDecision(
                SensorHealth.FAULT,
                0.0,
                confidence,
                'delay',
                ['latest_header_stamp_exceeded_delay_fault_threshold'],
                names,
                values,
                window_start_sec,
                window_end_sec,
                len(samples),
            )
        if (
            self._delay_warning_counts[sensor]
            >= self._delay_confirmation_cycles
        ):
            return HealthDecision(
                SensorHealth.DEGRADED,
                0.5,
                confidence,
                'delay',
                ['latest_header_stamp_exceeded_delay_warning_threshold'],
                names,
                values,
                window_start_sec,
                window_end_sec,
                len(samples),
            )
        return HealthDecision(
            SensorHealth.HEALTHY,
            1.0,
            confidence,
            'none',
            ['recent_messages_and_header_stamps_are_within_thresholds'],
            names,
            values,
            window_start_sec,
            window_end_sec,
            len(samples),
        )

    def _update_delay_counts(self, sensor, stamp_age_sec):
        """Track consecutive warning and fault-level header-delay checks."""
        if stamp_age_sec > self._delay_fault_sec:
            self._delay_fault_counts[sensor] += 1
            self._delay_warning_counts[sensor] += 1
        elif stamp_age_sec > self._delay_warning_sec:
            self._delay_fault_counts[sensor] = 0
            self._delay_warning_counts[sensor] += 1
        else:
            self._reset_delay_counts(sensor)

    def _reset_delay_counts(self, sensor):
        """Clear delay confirmation state after a healthy timing check."""
        self._delay_warning_counts[sensor] = 0
        self._delay_fault_counts[sensor] = 0

    def _unknown_decision(self, samples):
        count = len(samples)
        start_sec = samples[0].received_sec if samples else 0.0
        end_sec = samples[-1].received_sec if samples else 0.0
        reason = 'no_messages_received' if count == 0 else 'insufficient_samples'
        return HealthDecision(
            SensorHealth.UNKNOWN,
            -1.0,
            min(count / self._min_samples, 1.0),
            'unknown',
            [reason],
            [],
            [],
            start_sec,
            end_sec,
            count,
        )

    def _wheel_spans(self, samples):
        first = samples[0]
        pose_span_m = max(
            hypot(sample.pose_x - first.pose_x, sample.pose_y - first.pose_y)
            for sample in samples
        )
        linear_span_mps = max(
            sample.linear_x for sample in samples
        ) - min(sample.linear_x for sample in samples)
        angular_span_rad_s = max(
            sample.angular_z for sample in samples
        ) - min(sample.angular_z for sample in samples)
        yaw_span_rad = self._unwrapped_yaw_span(samples)
        return pose_span_m, yaw_span_rad, linear_span_mps, angular_span_rad_s

    @staticmethod
    def _unwrapped_yaw_span(samples):
        """Return yaw range after unwrapping consecutive angle samples."""
        previous_yaw_rad = samples[0].yaw_rad
        unwrapped_yaw_rad = previous_yaw_rad
        min_yaw_rad = unwrapped_yaw_rad
        max_yaw_rad = unwrapped_yaw_rad
        for sample in samples[1:]:
            delta_yaw_rad = (
                (sample.yaw_rad - previous_yaw_rad + pi) % (2.0 * pi)
            ) - pi
            unwrapped_yaw_rad += delta_yaw_rad
            min_yaw_rad = min(min_yaw_rad, unwrapped_yaw_rad)
            max_yaw_rad = max(max_yaw_rad, unwrapped_yaw_rad)
            previous_yaw_rad = sample.yaw_rad
        return max_yaw_rad - min_yaw_rad

    def _wheel_is_frozen(
        self,
        now_sec,
        pose_span_m,
        yaw_span_rad,
        samples,
    ):
        if not self._is_motion_commanded() or self._motion_command_started_sec is None:
            return False
        if now_sec - self._motion_command_started_sec < self._freeze_duration_sec:
            return False
        if (
            not samples
            or samples[-1].received_sec < self._motion_command_started_sec
        ):
            return False
        linear_is_frozen = (
            self._command_linear_abs_mps > self._command_linear_threshold_mps
            and pose_span_m < self._wheel_pose_span_threshold_m
        )
        angular_is_frozen = (
            self._command_angular_abs_rad_s
            > self._command_angular_threshold_rad_s
            and yaw_span_rad == 0.0
        )
        return linear_is_frozen or angular_is_frozen

    def _is_motion_commanded(self):
        return (
            self._command_linear_abs_mps > self._command_linear_threshold_mps
            or self._command_angular_abs_rad_s
            > self._command_angular_threshold_rad_s
        )

    @staticmethod
    def _with_metrics(decision, names, values):
        return HealthDecision(
            decision.state,
            decision.health_score,
            decision.confidence,
            decision.detected_fault,
            decision.reasons,
            names,
            values,
            decision.window_start_sec,
            decision.window_end_sec,
            decision.sample_count,
        )


def seconds_from_stamp(stamp):
    """Convert a ROS ``builtin_interfaces/Time`` message to seconds."""
    return stamp.sec + stamp.nanosec / 1_000_000_000.0


def yaw_from_quaternion(quaternion):
    """Return planar yaw in radians from an Odometry pose quaternion."""
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return atan2(sin_yaw, cos_yaw)


def stamp_from_seconds(seconds):
    """Convert non-negative seconds to a ROS Time message."""
    total_nanoseconds = max(int(seconds * 1_000_000_000), 0)
    stamp = rclpy.time.Time(nanoseconds=total_nanoseconds).to_msg()
    return stamp


def make_sensor_health(sensor, source_topic, stamp, decision):
    """Convert an assessment decision into the existing SensorHealth interface."""
    msg = SensorHealth()
    msg.header.stamp = stamp
    msg.header.frame_id = ''
    msg.sensor = sensor
    msg.source_topic = source_topic
    msg.state = decision.state
    msg.health_score = decision.health_score
    msg.confidence = decision.confidence
    msg.detected_fault = decision.detected_fault
    msg.reasons = decision.reasons
    msg.metric_names = decision.metric_names
    msg.metric_values = decision.metric_values
    msg.window_start = stamp_from_seconds(decision.window_start_sec)
    msg.window_end = stamp_from_seconds(decision.window_end_sec)
    msg.sample_count = decision.sample_count
    return msg


def make_unknown_sensor_health(sensor, source_topic, stamp):
    """Create the intentionally unimplemented scan-health output."""
    decision = HealthDecision(
        SensorHealth.UNKNOWN,
        -1.0,
        0.0,
        'unknown',
        ['detector_not_implemented'],
        [],
        [],
        seconds_from_stamp(stamp),
        seconds_from_stamp(stamp),
        0,
    )
    return make_sensor_health(sensor, source_topic, stamp, decision)


class SensorHealthMonitor(Node):
    """Publish health assessments for faulted IMU, wheel, and scan inputs."""

    def __init__(self):
        super().__init__('sensor_health_monitor')
        self._declare_parameters()
        publish_rate_hz = self._float_parameter('publish_rate_hz')
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than 0.0')
        self._sources = {
            'imu': self._string_parameter('imu_source_topic'),
            'wheel': self._string_parameter('wheel_source_topic'),
            'scan': self._string_parameter('scan_source_topic'),
        }
        self._evaluator = HealthEvaluator(
            self._float_parameter('window_duration_sec'),
            self._integer_parameter('min_samples'),
            self._float_parameter('stale_timeout_sec'),
            self._float_parameter('delay_warning_sec'),
            self._float_parameter('delay_fault_sec'),
            self._integer_parameter('delay_confirmation_cycles'),
            self._float_parameter('command_linear_threshold_mps'),
            self._float_parameter('command_angular_threshold_rad_s'),
            self._float_parameter('freeze_duration_sec'),
            self._float_parameter('wheel_pose_span_threshold_m'),
            self._float_parameter('wheel_linear_span_threshold_mps'),
            self._float_parameter('wheel_angular_span_threshold_rad_s'),
        )
        self._publishers = {
            'imu': self.create_publisher(SensorHealth, '/health/imu', 10),
            'wheel': self.create_publisher(SensorHealth, '/health/wheel', 10),
            'scan': self.create_publisher(SensorHealth, '/health/scan', 10),
        }
        self._create_sensor_subscriptions()
        self.create_subscription(Twist, '/cmd_vel', self._on_command, 10)
        self._timer = self.create_timer(
            1.0 / publish_rate_hz, self._publish_health
        )

    def _create_sensor_subscriptions(self):
        """Subscribe to the sensor inputs with the sensor-data QoS profile."""
        self.create_subscription(
            Imu,
            self._sources['imu'],
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self._sources['wheel'],
            self._on_wheel,
            qos_profile_sensor_data,
        )

    def _declare_parameters(self):
        defaults = {
            'publish_rate_hz': DEFAULT_PUBLISH_RATE_HZ,
            'imu_source_topic': DEFAULT_IMU_SOURCE_TOPIC,
            'wheel_source_topic': DEFAULT_WHEEL_SOURCE_TOPIC,
            'scan_source_topic': DEFAULT_SCAN_SOURCE_TOPIC,
            'window_duration_sec': 2.0,
            'min_samples': 3,
            'stale_timeout_sec': 0.5,
            'delay_warning_sec': 0.3,
            'delay_fault_sec': 0.5,
            'delay_confirmation_cycles': 3,
            'command_linear_threshold_mps': 0.05,
            'command_angular_threshold_rad_s': 0.10,
            'freeze_duration_sec': 1.0,
            'wheel_pose_span_threshold_m': 0.01,
            'wheel_linear_span_threshold_mps': 0.01,
            'wheel_angular_span_threshold_rad_s': 0.02,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _on_imu(self, msg):
        now_sec = self._now_sec()
        self._evaluator.add_imu(now_sec, seconds_from_stamp(msg.header.stamp))

    def _on_wheel(self, msg):
        now_sec = self._now_sec()
        self._evaluator.add_wheel(
            now_sec,
            seconds_from_stamp(msg.header.stamp),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation),
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        )

    def _on_command(self, msg):
        self._evaluator.set_command(
            self._now_sec(), msg.linear.x, msg.angular.z
        )

    def _publish_health(self):
        now_sec = self._now_sec()
        stamp = stamp_from_seconds(now_sec)
        self._publishers['imu'].publish(make_sensor_health(
            'imu', self._sources['imu'], stamp,
            self._evaluator.evaluate_imu(now_sec),
        ))
        self._publishers['wheel'].publish(make_sensor_health(
            'wheel', self._sources['wheel'], stamp,
            self._evaluator.evaluate_wheel(now_sec),
        ))
        self._publishers['scan'].publish(make_unknown_sensor_health(
            'scan', self._sources['scan'], stamp,
        ))

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _float_parameter(self, name):
        return self.get_parameter(name).value

    def _integer_parameter(self, name):
        return self.get_parameter(name).value

    def _string_parameter(self, name):
        return self.get_parameter(name).value


def main(args=None):
    """Run the sensor health monitor node."""
    rclpy.init(args=args)
    node = SensorHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
