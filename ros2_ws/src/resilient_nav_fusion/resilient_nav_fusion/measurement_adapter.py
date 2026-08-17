"""ROS 2 boundary adapter for health-aware anonymous fusion measurements."""

from copy import deepcopy
from math import isfinite

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from resilient_nav_interfaces.msg import FusionStatus, SensorHealth
from sensor_msgs.msg import Imu

from .fusion_policy import (
    FusionDecision,
    FusionPolicy,
    FusionPolicyConfig,
    FusionState,
    HealthState,
    MeasurementHealth,
)


# TwistWithCovariance is a 6x6 row-major matrix ordered as
# (x, y, z, rotation-X, rotation-Y, rotation-Z).
WHEEL_LINEAR_X_COVARIANCE_INDEX = 0
WHEEL_YAW_RATE_COVARIANCE_INDEX = 35
# Imu.angular_velocity_covariance is a 3x3 row-major matrix ordered x, y, z.
IMU_ANGULAR_Z_COVARIANCE_INDEX = 8


def measurement_health_from_sensor(msg: SensorHealth) -> MeasurementHealth:
    """Create an allowlisted policy input without retaining source metadata."""
    state_map = {
        SensorHealth.UNKNOWN: HealthState.UNKNOWN,
        SensorHealth.HEALTHY: HealthState.HEALTHY,
        SensorHealth.DEGRADED: HealthState.DEGRADED,
        SensorHealth.FAULT: HealthState.FAULT,
    }
    confidence = float(msg.confidence)
    if not isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        return MeasurementHealth(HealthState.UNKNOWN, confidence=0.0)
    return MeasurementHealth(state_map.get(msg.state, HealthState.UNKNOWN), confidence)


def prepare_wheel_odometry(
    message: Odometry,
    covariance_scale: float | None,
) -> Odometry | None:
    """Deep-copy wheel odometry and inflate only linear-x covariance safely."""
    scaled = _scaled_covariance(
        message.twist.covariance,
        WHEEL_LINEAR_X_COVARIANCE_INDEX,
        covariance_scale,
    )
    if scaled is None:
        return None
    output = deepcopy(message)
    output.twist.covariance[WHEEL_LINEAR_X_COVARIANCE_INDEX] = scaled
    return output


def prepare_imu(
    message: Imu,
    covariance_scale: float | None,
) -> Imu | None:
    """Deep-copy IMU data and inflate only angular-velocity-z covariance."""
    if not _has_known_imu_angular_velocity_covariance(
        message.angular_velocity_covariance
    ):
        return None
    scaled = _scaled_covariance(
        message.angular_velocity_covariance,
        IMU_ANGULAR_Z_COVARIANCE_INDEX,
        covariance_scale,
    )
    if scaled is None:
        return None
    output = deepcopy(message)
    output.angular_velocity_covariance[IMU_ANGULAR_Z_COVARIANCE_INDEX] = scaled
    return output


def prepare_wheel_yaw_rate(
    message: Odometry,
    covariance_scale: float | None,
) -> TwistWithCovarianceStamped | None:
    """Build an isolated wheel yaw-rate measurement in the twist frame."""
    if not message.child_frame_id:
        return None
    scaled = _scaled_covariance(
        message.twist.covariance,
        WHEEL_YAW_RATE_COVARIANCE_INDEX,
        covariance_scale,
    )
    if scaled is None:
        return None
    output = TwistWithCovarianceStamped()
    output.header = deepcopy(message.header)
    # Odometry documents its twist in child_frame_id, not header.frame_id.
    output.header.frame_id = message.child_frame_id
    output.twist = deepcopy(message.twist)
    output.twist.covariance[WHEEL_YAW_RATE_COVARIANCE_INDEX] = scaled
    return output


def adapt_wheel_measurements(
    message: Odometry,
    decision: FusionDecision,
) -> tuple[Odometry | None, TwistWithCovarianceStamped | None]:
    """Return allowed primary and fallback wheel outputs, suppressing failures."""
    wheel = None
    fallback = None
    if FusionPolicy.WHEEL_VELOCITY in decision.accepted_measurements:
        wheel = prepare_wheel_odometry(
            message,
            decision.covariance_scales[FusionPolicy.WHEEL_VELOCITY],
        )
    if decision.wheel_yaw_fallback_enabled:
        fallback = prepare_wheel_yaw_rate(
            message,
            decision.covariance_scales[FusionPolicy.WHEEL_YAW_RATE],
        )
    return wheel, fallback


def adapt_imu_measurement(
    message: Imu,
    decision: FusionDecision,
) -> Imu | None:
    """Return the allowed IMU output or suppress it without mutation."""
    if FusionPolicy.IMU_YAW_RATE not in decision.accepted_measurements:
        return None
    return prepare_imu(
        message,
        decision.covariance_scales[FusionPolicy.IMU_YAW_RATE],
    )


def fusion_status_from_decision(
    decision: FusionDecision,
    health: SensorHealth,
) -> FusionStatus:
    """Convert a pure decision into the matching ROS runtime-status message."""
    status = FusionStatus()
    status.header = deepcopy(health.header)
    status.chain_id = 'adaptive'
    status.state = {
        FusionState.UNKNOWN: FusionStatus.UNKNOWN,
        FusionState.NOMINAL: FusionStatus.NOMINAL,
        FusionState.DEGRADED: FusionStatus.DEGRADED,
        FusionState.HOLD: FusionStatus.HOLD,
    }[decision.state]
    status.accepted_measurements = list(decision.accepted_measurements)
    status.rejected_measurements = list(decision.rejected_measurements)
    status.reasons = list(decision.reasons)
    status.confidence = decision.confidence
    status.window_start = deepcopy(health.window_start)
    status.window_end = deepcopy(health.window_end)
    status.sample_count = health.sample_count
    return status


def _scaled_covariance(
    covariance: object,
    index: int,
    scale: float | None,
) -> float | None:
    """Return a finite non-negative scaled variance, otherwise fail closed."""
    if scale is None or not isfinite(scale) or scale < 1.0:
        return None
    try:
        value = float(covariance[index])
    except (IndexError, TypeError, ValueError):
        return None
    if not isfinite(value) or value < 0.0:
        return None
    scaled = value * scale
    return scaled if isfinite(scaled) else None


def _has_known_imu_angular_velocity_covariance(covariance: object) -> bool:
    """Apply the sentinel rules documented by sensor_msgs/msg/Imu."""
    try:
        values = [float(covariance[index]) for index in range(9)]
    except (IndexError, TypeError, ValueError):
        return False
    if not all(isfinite(value) for value in values):
        return False
    # Per Imu.msg, index 0 == -1 means angular velocity is unavailable and
    # an all-zero matrix means its covariance is unknown.
    return values[0] != -1.0 and any(value != 0.0 for value in values)


class MeasurementAdapter(Node):
    """Publish only policy-accepted anonymous measurements and fusion status."""

    def __init__(self):
        super().__init__('measurement_adapter')
        self._declare_parameters()
        self._policy = FusionPolicy(
            FusionPolicyConfig(
                wheel_degraded_covariance_scale=self._float_parameter(
                    'wheel_degraded_covariance_scale'
                ),
                imu_degraded_covariance_scale=self._float_parameter(
                    'imu_degraded_covariance_scale'
                ),
                wheel_yaw_fallback_covariance_scale=self._float_parameter(
                    'wheel_yaw_fallback_covariance_scale'
                ),
                recovery_confirmation_cycles=self._integer_parameter(
                    'recovery_confirmation_cycles'
                ),
            )
        )
        self._wheel_health = MeasurementHealth(HealthState.UNKNOWN, 0.0)
        self._imu_health = MeasurementHealth(HealthState.UNKNOWN, 0.0)
        self._decision = self._policy.decide(
            self._wheel_health,
            self._imu_health,
        )

        self._wheel_publisher = self.create_publisher(
            Odometry, self._string_parameter('wheel_output_topic'), 10
        )
        self._imu_publisher = self.create_publisher(
            Imu, self._string_parameter('imu_output_topic'), 10
        )
        self._fallback_publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            self._string_parameter('wheel_yaw_output_topic'),
            10,
        )
        self._status_publisher = self.create_publisher(
            FusionStatus, self._string_parameter('status_topic'), 10
        )
        self.create_subscription(
            Odometry,
            self._string_parameter('wheel_input_topic'),
            self._on_wheel,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            self._string_parameter('imu_input_topic'),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SensorHealth,
            self._string_parameter('wheel_health_topic'),
            self._on_wheel_health,
            10,
        )
        self.create_subscription(
            SensorHealth,
            self._string_parameter('imu_health_topic'),
            self._on_imu_health,
            10,
        )

    def _declare_parameters(self):
        defaults = {
            'wheel_input_topic': '/faulted/wheel/odometry',
            'imu_input_topic': '/faulted/imu/data',
            'wheel_health_topic': '/health/wheel',
            'imu_health_topic': '/health/imu',
            'wheel_output_topic': '/fusion/input/wheel/odometry',
            'imu_output_topic': '/fusion/input/imu/data',
            'wheel_yaw_output_topic': '/fusion/input/wheel/yaw_rate',
            'status_topic': '/fusion/status',
            'wheel_degraded_covariance_scale': 4.0,
            'imu_degraded_covariance_scale': 4.0,
            'wheel_yaw_fallback_covariance_scale': 8.0,
            'recovery_confirmation_cycles': 2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_wheel_health(self, message: SensorHealth):
        self._wheel_health = measurement_health_from_sensor(message)
        self._update_decision(message)

    def _on_imu_health(self, message: SensorHealth):
        self._imu_health = measurement_health_from_sensor(message)
        self._update_decision(message)

    def _update_decision(self, health: SensorHealth):
        self._decision = self._policy.decide(self._wheel_health, self._imu_health)
        self._status_publisher.publish(
            fusion_status_from_decision(self._decision, health)
        )

    def _on_wheel(self, message: Odometry):
        wheel, fallback = adapt_wheel_measurements(message, self._decision)
        if wheel is not None:
            self._wheel_publisher.publish(wheel)
        elif FusionPolicy.WHEEL_VELOCITY in self._decision.accepted_measurements:
            self.get_logger().warning('suppressed wheel measurement: invalid covariance')
        if fallback is not None:
            self._fallback_publisher.publish(fallback)
        elif self._decision.wheel_yaw_fallback_enabled:
            self.get_logger().warning('suppressed wheel yaw fallback: invalid covariance')

    def _on_imu(self, message: Imu):
        output = adapt_imu_measurement(message, self._decision)
        if output is not None:
            self._imu_publisher.publish(output)
        elif FusionPolicy.IMU_YAW_RATE in self._decision.accepted_measurements:
            self.get_logger().warning('suppressed IMU measurement: invalid covariance')

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _integer_parameter(self, name: str) -> int:
        return int(self.get_parameter(name).value)


def main(args=None):
    """Run the measurement adapter node when explicitly launched."""
    rclpy.init(args=args)
    node = MeasurementAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
