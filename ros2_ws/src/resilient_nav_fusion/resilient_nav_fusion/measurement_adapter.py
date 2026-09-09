"""ROS 2 boundary adapter for health-aware anonymous fusion measurements."""

from copy import deepcopy
from math import isfinite

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from resilient_nav_interfaces.msg import (
    FusionStatus,
    LidarOdometryStatus,
    MeasurementReliability as MeasurementReliabilityMsg,
    SensorHealth,
)
from sensor_msgs.msg import Imu

from .fusion_policy import (
    FusionDecision,
    FusionPolicy,
    FusionPolicyConfig,
    FusionState,
    HealthState,
    MeasurementHealth,
    ReliabilityScores,
)
from .physical_reliability_runtime import (
    ImuSample,
    LidarSample,
    OnlineFeatureWindow,
    ReliabilityModelBundle,
    WheelSample,
    lidar_quality_reliability,
)


# PoseWithCovariance and TwistWithCovariance are 6x6 row-major matrices
# ordered as (x, y, z, rotation-X, rotation-Y, rotation-Z).
WHEEL_LINEAR_X_COVARIANCE_INDEX = 0
WHEEL_YAW_POSE_COVARIANCE_INDEX = 35
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
    sensor = str(msg.sensor).strip().lower()
    reasons = tuple(msg.reasons)
    startup_is_provisional = (
        sensor in {'wheel', 'imu'}
        and msg.detected_fault == 'none'
        and reasons == ('startup_provisional_valid',)
    )
    imu_calibration_is_provisional = (
        sensor == 'imu'
        and msg.detected_fault == 'unknown'
        and reasons == ('imu_yaw_rate_bias_baseline_calibrating',)
    )
    if msg.state == SensorHealth.UNKNOWN and (
        startup_is_provisional or imu_calibration_is_provisional
    ):
        return MeasurementHealth(HealthState.PROVISIONAL, confidence)
    return MeasurementHealth(state_map.get(msg.state, HealthState.UNKNOWN), confidence)


def sensor_health_has_observation(msg: SensorHealth) -> bool:
    """Distinguish startup no-data timers from evidence about a measurement."""
    return int(msg.sample_count) > 0


def imu_measurement_health_from_sensor(msg: SensorHealth) -> MeasurementHealth:
    """Keep a fresh IMU usable when only its wheel cross-check is unavailable.

    The health monitor emits this exact UNKNOWN reason only after the IMU's own
    timing checks have passed.  It means bias observability is unavailable, not
    that the yaw-rate sample itself stopped arriving.  Fusion therefore admits
    it conservatively as DEGRADED while continuing to reject every other
    UNKNOWN state and every diagnosed IMU fault.
    """
    health = measurement_health_from_sensor(msg)
    if (
        health.state is HealthState.UNKNOWN
        and str(msg.sensor).strip().lower() == 'imu'
        and msg.detected_fault == 'none'
        and tuple(msg.reasons) == ('wheel_reference_unavailable',)
    ):
        return MeasurementHealth(HealthState.DEGRADED, health.confidence)
    return health


def prepare_wheel_odometry(
    message: Odometry,
    translation_covariance_scale: float | None,
    rotation_covariance_scale: float | None,
) -> Odometry | None:
    """Copy wheel odometry with independent translation/rotation weights."""
    scaled_linear_x = _scaled_covariance(
        message.twist.covariance,
        WHEEL_LINEAR_X_COVARIANCE_INDEX,
        translation_covariance_scale,
    )
    scaled_yaw = _scaled_covariance(
        message.pose.covariance,
        WHEEL_YAW_POSE_COVARIANCE_INDEX,
        rotation_covariance_scale,
    )
    if scaled_linear_x is None or scaled_yaw is None:
        return None
    output = deepcopy(message)
    output.twist.covariance[
        WHEEL_LINEAR_X_COVARIANCE_INDEX
    ] = scaled_linear_x
    output.pose.covariance[WHEEL_YAW_POSE_COVARIANCE_INDEX] = scaled_yaw
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


def prepare_lidar_velocity(
    message: TwistWithCovarianceStamped,
    covariance_scale: float | None,
) -> TwistWithCovarianceStamped | None:
    """Copy accepted LiDAR forward velocity and scale only its variance."""
    scaled = _scaled_covariance(
        message.twist.covariance,
        WHEEL_LINEAR_X_COVARIANCE_INDEX,
        covariance_scale,
    )
    if scaled is None or not isfinite(float(message.twist.twist.linear.x)):
        return None
    output = deepcopy(message)
    output.twist.covariance[WHEEL_LINEAR_X_COVARIANCE_INDEX] = scaled
    return output


def provisional_wheel_health(
    message: Odometry,
    now_sec: float,
    maximum_age_sec: float,
    future_tolerance_sec: float,
) -> MeasurementHealth | None:
    """Admit only a structurally legal first wheel measurement at startup."""
    if not message.header.frame_id or not message.child_frame_id:
        return None
    if not _startup_stamp_is_valid(
        message.header.stamp, now_sec, maximum_age_sec, future_tolerance_sec
    ):
        return None
    pose = message.pose.pose
    twist = message.twist.twist
    values = (
        pose.position.x,
        pose.position.y,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
        twist.linear.x,
        twist.angular.z,
    )
    if not all(isfinite(float(value)) for value in values):
        return None
    quaternion_norm_squared = sum(float(value) ** 2 for value in (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ))
    if quaternion_norm_squared <= 1.0e-12:
        return None
    if prepare_wheel_odometry(message, 1.0, 1.0) is None:
        return None
    return MeasurementHealth(HealthState.PROVISIONAL, confidence=0.5)


def provisional_imu_health(
    message: Imu,
    now_sec: float,
    maximum_age_sec: float,
    future_tolerance_sec: float,
) -> MeasurementHealth | None:
    """Admit only a structurally legal first IMU yaw-rate at startup."""
    if not message.header.frame_id:
        return None
    if not _startup_stamp_is_valid(
        message.header.stamp, now_sec, maximum_age_sec, future_tolerance_sec
    ):
        return None
    if not isfinite(float(message.angular_velocity.z)):
        return None
    if prepare_imu(message, 1.0) is None:
        return None
    return MeasurementHealth(HealthState.PROVISIONAL, confidence=0.5)


def lidar_admission_from_measurement(
    message: TwistWithCovarianceStamped,
    now_sec: float,
    maximum_age_sec: float,
    future_tolerance_sec: float,
) -> MeasurementHealth | None:
    """Admit only a fresh LiDAR velocity already accepted by Robust ICP."""
    if not message.header.frame_id:
        return None
    if not _startup_stamp_is_valid(
        message.header.stamp, now_sec, maximum_age_sec, future_tolerance_sec
    ):
        return None
    if not all(isfinite(float(value)) for value in (
        message.twist.twist.linear.x,
        message.twist.twist.linear.y,
    )):
        return None
    if prepare_lidar_velocity(message, 1.0) is None:
        return None
    return MeasurementHealth(HealthState.HEALTHY, confidence=1.0)


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
            decision.covariance_scales[FusionPolicy.WHEEL_ROTATION],
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


def adapt_lidar_measurement(
    message: TwistWithCovarianceStamped,
    decision: FusionDecision,
) -> TwistWithCovarianceStamped | None:
    """Forward LiDAR translation only while policy selects wheel fallback."""
    if FusionPolicy.LIDAR_VELOCITY not in decision.accepted_measurements:
        return None
    return prepare_lidar_velocity(
        message,
        decision.covariance_scales[FusionPolicy.LIDAR_VELOCITY],
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


def _startup_stamp_is_valid(
    stamp,
    now_sec: float,
    maximum_age_sec: float,
    future_tolerance_sec: float,
) -> bool:
    if (
        not isfinite(now_sec)
        or maximum_age_sec <= 0.0
        or future_tolerance_sec < 0.0
        or stamp.sec < 0
        or not 0 <= stamp.nanosec < 1_000_000_000
    ):
        return False
    stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    age_sec = now_sec - stamp_sec
    return (
        isfinite(stamp_sec)
        and -future_tolerance_sec <= age_sec <= maximum_age_sec
    )


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
                lidar_degraded_covariance_scale=self._float_parameter(
                    'lidar_degraded_covariance_scale'
                ),
                wheel_yaw_fallback_covariance_scale=self._float_parameter(
                    'wheel_yaw_fallback_covariance_scale'
                ),
                recovery_confirmation_cycles=self._integer_parameter(
                    'recovery_confirmation_cycles'
                ),
                nominal_reliability=self._float_parameter(
                    'nominal_reliability'
                ),
                nominal_state_reliability=self._float_parameter(
                    'nominal_state_reliability'
                ),
                reliability_floor=self._float_parameter(
                    'reliability_floor'
                ),
                maximum_covariance_scale=self._float_parameter(
                    'maximum_covariance_scale'
                ),
                fallback_reliability_threshold=self._float_parameter(
                    'fallback_reliability_threshold'
                ),
            )
        )
        self._feature_window = OnlineFeatureWindow(
            window_sec=self._float_parameter('rf_window_sec')
        )
        self._models = ReliabilityModelBundle(
            self._string_parameter('rf_model_directory')
        )
        self._reliability = ReliabilityScores()
        self._last_rf_prediction_sec = None
        self._wheel_health = MeasurementHealth(HealthState.UNKNOWN, 0.0)
        self._imu_health = MeasurementHealth(HealthState.UNKNOWN, 0.0)
        self._lidar_health = MeasurementHealth(HealthState.UNKNOWN, 0.0)
        self._health_observed = {'wheel': False, 'imu': False}
        self._decision = self._policy.decide(
            self._wheel_health,
            self._imu_health,
            self._lidar_health,
            self._reliability,
        )
        self._startup_maximum_age_sec = self._float_parameter(
            'startup_maximum_age_sec'
        )
        self._startup_future_tolerance_sec = self._float_parameter(
            'startup_future_tolerance_sec'
        )
        if (
            self._startup_maximum_age_sec <= 0.0
            or self._startup_future_tolerance_sec < 0.0
        ):
            raise ValueError('startup timing bounds are invalid')

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
        self._lidar_publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            self._string_parameter('lidar_output_topic'),
            10,
        )
        self._status_publisher = self.create_publisher(
            FusionStatus, self._string_parameter('status_topic'), 10
        )
        self._reliability_publisher = self.create_publisher(
            MeasurementReliabilityMsg,
            self._string_parameter('reliability_topic'),
            10,
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
            LidarOdometryStatus,
            self._string_parameter('lidar_input_topic'),
            self._on_lidar,
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
            'lidar_input_topic': '/lidar/odometry/status',
            'wheel_health_topic': '/health/wheel',
            'imu_health_topic': '/health/imu',
            'wheel_output_topic': '/fusion/input/wheel/odometry',
            'imu_output_topic': '/fusion/input/imu/data',
            'wheel_yaw_output_topic': '/fusion/input/wheel/yaw_rate',
            'lidar_output_topic': '/fusion/input/lidar/velocity',
            'status_topic': '/fusion/status',
            'reliability_topic': '/fusion/reliability',
            'rf_model_directory': '',
            'rf_window_sec': 0.4,
            'wheel_degraded_covariance_scale': 4.0,
            'imu_degraded_covariance_scale': 4.0,
            'lidar_degraded_covariance_scale': 4.0,
            'wheel_yaw_fallback_covariance_scale': 8.0,
            'recovery_confirmation_cycles': 1,
            'nominal_reliability': 0.95,
            'nominal_state_reliability': 0.80,
            'reliability_floor': 0.05,
            'maximum_covariance_scale': 100.0,
            'fallback_reliability_threshold': 0.10,
            'lidar_max_rmse_m': 0.08,
            'lidar_min_inlier_ratio': 0.45,
            'lidar_min_observability': 0.05,
            'startup_maximum_age_sec': 0.50,
            'startup_future_tolerance_sec': 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_wheel_health(self, message: SensorHealth):
        self._health_observed['wheel'] |= sensor_health_has_observation(message)
        self._wheel_health = measurement_health_from_sensor(message)
        self._update_decision(message)

    def _on_imu_health(self, message: SensorHealth):
        self._health_observed['imu'] |= sensor_health_has_observation(message)
        self._imu_health = imu_measurement_health_from_sensor(message)
        self._update_decision(message)

    def _update_decision(self, health: SensorHealth):
        self._refresh_decision()
        self._status_publisher.publish(
            fusion_status_from_decision(self._decision, health)
        )

    def _refresh_decision(self):
        self._decision = self._policy.decide(
            self._wheel_health,
            self._imu_health,
            self._lidar_health,
            self._reliability,
        )

    def _on_wheel(self, message: Odometry):
        self._feature_window.add_wheel(WheelSample(
            stamp_sec=_message_stamp_sec(message),
            vx_mps=float(message.twist.twist.linear.x),
            yaw_rate_radps=float(message.twist.twist.angular.z),
            vx_variance=float(
                message.twist.covariance[WHEEL_LINEAR_X_COVARIANCE_INDEX]
            ),
        ))
        if not self._health_observed['wheel']:
            provisional = provisional_wheel_health(
                message,
                self._now_sec(),
                self._startup_maximum_age_sec,
                self._startup_future_tolerance_sec,
            )
            if provisional is not None:
                self._wheel_health = provisional
                self._refresh_decision()
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
        self._feature_window.add_imu(ImuSample(
            stamp_sec=_message_stamp_sec(message),
            yaw_rate_radps=float(message.angular_velocity.z),
            accel_x_mps2=float(message.linear_acceleration.x),
            accel_y_mps2=float(message.linear_acceleration.y),
            gyro_x_radps=float(message.angular_velocity.x),
            gyro_y_radps=float(message.angular_velocity.y),
        ))
        if not self._health_observed['imu']:
            provisional = provisional_imu_health(
                message,
                self._now_sec(),
                self._startup_maximum_age_sec,
                self._startup_future_tolerance_sec,
            )
            if provisional is not None:
                self._imu_health = provisional
                self._refresh_decision()
        output = adapt_imu_measurement(message, self._decision)
        if output is not None:
            self._imu_publisher.publish(output)
        elif FusionPolicy.IMU_YAW_RATE in self._decision.accepted_measurements:
            self.get_logger().warning('suppressed IMU measurement: invalid covariance')

    def _on_lidar(self, message: LidarOdometryStatus):
        velocity = TwistWithCovarianceStamped()
        velocity.header = deepcopy(message.header)
        velocity.twist = deepcopy(message.velocity)
        lidar_sample = LidarSample(
            stamp_sec=_message_stamp_sec(message),
            valid=bool(message.valid),
            vx_mps=float(message.velocity.twist.linear.x),
            yaw_rate_radps=float(message.velocity.twist.angular.z),
            vx_variance=float(
                message.velocity.covariance[WHEEL_LINEAR_X_COVARIANCE_INDEX]
            ),
            rmse_m=float(message.icp_rmse_m),
            inlier_ratio=float(message.icp_inlier_ratio),
            observability=float(message.icp_observability),
        )
        self._feature_window.add_lidar(lidar_sample)
        lidar_reliability = lidar_quality_reliability(
            lidar_sample,
            max_rmse_m=self._float_parameter('lidar_max_rmse_m'),
            min_inlier_ratio=self._float_parameter('lidar_min_inlier_ratio'),
            min_observability=self._float_parameter(
                'lidar_min_observability'
            ),
        )
        features = self._feature_window.feature_vector(lidar_sample.stamp_sec)
        if features is None:
            prediction_is_recent = (
                self._last_rf_prediction_sec is not None
                and 0.0
                <= lidar_sample.stamp_sec - self._last_rf_prediction_sec
                <= self._feature_window.window_sec
            )
            if prediction_is_recent:
                self._reliability = ReliabilityScores(
                    wheel_translation=self._reliability.wheel_translation,
                    wheel_rotation=self._reliability.wheel_rotation,
                    imu_yaw_rate=self._reliability.imu_yaw_rate,
                    lidar_translation=lidar_reliability,
                    rf_ready=True,
                )
            else:
                self._reliability = ReliabilityScores(
                    lidar_translation=lidar_reliability
                )
        else:
            self._reliability = self._models.predict(
                features, lidar_reliability
            )
            self._last_rf_prediction_sec = lidar_sample.stamp_sec
        self._publish_reliability(message)
        if not message.valid:
            self._lidar_health = MeasurementHealth(HealthState.UNKNOWN, 0.0)
            self._refresh_decision()
            return
        admission = lidar_admission_from_measurement(
            velocity,
            self._now_sec(),
            self._startup_maximum_age_sec,
            self._startup_future_tolerance_sec,
        )
        if admission is None:
            return
        self._lidar_health = admission
        self._refresh_decision()
        output = adapt_lidar_measurement(velocity, self._decision)
        if output is not None:
            self._lidar_publisher.publish(output)
        elif FusionPolicy.LIDAR_VELOCITY in self._decision.accepted_measurements:
            self.get_logger().warning(
                'suppressed LiDAR velocity: invalid covariance'
            )

    def _publish_reliability(self, source: LidarOdometryStatus) -> None:
        message = MeasurementReliabilityMsg()
        message.header = deepcopy(source.header)
        message.rf_ready = self._reliability.rf_ready
        message.wheel_translation = self._reliability.wheel_translation
        message.wheel_rotation = self._reliability.wheel_rotation
        message.imu_yaw_rate = self._reliability.imu_yaw_rate
        message.lidar_translation = self._reliability.lidar_translation
        message.reasons = [
            'rf_predict_proba_continuous'
            if self._reliability.rf_ready
            else 'rf_window_warming_neutral_weight',
            'lidar_icp_quality_observability_gate',
        ]
        self._reliability_publisher.publish(message)

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _integer_parameter(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0


def _message_stamp_sec(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


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
