"""Assess IMU and wheel-odometry delivery and wheel-freeze health."""

from collections import deque
from dataclasses import dataclass
from math import atan2, hypot, isfinite, isinf, isnan, pi, sqrt

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from resilient_nav_interfaces.msg import SensorHealth
from sensor_msgs.msg import Imu, LaserScan


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
    ranges: tuple = ()
    angle_increment_rad: float = 0.0


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
        imu_wheel_max_pairing_time_diff_sec=0.05,
        imu_bias_min_pairs=10,
        imu_bias_warning_threshold_rad_s=0.08,
        imu_bias_fault_threshold_rad_s=0.12,
        imu_bias_max_stddev_rad_s=0.03,
        imu_bias_max_mad_rad_s=0.02,
        imu_bias_confirmation_cycles=3,
        imu_bias_baseline_calibration_min_pairs=10,
        scan_min_samples=3,
        scan_nan_warning_ratio=0.03,
        scan_nan_fault_ratio=0.08,
        scan_sector_warning_width_rad=0.50,
        scan_sector_fault_width_rad=0.90,
        scan_confirmation_cycles=3,
        scan_recovery_cycles=3,
        freeze_warning_duration_sec=0.40,
        wheel_yaw_span_threshold_rad=0.01,
        future_stamp_tolerance_sec=0.05,
    ):
        self._min_samples = min_samples
        self._stale_timeout_sec = stale_timeout_sec
        self._delay_warning_sec = delay_warning_sec
        self._delay_fault_sec = delay_fault_sec
        if delay_confirmation_cycles < 1:
            raise ValueError('delay_confirmation_cycles must be at least 1')
        self._delay_confirmation_cycles = delay_confirmation_cycles
        self._delay_warning_counts = {'imu': 0, 'wheel': 0, 'scan': 0}
        self._delay_fault_counts = {'imu': 0, 'wheel': 0, 'scan': 0}
        self._command_linear_threshold_mps = command_linear_threshold_mps
        self._command_angular_threshold_rad_s = command_angular_threshold_rad_s
        self._freeze_duration_sec = freeze_duration_sec
        self._freeze_warning_duration_sec = freeze_warning_duration_sec
        self._wheel_pose_span_threshold_m = wheel_pose_span_threshold_m
        self._wheel_yaw_span_threshold_rad = wheel_yaw_span_threshold_rad
        self._wheel_linear_span_threshold_mps = wheel_linear_span_threshold_mps
        self._wheel_angular_span_threshold_rad_s = (
            wheel_angular_span_threshold_rad_s
        )
        self._future_stamp_tolerance_sec = future_stamp_tolerance_sec
        if not 0.0 < freeze_warning_duration_sec < freeze_duration_sec:
            raise ValueError(
                'freeze warning duration must be positive and shorter than '
                'freeze duration'
            )
        if wheel_yaw_span_threshold_rad <= 0.0:
            raise ValueError('wheel yaw span threshold must be positive')
        if future_stamp_tolerance_sec < 0.0:
            raise ValueError('future stamp tolerance must be non-negative')
        if imu_wheel_max_pairing_time_diff_sec < 0.0:
            raise ValueError('imu_wheel_max_pairing_time_diff_sec must be non-negative')
        if imu_bias_min_pairs < 1:
            raise ValueError('imu_bias_min_pairs must be at least 1')
        if imu_bias_confirmation_cycles < 1:
            raise ValueError('imu_bias_confirmation_cycles must be at least 1')
        if imu_bias_baseline_calibration_min_pairs < imu_bias_min_pairs:
            raise ValueError(
                'imu_bias_baseline_calibration_min_pairs must be at least '
                'imu_bias_min_pairs'
            )
        if scan_min_samples < 1:
            raise ValueError('scan_min_samples must be at least 1')
        if not 0.0 <= scan_nan_warning_ratio <= scan_nan_fault_ratio <= 1.0:
            raise ValueError('scan NaN ratios must be ordered within [0.0, 1.0]')
        if (
            scan_sector_warning_width_rad < 0.0
            or scan_sector_fault_width_rad < scan_sector_warning_width_rad
        ):
            raise ValueError('scan sector widths must be non-negative and ordered')
        if scan_confirmation_cycles < 1 or scan_recovery_cycles < 1:
            raise ValueError('scan confirmation and recovery cycles must be at least 1')
        if imu_bias_warning_threshold_rad_s <= 0.0:
            raise ValueError('imu_bias_warning_threshold_rad_s must be greater than 0.0')
        if imu_bias_fault_threshold_rad_s < imu_bias_warning_threshold_rad_s:
            raise ValueError(
                'imu_bias_fault_threshold_rad_s must be at least the warning threshold'
            )
        self._imu_wheel_max_pairing_time_diff_sec = (
            imu_wheel_max_pairing_time_diff_sec
        )
        self._imu_bias_min_pairs = imu_bias_min_pairs
        self._imu_bias_warning_threshold_rad_s = imu_bias_warning_threshold_rad_s
        self._imu_bias_fault_threshold_rad_s = imu_bias_fault_threshold_rad_s
        self._imu_bias_max_stddev_rad_s = imu_bias_max_stddev_rad_s
        self._imu_bias_max_mad_rad_s = imu_bias_max_mad_rad_s
        self._imu_bias_confirmation_cycles = imu_bias_confirmation_cycles
        self._imu_bias_baseline_calibration_min_pairs = (
            imu_bias_baseline_calibration_min_pairs
        )
        self._bias_warning_count = 0
        self._bias_fault_count = 0
        self._imu_bias_baseline_residual = None
        self._imu_bias_baseline_samples = []
        self._imu_bias_baseline_pair_keys = set()
        self._scan_min_samples = scan_min_samples
        self._scan_nan_warning_ratio = scan_nan_warning_ratio
        self._scan_nan_fault_ratio = scan_nan_fault_ratio
        self._scan_sector_warning_width_rad = scan_sector_warning_width_rad
        self._scan_sector_fault_width_rad = scan_sector_fault_width_rad
        self._scan_confirmation_cycles = scan_confirmation_cycles
        self._scan_recovery_cycles = scan_recovery_cycles
        self._scan_warning_count = 0
        self._scan_fault_count = 0
        self._scan_recovery_count = 0
        self._scan_sector_fault_active = False
        self.imu_window = SensorWindow(window_duration_sec)
        self.wheel_window = SensorWindow(window_duration_sec)
        self.scan_window = SensorWindow(window_duration_sec)
        self._latest_scan_observation = None
        self._command_linear_abs_mps = 0.0
        self._command_angular_abs_rad_s = 0.0
        self._motion_command_started_sec = None

    def add_imu(self, received_sec, stamp_sec, angular_z=0.0):
        """Record a received IMU sample."""
        self.imu_window.add(
            Observation(received_sec, stamp_sec, angular_z=angular_z)
        )

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

    def add_scan(self, received_sec, stamp_sec, ranges, angle_increment_rad):
        """Record the latest LaserScan beams and timing metadata."""
        observation = Observation(
            received_sec,
            stamp_sec,
            ranges=tuple(ranges),
            angle_increment_rad=angle_increment_rad,
        )
        self.scan_window.add(observation)
        self._latest_scan_observation = observation

    def set_command(self, received_sec, linear_x, angular_z):
        """Update the most recent command and its continuous active interval."""
        self._command_linear_abs_mps = abs(linear_x)
        self._command_angular_abs_rad_s = abs(angular_z)
        if self._is_motion_commanded():
            if self._motion_command_started_sec is None:
                self._motion_command_started_sec = received_sec
        else:
            self._motion_command_started_sec = None

    def evaluate_imu(self, now_sec, wheel_decision=None):
        """
        Assess IMU timing, then its yaw-rate residual against wheel odometry.

        ``wheel_decision`` is supplied by the node's timer so the stateful wheel
        timing and freeze confirmation checks are evaluated only once per timer
        cycle.  It remains optional for direct unit-test use.
        """
        imu_samples = self.imu_window.samples(now_sec)
        wheel_samples = self.wheel_window.samples(now_sec)
        timing_decision = self._timing_decision('imu', imu_samples, now_sec)
        metrics = self._imu_wheel_residual_metrics(imu_samples, wheel_samples)
        names = list(timing_decision.metric_names) + metrics[0]
        values = list(timing_decision.metric_values) + metrics[1]

        # Transport problems always take precedence over cross-sensor residuals.
        if timing_decision.state != SensorHealth.HEALTHY:
            self._reset_bias_counts()
            return self._with_metrics(timing_decision, names, values)

        if wheel_decision is None:
            # Direct callers that only request timing retain the original IMU
            # timing assessment.  The node always supplies its single wheel
            # decision, which enables the cross-sensor detector.
            return self._with_metrics(timing_decision, names, values)
        reference_reason = self._wheel_reference_reason(
            wheel_decision, len(wheel_samples), metrics[2]
        )
        if reference_reason is not None:
            self._reset_bias_counts()
            return HealthDecision(
                SensorHealth.UNKNOWN,
                -1.0,
                min(timing_decision.confidence, metrics[2] / self._imu_bias_min_pairs),
                'none',
                [reference_reason],
                names,
                values,
                timing_decision.window_start_sec,
                timing_decision.window_end_sec,
                timing_decision.sample_count,
            )

        residual_mean, residual_median, residual_stddev, residual_mad = metrics[3:7]
        if self._imu_bias_baseline_residual is None:
            self._collect_imu_bias_baseline(metrics[7])
        baseline = self._imu_bias_baseline_residual
        corrected_mean = residual_mean - (baseline or 0.0)
        corrected_median = residual_median - (baseline or 0.0)
        names.extend([
            'imu_wheel_nominal_residual_baseline_rad_s',
            'imu_wheel_corrected_residual_mean_rad_s',
            'imu_wheel_corrected_residual_median_rad_s',
            'imu_wheel_baseline_calibration_pair_count',
            'imu_wheel_baseline_calibrated',
        ])
        values.extend([
            baseline or 0.0,
            corrected_mean,
            corrected_median,
            float(len(self._imu_bias_baseline_samples)),
            float(baseline is not None),
        ])
        if baseline is None:
            self._reset_bias_counts()
            return HealthDecision(
                SensorHealth.UNKNOWN,
                -1.0,
                min(
                    timing_decision.confidence,
                    len(self._imu_bias_baseline_samples)
                    / self._imu_bias_baseline_calibration_min_pairs,
                ),
                'unknown',
                ['imu_yaw_rate_bias_baseline_calibrating'],
                names,
                values,
                timing_decision.window_start_sec,
                timing_decision.window_end_sec,
                timing_decision.sample_count,
            )
        bias_level = self._bias_level(
            corrected_mean, corrected_median, residual_stddev, residual_mad
        )
        self._update_bias_counts(bias_level)
        if self._bias_fault_count >= self._imu_bias_confirmation_cycles:
            return HealthDecision(
                SensorHealth.FAULT,
                0.0,
                timing_decision.confidence,
                'bias',
                ['imu_yaw_rate_residual_exceeded_bias_fault_threshold'],
                names,
                values,
                timing_decision.window_start_sec,
                timing_decision.window_end_sec,
                timing_decision.sample_count,
            )
        if self._bias_warning_count >= self._imu_bias_confirmation_cycles:
            return HealthDecision(
                SensorHealth.DEGRADED,
                0.5,
                timing_decision.confidence,
                'bias',
                ['imu_yaw_rate_residual_exceeded_bias_warning_threshold'],
                names,
                values,
                timing_decision.window_start_sec,
                timing_decision.window_end_sec,
                timing_decision.sample_count,
            )
        if bias_level in ('warning', 'fault'):
            return HealthDecision(
                SensorHealth.DEGRADED,
                0.5,
                timing_decision.confidence,
                'bias',
                ['imu_yaw_rate_bias_confirmation_pending'],
                names,
                values,
                timing_decision.window_start_sec,
                timing_decision.window_end_sec,
                timing_decision.sample_count,
            )
        return self._with_metrics(timing_decision, names, values)

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

        freeze_level = self._wheel_freeze_level(now_sec, samples)
        if freeze_level == 'fault':
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
        if freeze_level == 'warning':
            return HealthDecision(
                SensorHealth.DEGRADED,
                0.5,
                decision.confidence,
                'freeze',
                ['short_horizon_wheel_progress_missing'],
                names,
                values,
                decision.window_start_sec,
                decision.window_end_sec,
                decision.sample_count,
            )

        return self._with_metrics(decision, names, values)

    def evaluate_scan(self, now_sec):
        """Assess LaserScan timing and persistent circular NaN sectors."""
        samples = self.scan_window.samples(now_sec)
        decision = self._timing_decision(
            'scan', samples, now_sec, self._scan_min_samples
        )
        names = list(decision.metric_names)
        values = list(decision.metric_values)
        scan_metrics = self._scan_metrics(samples[-1]) if samples else None
        if scan_metrics is not None:
            names.extend(scan_metrics[0])
            values.extend(scan_metrics[1])

        if decision.state != SensorHealth.HEALTHY:
            self._reset_scan_confirmation()
            return self._with_metrics(decision, names, values)

        sector_level = self._scan_sector_level(scan_metrics)
        if self._scan_sector_fault_active:
            if sector_level == 'none':
                self._scan_recovery_count += 1
                if self._scan_recovery_count >= self._scan_recovery_cycles:
                    self._scan_sector_fault_active = False
                    self._reset_scan_confirmation()
                    return self._with_metrics(decision, names, values)
                return HealthDecision(
                    SensorHealth.FAULT,
                    0.0,
                    decision.confidence,
                    'sector_blindness',
                    ['scan_sector_blindness_recovery_pending'],
                    names,
                    values,
                    decision.window_start_sec,
                    decision.window_end_sec,
                    decision.sample_count,
                )
            self._scan_recovery_count = 0
            return HealthDecision(
                SensorHealth.FAULT,
                0.0,
                decision.confidence,
                'sector_blindness',
                ['long_contiguous_nan_sector_exceeded_fault_threshold'],
                names,
                values,
                decision.window_start_sec,
                decision.window_end_sec,
                decision.sample_count,
            )

        if sector_level == 'fault':
            self._scan_fault_count += 1
            self._scan_warning_count += 1
        elif sector_level == 'warning':
            self._scan_fault_count = 0
            self._scan_warning_count += 1
        else:
            self._reset_scan_confirmation()
            return self._with_metrics(decision, names, values)

        if self._scan_fault_count >= self._scan_confirmation_cycles:
            self._scan_sector_fault_active = True
            self._scan_recovery_count = 0
            return HealthDecision(
                SensorHealth.FAULT,
                0.0,
                decision.confidence,
                'sector_blindness',
                ['long_contiguous_nan_sector_exceeded_fault_threshold'],
                names,
                values,
                decision.window_start_sec,
                decision.window_end_sec,
                decision.sample_count,
            )
        if self._scan_warning_count >= self._scan_confirmation_cycles:
            return HealthDecision(
                SensorHealth.DEGRADED,
                0.5,
                decision.confidence,
                'sector_blindness',
                ['long_contiguous_nan_sector_exceeded_warning_threshold'],
                names,
                values,
                decision.window_start_sec,
                decision.window_end_sec,
                decision.sample_count,
            )
        return self._with_metrics(decision, names, values)

    def _timing_decision(self, sensor, samples, now_sec, min_samples=None):
        min_samples = min_samples or self._min_samples
        if not samples:
            self._reset_delay_counts(sensor)
            return self._unknown_decision(samples, min_samples)

        last = samples[-1]
        message_age_sec = now_sec - last.received_sec
        stamp_age_sec = now_sec - last.stamp_sec
        interarrival_sec = (
            last.received_sec - samples[-2].received_sec
            if len(samples) >= 2 else 0.0
        )
        names = [
            'message_age_sec',
            'stamp_age_sec',
            'interarrival_sec',
        ]
        values = [message_age_sec, stamp_age_sec, interarrival_sec]
        confidence = min(len(samples) / min_samples, 1.0)
        window_start_sec = samples[0].received_sec
        window_end_sec = last.received_sec

        invalid_reason = self._observation_invalid_reason(sensor, last, now_sec)
        if invalid_reason is not None:
            self._reset_delay_counts(sensor)
            return HealthDecision(
                SensorHealth.FAULT,
                0.0,
                confidence,
                'invalid',
                [invalid_reason],
                names,
                values,
                window_start_sec,
                window_end_sec,
                len(samples),
            )

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
        if stamp_age_sec > self._delay_warning_sec:
            return HealthDecision(
                SensorHealth.DEGRADED,
                0.5,
                confidence,
                'delay',
                ['header_delay_confirmation_pending'],
                names,
                values,
                window_start_sec,
                window_end_sec,
                len(samples),
            )
        if len(samples) < min_samples:
            return HealthDecision(
                SensorHealth.UNKNOWN,
                -1.0,
                confidence,
                'none',
                ['startup_provisional_valid'],
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

    def _observation_invalid_reason(self, sensor, observation, now_sec):
        """Reject malformed values before provisional startup acceptance."""
        timing_values = (
            now_sec,
            observation.received_sec,
            observation.stamp_sec,
        )
        if not all(isfinite(value) for value in timing_values):
            return 'non_finite_message_timing'
        if observation.received_sec < 0.0 or observation.stamp_sec < 0.0:
            return 'negative_message_timing'
        if observation.received_sec > now_sec + self._future_stamp_tolerance_sec:
            return 'receive_time_is_in_the_future'
        if observation.stamp_sec > now_sec + self._future_stamp_tolerance_sec:
            return 'header_stamp_is_in_the_future'
        if sensor == 'imu' and not isfinite(observation.angular_z):
            return 'non_finite_imu_yaw_rate'
        if sensor == 'wheel':
            wheel_values = (
                observation.pose_x,
                observation.pose_y,
                observation.yaw_rad,
                observation.linear_x,
                observation.angular_z,
            )
            if not all(isfinite(value) for value in wheel_values):
                return 'non_finite_wheel_measurement'
        return None

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

    def _scan_metrics(self, observation):
        """Summarize invalid beams and the largest circular NaN sector."""
        ranges = observation.ranges
        total_beams = len(ranges)
        nan_flags = [isnan(value) for value in ranges]
        nan_count = sum(nan_flags)
        finite_count = sum(isfinite(value) for value in ranges)
        inf_count = sum(isinf(value) for value in ranges)
        longest_nan_beams = self._longest_circular_nan_run(nan_flags)
        longest_nan_width_rad = (
            longest_nan_beams * abs(observation.angle_increment_rad)
        )
        return (
            [
                'scan_total_beams',
                'scan_nan_count',
                'scan_nan_ratio',
                'scan_finite_count',
                'scan_inf_count',
                'scan_longest_nan_sector_beams',
                'scan_longest_nan_sector_width_rad',
            ],
            [
                float(total_beams),
                float(nan_count),
                nan_count / total_beams if total_beams else 0.0,
                float(finite_count),
                float(inf_count),
                float(longest_nan_beams),
                longest_nan_width_rad,
            ],
        )

    @staticmethod
    def _longest_circular_nan_run(nan_flags):
        """Return the longest NaN run while treating scan ends as adjacent."""
        total_beams = len(nan_flags)
        if not total_beams or not any(nan_flags):
            return 0
        if all(nan_flags):
            return total_beams
        first_finite = nan_flags.index(False)
        longest_run = 0
        current_run = 0
        for offset in range(1, total_beams + 1):
            if nan_flags[(first_finite + offset) % total_beams]:
                current_run += 1
                longest_run = max(longest_run, current_run)
            else:
                current_run = 0
        return longest_run

    def _scan_sector_level(self, scan_metrics):
        """Classify a NaN sector using both coverage and angular extent."""
        metrics = dict(zip(scan_metrics[0], scan_metrics[1]))
        nan_ratio = metrics['scan_nan_ratio']
        sector_width_rad = metrics['scan_longest_nan_sector_width_rad']
        if (
            nan_ratio >= self._scan_nan_fault_ratio
            and sector_width_rad >= self._scan_sector_fault_width_rad
        ):
            return 'fault'
        if (
            nan_ratio >= self._scan_nan_warning_ratio
            and sector_width_rad >= self._scan_sector_warning_width_rad
        ):
            return 'warning'
        return 'none'

    def _reset_scan_confirmation(self):
        """Clear non-latched scan-sector confirmation and recovery counters."""
        self._scan_warning_count = 0
        self._scan_fault_count = 0
        self._scan_recovery_count = 0

    def _imu_wheel_residual_metrics(self, imu_samples, wheel_samples):
        """Pair by header stamp and calculate robust IMU-wheel yaw-rate metrics."""
        paired_residuals = []
        time_differences = []
        # Each observation is eligible for one residual only.  Reusing the
        # nearest wheel observation for several IMU observations would inflate
        # pair_count when the topic rates differ or an input is duplicated.
        available_wheel_samples = sorted(
            wheel_samples, key=lambda sample: sample.stamp_sec
        )
        for imu_sample in sorted(imu_samples, key=lambda sample: sample.stamp_sec):
            if not available_wheel_samples:
                break
            wheel_index, wheel_sample = min(
                enumerate(available_wheel_samples),
                key=lambda item: abs(item[1].stamp_sec - imu_sample.stamp_sec),
            )
            time_difference_sec = abs(
                wheel_sample.stamp_sec - imu_sample.stamp_sec
            )
            if time_difference_sec <= self._imu_wheel_max_pairing_time_diff_sec:
                paired_residuals.append((
                    imu_sample.stamp_sec,
                    wheel_sample.stamp_sec,
                    imu_sample.angular_z - wheel_sample.angular_z,
                ))
                time_differences.append(time_difference_sec)
                del available_wheel_samples[wheel_index]

        residuals = [pair[2] for pair in paired_residuals]
        pair_count = len(residuals)
        if pair_count:
            residual_mean = sum(residuals) / pair_count
            residual_median = self._median(residuals)
            residual_stddev = sqrt(
                sum((residual - residual_mean) ** 2 for residual in residuals)
                / pair_count
            )
            residual_mad = self._median([
                abs(residual - residual_median) for residual in residuals
            ])
            max_time_difference_sec = max(time_differences)
        else:
            residual_mean = residual_median = residual_stddev = residual_mad = 0.0
            max_time_difference_sec = 0.0
        return (
            [
                'imu_wheel_pair_count',
                'imu_wheel_residual_mean_rad_s',
                'imu_wheel_residual_median_rad_s',
                'imu_wheel_residual_stddev_rad_s',
                'imu_wheel_residual_mad_rad_s',
                'imu_wheel_max_pairing_time_difference_sec',
            ],
            [
                float(pair_count),
                residual_mean,
                residual_median,
                residual_stddev,
                residual_mad,
                max_time_difference_sec,
            ],
            pair_count,
            residual_mean,
            residual_median,
            residual_stddev,
            residual_mad,
            paired_residuals,
        )

    @staticmethod
    def _median(values):
        ordered_values = sorted(values)
        middle = len(ordered_values) // 2
        if len(ordered_values) % 2:
            return ordered_values[middle]
        return (ordered_values[middle - 1] + ordered_values[middle]) / 2.0

    def _wheel_reference_reason(self, wheel_decision, wheel_sample_count, pair_count):
        """Explain why a wheel reference cannot support a bias decision."""
        if wheel_decision.state != SensorHealth.HEALTHY:
            if wheel_sample_count < self._min_samples:
                return 'wheel_reference_insufficient'
            return 'wheel_reference_unavailable'
        if pair_count == 0:
            return 'imu_wheel_timestamp_pairs_unavailable'
        if pair_count < self._imu_bias_min_pairs:
            return 'imu_wheel_pairs_insufficient'
        return None

    def _bias_level(
        self, residual_mean, residual_median, residual_stddev, residual_mad
    ):
        """Classify a stable signed residual against conservative fixed thresholds."""
        if (
            residual_stddev > self._imu_bias_max_stddev_rad_s
            or residual_mad > self._imu_bias_max_mad_rad_s
        ):
            return 'none'
        signed_residual = min(abs(residual_mean), abs(residual_median))
        if signed_residual >= self._imu_bias_fault_threshold_rad_s:
            return 'fault'
        if signed_residual >= self._imu_bias_warning_threshold_rad_s:
            return 'warning'
        return 'none'

    def _collect_imu_bias_baseline(self, paired_residuals):
        """Freeze a startup nominal residual from unique healthy pair samples."""
        for imu_stamp_sec, wheel_stamp_sec, residual in paired_residuals:
            pair_key = (imu_stamp_sec, wheel_stamp_sec)
            if pair_key not in self._imu_bias_baseline_pair_keys:
                self._imu_bias_baseline_pair_keys.add(pair_key)
                self._imu_bias_baseline_samples.append(residual)
        if (
            len(self._imu_bias_baseline_samples)
            >= self._imu_bias_baseline_calibration_min_pairs
        ):
            self._imu_bias_baseline_residual = self._median(
                self._imu_bias_baseline_samples
            )

    def _update_bias_counts(self, bias_level):
        """Track uninterrupted warning and fault-level yaw-rate residuals."""
        if bias_level == 'fault':
            self._bias_fault_count += 1
            self._bias_warning_count += 1
        elif bias_level == 'warning':
            self._bias_fault_count = 0
            self._bias_warning_count += 1
        else:
            self._reset_bias_counts()

    def _reset_bias_counts(self):
        """Clear bias confirmation after a healthy or unusable assessment."""
        self._bias_warning_count = 0
        self._bias_fault_count = 0

    def _unknown_decision(self, samples, min_samples=None):
        min_samples = min_samples or self._min_samples
        count = len(samples)
        start_sec = samples[0].received_sec if samples else 0.0
        end_sec = samples[-1].received_sec if samples else 0.0
        reason = 'no_messages_received' if count == 0 else 'insufficient_samples'
        return HealthDecision(
            SensorHealth.UNKNOWN,
            -1.0,
            min(count / min_samples, 1.0),
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

    def _wheel_freeze_level(self, now_sec, samples):
        """Return warning/fault from short and full no-progress horizons."""
        if not self._is_motion_commanded() or self._motion_command_started_sec is None:
            return 'none'
        fault_samples = self._covered_recent_samples(
            now_sec, samples, self._freeze_duration_sec
        )
        if fault_samples is not None and self._wheel_lacks_progress(fault_samples):
            return 'fault'
        warning_samples = self._covered_recent_samples(
            now_sec, samples, self._freeze_warning_duration_sec
        )
        if warning_samples is None or not self._wheel_lacks_progress(
            warning_samples
        ):
            return 'none'
        return 'warning'

    def _covered_recent_samples(self, now_sec, samples, duration_sec):
        if now_sec - self._motion_command_started_sec < duration_sec:
            return None
        start_sec = max(
            now_sec - duration_sec,
            self._motion_command_started_sec,
        )
        recent = [
            sample for sample in samples if sample.received_sec >= start_sec
        ]
        if len(recent) < 2:
            return None
        if recent[-1].received_sec - recent[0].received_sec < 0.8 * duration_sec:
            return None
        return recent

    def _wheel_lacks_progress(self, samples):
        pose_span_m, yaw_span_rad, _, _ = self._wheel_spans(samples)
        linear_is_frozen = (
            self._command_linear_abs_mps > self._command_linear_threshold_mps
            and pose_span_m < self._wheel_pose_span_threshold_m
        )
        angular_is_frozen = (
            self._command_angular_abs_rad_s
            > self._command_angular_threshold_rad_s
            and yaw_span_rad < self._wheel_yaw_span_threshold_rad
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
            self._float_parameter('imu_wheel_max_pairing_time_diff_sec'),
            self._integer_parameter('imu_bias_min_pairs'),
            self._float_parameter('imu_bias_warning_threshold_rad_s'),
            self._float_parameter('imu_bias_fault_threshold_rad_s'),
            self._float_parameter('imu_bias_max_stddev_rad_s'),
            self._float_parameter('imu_bias_max_mad_rad_s'),
            self._integer_parameter('imu_bias_confirmation_cycles'),
            self._integer_parameter('imu_bias_baseline_calibration_min_pairs'),
            self._integer_parameter('scan_min_samples'),
            self._float_parameter('scan_nan_warning_ratio'),
            self._float_parameter('scan_nan_fault_ratio'),
            self._float_parameter('scan_sector_warning_width_rad'),
            self._float_parameter('scan_sector_fault_width_rad'),
            self._integer_parameter('scan_confirmation_cycles'),
            self._integer_parameter('scan_recovery_cycles'),
            self._float_parameter('freeze_warning_duration_sec'),
            self._float_parameter('wheel_yaw_span_threshold_rad'),
            self._float_parameter('future_stamp_tolerance_sec'),
        )
        self._health_publishers = {
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
        self.create_subscription(
            LaserScan,
            self._sources['scan'],
            self._on_scan,
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
            'freeze_warning_duration_sec': 0.40,
            'wheel_pose_span_threshold_m': 0.01,
            'wheel_yaw_span_threshold_rad': 0.01,
            'wheel_linear_span_threshold_mps': 0.01,
            'wheel_angular_span_threshold_rad_s': 0.02,
            'imu_wheel_max_pairing_time_diff_sec': 0.05,
            'imu_bias_min_pairs': 10,
            'imu_bias_warning_threshold_rad_s': 0.08,
            'imu_bias_fault_threshold_rad_s': 0.12,
            'imu_bias_max_stddev_rad_s': 0.03,
            'imu_bias_max_mad_rad_s': 0.02,
            'imu_bias_confirmation_cycles': 3,
            'imu_bias_baseline_calibration_min_pairs': 10,
            'scan_min_samples': 3,
            'scan_nan_warning_ratio': 0.03,
            'scan_nan_fault_ratio': 0.08,
            'scan_sector_warning_width_rad': 0.50,
            'scan_sector_fault_width_rad': 0.90,
            'scan_confirmation_cycles': 3,
            'scan_recovery_cycles': 3,
            'future_stamp_tolerance_sec': 0.05,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _on_imu(self, msg):
        now_sec = self._now_sec()
        self._evaluator.add_imu(
            now_sec,
            seconds_from_stamp(msg.header.stamp),
            msg.angular_velocity.z,
        )

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

    def _on_scan(self, msg):
        now_sec = self._now_sec()
        self._evaluator.add_scan(
            now_sec,
            seconds_from_stamp(msg.header.stamp),
            msg.ranges,
            msg.angle_increment,
        )

    def _on_command(self, msg):
        self._evaluator.set_command(
            self._now_sec(), msg.linear.x, msg.angular.z
        )

    def _publish_health(self):
        now_sec = self._now_sec()
        stamp = stamp_from_seconds(now_sec)
        wheel_decision = self._evaluator.evaluate_wheel(now_sec)
        self._health_publishers['imu'].publish(make_sensor_health(
            'imu', self._sources['imu'], stamp,
            self._evaluator.evaluate_imu(now_sec, wheel_decision),
        ))
        self._health_publishers['wheel'].publish(make_sensor_health(
            'wheel', self._sources['wheel'], stamp,
            wheel_decision,
        ))
        self._health_publishers['scan'].publish(make_sensor_health(
            'scan', self._sources['scan'], stamp,
            self._evaluator.evaluate_scan(now_sec),
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
