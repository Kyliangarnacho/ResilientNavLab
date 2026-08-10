"""Publish conservative C920 timing and visual health decisions."""

from collections import deque
from dataclasses import dataclass
from math import isfinite

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from resilient_nav_health_assessment.camera_health_calibrate import (
    image_message_to_numpy,
    VISUAL_METRICS,
)
from resilient_nav_health_assessment.camera_health_features import (
    compute_camera_health_features,
)
from resilient_nav_health_assessment.sensor_health_monitor import (
    HealthDecision,
    make_sensor_health,
    seconds_from_stamp,
    stamp_from_seconds,
)
from resilient_nav_interfaces.msg import SensorHealth
from sensor_msgs.msg import Image


DEFAULT_IMAGE_TOPIC = '/camera/c920/image_raw'
DEFAULT_HEALTH_TOPIC = '/health/camera'


@dataclass(frozen=True)
class CameraObservation:
    """One valid image with transport deltas and extracted features."""

    received_sec: float
    stamp_sec: float
    interarrival_sec: object
    stamp_delta_sec: object
    fingerprint_changed: bool
    features: dict


class CameraHealthEvaluator:
    """Evaluate time-confirmed camera timing and visual faults."""

    def __init__(
        self,
        *,
        window_duration_sec=3.0,
        min_samples=3,
        stale_timeout_sec=1.0,
        freeze_duration_sec=2.0,
        fault_confirmation_sec=0.6,
        recovery_confirmation_sec=1.0,
        dark_mean_gray_candidate=6.0,
        dark_p95_candidate=8.0,
        bright_mean_gray_candidate=170.0,
        bright_p05_candidate=150.0,
        bright_p95_candidate=180.0,
        dark_ratio_candidate=0.90,
        bright_ratio_candidate=0.95,
        blur_reference_max_age_sec=3.0,
        blur_reference_laplacian_variance_min=100.0,
        blur_reference_edge_density_min=0.01,
        blur_laplacian_variance_candidate=10.0,
        blur_edge_density_candidate=0.001,
        blur_gray_std_min=20.0,
        blur_entropy_min=5.5,
        low_information_reference_max_age_sec=3.0,
        low_information_reference_edge_density_min=0.01,
        low_information_reference_entropy_min=5.2,
        low_information_edge_density_candidate=0.0005,
        low_information_entropy_candidate=5.8,
        low_information_dark_ratio_min=0.20,
        low_information_gray_std_min=20.0,
    ):
        self._validate_positive('window_duration_sec', window_duration_sec)
        self._validate_positive('stale_timeout_sec', stale_timeout_sec)
        self._validate_positive('freeze_duration_sec', freeze_duration_sec)
        self._validate_positive(
            'fault_confirmation_sec', fault_confirmation_sec
        )
        self._validate_positive(
            'recovery_confirmation_sec', recovery_confirmation_sec
        )
        if min_samples < 2:
            raise ValueError('min_samples must be at least 2')
        if not (
            0.0
            <= dark_mean_gray_candidate
            < bright_mean_gray_candidate
            <= 255.0
        ):
            raise ValueError('mean-gray candidate thresholds must be ordered')
        if not 0.0 <= dark_p95_candidate <= 255.0:
            raise ValueError('dark_p95_candidate must be within [0, 255]')
        if not (
            0.0
            <= bright_p05_candidate
            <= bright_p95_candidate
            <= 255.0
        ):
            raise ValueError(
                'bright percentile candidate thresholds must be ordered'
            )
        if not 0.0 <= dark_ratio_candidate <= 1.0:
            raise ValueError('dark_ratio_candidate must be within [0, 1]')
        if not 0.0 <= bright_ratio_candidate <= 1.0:
            raise ValueError('bright_ratio_candidate must be within [0, 1]')
        if blur_laplacian_variance_candidate < 0.0:
            raise ValueError(
                'blur_laplacian_variance_candidate must be non-negative'
            )
        self._validate_positive(
            'blur_reference_max_age_sec', blur_reference_max_age_sec
        )
        if (
            blur_reference_laplacian_variance_min
            <= blur_laplacian_variance_candidate
        ):
            raise ValueError(
                'blur Laplacian reference must exceed candidate threshold'
            )
        if not (
            0.0
            <= blur_edge_density_candidate
            < blur_reference_edge_density_min
            <= 1.0
        ):
            raise ValueError(
                'blur edge candidate/reference thresholds must be ordered'
            )
        if blur_gray_std_min < 0.0:
            raise ValueError('blur_gray_std_min must be non-negative')
        if blur_entropy_min < 0.0:
            raise ValueError('blur_entropy_min must be non-negative')
        self._validate_positive(
            'low_information_reference_max_age_sec',
            low_information_reference_max_age_sec,
        )
        if low_information_reference_entropy_min < 0.0:
            raise ValueError(
                'low_information_reference_entropy_min must be non-negative'
            )
        if low_information_entropy_candidate < 0.0:
            raise ValueError(
                'low_information_entropy_candidate must be non-negative'
            )
        if not (
            0.0
            <= low_information_edge_density_candidate
            < low_information_reference_edge_density_min
            <= 1.0
        ):
            raise ValueError(
                'low-information edge candidate/reference thresholds '
                'must be ordered'
            )
        if not 0.0 <= low_information_dark_ratio_min <= 1.0:
            raise ValueError(
                'low_information_dark_ratio_min must be within [0, 1]'
            )
        if low_information_gray_std_min < 0.0:
            raise ValueError(
                'low_information_gray_std_min must be non-negative'
            )

        self._window_duration_sec = float(window_duration_sec)
        self._min_samples = int(min_samples)
        self._stale_timeout_sec = float(stale_timeout_sec)
        self._freeze_duration_sec = float(freeze_duration_sec)
        self._fault_confirmation_sec = float(fault_confirmation_sec)
        self._recovery_confirmation_sec = float(recovery_confirmation_sec)
        self._dark_mean_gray_candidate = float(dark_mean_gray_candidate)
        self._dark_p95_candidate = float(dark_p95_candidate)
        self._bright_mean_gray_candidate = float(bright_mean_gray_candidate)
        self._bright_p05_candidate = float(bright_p05_candidate)
        self._bright_p95_candidate = float(bright_p95_candidate)
        self._dark_ratio_candidate = float(dark_ratio_candidate)
        self._bright_ratio_candidate = float(bright_ratio_candidate)
        self._blur_reference_max_age_sec = float(
            blur_reference_max_age_sec
        )
        self._blur_reference_laplacian_variance_min = float(
            blur_reference_laplacian_variance_min
        )
        self._blur_reference_edge_density_min = float(
            blur_reference_edge_density_min
        )
        self._blur_laplacian_variance_candidate = float(
            blur_laplacian_variance_candidate
        )
        self._blur_edge_density_candidate = float(
            blur_edge_density_candidate
        )
        self._blur_gray_std_min = float(blur_gray_std_min)
        self._blur_entropy_min = float(blur_entropy_min)
        self._low_information_reference_max_age_sec = float(
            low_information_reference_max_age_sec
        )
        self._low_information_reference_edge_density_min = float(
            low_information_reference_edge_density_min
        )
        self._low_information_reference_entropy_min = float(
            low_information_reference_entropy_min
        )
        self._low_information_entropy_candidate = float(
            low_information_entropy_candidate
        )
        self._low_information_edge_density_candidate = float(
            low_information_edge_density_candidate
        )
        self._low_information_dark_ratio_min = float(
            low_information_dark_ratio_min
        )
        self._low_information_gray_std_min = float(
            low_information_gray_std_min
        )

        self._samples = deque()
        self._rejected_receive_times = deque()
        self._previous_image = None
        self._latest_message_received_sec = None
        self._latest_message_stamp_sec = None
        self._latest_valid_received_sec = None
        self._latest_fingerprint = None
        self._fingerprint_run_start_sec = None
        self._fingerprint_identical_count = 0
        self._fingerprint_stamp_progress_continuous = False
        self._monitor_started_sec = None
        self._candidate_fault = None
        self._candidate_started_sec = None
        self._active_fault = None
        self._recovery_started_sec = None
        self._blur_reference_received_sec = None
        self._blur_reference_laplacian_variance = 0.0
        self._blur_reference_edge_density = 0.0
        self._low_information_reference_received_sec = None
        self._low_information_reference_edge_density = 0.0
        self._low_information_reference_entropy = 0.0

    @staticmethod
    def _validate_positive(name, value):
        if not isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and greater than zero')

    def start(self, now_sec):
        """Set the no-image stale clock once at node startup."""
        if self._monitor_started_sec is None:
            self._monitor_started_sec = float(now_sec)

    def add_image(
        self,
        image,
        *,
        channel_order,
        received_sec,
        stamp_sec,
    ):
        """Extract the existing features and record one valid Image."""
        received_sec = float(received_sec)
        stamp_sec = float(stamp_sec)
        self.start(received_sec)
        features = compute_camera_health_features(
            image,
            self._previous_image,
            channel_order=channel_order,
        )
        self._record_blur_evidence(features, received_sec)
        self._record_low_information_evidence(features, received_sec)
        interarrival_sec, stamp_delta_sec = self._record_arrival(
            received_sec, stamp_sec
        )
        fingerprint = features['frame_fingerprint']
        continuity_available = self._latest_fingerprint is not None
        fingerprint_changed = (
            continuity_available and fingerprint != self._latest_fingerprint
        )
        stamp_progressed = (
            stamp_delta_sec is not None and stamp_delta_sec > 0.0
        )
        if continuity_available and not fingerprint_changed:
            self._fingerprint_identical_count += 1
            self._fingerprint_stamp_progress_continuous = (
                self._fingerprint_stamp_progress_continuous
                and stamp_progressed
            )
        else:
            self._fingerprint_run_start_sec = received_sec
            self._fingerprint_identical_count = 1
            self._fingerprint_stamp_progress_continuous = True

        observation = CameraObservation(
            received_sec=received_sec,
            stamp_sec=stamp_sec,
            interarrival_sec=interarrival_sec,
            stamp_delta_sec=stamp_delta_sec,
            fingerprint_changed=(
                fingerprint_changed if continuity_available else True
            ),
            features=features,
        )
        self._samples.append(observation)
        self._latest_valid_received_sec = received_sec
        self._latest_fingerprint = fingerprint
        self._previous_image = np.array(image, copy=True)
        self._prune(received_sec)
        return features

    def record_rejected_image(self, *, received_sec, stamp_sec):
        """Record Image transport while breaking unobservable continuity."""
        received_sec = float(received_sec)
        stamp_sec = float(stamp_sec)
        self.start(received_sec)
        self._record_arrival(received_sec, stamp_sec)
        self._rejected_receive_times.append(received_sec)
        self._previous_image = None
        self._latest_fingerprint = None
        self._fingerprint_run_start_sec = None
        self._fingerprint_identical_count = 0
        self._fingerprint_stamp_progress_continuous = False
        self._prune(received_sec)

    def _record_arrival(self, received_sec, stamp_sec):
        interarrival_sec = None
        stamp_delta_sec = None
        if self._latest_message_received_sec is not None:
            interarrival_sec = (
                received_sec - self._latest_message_received_sec
            )
            if interarrival_sec < 0.0:
                raise ValueError('received_sec must not move backwards')
            stamp_delta_sec = stamp_sec - self._latest_message_stamp_sec
        self._latest_message_received_sec = received_sec
        self._latest_message_stamp_sec = stamp_sec
        return interarrival_sec, stamp_delta_sec

    def evaluate(self, now_sec):
        """Return one fixed-rate SensorHealth-compatible decision."""
        now_sec = float(now_sec)
        self.start(now_sec)
        samples = self._window_samples(now_sec)
        metrics = self._metrics(now_sec, samples)
        raw_fault = self._raw_fault(metrics)
        visual_fault_prefix = self._visual_fault_prefix(raw_fault)
        if visual_fault_prefix is not None:
            metrics['fault_confirmation_elapsed_sec'] = metrics[
                f'{visual_fault_prefix}_candidate_duration_sec'
            ]
        stable = (
            raw_fault is None
            and len(samples) >= self._min_samples
            and metrics['message_age_sec'] <= self._stale_timeout_sec
            and metrics['latest_valid_image_age_sec'] >= 0.0
            and metrics['latest_valid_image_age_sec'] <= self._stale_timeout_sec
            and metrics['header_stamp_progress'] == 1.0
        )
        if self._active_fault == 'freeze':
            stable = stable and metrics['fingerprint_changed'] == 1.0
        decision = self._state_decision(
            now_sec, samples, metrics, raw_fault, stable
        )
        return self._with_metrics(decision, metrics)

    def _raw_fault(self, metrics):
        if metrics['message_age_sec'] > self._stale_timeout_sec:
            return 'stale'
        if metrics['exposure_dark_candidate'] == 1.0:
            return 'underexposed'
        if metrics['exposure_bright_candidate'] == 1.0:
            return 'overexposed'
        # The low-information rule is more specific than blur because it also
        # requires the occlusion-development entropy/dark/std signature.  It
        # therefore wins only when both candidates are simultaneously true.
        if metrics['low_information_candidate'] == 1.0:
            return 'low_information'
        if metrics['blur_candidate'] == 1.0:
            return 'blurred'
        if (
            metrics['latest_valid_image_age_sec'] <= self._stale_timeout_sec
            and metrics['fingerprint_identical_count'] >= 2.0
            and metrics['fingerprint_identical_duration_sec']
            >= self._freeze_duration_sec
            and metrics['fingerprint_stamp_progress_continuous'] == 1.0
        ):
            return 'freeze'
        return None

    def _state_decision(self, now_sec, samples, metrics, raw_fault, stable):
        if self._active_fault is not None:
            if raw_fault is not None:
                self._active_fault = raw_fault
                self._recovery_started_sec = None
                return self._fault_decision(samples, metrics, raw_fault, False)
            if stable:
                if self._recovery_started_sec is None:
                    self._recovery_started_sec = now_sec
                recovery_elapsed = now_sec - self._recovery_started_sec
                if recovery_elapsed >= self._recovery_confirmation_sec:
                    self._active_fault = None
                    self._recovery_started_sec = None
                    self._reset_candidate()
                    return self._healthy_decision(samples, metrics)
                return self._fault_decision(
                    samples, metrics, self._active_fault, True
                )
            self._recovery_started_sec = None
            return self._fault_decision(
                samples, metrics, self._active_fault, True
            )

        if raw_fault is not None:
            if self._candidate_fault != raw_fault:
                self._candidate_fault = raw_fault
                self._candidate_started_sec = now_sec
            visual_fault_prefix = self._visual_fault_prefix(raw_fault)
            if visual_fault_prefix is not None:
                confirmation_elapsed = metrics[
                    f'{visual_fault_prefix}_candidate_duration_sec'
                ]
            else:
                confirmation_elapsed = now_sec - self._candidate_started_sec
            confirmation_complete = (
                confirmation_elapsed >= self._fault_confirmation_sec
            )
            if visual_fault_prefix is not None:
                confirmation_complete = confirmation_complete and (
                    metrics[f'{visual_fault_prefix}_candidate_count']
                    >= self._min_samples
                )
            if confirmation_complete:
                self._active_fault = raw_fault
                self._recovery_started_sec = None
                return self._fault_decision(
                    samples, metrics, raw_fault, False
                )
            return self._pending_decision(
                samples, metrics, raw_fault, confirmation_elapsed
            )

        self._reset_candidate()
        if len(samples) < self._min_samples:
            return self._unknown_decision(samples, metrics)
        return self._healthy_decision(samples, metrics)

    def _healthy_decision(self, samples, metrics):
        visual_candidates = self._visual_candidate_names(metrics)
        rejection_ratio = metrics['recent_rejected_image_ratio']
        score = max(
            0.7,
            1.0 - 0.05 * len(visual_candidates) - 0.2 * rejection_ratio,
        )
        reasons = ['camera_stream_recent_and_content_progressing']
        if visual_candidates:
            reasons.append('visual_candidates_are_observational_only')
        return self._decision(
            SensorHealth.HEALTHY,
            score,
            min(len(samples) / self._min_samples, 1.0),
            'none',
            reasons,
            samples,
        )

    def _pending_decision(
        self, samples, metrics, raw_fault, confirmation_elapsed
    ):
        progress = min(
            confirmation_elapsed / self._fault_confirmation_sec, 1.0
        )
        if raw_fault == 'stale':
            score = max(
                0.0,
                1.0 - metrics['message_age_sec'] / (
                    self._stale_timeout_sec + self._fault_confirmation_sec
                ),
            )
        else:
            score = 0.5
        return self._decision(
            SensorHealth.DEGRADED,
            score,
            0.5 + 0.5 * progress,
            raw_fault,
            [f'{raw_fault}_confirmation_pending'],
            samples,
        )

    def _fault_decision(self, samples, metrics, fault, recovery_pending):
        if recovery_pending:
            recovery_elapsed = self._recovery_elapsed(metrics['now_sec'])
            progress = min(
                recovery_elapsed / self._recovery_confirmation_sec, 1.0
            )
            return self._decision(
                SensorHealth.FAULT,
                0.4,
                1.0 - 0.5 * progress,
                fault,
                [f'{fault}_recovery_pending'],
                samples,
            )
        reasons = {
            'stale': ['image_stream_absent_beyond_stale_timeout'],
            'underexposed': [
                'mean_gray_p95_and_dark_ratio_confirm_severe_underexposure'
            ],
            'overexposed': [
                'mean_gray_p05_and_p95_confirm_severe_overexposure'
            ],
            'blurred': [
                'textured_reference_lost_laplacian_and_edge_detail'
            ],
            'low_information': [
                'informative_reference_lost_edge_entropy_and_spatial_detail'
            ],
            'freeze': [
                'image_stamps_progressed_while_fingerprint_remained_identical'
            ],
        }
        fault_scores = {
            'stale': 0.0,
            'freeze': 0.1,
            'underexposed': 0.2,
            'overexposed': 0.2,
            'blurred': 0.2,
            'low_information': 0.2,
        }
        return self._decision(
            SensorHealth.FAULT,
            fault_scores[fault],
            1.0,
            fault,
            reasons[fault],
            samples,
        )

    def _unknown_decision(self, samples, metrics):
        if self._latest_message_received_sec is None:
            reason = 'no_images_received'
        elif self._latest_valid_received_sec is None:
            reason = 'no_valid_images_received'
        else:
            reason = 'insufficient_camera_samples'
        return self._decision(
            SensorHealth.UNKNOWN,
            -1.0,
            min(len(samples) / self._min_samples, 1.0),
            'unknown',
            [reason],
            samples,
        )

    def _decision(
        self, state, score, confidence, fault, reasons, samples
    ):
        if samples:
            window_start_sec = samples[0].received_sec
            window_end_sec = samples[-1].received_sec
        else:
            fallback = self._latest_message_received_sec
            if fallback is None:
                fallback = self._monitor_started_sec or 0.0
            window_start_sec = window_end_sec = fallback
        return HealthDecision(
            state,
            float(score),
            float(confidence),
            fault,
            reasons,
            [],
            [],
            window_start_sec,
            window_end_sec,
            len(samples),
        )

    def _metrics(self, now_sec, samples):
        message_reference = self._latest_message_received_sec
        if message_reference is None:
            message_reference = self._monitor_started_sec
        message_age_sec = max(now_sec - message_reference, 0.0)
        valid_age_sec = -1.0
        if self._latest_valid_received_sec is not None:
            valid_age_sec = max(
                now_sec - self._latest_valid_received_sec, 0.0
            )

        interarrivals = [
            sample.interarrival_sec
            for sample in samples
            if sample.interarrival_sec is not None
        ]
        latest_interarrival_sec = (
            interarrivals[-1] if interarrivals else -1.0
        )
        max_recent_gap_sec = max(interarrivals) if interarrivals else 0.0
        rolling_observed_fps = 0.0
        if len(samples) >= 2:
            span_sec = samples[-1].received_sec - samples[0].received_sec
            if span_sec > 0.0:
                rolling_observed_fps = (len(samples) - 1) / span_sec

        stamp_deltas = [
            sample.stamp_delta_sec
            for sample in samples
            if sample.stamp_delta_sec is not None
        ]
        header_progress_count = sum(delta > 0.0 for delta in stamp_deltas)
        header_regression_count = sum(delta < 0.0 for delta in stamp_deltas)
        header_progress_ratio = (
            header_progress_count / len(stamp_deltas)
            if stamp_deltas else 0.0
        )
        fingerprint_duration_sec = 0.0
        if (
            self._fingerprint_run_start_sec is not None
            and self._latest_valid_received_sec is not None
        ):
            fingerprint_duration_sec = max(
                self._latest_valid_received_sec
                - self._fingerprint_run_start_sec,
                0.0,
            )

        latest = samples[-1] if samples else None
        features = latest.features if latest is not None else {}
        underexposure_count, underexposure_duration_sec = (
            self._trailing_evidence(samples, self._underexposure_evidence)
        )
        overexposure_count, overexposure_duration_sec = (
            self._trailing_evidence(samples, self._overexposure_evidence)
        )
        blur_count, blur_duration_sec = self._trailing_evidence(
            samples,
            lambda sample_features: (
                sample_features.get('blur_candidate_evidence', 0.0) == 1.0
            ),
        )
        low_information_count, low_information_duration_sec = (
            self._trailing_evidence(
                samples,
                lambda sample_features: (
                    sample_features.get(
                        'low_information_candidate_evidence', 0.0
                    ) == 1.0
                ),
            )
        )
        metrics = {
            'now_sec': now_sec,
            'message_age_sec': message_age_sec,
            'latest_valid_image_age_sec': valid_age_sec,
            'latest_interarrival_sec': latest_interarrival_sec,
            'rolling_observed_fps': rolling_observed_fps,
            'max_recent_gap_sec': max_recent_gap_sec,
            'latest_header_stamp_sec': (
                self._latest_message_stamp_sec
                if self._latest_message_stamp_sec is not None else -1.0
            ),
            'header_stamp_delta_sec': (
                stamp_deltas[-1] if stamp_deltas else 0.0
            ),
            'header_stamp_progress': float(
                bool(stamp_deltas and stamp_deltas[-1] > 0.0)
            ),
            'header_stamp_progress_ratio': header_progress_ratio,
            'header_stamp_regression_count': float(header_regression_count),
            'fingerprint_changed': float(
                latest.fingerprint_changed if latest is not None else False
            ),
            'fingerprint_identical_duration_sec': fingerprint_duration_sec,
            'fingerprint_identical_count': float(
                self._fingerprint_identical_count
            ),
            'fingerprint_stamp_progress_continuous': float(
                self._fingerprint_stamp_progress_continuous
            ),
            'recent_rejected_image_count': float(
                len(self._rejected_receive_times)
            ),
            'recent_rejected_image_ratio': (
                len(self._rejected_receive_times)
                / (len(samples) + len(self._rejected_receive_times))
                if samples or self._rejected_receive_times else 0.0
            ),
            'underexposure_candidate_count': float(
                underexposure_count
            ),
            'underexposure_candidate_duration_sec': (
                underexposure_duration_sec
            ),
            'overexposure_candidate_count': float(overexposure_count),
            'overexposure_candidate_duration_sec': (
                overexposure_duration_sec
            ),
            'blur_candidate_count': float(blur_count),
            'blur_candidate_duration_sec': blur_duration_sec,
            'blur_reference_ready': float(
                self._blur_reference_available(now_sec)
            ),
            'blur_reference_laplacian_variance': (
                self._blur_reference_laplacian_variance
            ),
            'blur_reference_edge_density': (
                self._blur_reference_edge_density
            ),
            'low_information_candidate_count': float(
                low_information_count
            ),
            'low_information_candidate_duration_sec': (
                low_information_duration_sec
            ),
            'low_information_reference_ready': float(
                self._low_information_reference_available(now_sec)
            ),
            'low_information_reference_edge_density': (
                self._low_information_reference_edge_density
            ),
            'low_information_reference_entropy': (
                self._low_information_reference_entropy
            ),
            'fault_confirmation_elapsed_sec': self._candidate_elapsed(now_sec),
            'recovery_elapsed_sec': self._recovery_elapsed(now_sec),
        }
        for name in VISUAL_METRICS:
            value = features.get(name)
            metrics[name] = float(value) if value is not None else -1.0
        metrics['blur_candidate_evidence'] = float(
            features.get('blur_candidate_evidence', 0.0)
        )
        metrics['low_information_candidate_evidence'] = float(
            features.get('low_information_candidate_evidence', 0.0)
        )
        metrics.update(self._visual_candidates(metrics))
        return metrics

    def _visual_candidates(self, metrics):
        visual_available = metrics['mean_gray'] >= 0.0
        dark = self._underexposure_evidence(metrics)
        bright = self._overexposure_evidence(metrics)
        bright_ratio = visual_available and (
            metrics['bright_ratio'] >= self._bright_ratio_candidate
        )
        blur = metrics.get('blur_candidate_evidence', 0.0) == 1.0
        low_information = (
            metrics.get('low_information_candidate_evidence', 0.0) == 1.0
        )
        return {
            'exposure_dark_candidate': float(dark),
            'exposure_bright_candidate': float(bright),
            'bright_ratio_observation_candidate': float(bright_ratio),
            'blur_candidate': float(blur),
            'low_information_candidate': float(low_information),
        }

    def _underexposure_evidence(self, features):
        return (
            features.get('mean_gray', -1.0) >= 0.0
            and features.get('mean_gray', 256.0)
            <= self._dark_mean_gray_candidate
            and features.get('p95', 256.0) <= self._dark_p95_candidate
            and features.get('dark_ratio', -1.0)
            >= self._dark_ratio_candidate
        )

    def _overexposure_evidence(self, features):
        return (
            features.get('mean_gray', 256.0)
            >= self._bright_mean_gray_candidate
            and features.get('p05', -1.0) >= self._bright_p05_candidate
            and features.get('p95', -1.0) >= self._bright_p95_candidate
        )

    def _record_blur_evidence(self, features, received_sec):
        if self._sharp_blur_reference(features):
            self._blur_reference_received_sec = received_sec
            self._blur_reference_laplacian_variance = float(
                features['laplacian_variance']
            )
            self._blur_reference_edge_density = float(
                features['edge_density']
            )
        features['blur_candidate_evidence'] = float(
            self._blur_reference_available(received_sec)
            and self._blurred_current_frame(features)
        )

    def _sharp_blur_reference(self, features):
        return (
            features.get('laplacian_variance', 0.0)
            >= self._blur_reference_laplacian_variance_min
            and features.get('edge_density', 0.0)
            >= self._blur_reference_edge_density_min
        )

    def _blurred_current_frame(self, features):
        return (
            features.get('laplacian_variance', float('inf'))
            <= self._blur_laplacian_variance_candidate
            and features.get('edge_density', float('inf'))
            <= self._blur_edge_density_candidate
            and features.get('gray_std', -1.0) >= self._blur_gray_std_min
            and features.get('entropy', -1.0) >= self._blur_entropy_min
        )

    def _blur_reference_available(self, now_sec):
        if self._blur_reference_received_sec is None:
            return False
        if (
            self._candidate_fault == 'blurred'
            or self._active_fault == 'blurred'
        ):
            return True
        return (
            now_sec - self._blur_reference_received_sec
            <= self._blur_reference_max_age_sec
        )

    def _record_low_information_evidence(self, features, received_sec):
        if self._informative_low_information_reference(features):
            self._low_information_reference_received_sec = received_sec
            self._low_information_reference_edge_density = float(
                features['edge_density']
            )
            self._low_information_reference_entropy = float(
                features['entropy']
            )
        features['low_information_candidate_evidence'] = float(
            self._low_information_reference_available(received_sec)
            and self._low_information_current_frame(features)
        )

    def _informative_low_information_reference(self, features):
        return (
            features.get('edge_density', 0.0)
            >= self._low_information_reference_edge_density_min
            and features.get('entropy', 0.0)
            >= self._low_information_reference_entropy_min
        )

    def _low_information_current_frame(self, features):
        return (
            features.get('edge_density', float('inf'))
            <= self._low_information_edge_density_candidate
            and features.get('entropy', float('inf'))
            <= self._low_information_entropy_candidate
            and features.get('dark_ratio', -1.0)
            >= self._low_information_dark_ratio_min
            and features.get('gray_std', -1.0)
            >= self._low_information_gray_std_min
        )

    def _low_information_reference_available(self, now_sec):
        if self._low_information_reference_received_sec is None:
            return False
        if (
            self._candidate_fault == 'low_information'
            or self._active_fault == 'low_information'
        ):
            return True
        return (
            now_sec - self._low_information_reference_received_sec
            <= self._low_information_reference_max_age_sec
        )

    @staticmethod
    def _trailing_evidence(samples, predicate):
        evidence_samples = []
        for sample in reversed(samples):
            if not predicate(sample.features):
                break
            evidence_samples.append(sample)
        duration_sec = 0.0
        if len(evidence_samples) >= 2:
            duration_sec = (
                evidence_samples[0].received_sec
                - evidence_samples[-1].received_sec
            )
        return len(evidence_samples), duration_sec

    @staticmethod
    def _visual_fault_prefix(fault):
        return {
            'underexposed': 'underexposure',
            'overexposed': 'overexposure',
            'blurred': 'blur',
            'low_information': 'low_information',
        }.get(fault)

    @staticmethod
    def _visual_candidate_names(metrics):
        return [
            name
            for name in (
                'exposure_dark_candidate',
                'exposure_bright_candidate',
                'bright_ratio_observation_candidate',
                'blur_candidate',
                'low_information_candidate',
            )
            if metrics[name] == 1.0
        ]

    def _window_samples(self, now_sec):
        self._prune(now_sec)
        return list(self._samples)

    def _prune(self, now_sec):
        cutoff_sec = now_sec - self._window_duration_sec
        while self._samples and self._samples[0].received_sec < cutoff_sec:
            self._samples.popleft()
        while (
            self._rejected_receive_times
            and self._rejected_receive_times[0] < cutoff_sec
        ):
            self._rejected_receive_times.popleft()

    def _candidate_elapsed(self, now_sec):
        if self._candidate_started_sec is None:
            return 0.0
        return max(now_sec - self._candidate_started_sec, 0.0)

    def _recovery_elapsed(self, now_sec):
        if self._recovery_started_sec is None:
            return 0.0
        return max(now_sec - self._recovery_started_sec, 0.0)

    def _reset_candidate(self):
        self._candidate_fault = None
        self._candidate_started_sec = None

    @staticmethod
    def _with_metrics(decision, metrics):
        names = [name for name in metrics if name != 'now_sec']
        values = [metrics[name] for name in names]
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


class CameraHealthMonitor(Node):
    """Subscribe to C920 images and publish fixed-rate camera health."""

    def __init__(self):
        super().__init__('camera_health_monitor')
        self._declare_parameters()
        publish_rate_hz = self._float_parameter('publish_rate_hz')
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')
        self._image_topic = self._string_parameter('image_topic')
        health_topic = self._string_parameter('health_topic')
        self._evaluator = CameraHealthEvaluator(
            window_duration_sec=self._float_parameter('window_duration_sec'),
            min_samples=self._integer_parameter('min_samples'),
            stale_timeout_sec=self._float_parameter('stale_timeout_sec'),
            freeze_duration_sec=self._float_parameter('freeze_duration_sec'),
            fault_confirmation_sec=self._float_parameter(
                'fault_confirmation_sec'
            ),
            recovery_confirmation_sec=self._float_parameter(
                'recovery_confirmation_sec'
            ),
            dark_mean_gray_candidate=self._float_parameter(
                'dark_mean_gray_candidate'
            ),
            dark_p95_candidate=self._float_parameter(
                'dark_p95_candidate'
            ),
            bright_mean_gray_candidate=self._float_parameter(
                'bright_mean_gray_candidate'
            ),
            bright_p05_candidate=self._float_parameter(
                'bright_p05_candidate'
            ),
            bright_p95_candidate=self._float_parameter(
                'bright_p95_candidate'
            ),
            dark_ratio_candidate=self._float_parameter(
                'dark_ratio_candidate'
            ),
            bright_ratio_candidate=self._float_parameter(
                'bright_ratio_candidate'
            ),
            blur_reference_max_age_sec=self._float_parameter(
                'blur_reference_max_age_sec'
            ),
            blur_reference_laplacian_variance_min=self._float_parameter(
                'blur_reference_laplacian_variance_min'
            ),
            blur_reference_edge_density_min=self._float_parameter(
                'blur_reference_edge_density_min'
            ),
            blur_laplacian_variance_candidate=self._float_parameter(
                'blur_laplacian_variance_candidate'
            ),
            blur_edge_density_candidate=self._float_parameter(
                'blur_edge_density_candidate'
            ),
            blur_gray_std_min=self._float_parameter('blur_gray_std_min'),
            blur_entropy_min=self._float_parameter('blur_entropy_min'),
            low_information_reference_max_age_sec=self._float_parameter(
                'low_information_reference_max_age_sec'
            ),
            low_information_reference_edge_density_min=self._float_parameter(
                'low_information_reference_edge_density_min'
            ),
            low_information_reference_entropy_min=self._float_parameter(
                'low_information_reference_entropy_min'
            ),
            low_information_entropy_candidate=self._float_parameter(
                'low_information_entropy_candidate'
            ),
            low_information_edge_density_candidate=self._float_parameter(
                'low_information_edge_density_candidate'
            ),
            low_information_dark_ratio_min=self._float_parameter(
                'low_information_dark_ratio_min'
            ),
            low_information_gray_std_min=self._float_parameter(
                'low_information_gray_std_min'
            ),
        )
        self._bridge = CvBridge()
        self._health_publisher = self.create_publisher(
            SensorHealth, health_topic, 10
        )
        self._subscription = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._evaluator.start(self._now_sec())
        self._timer = self.create_timer(
            1.0 / publish_rate_hz, self._publish_health
        )

    def _declare_parameters(self):
        defaults = {
            'image_topic': DEFAULT_IMAGE_TOPIC,
            'health_topic': DEFAULT_HEALTH_TOPIC,
            'publish_rate_hz': 5.0,
            'window_duration_sec': 3.0,
            'min_samples': 3,
            'stale_timeout_sec': 1.0,
            'freeze_duration_sec': 2.0,
            'fault_confirmation_sec': 0.6,
            'recovery_confirmation_sec': 1.0,
            'dark_mean_gray_candidate': 6.0,
            'dark_p95_candidate': 8.0,
            'bright_mean_gray_candidate': 170.0,
            'bright_p05_candidate': 150.0,
            'bright_p95_candidate': 180.0,
            'dark_ratio_candidate': 0.90,
            'bright_ratio_candidate': 0.95,
            'blur_reference_max_age_sec': 3.0,
            'blur_reference_laplacian_variance_min': 100.0,
            'blur_reference_edge_density_min': 0.01,
            'blur_laplacian_variance_candidate': 10.0,
            'blur_edge_density_candidate': 0.001,
            'blur_gray_std_min': 20.0,
            'blur_entropy_min': 5.5,
            'low_information_reference_max_age_sec': 3.0,
            'low_information_reference_edge_density_min': 0.01,
            'low_information_reference_entropy_min': 5.2,
            'low_information_edge_density_candidate': 0.0005,
            'low_information_entropy_candidate': 5.8,
            'low_information_dark_ratio_min': 0.20,
            'low_information_gray_std_min': 20.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _on_image(self, message):
        received_sec = self._now_sec()
        stamp_sec = seconds_from_stamp(message.header.stamp)
        try:
            image, channel_order = image_message_to_numpy(
                message, self._bridge
            )
            self._evaluator.add_image(
                image,
                channel_order=channel_order,
                received_sec=received_sec,
                stamp_sec=stamp_sec,
            )
        except (TypeError, ValueError) as error:
            self._evaluator.record_rejected_image(
                received_sec=received_sec, stamp_sec=stamp_sec
            )
            self.get_logger().warning(f'Image sample rejected: {error}')

    def _publish_health(self):
        now_sec = self._now_sec()
        stamp = stamp_from_seconds(now_sec)
        self._health_publisher.publish(make_sensor_health(
            'camera',
            self._image_topic,
            stamp,
            self._evaluator.evaluate(now_sec),
        ))

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _float_parameter(self, name):
        return float(self.get_parameter(name).value)

    def _integer_parameter(self, name):
        return int(self.get_parameter(name).value)

    def _string_parameter(self, name):
        return str(self.get_parameter(name).value)


def main(args=None):
    """Run the fixed-rate camera health monitor."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraHealthMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
