"""Build a leakage-safe window dataset from physical-disturbance rosbags.

The feature table contains runtime-available wheel, IMU, and LiDAR signals.
Independent Gazebo pose is used only to create labels and a separate audit
table; no ground-truth value is admitted to the feature allowlist.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
from math import atan2, isfinite
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .scan_matching import (
    PlanarScanMatcher,
    ScanMatcherConfig,
    laser_scan_points,
    scan_observability,
)


METADATA_COLUMNS = (
    'window_id',
    'run_id',
    'window_start_sec',
    'window_end_sec',
)

RESIDUAL_NAMES = (
    'wheel_lidar_vx',
    'wheel_lidar_vx_normalized',
    'wheel_imu_yaw',
    'wheel_lidar_yaw',
    'imu_lidar_yaw',
)

FEATURE_COLUMNS = tuple(
    column
    for name in RESIDUAL_NAMES
    for column in (
        f'{name}_residual_mean',
        f'{name}_residual_abs_mean',
        f'{name}_residual_abs_max',
        f'{name}_residual_std',
        f'{name}_residual_persistence',
    )
) + (
    'wheel_vx_abs_mean',
    'wheel_yaw_rate_abs_mean',
    'imu_yaw_rate_abs_mean',
    'lidar_vx_abs_mean',
    'lidar_yaw_rate_abs_mean',
    'imu_horizontal_accel_rms',
    'imu_horizontal_accel_std',
    'imu_horizontal_accel_mad',
    'imu_roll_pitch_gyro_energy',
    'icp_valid_fraction',
    'icp_rmse_mean',
    'icp_rmse_max',
    'icp_rmse_std',
    'icp_inlier_ratio_mean',
    'icp_inlier_ratio_min',
    'icp_inlier_ratio_std',
    'icp_observability_mean',
    'icp_observability_min',
    'icp_observability_std',
)

LABEL_COLUMNS = (
    'wheel_translation_reliable',
    'wheel_rotation_reliable',
    'imu_yaw_rate_reliable',
    'lidar_translation_reliable',
)

DATASET_SCHEMA_VERSION = 2
SCAN_MATCHER_ALGORITHM = 'robust_point_to_line_v2'


@dataclass(frozen=True)
class DatasetConfig:
    """Fixed extraction and independent-GT labelling contract."""

    window_sec: float = 0.4
    stride_sec: float = 0.2
    gt_derivative_radius_sec: float = 0.10
    wheel_translation_tolerance_mps: float = 0.06
    wheel_rotation_tolerance_radps: float = 0.08
    imu_yaw_rate_tolerance_radps: float = 0.08
    lidar_translation_tolerance_mps: float = 0.12
    label_min_within_fraction: float = 0.80
    label_min_lidar_valid_fraction: float = 0.50
    minimum_wheel_samples: int = 5
    minimum_imu_samples: int = 10
    minimum_lidar_samples: int = 2
    nominal_wheel_vx_variance: float = 0.0025
    minimum_lidar_vx_variance: float = 0.0025
    vx_residual_persistence_mps: float = 0.08
    normalized_vx_persistence: float = 3.0
    yaw_residual_persistence_radps: float = 0.10

    def __post_init__(self) -> None:
        if not 0.3 <= self.window_sec <= 0.5:
            raise ValueError('window_sec must remain in the declared 0.3-0.5 s range')
        if not 0.0 < self.stride_sec <= self.window_sec:
            raise ValueError('stride_sec must be in (0, window_sec]')
        if not 0.0 < self.label_min_within_fraction <= 1.0:
            raise ValueError('label_min_within_fraction must be in (0, 1]')
        if not 0.0 < self.label_min_lidar_valid_fraction <= 1.0:
            raise ValueError('label_min_lidar_valid_fraction must be in (0, 1]')


@dataclass
class BagSignals:
    """Only signals needed by the offline dataset builder."""

    run_id: str
    command: list[tuple[float, float, float]]
    wheel: list[tuple[float, float, float, float]]
    imu: list[tuple[float, float, float, float, float, float]]
    ground_truth: list[tuple[float, float, float, float]]
    scans: list[tuple[float, Any]]


@dataclass
class LidarSample:
    """One adjacent-scan motion estimate and its quality metadata."""

    stamp_sec: float
    valid: bool
    vx_mps: float = 0.0
    yaw_rate_radps: float = 0.0
    vx_variance: float = 0.0
    rmse_m: float = 0.0
    inlier_ratio: float = 0.0
    observability: float = 0.0


@dataclass
class TruthSeries:
    """Independent world pose differentiated into body-frame motion."""

    time: np.ndarray
    vx_body: np.ndarray
    yaw_rate: np.ndarray


def _stamp_sec(message: Any) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _yaw_from_quaternion(orientation: Any) -> float:
    return atan2(
        2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def read_bag(bag_path: Path) -> BagSignals:
    """Deserialize the small topic allowlist from one MCAP rosbag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_path = Path(bag_path)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    required = {
        '/clock',
        '/cmd_vel',
        '/odom',
        '/imu/data',
        '/scan',
        '/evaluation/ground_truth_pose',
    }
    missing = sorted(required - topic_types.keys())
    if missing:
        raise ValueError(f'{bag_path.name} is missing required topics: {missing}')
    message_types = {
        topic: get_message(topic_types[topic]) for topic in required
    }

    command: list[tuple[float, float, float]] = []
    wheel: list[tuple[float, float, float, float]] = []
    imu: list[tuple[float, float, float, float, float, float]] = []
    ground_truth: list[tuple[float, float, float, float]] = []
    scans: list[tuple[float, Any]] = []
    clock_sec: float | None = None
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic == '/clock':
            clock_sec = (
                float(message.clock.sec)
                + float(message.clock.nanosec) * 1.0e-9
            )
        elif topic == '/cmd_vel':
            if clock_sec is not None:
                command.append((
                    clock_sec,
                    float(message.linear.x),
                    float(message.angular.z),
                ))
        elif topic == '/odom':
            vx_variance = float(message.twist.covariance[0])
            wheel.append((
                _stamp_sec(message),
                float(message.twist.twist.linear.x),
                float(message.twist.twist.angular.z),
                vx_variance,
            ))
        elif topic == '/imu/data':
            imu.append((
                _stamp_sec(message),
                float(message.angular_velocity.z),
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
            ))
        elif topic == '/evaluation/ground_truth_pose':
            ground_truth.append((
                _stamp_sec(message),
                float(message.pose.position.x),
                float(message.pose.position.y),
                _yaw_from_quaternion(message.pose.orientation),
            ))
        elif topic == '/scan':
            scans.append((_stamp_sec(message), message))

    signals = BagSignals(
        run_id=bag_path.name,
        command=command,
        wheel=wheel,
        imu=imu,
        ground_truth=ground_truth,
        scans=scans,
    )
    for name in ('command', 'wheel', 'imu', 'ground_truth', 'scans'):
        if not getattr(signals, name):
            raise ValueError(f'{bag_path.name} has no usable {name} messages')
    return signals


def _strictly_increasing_rows(rows: Iterable[tuple]) -> np.ndarray:
    values = np.asarray(list(rows), dtype=float)
    order = np.argsort(values[:, 0], kind='stable')
    values = values[order]
    unique = np.concatenate(([True], np.diff(values[:, 0]) > 1.0e-9))
    return values[unique]


def _local_slope(time: np.ndarray, values: np.ndarray, radius: float) -> np.ndarray:
    slopes = np.zeros_like(values)
    for index, center in enumerate(time):
        first = int(np.searchsorted(time, center - radius, side='left'))
        last = int(np.searchsorted(time, center + radius, side='right'))
        local_time = time[first:last] - center
        local_values = values[first:last]
        if local_time.size < 3:
            slopes[index] = np.nan
            continue
        denominator = float(np.dot(local_time, local_time))
        if denominator <= 1.0e-12:
            slopes[index] = np.nan
            continue
        slopes[index] = float(
            np.dot(local_time, local_values - np.mean(local_values))
            / denominator
        )
    return slopes


def ground_truth_motion(
    poses: Iterable[tuple[float, float, float, float]],
    derivative_radius_sec: float,
) -> TruthSeries:
    """Differentiate independent pose without using wheel or IMU signals."""
    pose = _strictly_increasing_rows(poses)
    if pose.shape[0] < 5:
        raise ValueError('at least five ground-truth poses are required')
    time, x_value, y_value = pose[:, 0], pose[:, 1], pose[:, 2]
    yaw = np.unwrap(pose[:, 3])
    vx_world = _local_slope(time, x_value, derivative_radius_sec)
    vy_world = _local_slope(time, y_value, derivative_radius_sec)
    yaw_rate = _local_slope(time, yaw, derivative_radius_sec)
    vx_body = np.cos(yaw) * vx_world + np.sin(yaw) * vy_world
    valid = np.isfinite(vx_body) & np.isfinite(yaw_rate)
    return TruthSeries(time=time[valid], vx_body=vx_body[valid], yaw_rate=yaw_rate[valid])


def extract_lidar_samples(
    scans: Iterable[tuple[float, Any]],
    config: DatasetConfig,
) -> list[LidarSample]:
    """Run the same robust ICP core used by the runtime LiDAR fallback."""
    matcher = PlanarScanMatcher(ScanMatcherConfig())
    output: list[LidarSample] = []
    previous_points = None
    previous_stamp = None
    for stamp_sec, scan in sorted(scans, key=lambda item: item[0]):
        points = laser_scan_points(
            scan.ranges,
            angle_min=float(scan.angle_min),
            angle_increment=float(scan.angle_increment),
            range_min=float(scan.range_min),
            range_max=float(scan.range_max),
            max_points=matcher.config.max_points,
        )
        if points is None:
            previous_points = None
            previous_stamp = None
            continue
        observability = scan_observability(points)
        if previous_points is None or previous_stamp is None:
            previous_points, previous_stamp = points, stamp_sec
            continue
        interval = stamp_sec - previous_stamp
        if interval <= 0.0 or interval > 0.50:
            previous_points, previous_stamp = points, stamp_sec
            continue
        match = matcher.match(previous_points, points)
        previous_points, previous_stamp = points, stamp_sec
        if match is None:
            output.append(LidarSample(
                stamp_sec=stamp_sec,
                valid=False,
                observability=observability,
            ))
            continue
        forward, _ = match.translation_current_frame
        output.append(LidarSample(
            stamp_sec=stamp_sec,
            valid=True,
            vx_mps=float(forward / interval),
            yaw_rate_radps=float(match.rotation_rad / interval),
            vx_variance=max(
                config.minimum_lidar_vx_variance,
                float((match.rmse_m / interval) ** 2),
            ),
            rmse_m=float(match.rmse_m),
            inlier_ratio=float(match.inlier_ratio),
            observability=observability,
        ))
    return output


def aggregate_residual(values: Iterable[float], threshold: float) -> dict[str, float]:
    """Summarize signed residual and its within-window persistence."""
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return {
            'mean': 0.0,
            'abs_mean': 0.0,
            'abs_max': 0.0,
            'std': 0.0,
            'persistence': 0.0,
        }
    absolute = np.abs(array)
    return {
        'mean': float(np.mean(array)),
        'abs_mean': float(np.mean(absolute)),
        'abs_max': float(np.max(absolute)),
        'std': float(np.std(array)),
        'persistence': float(np.mean(absolute > threshold)),
    }


def _simple_stats(values: Iterable[float]) -> tuple[float, float, float]:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return 0.0, 0.0, 0.0
    return float(np.mean(array)), float(np.max(array)), float(np.std(array))


def _mean_min_std(values: Iterable[float]) -> tuple[float, float, float]:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return 0.0, 0.0, 0.0
    return float(np.mean(array)), float(np.min(array)), float(np.std(array))


def _interp(time: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    return np.interp(query, time, values, left=np.nan, right=np.nan)


def _window(array: np.ndarray, start: float, end: float) -> np.ndarray:
    return array[(array[:, 0] >= start) & (array[:, 0] < end)]


def _label_audit(
    errors: np.ndarray,
    *,
    tolerance: float,
    sample_count: int,
    minimum_samples: int,
    minimum_within_fraction: float,
    availability_fraction: float = 1.0,
    minimum_availability_fraction: float = 0.0,
) -> tuple[int, dict[str, float]]:
    errors = np.asarray(errors, dtype=float)
    errors = errors[np.isfinite(errors)]
    absolute = np.abs(errors)
    if absolute.size:
        mean = float(np.mean(absolute))
        p90 = float(np.quantile(absolute, 0.90))
        rmse = float(np.sqrt(np.mean(np.square(errors))))
        within = float(np.mean(absolute <= tolerance))
    else:
        mean = p90 = rmse = within = 0.0
    enough = (
        sample_count >= minimum_samples
        and availability_fraction >= minimum_availability_fraction
    )
    reliable = int(
        enough
        and rmse <= tolerance
        and within >= minimum_within_fraction
    )
    return reliable, {
        'sample_count': float(sample_count),
        'availability_fraction': float(availability_fraction),
        'error_abs_mean': mean,
        'error_abs_p90': p90,
        'error_rmse': rmse,
        'within_tolerance_fraction': within,
        'tolerance': float(tolerance),
    }


def _put_residual(
    features: dict[str, float],
    name: str,
    values: Iterable[float],
    threshold: float,
) -> None:
    for statistic, value in aggregate_residual(values, threshold).items():
        features[f'{name}_residual_{statistic}'] = value


def build_run_windows(
    signals: BagSignals,
    config: DatasetConfig,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    """Create feature/label rows and GT-only audit rows for one run."""
    wheel = _strictly_increasing_rows(signals.wheel)
    imu = _strictly_increasing_rows(signals.imu)
    truth = ground_truth_motion(
        signals.ground_truth, config.gt_derivative_radius_sec
    )
    lidar_samples = extract_lidar_samples(signals.scans, config)
    if not lidar_samples:
        raise ValueError(f'{signals.run_id} produced no adjacent-scan records')

    active_commands = [
        command for command in signals.command
        if abs(command[1]) > 1.0e-4 or abs(command[2]) > 1.0e-4
    ]
    if not active_commands:
        raise ValueError(f'{signals.run_id} contains no active route commands')
    start = max(
        active_commands[0][0], wheel[0, 0], imu[0, 0], truth.time[0]
    )
    end = min(
        active_commands[-1][0] + config.window_sec,
        wheel[-1, 0],
        imu[-1, 0],
        truth.time[-1],
    )
    if end - start < config.window_sec:
        raise ValueError(f'{signals.run_id} has no complete active-route window')

    lidar_time = np.asarray([item.stamp_sec for item in lidar_samples])
    valid_lidar = [item for item in lidar_samples if item.valid]
    valid_lidar_time = np.asarray([item.stamp_sec for item in valid_lidar])
    valid_lidar_vx = np.asarray([item.vx_mps for item in valid_lidar])
    valid_lidar_yaw = np.asarray([item.yaw_rate_radps for item in valid_lidar])
    wheel_at_lidar_vx = _interp(wheel[:, 0], wheel[:, 1], valid_lidar_time)
    wheel_at_lidar_yaw = _interp(wheel[:, 0], wheel[:, 2], valid_lidar_time)
    wheel_at_lidar_var = _interp(wheel[:, 0], wheel[:, 3], valid_lidar_time)
    imu_at_lidar_yaw = _interp(imu[:, 0], imu[:, 1], valid_lidar_time)
    lidar_variance = np.asarray([item.vx_variance for item in valid_lidar])

    rows: list[dict[str, Any]] = []
    audits: list[dict[str, Any]] = []
    starts = np.arange(start, end - config.window_sec + 1.0e-9, config.stride_sec)
    for index, window_start in enumerate(starts):
        window_end = float(window_start + config.window_sec)
        wheel_window = _window(wheel, window_start, window_end)
        imu_window = _window(imu, window_start, window_end)
        lidar_mask = (
            (valid_lidar_time >= window_start)
            & (valid_lidar_time < window_end)
        )
        quality_mask = (lidar_time >= window_start) & (lidar_time < window_end)
        quality = [
            sample for sample, selected in zip(lidar_samples, quality_mask)
            if selected
        ]
        valid_quality = [sample for sample in quality if sample.valid]

        wheel_imu_yaw = (
            wheel_window[:, 2]
            - _interp(imu[:, 0], imu[:, 1], wheel_window[:, 0])
        )
        wheel_lidar_vx = wheel_at_lidar_vx[lidar_mask] - valid_lidar_vx[lidar_mask]
        combined_variance = (
            np.maximum(
                wheel_at_lidar_var[lidar_mask],
                config.nominal_wheel_vx_variance,
            )
            + lidar_variance[lidar_mask]
        )
        normalized_vx = wheel_lidar_vx / np.sqrt(combined_variance)
        wheel_lidar_yaw = (
            wheel_at_lidar_yaw[lidar_mask] - valid_lidar_yaw[lidar_mask]
        )
        imu_lidar_yaw = (
            imu_at_lidar_yaw[lidar_mask] - valid_lidar_yaw[lidar_mask]
        )

        features: dict[str, float] = {}
        _put_residual(
            features, 'wheel_lidar_vx', wheel_lidar_vx,
            config.vx_residual_persistence_mps,
        )
        _put_residual(
            features, 'wheel_lidar_vx_normalized', normalized_vx,
            config.normalized_vx_persistence,
        )
        _put_residual(
            features, 'wheel_imu_yaw', wheel_imu_yaw,
            config.yaw_residual_persistence_radps,
        )
        _put_residual(
            features, 'wheel_lidar_yaw', wheel_lidar_yaw,
            config.yaw_residual_persistence_radps,
        )
        _put_residual(
            features, 'imu_lidar_yaw', imu_lidar_yaw,
            config.yaw_residual_persistence_radps,
        )

        features['wheel_vx_abs_mean'] = float(
            np.mean(np.abs(wheel_window[:, 1]))
        ) if wheel_window.size else 0.0
        features['wheel_yaw_rate_abs_mean'] = float(
            np.mean(np.abs(wheel_window[:, 2]))
        ) if wheel_window.size else 0.0
        features['imu_yaw_rate_abs_mean'] = float(
            np.mean(np.abs(imu_window[:, 1]))
        ) if imu_window.size else 0.0
        features['lidar_vx_abs_mean'] = float(
            np.mean(np.abs(valid_lidar_vx[lidar_mask]))
        ) if np.any(lidar_mask) else 0.0
        features['lidar_yaw_rate_abs_mean'] = float(
            np.mean(np.abs(valid_lidar_yaw[lidar_mask]))
        ) if np.any(lidar_mask) else 0.0

        horizontal = np.hypot(imu_window[:, 2], imu_window[:, 3])
        features['imu_horizontal_accel_rms'] = float(
            np.sqrt(np.mean(np.square(horizontal)))
        ) if horizontal.size else 0.0
        features['imu_horizontal_accel_std'] = float(
            np.std(horizontal)
        ) if horizontal.size else 0.0
        features['imu_horizontal_accel_mad'] = float(
            np.median(np.abs(horizontal - np.median(horizontal)))
        ) if horizontal.size else 0.0
        features['imu_roll_pitch_gyro_energy'] = float(np.mean(
            np.square(imu_window[:, 4]) + np.square(imu_window[:, 5])
        )) if imu_window.size else 0.0

        features['icp_valid_fraction'] = (
            float(len(valid_quality) / len(quality)) if quality else 0.0
        )
        rmse_mean, rmse_max, rmse_std = _simple_stats(
            item.rmse_m for item in valid_quality
        )
        features.update({
            'icp_rmse_mean': rmse_mean,
            'icp_rmse_max': rmse_max,
            'icp_rmse_std': rmse_std,
        })
        inlier_mean, inlier_min, inlier_std = _mean_min_std(
            item.inlier_ratio for item in valid_quality
        )
        features.update({
            'icp_inlier_ratio_mean': inlier_mean,
            'icp_inlier_ratio_min': inlier_min,
            'icp_inlier_ratio_std': inlier_std,
        })
        obs_mean, obs_min, obs_std = _mean_min_std(
            item.observability for item in quality
        )
        features.update({
            'icp_observability_mean': obs_mean,
            'icp_observability_min': obs_min,
            'icp_observability_std': obs_std,
        })

        truth_wheel_vx = _interp(
            truth.time, truth.vx_body, wheel_window[:, 0]
        )
        truth_wheel_yaw = _interp(
            truth.time, truth.yaw_rate, wheel_window[:, 0]
        )
        truth_imu_yaw = _interp(
            truth.time, truth.yaw_rate, imu_window[:, 0]
        )
        truth_lidar_vx = _interp(
            truth.time, truth.vx_body, valid_lidar_time[lidar_mask]
        )
        labels_and_audits = {
            'wheel_translation': _label_audit(
                wheel_window[:, 1] - truth_wheel_vx,
                tolerance=config.wheel_translation_tolerance_mps,
                sample_count=wheel_window.shape[0],
                minimum_samples=config.minimum_wheel_samples,
                minimum_within_fraction=config.label_min_within_fraction,
            ),
            'wheel_rotation': _label_audit(
                wheel_window[:, 2] - truth_wheel_yaw,
                tolerance=config.wheel_rotation_tolerance_radps,
                sample_count=wheel_window.shape[0],
                minimum_samples=config.minimum_wheel_samples,
                minimum_within_fraction=config.label_min_within_fraction,
            ),
            'imu_yaw_rate': _label_audit(
                imu_window[:, 1] - truth_imu_yaw,
                tolerance=config.imu_yaw_rate_tolerance_radps,
                sample_count=imu_window.shape[0],
                minimum_samples=config.minimum_imu_samples,
                minimum_within_fraction=config.label_min_within_fraction,
            ),
            'lidar_translation': _label_audit(
                valid_lidar_vx[lidar_mask] - truth_lidar_vx,
                tolerance=config.lidar_translation_tolerance_mps,
                sample_count=int(np.count_nonzero(lidar_mask)),
                minimum_samples=config.minimum_lidar_samples,
                minimum_within_fraction=config.label_min_within_fraction,
                availability_fraction=features['icp_valid_fraction'],
                minimum_availability_fraction=(
                    config.label_min_lidar_valid_fraction
                ),
            ),
        }
        window_id = f'{signals.run_id}:{index:04d}'
        row: dict[str, Any] = {
            'window_id': window_id,
            'run_id': signals.run_id,
            'window_start_sec': float(window_start),
            'window_end_sec': window_end,
            **features,
            **{
                f'{name}_reliable': result[0]
                for name, result in labels_and_audits.items()
            },
        }
        if set(features) != set(FEATURE_COLUMNS):
            raise RuntimeError('feature schema and extractor output diverged')
        if not all(isfinite(float(row[name])) for name in FEATURE_COLUMNS):
            raise RuntimeError(f'{window_id} contains a non-finite feature')
        rows.append(row)

        audit: dict[str, Any] = {
            'window_id': window_id,
            'run_id': signals.run_id,
            'window_start_sec': float(window_start),
            'window_end_sec': window_end,
        }
        for name, (_, details) in labels_and_audits.items():
            for key, value in details.items():
                audit[f'{name}_{key}'] = value
        audits.append(audit)

    run_summary = {
        'run_id': signals.run_id,
        'route_start_sec': float(start),
        'route_end_sec': float(end),
        'window_count': len(rows),
        'lidar_pair_count': len(lidar_samples),
        'lidar_valid_pair_count': sum(item.valid for item in lidar_samples),
        'label_unreliable_count': {
            label: sum(int(row[label]) == 0 for row in rows)
            for label in LABEL_COLUMNS
        },
    }
    return rows, audits, run_summary


def _write_csv(path: Path, rows: list[dict[str, Any]], columns: Iterable[str]) -> None:
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(columns))
        writer.writeheader()
        writer.writerows(rows)


def discover_bags(input_root: Path) -> list[Path]:
    """Return direct child bag directories in deterministic order."""
    bags = sorted(
        path.parent for path in Path(input_root).glob('*/metadata.yaml')
    )
    if not bags:
        raise ValueError(f'no rosbag metadata found below {input_root}')
    return bags


def build_dataset(input_root: Path, output_dir: Path, config: DatasetConfig) -> dict[str, Any]:
    """Build CSV tables plus the feature/label manifest; never train a model."""
    input_root = Path(input_root).resolve()
    output_dir = Path(output_dir).resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f'refusing to overwrite non-empty {output_dir}')
    output_dir.mkdir(parents=True, exist_ok=True)

    all_rows: list[dict[str, Any]] = []
    all_audits: list[dict[str, Any]] = []
    run_summaries: list[dict[str, Any]] = []
    bags = discover_bags(input_root)
    for bag in bags:
        rows, audits, summary = build_run_windows(read_bag(bag), config)
        all_rows.extend(rows)
        all_audits.extend(audits)
        run_summaries.append(summary)

    if not all_rows:
        raise RuntimeError('dataset builder produced no windows')
    dataset_columns = METADATA_COLUMNS + FEATURE_COLUMNS + LABEL_COLUMNS
    audit_columns = tuple(all_audits[0].keys())
    _write_csv(output_dir / 'windows.csv', all_rows, dataset_columns)
    _write_csv(output_dir / 'label_audit_gt_only.csv', all_audits, audit_columns)

    summary = {
        'dataset_schema_version': DATASET_SCHEMA_VERSION,
        'window_count': len(all_rows),
        'run_count': len(run_summaries),
        'feature_count': len(FEATURE_COLUMNS),
        'label_balance': {
            label: {
                'reliable': sum(int(row[label]) == 1 for row in all_rows),
                'unreliable': sum(int(row[label]) == 0 for row in all_rows),
            }
            for label in LABEL_COLUMNS
        },
        'runs': run_summaries,
    }
    manifest = {
        'dataset_schema_version': DATASET_SCHEMA_VERSION,
        'purpose': 'pre-Random-Forest physical measurement reliability dataset',
        'scan_matcher_algorithm': SCAN_MATCHER_ALGORITHM,
        'input_root': str(input_root),
        'input_runs': [bag.name for bag in bags],
        'config': asdict(config),
        'metadata_columns': list(METADATA_COLUMNS),
        'feature_columns': list(FEATURE_COLUMNS),
        'label_columns': list(LABEL_COLUMNS),
        'ground_truth_topic': '/evaluation/ground_truth_pose',
        'ground_truth_contract': (
            'Independent GT is used only for labels and label_audit_gt_only.csv; '
            'it is forbidden from feature inputs and later runtime inference.'
        ),
        'missing_value_contract': (
            'All features are finite. Missing ICP aggregates are zero-filled '
            'only together with icp_valid_fraction=0; the validity feature must '
            'remain in every trained model.'
        ),
        'split_contract': (
            'Do not randomly split overlapping windows. Future validation must '
            'group by complete run and hold out disturbance intensities.'
        ),
        'training_status': 'NOT_STARTED',
        'training_forbidden_files': ['label_audit_gt_only.csv'],
    }
    (output_dir / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'summary.json').write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    return summary


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description='Build windowed RF features and independent-GT labels.'
    )
    parser.add_argument('--input-root', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--window-sec', type=float, default=0.4)
    parser.add_argument('--stride-sec', type=float, default=0.2)
    arguments = parser.parse_args(argv)
    config = DatasetConfig(
        window_sec=arguments.window_sec,
        stride_sec=arguments.stride_sec,
    )
    summary = build_dataset(arguments.input_root, arguments.output_dir, config)
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
