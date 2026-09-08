"""GT-free online feature extraction and RF reliability inference."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Mapping

import joblib
import numpy as np

from .fusion_policy import ReliabilityScores


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
TARGETS = (
    'wheel_translation_reliable',
    'wheel_rotation_reliable',
    'imu_yaw_rate_reliable',
)


@dataclass(frozen=True)
class WheelSample:
    stamp_sec: float
    vx_mps: float
    yaw_rate_radps: float
    vx_variance: float


@dataclass(frozen=True)
class ImuSample:
    stamp_sec: float
    yaw_rate_radps: float
    accel_x_mps2: float
    accel_y_mps2: float
    gyro_x_radps: float
    gyro_y_radps: float


@dataclass(frozen=True)
class LidarSample:
    stamp_sec: float
    valid: bool
    vx_mps: float = 0.0
    yaw_rate_radps: float = 0.0
    vx_variance: float = 0.0
    rmse_m: float = 0.0
    inlier_ratio: float = 0.0
    observability: float = 0.0


class OnlineFeatureWindow:
    """Maintain the schema-v2 0.4 s runtime signal window."""

    def __init__(
        self,
        *,
        window_sec: float = 0.4,
        minimum_wheel_samples: int = 5,
        minimum_imu_samples: int = 10,
        minimum_lidar_samples: int = 2,
        nominal_wheel_vx_variance: float = 0.0025,
    ) -> None:
        if not 0.3 <= window_sec <= 0.5:
            raise ValueError('window_sec must be in [0.3, 0.5]')
        self.window_sec = float(window_sec)
        self.minimum_wheel_samples = int(minimum_wheel_samples)
        self.minimum_imu_samples = int(minimum_imu_samples)
        self.minimum_lidar_samples = int(minimum_lidar_samples)
        self.nominal_wheel_vx_variance = float(nominal_wheel_vx_variance)
        self._wheel: deque[WheelSample] = deque()
        self._imu: deque[ImuSample] = deque()
        self._lidar: deque[LidarSample] = deque()

    def add_wheel(self, sample: WheelSample) -> None:
        self._append(self._wheel, sample)

    def add_imu(self, sample: ImuSample) -> None:
        self._append(self._imu, sample)

    def add_lidar(self, sample: LidarSample) -> None:
        self._append(self._lidar, sample)

    def feature_vector(self, end_sec: float) -> dict[str, float] | None:
        """Return one exact schema-v2 feature row when all streams are ready."""
        start_sec = float(end_sec) - self.window_sec
        wheel_samples = [x for x in self._wheel if start_sec <= x.stamp_sec <= end_sec]
        imu_samples = [x for x in self._imu if start_sec <= x.stamp_sec <= end_sec]
        wheel_context = [x for x in self._wheel if x.stamp_sec <= end_sec]
        imu_context = [x for x in self._imu if x.stamp_sec <= end_sec]
        lidar_samples = [x for x in self._lidar if start_sec <= x.stamp_sec <= end_sec]
        valid_lidar = [x for x in lidar_samples if x.valid]
        if (
            len(wheel_samples) < self.minimum_wheel_samples
            or len(imu_samples) < self.minimum_imu_samples
            or len(valid_lidar) < self.minimum_lidar_samples
        ):
            return None

        wheel = np.asarray([
            (x.stamp_sec, x.vx_mps, x.yaw_rate_radps, x.vx_variance)
            for x in wheel_samples
        ], dtype=float)
        wheel_for_interp = np.asarray([
            (x.stamp_sec, x.vx_mps, x.yaw_rate_radps, x.vx_variance)
            for x in wheel_context
        ], dtype=float)
        imu = np.asarray([
            (
                x.stamp_sec,
                x.yaw_rate_radps,
                x.accel_x_mps2,
                x.accel_y_mps2,
                x.gyro_x_radps,
                x.gyro_y_radps,
            )
            for x in imu_samples
        ], dtype=float)
        imu_for_interp = np.asarray([
            (x.stamp_sec, x.yaw_rate_radps)
            for x in imu_context
        ], dtype=float)
        lidar_time = np.asarray([x.stamp_sec for x in valid_lidar], dtype=float)
        lidar_vx = np.asarray([x.vx_mps for x in valid_lidar], dtype=float)
        lidar_yaw = np.asarray([x.yaw_rate_radps for x in valid_lidar], dtype=float)
        lidar_variance = np.asarray([x.vx_variance for x in valid_lidar], dtype=float)
        wheel_at_lidar_vx = _interp(
            wheel_for_interp[:, 0], wheel_for_interp[:, 1], lidar_time
        )
        wheel_at_lidar_yaw = _interp(
            wheel_for_interp[:, 0], wheel_for_interp[:, 2], lidar_time
        )
        wheel_at_lidar_variance = _interp(
            wheel_for_interp[:, 0], wheel_for_interp[:, 3], lidar_time
        )
        imu_at_lidar_yaw = _interp(
            imu_for_interp[:, 0], imu_for_interp[:, 1], lidar_time
        )

        wheel_lidar_vx = wheel_at_lidar_vx - lidar_vx
        normalized_vx = wheel_lidar_vx / np.sqrt(
            np.maximum(wheel_at_lidar_variance, self.nominal_wheel_vx_variance)
            + lidar_variance
        )
        residuals = {
            'wheel_lidar_vx': (wheel_lidar_vx, 0.08),
            'wheel_lidar_vx_normalized': (normalized_vx, 3.0),
            'wheel_imu_yaw': (
                wheel[:, 2] - _interp(
                    imu_for_interp[:, 0], imu_for_interp[:, 1], wheel[:, 0]
                ),
                0.10,
            ),
            'wheel_lidar_yaw': (wheel_at_lidar_yaw - lidar_yaw, 0.10),
            'imu_lidar_yaw': (imu_at_lidar_yaw - lidar_yaw, 0.10),
        }
        features: dict[str, float] = {}
        for name, (values, threshold) in residuals.items():
            for statistic, value in _aggregate(values, threshold).items():
                features[f'{name}_residual_{statistic}'] = value

        features.update({
            'wheel_vx_abs_mean': float(np.mean(np.abs(wheel[:, 1]))),
            'wheel_yaw_rate_abs_mean': float(np.mean(np.abs(wheel[:, 2]))),
            'imu_yaw_rate_abs_mean': float(np.mean(np.abs(imu[:, 1]))),
            'lidar_vx_abs_mean': float(np.mean(np.abs(lidar_vx))),
            'lidar_yaw_rate_abs_mean': float(np.mean(np.abs(lidar_yaw))),
        })
        horizontal = np.hypot(imu[:, 2], imu[:, 3])
        features.update({
            'imu_horizontal_accel_rms': float(np.sqrt(np.mean(np.square(horizontal)))),
            'imu_horizontal_accel_std': float(np.std(horizontal)),
            'imu_horizontal_accel_mad': float(
                np.median(np.abs(horizontal - np.median(horizontal)))
            ),
            'imu_roll_pitch_gyro_energy': float(np.mean(
                np.square(imu[:, 4]) + np.square(imu[:, 5])
            )),
            'icp_valid_fraction': float(len(valid_lidar) / len(lidar_samples)),
        })
        rmse_mean, rmse_max, rmse_std = _mean_max_std(
            x.rmse_m for x in valid_lidar
        )
        inlier_mean, inlier_min, inlier_std = _mean_min_std(
            x.inlier_ratio for x in valid_lidar
        )
        obs_mean, obs_min, obs_std = _mean_min_std(
            x.observability for x in lidar_samples
        )
        features.update({
            'icp_rmse_mean': rmse_mean,
            'icp_rmse_max': rmse_max,
            'icp_rmse_std': rmse_std,
            'icp_inlier_ratio_mean': inlier_mean,
            'icp_inlier_ratio_min': inlier_min,
            'icp_inlier_ratio_std': inlier_std,
            'icp_observability_mean': obs_mean,
            'icp_observability_min': obs_min,
            'icp_observability_std': obs_std,
        })
        if tuple(features) != FEATURE_COLUMNS:
            raise RuntimeError('online feature order diverged from schema-v2')
        if not all(isfinite(value) for value in features.values()):
            return None
        return features

    def _append(self, buffer: deque, sample: object) -> None:
        stamp_sec = float(sample.stamp_sec)
        if not isfinite(stamp_sec):
            return
        if buffer and stamp_sec <= buffer[-1].stamp_sec:
            if stamp_sec < buffer[-1].stamp_sec:
                buffer.clear()
            else:
                return
        buffer.append(sample)
        cutoff = stamp_sec - self.window_sec - 0.10
        while buffer and buffer[0].stamp_sec < cutoff:
            buffer.popleft()


class ReliabilityModelBundle:
    """Load exactly the three selected sklearn artifacts and emit class-1 proba."""

    def __init__(self, model_directory: Path) -> None:
        directory = Path(model_directory)
        self._estimators = {}
        for target in TARGETS:
            artifact = joblib.load(directory / f'{target}.joblib')
            if artifact.get('artifact_schema_version') != 1:
                raise ValueError(f'{target} artifact schema is unsupported')
            if artifact.get('dataset_schema_version') != 2:
                raise ValueError(f'{target} dataset schema is unsupported')
            if artifact.get('target') != target:
                raise ValueError(f'{target} artifact target mismatch')
            if tuple(artifact.get('feature_columns', ())) != FEATURE_COLUMNS:
                raise ValueError(f'{target} feature schema mismatch')
            estimator = artifact.get('estimator')
            if estimator is None or tuple(int(x) for x in estimator.classes_) != (0, 1):
                raise ValueError(f'{target} class contract mismatch')
            self._estimators[target] = estimator

    def predict(self, features: Mapping[str, float], lidar_translation: float) -> ReliabilityScores:
        """Predict continuous reliability without applying training thresholds."""
        if tuple(features) != FEATURE_COLUMNS:
            raise ValueError('runtime feature order mismatch')
        vector = np.asarray([[float(features[name]) for name in FEATURE_COLUMNS]])
        if not np.all(np.isfinite(vector)):
            raise ValueError('runtime features must be finite')
        probabilities = {}
        for target, estimator in self._estimators.items():
            probabilities[target] = float(estimator.predict_proba(vector)[0, 1])
        return ReliabilityScores(
            wheel_translation=probabilities['wheel_translation_reliable'],
            wheel_rotation=probabilities['wheel_rotation_reliable'],
            imu_yaw_rate=probabilities['imu_yaw_rate_reliable'],
            lidar_translation=float(lidar_translation),
            rf_ready=True,
        )


def lidar_quality_reliability(
    sample: LidarSample,
    *,
    max_rmse_m: float,
    min_inlier_ratio: float,
    min_observability: float,
) -> float:
    """Map the existing hard ICP gate's headroom to a bounded confidence."""
    if not sample.valid:
        return 0.0
    if (
        sample.rmse_m > max_rmse_m
        or sample.inlier_ratio < min_inlier_ratio
        or sample.observability < min_observability
    ):
        return 0.0
    rmse_score = 1.0 - 0.5 * sample.rmse_m / max_rmse_m
    inlier_score = 0.5 + 0.5 * (
        sample.inlier_ratio - min_inlier_ratio
    ) / (1.0 - min_inlier_ratio)
    observability_score = 0.5 + 0.5 * min(
        1.0,
        (sample.observability - min_observability) / (4.0 * min_observability),
    )
    return float(np.clip(min(rmse_score, inlier_score, observability_score), 0.0, 1.0))


def _interp(time: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    return np.interp(query, time, values, left=np.nan, right=np.nan)


def _aggregate(values, threshold: float) -> dict[str, float]:
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return {'mean': 0.0, 'abs_mean': 0.0, 'abs_max': 0.0, 'std': 0.0, 'persistence': 0.0}
    absolute = np.abs(array)
    return {
        'mean': float(np.mean(array)),
        'abs_mean': float(np.mean(absolute)),
        'abs_max': float(np.max(absolute)),
        'std': float(np.std(array)),
        'persistence': float(np.mean(absolute > threshold)),
    }


def _mean_max_std(values) -> tuple[float, float, float]:
    array = np.asarray(list(values), dtype=float)
    return float(np.mean(array)), float(np.max(array)), float(np.std(array))


def _mean_min_std(values) -> tuple[float, float, float]:
    array = np.asarray(list(values), dtype=float)
    return float(np.mean(array)), float(np.min(array)), float(np.std(array))
