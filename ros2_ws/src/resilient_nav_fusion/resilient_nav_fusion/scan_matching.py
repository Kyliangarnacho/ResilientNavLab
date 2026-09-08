"""Small ROS-free point-to-line scan matcher for LiDAR velocity fallback."""

from dataclasses import dataclass
from math import isfinite

import numpy as np


@dataclass(frozen=True)
class ScanMatcherConfig:
    """Conservative bounds for adjacent-scan robust ICP."""

    min_points: int = 40
    max_points: int = 180
    max_iterations: int = 12
    max_correspondence_distance_m: float = 0.35
    robust_mad_scale: float = 3.5
    robust_residual_floor_m: float = 0.02
    trim_fraction: float = 0.80
    min_inlier_ratio: float = 0.45
    max_rmse_m: float = 0.08
    max_translation_m: float = 0.20
    max_rotation_rad: float = 0.25
    convergence_translation_m: float = 1.0e-4
    convergence_rotation_rad: float = 1.0e-4
    normal_neighbor_max_distance_m: float = 0.50
    normal_max_curvature_ratio: float = 0.20
    min_point_to_line_observability: float = 0.05
    point_to_line_damping: float = 1.0e-6

    def __post_init__(self) -> None:
        """Reject configurations that could silently disable quality gates."""
        if self.min_points < 3 or self.max_points < self.min_points:
            raise ValueError('scan point limits are invalid')
        if self.max_iterations < 1:
            raise ValueError('max_iterations must be positive')
        positive = (
            self.max_correspondence_distance_m,
            self.robust_mad_scale,
            self.robust_residual_floor_m,
            self.max_rmse_m,
            self.max_translation_m,
            self.max_rotation_rad,
            self.convergence_translation_m,
            self.convergence_rotation_rad,
            self.normal_neighbor_max_distance_m,
            self.normal_max_curvature_ratio,
            self.min_point_to_line_observability,
            self.point_to_line_damping,
        )
        if not all(isfinite(value) and value > 0.0 for value in positive):
            raise ValueError(
                'scan matcher metric bounds must be finite and positive'
            )
        if not 0.0 < self.trim_fraction <= 1.0:
            raise ValueError('trim_fraction must be in (0, 1]')
        if not 0.0 < self.min_inlier_ratio <= 1.0:
            raise ValueError('min_inlier_ratio must be in (0, 1]')
        if not 0.0 < self.normal_max_curvature_ratio < 1.0:
            raise ValueError('normal_max_curvature_ratio must be in (0, 1)')
        if not 0.0 < self.min_point_to_line_observability <= 1.0:
            raise ValueError(
                'min_point_to_line_observability must be in (0, 1]'
            )


@dataclass(frozen=True)
class ScanMatchResult:
    """Motion from the previous sensor pose to the current sensor pose."""

    translation_previous_frame: tuple[float, float]
    rotation_rad: float
    rmse_m: float
    inlier_ratio: float
    inlier_count: int

    @property
    def translation_current_frame(self) -> tuple[float, float]:
        """Express the estimated translation in the current sensor frame."""
        cosine = float(np.cos(self.rotation_rad))
        sine = float(np.sin(self.rotation_rad))
        x_value, y_value = self.translation_previous_frame
        return (
            cosine * x_value + sine * y_value,
            -sine * x_value + cosine * y_value,
        )


def laser_scan_points(
    ranges,
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    max_points: int,
) -> np.ndarray | None:
    """Convert valid LaserScan ranges into a bounded planar point array."""
    if max_points < 3:
        raise ValueError('max_points must be at least 3')
    values = np.asarray(ranges, dtype=float)
    if values.ndim != 1 or values.size < 3:
        return None
    metadata = (angle_min, angle_increment, range_min, range_max)
    if not all(isfinite(value) for value in metadata):
        return None
    if angle_increment == 0.0 or range_min < 0.0 or range_max <= range_min:
        return None
    valid = np.isfinite(values) & (values >= range_min) & (values <= range_max)
    indices = np.flatnonzero(valid)
    if indices.size < 3:
        return None
    if indices.size > max_points:
        selected = np.linspace(0, indices.size - 1, max_points, dtype=int)
        indices = indices[selected]
    selected_ranges = values[indices]
    angles = angle_min + indices * angle_increment
    return np.column_stack((
        selected_ranges * np.cos(angles),
        selected_ranges * np.sin(angles),
    ))


def scan_observability(points: np.ndarray) -> float:
    """Estimate planar translation observability from local scan normals."""
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[0] < 5 or values.shape[1] != 2:
        return 0.0
    tangents = values[2:] - values[:-2]
    lengths = np.linalg.norm(tangents, axis=1)
    valid = np.isfinite(lengths) & (lengths > 0.01) & (lengths < 0.75)
    if int(np.count_nonzero(valid)) < 3:
        return 0.0
    tangents = tangents[valid] / lengths[valid, np.newaxis]
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    information = normals.T @ normals / float(normals.shape[0])
    eigenvalues = np.linalg.eigvalsh(information)
    largest = float(eigenvalues[-1])
    if largest <= 1.0e-12:
        return 0.0
    return float(np.clip(eigenvalues[0] / largest, 0.0, 1.0))


class PlanarScanMatcher:
    """Estimate adjacent-scan planar motion with quality-gated ICP."""

    def __init__(self, config: ScanMatcherConfig | None = None) -> None:
        """Create a matcher with explicit or conservative default bounds."""
        self.config = config or ScanMatcherConfig()

    def match(
        self,
        previous_points: np.ndarray,
        current_points: np.ndarray,
    ) -> ScanMatchResult | None:
        """Return sensor motion, or None when matching is unreliable."""
        previous = self._validated_points(previous_points)
        current = self._validated_points(current_points)
        if previous is None or current is None:
            return None

        surface_fit_available, surface_fit = self._point_to_line_fit(
            previous, current
        )
        if surface_fit_available:
            if surface_fit is None:
                return None
            rotation, translation, distances, raw_inlier_count = surface_fit
            return self._quality_gated_result(
                rotation,
                translation,
                distances,
                raw_inlier_count,
                current.shape[0],
            )

        # Point-to-point remains a compatibility fallback for unstructured
        # point sets. Laser scans with enough surface normals never use it:
        # an unobservable surface fit must fail closed instead of publishing
        # the near-zero local minimum that motivated this repair.
        rotation = np.eye(2)
        translation = np.zeros(2)
        for _ in range(self.config.max_iterations):
            transformed = current @ rotation.T + translation
            pairs = self._correspondences(previous, transformed)
            if pairs is None:
                return None
            source, target, _, _ = pairs
            update_rotation, update_translation = _rigid_transform(
                source, target
            )
            rotation = update_rotation @ rotation
            translation = update_rotation @ translation + update_translation
            update_angle = _rotation_angle(update_rotation)
            if (
                float(np.linalg.norm(update_translation))
                <= self.config.convergence_translation_m
                and abs(update_angle) <= self.config.convergence_rotation_rad
            ):
                break

        transformed = current @ rotation.T + translation
        pairs = self._correspondences(previous, transformed)
        if pairs is None:
            return None
        _, _, distances, raw_inlier_count = pairs
        return self._quality_gated_result(
            rotation,
            translation,
            distances,
            raw_inlier_count,
            current.shape[0],
        )

    def _quality_gated_result(
        self,
        rotation,
        translation,
        distances,
        raw_inlier_count,
        source_count,
    ):
        inlier_ratio = raw_inlier_count / float(source_count)
        rmse = float(np.sqrt(np.mean(np.square(distances))))
        angle = _rotation_angle(rotation)
        distance = float(np.linalg.norm(translation))
        if (
            inlier_ratio < self.config.min_inlier_ratio
            or rmse > self.config.max_rmse_m
            or distance > self.config.max_translation_m
            or abs(angle) > self.config.max_rotation_rad
        ):
            return None
        return ScanMatchResult(
            translation_previous_frame=(
                float(translation[0]),
                float(translation[1]),
            ),
            rotation_rad=angle,
            rmse_m=rmse,
            inlier_ratio=inlier_ratio,
            inlier_count=int(distances.size),
        )

    def _point_to_line_fit(self, target, source):
        """Fit scan surfaces, returning whether surface ICP was applicable."""
        normals = self._surface_normals(target)
        if int(np.count_nonzero(np.isfinite(normals[:, 0]))) < self.config.min_points:
            return False, None

        rotation = np.eye(2)
        translation = np.zeros(2)
        final_distances = None
        final_raw_count = 0
        for _ in range(self.config.max_iterations):
            transformed = source @ rotation.T + translation
            pairs = self._point_to_line_correspondences(
                target, normals, transformed
            )
            if pairs is None:
                return True, None
            source_points, target_points, pair_normals, residuals, raw_count = pairs
            normal_information = pair_normals.T @ pair_normals
            eigenvalues = np.linalg.eigvalsh(normal_information)
            if (
                float(eigenvalues[-1]) <= 1.0e-12
                or float(eigenvalues[0] / eigenvalues[-1])
                < self.config.min_point_to_line_observability
            ):
                return True, None

            jacobian = np.column_stack((
                pair_normals[:, 0],
                pair_normals[:, 1],
                (
                    -pair_normals[:, 0] * source_points[:, 1]
                    + pair_normals[:, 1] * source_points[:, 0]
                ),
            ))
            hessian = jacobian.T @ jacobian
            gradient = jacobian.T @ residuals
            try:
                update = -np.linalg.solve(
                    hessian + self.config.point_to_line_damping * np.eye(3),
                    gradient,
                )
            except np.linalg.LinAlgError:
                return True, None
            if not np.all(np.isfinite(update)):
                return True, None

            update_angle = float(update[2])
            update_rotation = np.array((
                (np.cos(update_angle), -np.sin(update_angle)),
                (np.sin(update_angle), np.cos(update_angle)),
            ))
            rotation = update_rotation @ rotation
            translation = update_rotation @ translation + update[:2]
            final_distances = np.abs(residuals)
            final_raw_count = raw_count
            if (
                float(np.linalg.norm(update[:2]))
                <= self.config.convergence_translation_m
                and abs(update_angle)
                <= self.config.convergence_rotation_rad
            ):
                break

        transformed = source @ rotation.T + translation
        final_pairs = self._point_to_line_correspondences(
            target, normals, transformed
        )
        if final_pairs is None:
            return True, None
        _, _, pair_normals, residuals, raw_count = final_pairs
        normal_information = pair_normals.T @ pair_normals
        eigenvalues = np.linalg.eigvalsh(normal_information)
        if (
            float(eigenvalues[-1]) <= 1.0e-12
            or float(eigenvalues[0] / eigenvalues[-1])
            < self.config.min_point_to_line_observability
        ):
            return True, None
        final_distances = np.abs(residuals)
        final_raw_count = raw_count
        return True, (
            rotation,
            translation,
            final_distances,
            final_raw_count,
        )

    def _surface_normals(self, points):
        """Estimate ordered LaserScan normals while rejecting range edges."""
        normals = np.full(points.shape, np.nan, dtype=float)
        for index in range(2, points.shape[0] - 2):
            neighborhood = points[index - 2:index + 3]
            gaps = np.linalg.norm(np.diff(neighborhood, axis=0), axis=1)
            if np.max(gaps) > self.config.normal_neighbor_max_distance_m:
                continue
            centered = neighborhood - np.mean(neighborhood, axis=0)
            eigenvalues, eigenvectors = np.linalg.eigh(centered.T @ centered)
            if (
                float(eigenvalues[-1]) <= 1.0e-12
                or float(eigenvalues[0] / eigenvalues[-1])
                > self.config.normal_max_curvature_ratio
            ):
                continue
            normals[index] = eigenvectors[:, 0]
        return normals

    def _point_to_line_correspondences(
        self,
        target,
        target_normals,
        source,
    ):
        squared = np.sum(
            np.square(source[:, np.newaxis, :] - target[np.newaxis, :, :]),
            axis=2,
        )
        target_indices = np.argmin(squared, axis=1)
        euclidean = np.sqrt(
            squared[np.arange(source.shape[0]), target_indices]
        )
        source_indices = np.arange(source.shape[0])
        reciprocal = (
            np.argmin(squared, axis=0)[target_indices] == source_indices
        )
        normals = target_normals[target_indices]
        candidate = (
            reciprocal
            & np.isfinite(normals[:, 0])
            & (euclidean <= self.config.max_correspondence_distance_m)
        )
        candidate_indices = np.flatnonzero(candidate)
        if candidate_indices.size < self.config.min_points:
            return None

        residuals = np.sum(
            normals[candidate_indices]
            * (
                source[candidate_indices]
                - target[target_indices[candidate_indices]]
            ),
            axis=1,
        )
        median = float(np.median(residuals))
        mad = float(np.median(np.abs(residuals - median)))
        robust_limit = max(
            self.config.robust_residual_floor_m,
            self.config.robust_mad_scale * 1.4826 * mad,
        )
        robust_limit = min(
            robust_limit,
            self.config.max_correspondence_distance_m,
        )
        gated = candidate_indices[np.abs(residuals - median) <= robust_limit]
        raw_count = int(gated.size)
        if raw_count < self.config.min_points:
            return None

        gated_residuals = np.sum(
            normals[gated]
            * (source[gated] - target[target_indices[gated]]),
            axis=1,
        )
        keep_count = max(
            self.config.min_points,
            int(raw_count * self.config.trim_fraction),
        )
        kept = gated[np.argsort(np.abs(gated_residuals))[:keep_count]]
        kept_normals = normals[kept]
        kept_residuals = np.sum(
            kept_normals
            * (source[kept] - target[target_indices[kept]]),
            axis=1,
        )
        return (
            source[kept],
            target[target_indices[kept]],
            kept_normals,
            kept_residuals,
            raw_count,
        )

    def _validated_points(self, points) -> np.ndarray | None:
        values = np.asarray(points, dtype=float)
        if values.ndim != 2 or values.shape[1] != 2:
            return None
        if values.shape[0] < self.config.min_points:
            return None
        if not np.all(np.isfinite(values)):
            return None
        if values.shape[0] > self.config.max_points:
            indices = np.linspace(
                0,
                values.shape[0] - 1,
                self.config.max_points,
                dtype=int,
            )
            values = values[indices]
        covariance = np.cov(values, rowvar=False)
        if float(np.min(np.linalg.eigvalsh(covariance))) <= 1.0e-6:
            return None
        return values

    def _correspondences(self, target, source):
        """Select reciprocal, robust-residual correspondences.

        Points from a newly appeared pedestrian or obstacle have no stable
        reciprocal match in the preceding scan.  The median/MAD gate then
        removes the remaining high-residual pairs before the rigid update.
        """
        squared = np.sum(
            np.square(source[:, np.newaxis, :] - target[np.newaxis, :, :]),
            axis=2,
        )
        target_indices = np.argmin(squared, axis=1)
        distances = np.sqrt(
            squared[np.arange(source.shape[0]), target_indices]
        )
        source_indices = np.arange(source.shape[0])
        nearest_sources = np.argmin(squared, axis=0)
        reciprocal = nearest_sources[target_indices] == source_indices
        candidate_mask = (
            reciprocal
            & (distances <= self.config.max_correspondence_distance_m)
        )
        candidate_indices = np.flatnonzero(candidate_mask)
        if candidate_indices.size < self.config.min_points:
            return None

        candidate_distances = distances[candidate_indices]
        median = float(np.median(candidate_distances))
        mad = float(np.median(np.abs(candidate_distances - median)))
        robust_limit = max(
            self.config.robust_residual_floor_m,
            median + self.config.robust_mad_scale * 1.4826 * mad,
        )
        robust_limit = min(
            robust_limit,
            self.config.max_correspondence_distance_m,
        )
        robust_mask = candidate_distances <= robust_limit
        gated_indices = candidate_indices[robust_mask]
        raw_inlier_count = int(gated_indices.size)
        if raw_inlier_count < self.config.min_points:
            return None
        keep_count = max(
            self.config.min_points,
            int(raw_inlier_count * self.config.trim_fraction),
        )
        order = np.argsort(distances[gated_indices])[:keep_count]
        source_indices = gated_indices[order]
        return (
            source[source_indices],
            target[target_indices[source_indices]],
            distances[source_indices],
            raw_inlier_count,
        )


def _rigid_transform(source: np.ndarray, target: np.ndarray):
    source_centroid = np.mean(source, axis=0)
    target_centroid = np.mean(target, axis=0)
    centered_source = source - source_centroid
    centered_target = target - target_centroid
    covariance = centered_source.T @ centered_target
    left, _, right_transpose = np.linalg.svd(covariance)
    rotation = right_transpose.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_transpose[-1, :] *= -1.0
        rotation = right_transpose.T @ left.T
    translation = target_centroid - rotation @ source_centroid
    return rotation, translation


def _rotation_angle(rotation: np.ndarray) -> float:
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))
