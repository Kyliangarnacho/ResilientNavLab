"""ROS-free LiDAR clustering and lightweight dynamic-object tracking."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class LidarTrackerConfig:
    """Small V1 profile for the 15 Hz Scene 1 planar LiDAR."""

    cluster_gap: float = 0.12
    minimum_cluster_points: int = 3
    maximum_usable_range: float = 6.0
    maximum_track_diameter: float = 0.60
    association_distance: float = 0.35
    minimum_common_matches: int = 3
    history_size: int = 6
    minimum_confirmations: int = 6
    candidate_minimum_confirmations: int = 3
    candidate_minimum_displacement: float = 0.02
    dynamic_minimum_speed: float = 0.08
    dynamic_minimum_displacement: float = 0.04
    direction_consistency: float = 0.75
    minimum_direction_step: float = 0.005
    velocity_ema_alpha: float = 0.25
    velocity_ema_stability_window: int = 3
    velocity_ema_max_direction_change_rad: float = 0.35
    velocity_ema_outlier_direction_change_rad: float = 0.70
    stationary_maximum_speed: float = 0.04
    stationary_confirmation_frames: int = 8

    def __post_init__(self) -> None:
        positive = (
            self.cluster_gap,
            self.maximum_usable_range,
            self.maximum_track_diameter,
            self.association_distance,
            self.candidate_minimum_displacement,
            self.dynamic_minimum_speed,
            self.dynamic_minimum_displacement,
            self.minimum_direction_step,
            self.stationary_maximum_speed,
        )
        if (
            not all(math.isfinite(value) and value > 0.0 for value in positive)
            or self.minimum_cluster_points < 2
            or self.minimum_common_matches < 2
            or self.history_size < 3
            or not 3 <= self.minimum_confirmations <= self.history_size
            or not 2 <= self.candidate_minimum_confirmations < self.minimum_confirmations
            or not 0.0 < self.direction_consistency <= 1.0
            or not 0.0 < self.velocity_ema_alpha <= 1.0
            or self.velocity_ema_stability_window < 2
            or not 0.0 < self.velocity_ema_max_direction_change_rad <= math.pi
            or not 0.0 < self.velocity_ema_outlier_direction_change_rad <= math.pi
            or self.stationary_maximum_speed >= self.dynamic_minimum_speed
            or self.stationary_confirmation_frames < 2
        ):
            raise ValueError('LiDAR tracker parameters are invalid')


@dataclass(frozen=True)
class LidarCluster:
    """One contiguous scan cluster represented in a caller-selected frame."""

    position: np.ndarray
    point_count: int
    diameter: float
    first_scan_index: int | None = None
    last_scan_index: int | None = None


@dataclass(frozen=True)
class DynamicAgent:
    """Sensor-derived dynamic state ready for the BRNE pedestrian contract."""

    track_id: int
    position: tuple[float, float]
    velocity: tuple[float, float]
    scan_index_span: tuple[int, int] | None = None


@dataclass(frozen=True)
class TrackerDiagnostics:
    """Minimal evidence for common-mode compensation and false positives."""

    cluster_count: int
    matched_cluster_count: int
    common_drift: tuple[float, float]
    track_count: int
    dynamic_agent_count: int


@dataclass
class _Track:
    track_id: int
    last_raw_position: np.ndarray
    corrected_position: np.ndarray
    history: deque[tuple[float, np.ndarray]]
    filtered_velocity_history: deque[np.ndarray]
    current_scan_index_span: tuple[int, int] | None
    filtered_velocity: np.ndarray | None = None
    confirmations: int = 1
    dynamic: bool = False
    velocity_stable: bool = False
    static_confirmed: bool = False
    stationary_frames: int = 0


@dataclass(frozen=True)
class _PreviousCluster:
    cluster: LidarCluster
    track_id: int | None


def clusters_from_scan(
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: LidarTrackerConfig,
) -> list[LidarCluster]:
    """Split adjacent finite scan returns into simple Euclidean clusters."""
    if (
        not all(math.isfinite(value) for value in (angle_min, angle_increment))
        or angle_increment <= 0.0
        or not math.isfinite(range_min)
        or not math.isfinite(range_max)
        or range_min < 0.0
        or range_max <= range_min
    ):
        return []
    upper_range = min(range_max, config.maximum_usable_range)
    clusters: list[LidarCluster] = []
    current: list[tuple[float, float]] = []
    current_indices: list[int] = []

    def flush() -> None:
        if len(current) < config.minimum_cluster_points:
            current.clear()
            current_indices.clear()
            return
        points = np.asarray(current, dtype=float)
        extent = np.max(points, axis=0) - np.min(points, axis=0)
        clusters.append(LidarCluster(
            position=np.mean(points, axis=0),
            point_count=len(current),
            diameter=float(np.linalg.norm(extent)),
            first_scan_index=current_indices[0],
            last_scan_index=current_indices[-1],
        ))
        current.clear()
        current_indices.clear()

    previous = None
    for index, raw_range in enumerate(ranges):
        scan_range = float(raw_range)
        if not math.isfinite(scan_range) or not range_min <= scan_range <= upper_range:
            flush()
            previous = None
            continue
        angle = angle_min + index * angle_increment
        point = (scan_range * math.cos(angle), scan_range * math.sin(angle))
        if previous is not None and math.dist(previous, point) > config.cluster_gap:
            flush()
        current.append(point)
        current_indices.append(index)
        previous = point
    flush()
    return clusters


def transform_clusters(
    clusters: Sequence[LidarCluster],
    *,
    translation: Sequence[float],
    yaw: float,
) -> list[LidarCluster]:
    """Rigidly transform cluster centroids while preserving scan geometry."""
    offset = np.asarray(translation, dtype=float)
    if offset.shape != (2,) or not np.isfinite(offset).all() or not math.isfinite(yaw):
        return []
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return [
        LidarCluster(
            position=rotation @ cluster.position + offset,
            point_count=cluster.point_count,
            diameter=cluster.diameter,
            first_scan_index=cluster.first_scan_index,
            last_scan_index=cluster.last_scan_index,
        )
        for cluster in clusters
    ]


def agents_within_range(
    agents: Sequence[DynamicAgent],
    *,
    observer_position: Sequence[float],
    maximum_range: float,
) -> list[DynamicAgent]:
    """Keep only dynamic agents relevant to the local interaction horizon."""
    observer = np.asarray(observer_position, dtype=float)
    if (
        observer.shape != (2,)
        or not np.isfinite(observer).all()
        or not math.isfinite(maximum_range)
        or maximum_range <= 0.0
    ):
        return []
    return [
        agent
        for agent in agents
        if np.linalg.norm(np.asarray(agent.position) - observer) <= maximum_range
    ]


def ranges_without_dynamic_agents(
    ranges: Sequence[float],
    agents: Sequence[DynamicAgent],
    *,
    clear_range: float,
) -> list[float]:
    """Replace dynamic or moving-candidate cluster beams with clearing rays."""
    filtered = [float(value) for value in ranges]
    if not math.isfinite(clear_range) or clear_range <= 0.0:
        return filtered
    for agent in agents:
        span = agent.scan_index_span
        if (
            span is None
            or len(span) != 2
            or not all(isinstance(index, int) for index in span)
        ):
            continue
        first_index = max(0, span[0])
        last_index = min(len(filtered) - 1, span[1])
        if first_index > last_index:
            continue
        filtered[first_index:last_index + 1] = [clear_range] * (
            last_index - first_index + 1
        )
    return filtered


class LidarDynamicTracker:
    """Track cluster motion after subtracting frame-wise common displacement."""

    def __init__(self, config: LidarTrackerConfig | None = None) -> None:
        self.config = config or LidarTrackerConfig()
        self._tracks: dict[int, _Track] = {}
        self._previous_clusters: list[_PreviousCluster] = []
        self._last_stamp: float | None = None
        self._next_track_id = 1

    def reset(self) -> None:
        """Drop observations immediately; never carry tracks through a gap."""
        self._tracks.clear()
        self._previous_clusters.clear()
        self._last_stamp = None

    def update(
        self,
        clusters: Sequence[LidarCluster],
        stamp_sec: float,
    ) -> tuple[list[DynamicAgent], TrackerDiagnostics]:
        """Associate one frame and return only confirmed dynamic tracks."""
        valid_clusters = [cluster for cluster in clusters if _valid_cluster(cluster)]
        if not math.isfinite(stamp_sec):
            self.reset()
            return [], self._diagnostics(len(valid_clusters), 0, np.zeros(2))
        if self._last_stamp is not None and stamp_sec <= self._last_stamp:
            self.reset()

        associations = _greedy_associations(
            [entry.cluster for entry in self._previous_clusters],
            valid_clusters,
            self.config.association_distance,
        )
        drift_samples = [
            valid_clusters[current_index].position
            - self._previous_clusters[previous_index].cluster.position
            for previous_index, current_index in associations
        ]
        common_drift = (
            np.median(np.asarray(drift_samples), axis=0)
            if len(drift_samples) >= self.config.minimum_common_matches
            else np.zeros(2)
        )
        previous_by_current = {
            current_index: previous_index
            for previous_index, current_index in associations
        }
        new_tracks: dict[int, _Track] = {}
        current_track_ids: dict[int, int] = {}
        for current_index, cluster in enumerate(valid_clusters):
            if cluster.diameter > self.config.maximum_track_diameter:
                continue
            previous_index = previous_by_current.get(current_index)
            previous_entry = (
                None
                if previous_index is None
                else self._previous_clusters[previous_index]
            )
            old_track = (
                None
                if previous_entry is None or previous_entry.track_id is None
                else self._tracks.get(previous_entry.track_id)
            )
            if old_track is None:
                track = self._new_track(cluster, stamp_sec)
            else:
                residual = (
                    cluster.position - old_track.last_raw_position - common_drift
                )
                track = old_track
                track.last_raw_position = cluster.position.copy()
                track.corrected_position = track.corrected_position + residual
                track.current_scan_index_span = _cluster_scan_index_span(cluster)
                track.confirmations += 1
                track.history.append((stamp_sec, track.corrected_position.copy()))
                if not track.dynamic and self._is_dynamic(track):
                    track.dynamic = True
                    track.static_confirmed = False
                elif not track.dynamic and self._is_stationary_candidate(track):
                    track.static_confirmed = True
            new_tracks[track.track_id] = track
            current_track_ids[current_index] = track.track_id

        self._tracks = new_tracks
        self._previous_clusters = [
            _PreviousCluster(cluster, current_track_ids.get(index))
            for index, cluster in enumerate(valid_clusters)
        ]
        self._last_stamp = stamp_sec
        agents = []
        for _, track in sorted(self._tracks.items()):
            if not track.dynamic:
                continue
            agent = self._agent_from_track(track)
            if not track.velocity_stable:
                track.velocity_stable = _velocity_ema_window_is_stable(
                    track.filtered_velocity_history,
                    window_size=self.config.velocity_ema_stability_window,
                    minimum_speed=self.config.dynamic_minimum_speed,
                    maximum_direction_change_rad=(
                        self.config.velocity_ema_max_direction_change_rad
                    ),
                )
            if track.velocity_stable:
                if np.linalg.norm(agent.velocity) <= self.config.stationary_maximum_speed:
                    track.stationary_frames += 1
                else:
                    track.stationary_frames = 0
                if track.stationary_frames >= self.config.stationary_confirmation_frames:
                    self._demote_stationary_track(track)
                    continue
                agents.append(agent)
        return agents, self._diagnostics(
            len(valid_clusters), len(associations), common_drift, len(agents)
        )

    def costmap_exclusion_agents(self) -> list[DynamicAgent]:
        """Return quarantined candidates and dynamic agents excluded from Navfn."""
        exclusions = []
        for _, track in sorted(self._tracks.items()):
            if (
                track.static_confirmed
                and not track.dynamic
                and not self._is_moving_candidate(track)
            ):
                continue
            velocity = (
                track.filtered_velocity
                if track.filtered_velocity is not None
                else _fitted_velocity(track.history)
            )
            exclusions.append(DynamicAgent(
                track_id=track.track_id,
                position=tuple(float(value) for value in track.corrected_position),
                velocity=tuple(float(value) for value in velocity),
                scan_index_span=track.current_scan_index_span,
            ))
        return exclusions

    def _new_track(self, cluster: LidarCluster, stamp_sec: float) -> _Track:
        track_id = self._next_track_id
        self._next_track_id += 1
        position = cluster.position.copy()
        return _Track(
            track_id=track_id,
            last_raw_position=position.copy(),
            corrected_position=position,
            history=deque([(stamp_sec, position.copy())], maxlen=self.config.history_size),
            filtered_velocity_history=deque(
                maxlen=self.config.velocity_ema_stability_window
            ),
            current_scan_index_span=_cluster_scan_index_span(cluster),
        )

    def _is_dynamic(self, track: _Track) -> bool:
        if track.confirmations < self.config.minimum_confirmations:
            return False
        positions = np.asarray([position for _, position in track.history])
        displacement = float(np.linalg.norm(positions[-1] - positions[0]))
        velocity = _fitted_velocity(track.history)
        if (
            displacement < self.config.dynamic_minimum_displacement
            or float(np.linalg.norm(velocity)) < self.config.dynamic_minimum_speed
        ):
            return False
        steps = np.diff(positions, axis=0)
        lengths = np.linalg.norm(steps, axis=1)
        moving_steps = steps[lengths >= self.config.minimum_direction_step]
        moving_lengths = lengths[lengths >= self.config.minimum_direction_step]
        if len(moving_steps) < self.config.minimum_confirmations - 1:
            return False
        unit_steps = moving_steps / moving_lengths[:, np.newaxis]
        consistency = float(np.linalg.norm(np.sum(unit_steps, axis=0)) / len(unit_steps))
        return consistency >= self.config.direction_consistency

    def _is_moving_candidate(self, track: _Track) -> bool:
        if track.confirmations < self.config.candidate_minimum_confirmations:
            return False
        positions = np.asarray([position for _, position in track.history])
        if (
            np.linalg.norm(positions[-1] - positions[0])
            < self.config.candidate_minimum_displacement
            or np.linalg.norm(_fitted_velocity(track.history))
            < self.config.dynamic_minimum_speed
        ):
            return False
        steps = np.diff(positions, axis=0)
        lengths = np.linalg.norm(steps, axis=1)
        moving_steps = steps[lengths >= self.config.minimum_direction_step]
        moving_lengths = lengths[lengths >= self.config.minimum_direction_step]
        if len(moving_steps) < self.config.candidate_minimum_confirmations - 1:
            return False
        unit_steps = moving_steps / moving_lengths[:, np.newaxis]
        consistency = float(np.linalg.norm(np.sum(unit_steps, axis=0)) / len(unit_steps))
        return consistency >= self.config.direction_consistency

    def _is_stationary_candidate(self, track: _Track) -> bool:
        if track.confirmations < self.config.minimum_confirmations:
            return False
        positions = np.asarray([position for _, position in track.history])
        maximum_excursion = float(np.max(np.linalg.norm(
            positions - positions[0], axis=1
        )))
        return (
            maximum_excursion <= self.config.candidate_minimum_displacement
            and np.linalg.norm(_fitted_velocity(track.history))
            <= self.config.stationary_maximum_speed
        )

    def _demote_stationary_track(self, track: _Track) -> None:
        latest_observation = track.history[-1]
        track.history.clear()
        track.history.append(latest_observation)
        track.filtered_velocity_history.clear()
        track.filtered_velocity = None
        track.confirmations = 1
        track.dynamic = False
        track.velocity_stable = False
        track.static_confirmed = True
        track.stationary_frames = 0

    def _agent_from_track(self, track: _Track) -> DynamicAgent:
        fitted_velocity = _fitted_velocity(track.history)
        if track.velocity_stable:
            fitted_velocity = _reject_velocity_direction_outlier(
                fitted_velocity,
                track.filtered_velocity,
                minimum_speed=self.config.dynamic_minimum_speed,
                maximum_direction_change_rad=(
                    self.config.velocity_ema_outlier_direction_change_rad
                ),
            )
        track.filtered_velocity = _ema_velocity(
            fitted_velocity,
            track.filtered_velocity,
            self.config.velocity_ema_alpha,
        )
        track.filtered_velocity_history.append(track.filtered_velocity.copy())
        return DynamicAgent(
            track_id=track.track_id,
            position=tuple(float(value) for value in track.corrected_position),
            velocity=tuple(float(value) for value in track.filtered_velocity),
            scan_index_span=track.current_scan_index_span,
        )

    def _diagnostics(
        self,
        cluster_count: int,
        matched_count: int,
        common_drift: np.ndarray,
        dynamic_count: int = 0,
    ) -> TrackerDiagnostics:
        return TrackerDiagnostics(
            cluster_count=cluster_count,
            matched_cluster_count=matched_count,
            common_drift=tuple(float(value) for value in common_drift),
            track_count=len(self._tracks),
            dynamic_agent_count=dynamic_count,
        )


def _valid_cluster(cluster: LidarCluster) -> bool:
    return (
        isinstance(cluster, LidarCluster)
        and cluster.position.shape == (2,)
        and np.isfinite(cluster.position).all()
        and cluster.point_count > 0
        and math.isfinite(cluster.diameter)
        and cluster.diameter >= 0.0
    )


def _cluster_scan_index_span(cluster: LidarCluster) -> tuple[int, int] | None:
    first_index = cluster.first_scan_index
    last_index = cluster.last_scan_index
    if (
        not isinstance(first_index, int)
        or not isinstance(last_index, int)
        or first_index < 0
        or last_index < first_index
    ):
        return None
    return first_index, last_index


def _greedy_associations(
    previous: Sequence[LidarCluster],
    current: Sequence[LidarCluster],
    maximum_distance: float,
) -> list[tuple[int, int]]:
    candidates = sorted(
        (
            float(np.linalg.norm(old.position - new.position)),
            old_index,
            new_index,
        )
        for old_index, old in enumerate(previous)
        for new_index, new in enumerate(current)
        if float(np.linalg.norm(old.position - new.position)) <= maximum_distance
    )
    used_old: set[int] = set()
    used_new: set[int] = set()
    associations = []
    for _, old_index, new_index in candidates:
        if old_index in used_old or new_index in used_new:
            continue
        used_old.add(old_index)
        used_new.add(new_index)
        associations.append((old_index, new_index))
    return associations


def _fitted_velocity(history: deque[tuple[float, np.ndarray]]) -> np.ndarray:
    if len(history) < 2:
        return np.zeros(2)
    times = np.asarray([stamp for stamp, _ in history], dtype=float)
    positions = np.asarray([position for _, position in history], dtype=float)
    centered = times - np.mean(times)
    denominator = float(centered @ centered)
    if denominator <= 1e-12:
        return np.zeros(2)
    return centered @ positions / denominator


def _ema_velocity(
    current: np.ndarray,
    previous: np.ndarray | None,
    alpha: float,
) -> np.ndarray:
    """Apply EMA after the unchanged short-window velocity fit."""
    if previous is None:
        return current.copy()
    return alpha * current + (1.0 - alpha) * previous


def _reject_velocity_direction_outlier(
    current: np.ndarray,
    previous: np.ndarray | None,
    *,
    minimum_speed: float,
    maximum_direction_change_rad: float,
) -> np.ndarray:
    """Keep the last stable velocity through one implausible direction jump."""
    if previous is None:
        return current.copy()
    current_speed = float(np.linalg.norm(current))
    previous_speed = float(np.linalg.norm(previous))
    if (
        not np.isfinite(current).all()
        or not np.isfinite(previous).all()
        or current_speed < minimum_speed
        or previous_speed < minimum_speed
    ):
        return current.copy()
    cosine = float(np.clip(
        current @ previous / (current_speed * previous_speed),
        -1.0,
        1.0,
    ))
    if math.acos(cosine) > maximum_direction_change_rad:
        return previous.copy()
    return current.copy()


def _velocity_ema_window_is_stable(
    history: Sequence[np.ndarray],
    *,
    window_size: int,
    minimum_speed: float,
    maximum_direction_change_rad: float,
) -> bool:
    """Require one scene-independent window of mutually stable EMA directions."""
    if len(history) < window_size:
        return False
    velocities = np.asarray(list(history)[-window_size:], dtype=float)
    speeds = np.linalg.norm(velocities, axis=1)
    if not np.isfinite(velocities).all() or np.any(speeds < minimum_speed):
        return False
    directions = velocities / speeds[:, np.newaxis]
    newest_direction = directions[-1]
    cosines = np.clip(directions @ newest_direction, -1.0, 1.0)
    return bool(np.all(np.arccos(cosines) <= maximum_direction_change_rad))
