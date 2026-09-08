"""ROS-free contracts for the lightweight sensor-input dynamic tracker."""

import numpy as np
import pytest

from resilient_nav_brne.lidar_dynamic_tracker import (
    _ema_velocity,
    _reject_velocity_direction_outlier,
    _velocity_ema_window_is_stable,
    agents_within_range,
    clusters_from_scan,
    DynamicAgent,
    LidarCluster,
    LidarDynamicTracker,
    LidarTrackerConfig,
    transform_clusters,
)


def _cluster(
    x_position,
    y_position,
    diameter=0.20,
    first_scan_index=None,
    last_scan_index=None,
):
    return LidarCluster(
        position=np.array([x_position, y_position], dtype=float),
        point_count=8,
        diameter=diameter,
        first_scan_index=first_scan_index,
        last_scan_index=last_scan_index,
    )


def test_scan_clustering_splits_invalid_and_geometrically_separate_returns():
    config = LidarTrackerConfig(minimum_cluster_points=2, cluster_gap=0.08)
    clusters = clusters_from_scan(
        [float('inf'), 1.0, 1.0, 1.0, float('inf'), 2.0, 2.0],
        angle_min=0.0,
        angle_increment=0.01,
        range_min=0.08,
        range_max=12.0,
        config=config,
    )

    assert len(clusters) == 2
    assert [cluster.point_count for cluster in clusters] == [3, 2]
    assert [
        (cluster.first_scan_index, cluster.last_scan_index)
        for cluster in clusters
    ] == [(1, 3), (5, 6)]
    assert clusters[0].diameter < config.cluster_gap
    assert clusters[1].diameter < config.cluster_gap


def test_cluster_transform_is_a_rigid_odom_conversion():
    transformed = transform_clusters(
        [_cluster(1.0, 0.0, first_scan_index=4, last_scan_index=6)],
        translation=(2.0, -1.0),
        yaw=np.pi / 2.0,
    )

    assert len(transformed) == 1
    assert np.allclose(transformed[0].position, [2.0, 0.0])
    assert transformed[0].diameter == 0.20
    assert transformed[0].first_scan_index == 4
    assert transformed[0].last_scan_index == 6


def test_only_local_dynamic_agents_reach_the_brne_input():
    agents = [
        DynamicAgent(1, (1.0, 1.0), (0.0, 0.25)),
        DynamicAgent(2, (1.7, 2.28), (0.0, 0.25)),
    ]

    local = agents_within_range(
        agents,
        observer_position=(0.0, 0.0),
        maximum_range=2.0,
    )

    assert [agent.track_id for agent in local] == [1]


def test_common_drift_is_removed_before_dynamic_agent_promotion():
    tracker = LidarDynamicTracker()
    bases = np.array([
        [0.0, 0.0],
        [2.0, 0.0],
        [0.0, 2.0],
        [1.0, 1.0],
    ])
    agents = []
    diagnostics = None
    for frame_index in range(9):
        common = frame_index * np.array([0.01, -0.005])
        positions = bases + common
        positions[-1] += frame_index * np.array([0.0, 0.025])
        clusters = [_cluster(*position) for position in positions]
        clusters[-1] = _cluster(
            *positions[-1], first_scan_index=20, last_scan_index=24
        )
        agents, diagnostics = tracker.update(
            clusters,
            stamp_sec=frame_index * 0.1,
        )

    assert diagnostics is not None
    assert diagnostics.matched_cluster_count == 4
    assert np.allclose(diagnostics.common_drift, [0.01, -0.005])
    assert diagnostics.common_rotation_rad == pytest.approx(0.0)
    assert diagnostics.track_count == 4
    assert diagnostics.dynamic_agent_count == 1
    assert len(agents) == 1
    assert agents[0].track_id == 4
    assert np.allclose(agents[0].position, [1.0, 1.2])
    assert np.allclose(agents[0].velocity, [0.0, 0.25])
    assert agents[0].scan_index_span == (20, 24)


def test_velocity_ema_is_applied_after_the_existing_fitted_velocity():
    current_fit = np.array([1.0, -1.0])
    previous_output = np.array([0.2, 0.4])

    first_output = _ema_velocity(current_fit, None, alpha=0.25)
    filtered_output = _ema_velocity(current_fit, previous_output, alpha=0.25)

    assert np.allclose(first_output, current_fit)
    assert np.allclose(filtered_output, [0.40, 0.05])


def test_velocity_ema_alpha_must_be_in_unit_interval():
    config = LidarTrackerConfig()
    assert config.velocity_ema_alpha == 0.25
    assert config.velocity_ema_stability_window == 3
    assert config.velocity_ema_max_direction_change_rad == 0.35
    assert config.velocity_ema_outlier_direction_change_rad == 0.70
    assert config.common_motion_inlier_distance == 0.02
    assert config.stationary_maximum_speed == 0.04
    assert config.stationary_confirmation_frames == 8
    with pytest.raises(ValueError):
        LidarTrackerConfig(velocity_ema_alpha=0.0)
    with pytest.raises(ValueError):
        LidarTrackerConfig(velocity_ema_alpha=1.01)
    with pytest.raises(ValueError):
        LidarTrackerConfig(velocity_ema_stability_window=1)
    with pytest.raises(ValueError):
        LidarTrackerConfig(velocity_ema_max_direction_change_rad=0.0)
    with pytest.raises(ValueError):
        LidarTrackerConfig(velocity_ema_outlier_direction_change_rad=0.0)
    with pytest.raises(ValueError):
        LidarTrackerConfig(common_motion_inlier_distance=0.0)
    with pytest.raises(ValueError):
        LidarTrackerConfig(stationary_maximum_speed=0.08)
    with pytest.raises(ValueError):
        LidarTrackerConfig(stationary_confirmation_frames=1)


def test_post_stability_direction_outlier_keeps_the_previous_velocity():
    previous = np.array([0.25, 0.0])
    wild_turn = np.array([0.0, 0.30])
    modest_turn = np.array([0.24, 0.08])

    rejected = _reject_velocity_direction_outlier(
        wild_turn,
        previous,
        minimum_speed=0.08,
        maximum_direction_change_rad=0.70,
    )
    accepted = _reject_velocity_direction_outlier(
        modest_turn,
        previous,
        minimum_speed=0.08,
        maximum_direction_change_rad=0.70,
    )

    assert np.array_equal(rejected, previous)
    assert np.array_equal(accepted, modest_turn)


def test_velocity_ema_stability_window_rejects_large_direction_change():
    stable = [
        np.array([np.cos(angle), np.sin(angle)])
        for angle in (0.0, 0.10, 0.20, 0.30)
    ]
    unstable = stable[:-1] + [
        np.array([np.cos(0.60), np.sin(0.60)])
    ]

    assert _velocity_ema_window_is_stable(
        stable,
        window_size=4,
        minimum_speed=0.08,
        maximum_direction_change_rad=0.35,
    )
    assert not _velocity_ema_window_is_stable(
        unstable,
        window_size=4,
        minimum_speed=0.08,
        maximum_direction_change_rad=0.35,
    )
    assert not _velocity_ema_window_is_stable(
        stable[:3],
        window_size=4,
        minimum_speed=0.08,
        maximum_direction_change_rad=0.35,
    )


def test_dynamic_agent_waits_for_three_stable_velocity_ema_samples():
    tracker = LidarDynamicTracker()
    static = np.array([[0.0, 0.0], [2.0, 0.0], [0.0, 2.0]])
    outputs = []
    for frame_index in range(9):
        moving = np.array([1.0, 1.0 + frame_index * 0.025])
        agents, _ = tracker.update(
            [_cluster(*position) for position in np.vstack((static, moving))],
            stamp_sec=frame_index * 0.1,
        )
        outputs.append(len(agents))

    assert outputs[5:9] == [0, 0, 1, 1]


def test_sustained_stationary_dynamic_track_returns_to_non_dynamic_state():
    tracker = LidarDynamicTracker()
    static = np.array([[0.0, 0.0], [2.0, 0.0], [0.0, 2.0]])
    agents = []
    for frame_index in range(9):
        moving = _cluster(
            1.0,
            1.0 + frame_index * 0.025,
            first_scan_index=20,
            last_scan_index=24,
        )
        agents, _ = tracker.update(
            [_cluster(*position) for position in static] + [moving],
            stamp_sec=frame_index * 0.1,
        )
    assert len(agents) == 1

    stopped_position = 1.0 + 8 * 0.025
    for frame_index in range(9, 39):
        stopped = _cluster(
            1.0,
            stopped_position,
            first_scan_index=20,
            last_scan_index=24,
        )
        agents, _ = tracker.update(
            [_cluster(*position) for position in static] + [stopped],
            stamp_sec=frame_index * 0.1,
        )

    assert agents == []
    pedestrian_track = tracker._tracks[4]
    assert not pedestrian_track.dynamic
    assert not pedestrian_track.velocity_stable


def test_common_motion_alone_does_not_promote_static_clusters():
    tracker = LidarDynamicTracker()
    bases = np.array([[0.0, 0.0], [2.0, 0.0], [0.0, 2.0]])
    agents = []
    for frame_index in range(7):
        common = frame_index * np.array([0.02, -0.01])
        agents, _ = tracker.update(
            [_cluster(*(position + common)) for position in bases],
            stamp_sec=frame_index * 0.1,
        )

    assert agents == []


@pytest.mark.parametrize('rotation_step', [-0.035, 0.035])
def test_common_rotation_does_not_create_direction_dependent_motion(
    rotation_step,
):
    tracker = LidarDynamicTracker()
    bases = np.array([
        [-2.0, -1.0],
        [2.0, -1.0],
        [-2.0, 1.0],
        [2.0, 1.0],
    ])
    diagnostics = None
    agents = []
    for frame_index in range(9):
        angle = frame_index * rotation_step
        rotation = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)],
        ])
        translation = frame_index * np.array([0.005, -0.003])
        positions = bases @ rotation.T + translation
        agents, diagnostics = tracker.update(
            [_cluster(*position) for position in positions],
            stamp_sec=frame_index * 0.1,
        )

    assert agents == []
    assert diagnostics is not None
    assert diagnostics.common_rotation_rad == pytest.approx(rotation_step)
    assert diagnostics.dynamic_agent_count == 0


def test_common_rigid_motion_keeps_independent_target_motion():
    tracker = LidarDynamicTracker()
    bases = np.array([
        [-2.0, -1.0],
        [2.0, -1.0],
        [-2.0, 1.0],
        [1.0, 0.0],
    ])
    agents = []
    for frame_index in range(9):
        angle = frame_index * 0.025
        rotation = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)],
        ])
        positions = bases @ rotation.T
        positions[-1] += frame_index * np.array([0.0, 0.025])
        clusters = [_cluster(*position) for position in positions]
        clusters[-1] = _cluster(
            *positions[-1], first_scan_index=20, last_scan_index=24
        )
        agents, _ = tracker.update(clusters, stamp_sec=frame_index * 0.1)

    assert len(agents) == 1
    assert agents[0].track_id == 4
    assert np.linalg.norm(agents[0].velocity) == pytest.approx(0.25, rel=0.05)


def test_lost_dynamic_track_is_deleted_and_reappearance_gets_a_new_id():
    tracker = LidarDynamicTracker()
    static = np.array([[0.0, 0.0], [2.0, 0.0], [0.0, 2.0]])
    agents = []
    for frame_index in range(9):
        moving = np.array([1.0, 1.0 + frame_index * 0.025])
        agents, _ = tracker.update(
            [_cluster(*position) for position in np.vstack((static, moving))],
            stamp_sec=frame_index * 0.1,
        )
    old_id = agents[0].track_id

    agents, diagnostics = tracker.update([], stamp_sec=0.9)
    assert agents == []
    assert diagnostics.track_count == 0

    agents, _ = tracker.update(
        [_cluster(1.0, 1.2)],
        stamp_sec=1.0,
    )
    assert agents == []
    assert tracker._next_track_id > old_id + 1


def test_nonincreasing_timestamp_clears_motion_history():
    tracker = LidarDynamicTracker()
    tracker.update([_cluster(1.0, 1.0)], stamp_sec=1.0)
    tracker.update([_cluster(1.0, 1.1)], stamp_sec=1.0)

    assert len(tracker._tracks) == 1
    track = next(iter(tracker._tracks.values()))
    assert track.confirmations == 1
    assert not track.dynamic
