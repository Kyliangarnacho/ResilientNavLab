"""Focused checks for the ROS-free Phase 8 healthy smoke contract."""

import pytest

from resilient_nav_fusion.healthy_smoke import (
    HeaderObservation,
    HealthySmokeObserver,
)


def header(frame_id='odom', sec=1, nanosec=0, child_frame_id=''):
    """Create a valid minimal header observation."""
    return HeaderObservation(frame_id, sec, nanosec, child_frame_id)


def test_observer_requires_continuous_nominal_measurements_and_output():
    """Three valid samples per stream plus nominal status form smoke evidence."""
    observer = HealthySmokeObserver(required_samples=3)
    for index in range(3):
        observer.observe_wheel(header(sec=index))
        observer.observe_imu(header(frame_id='imu_link', sec=index))
        observer.observe_adaptive(
            header(sec=index, child_frame_id='base_footprint')
        )
        observer.observe_fusion_state(1)

    assert observer.passed is True
    assert observer.summary() == {
        'wheel_samples': 3,
        'imu_samples': 3,
        'adaptive_samples': 3,
        'nominal_status_samples': 3,
        'failures': [],
    }


@pytest.mark.parametrize(
    ('callback', 'observation', 'failure'),
    [
        ('observe_wheel', header(frame_id='map'), 'wheel_frame_mismatch'),
        (
            'observe_adaptive',
            header(child_frame_id='base_link'),
            'adaptive_child_frame_mismatch',
        ),
        (
            'observe_imu',
            header(frame_id='imu_link', nanosec=1_000_000_000),
            'imu_invalid_stamp',
        ),
    ],
)
def test_observer_fails_closed_on_frame_or_timestamp_contract_breaks(
    callback,
    observation,
    failure,
):
    """Invalid runtime evidence cannot be reported as a successful smoke."""
    observer = HealthySmokeObserver()
    getattr(observer, callback)(observation)

    assert observer.passed is False
    assert observer.failures == [failure]


def test_observer_rejects_invalid_required_sample_count():
    """The smoke contract cannot be accidentally reduced to zero observations."""
    with pytest.raises(ValueError, match='at least one'):
        HealthySmokeObserver(required_samples=0)
