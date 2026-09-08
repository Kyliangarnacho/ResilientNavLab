"""Targeted, ROS-free contracts for controlled benchmark evidence."""

from resilient_nav_fusion.benchmark_observer import ImuBenchmarkObserver
from resilient_nav_fusion.benchmark_observer import SensorBenchmarkObserver


def metrics():
    """Build the minimum complete evaluator payload."""
    group = {
        'position_rmse': 1.0,
        'yaw_rmse': 0.1,
        'max_position_error': 2.0,
        'max_yaw_error': 0.2,
        'final_position_drift': 0.5,
        'final_yaw_drift': 0.05,
        'sample_count': 20,
    }
    return {'fixed': group, 'adaptive': group, 'position_rmse_benefit': 0.0}


def test_observer_requires_ordered_fault_health_fusion_recovery_and_fresh_metrics():
    """A passing record proves every requested benchmark phase was observed."""
    observer = ImuBenchmarkObserver()
    observer.observe_imu_health(3, 1.0)
    observer.observe_fusion_status(2, 1.0)
    observer.observe_fault_status(1, 5.0)
    observer.observe_imu_health(3, 5.6)
    observer.observe_fusion_status(2, 5.8)
    observer.observe_fault_status(2, 15.0)
    observer.observe_imu_health(1, 15.6)
    observer.observe_fusion_status(1, 15.8)
    observer.observe_metrics(metrics())
    observer.observe_metrics(metrics())

    assert observer.complete is True
    result = observer.result('phase8_imu_test', 'PASS')
    assert result['events'] == {
        'fault_start': 5.0,
        'imu_health_first_abnormal': 5.6,
        'fusion_first_non_nominal': 5.8,
        'fault_end': 15.0,
        'imu_health_recovered_healthy': 15.6,
        'fusion_recovered_nominal': 15.8,
    }
    assert result['metrics'] == metrics()
    assert result['benchmark_outcome'] == 'PASS'
    assert result['adaptive_improved'] is False


def test_observer_fails_closed_when_metrics_or_recovery_are_missing():
    """No partial trace can accidentally become a successful benchmark."""
    observer = ImuBenchmarkObserver()
    observer.observe_fault_status(1, 5.0)
    observer.observe_imu_health(3, 5.2)
    observer.observe_fusion_status(2, 5.2)
    observer.observe_fault_status(2, 15.0)

    assert observer.complete is False
    assert observer.missing_requirements() == [
        'imu_health_recovered_healthy',
        'fusion_recovered_nominal',
        'evaluator_metrics',
    ]


def test_observer_rejects_incomplete_evaluator_payload():
    """The recorder will not report metrics that omit required comparisons."""
    observer = ImuBenchmarkObserver()
    observer.observe_metrics({'fixed': {}, 'adaptive': {}})

    assert observer.metrics is None


def test_observer_rejects_missing_yaw_shape_and_separates_outcome_from_improvement():
    """A complete timeline cannot mask incomplete metrics or a non-improvement."""
    observer = ImuBenchmarkObserver()
    invalid = metrics()
    invalid['fixed'] = dict(invalid['fixed'])
    invalid['fixed'].pop('max_yaw_error')
    observer.observe_metrics(invalid)

    assert observer.metrics is None
    failed = observer.result('phase8_imu_test', 'FAIL', error='no evidence')
    assert failed['benchmark_outcome'] == 'FAIL'
    assert failed['adaptive_improved'] is None


def test_wheel_observer_requires_policy_to_reject_wheel_and_keep_imu_yaw_rate():
    """Wheel-freeze PASS needs policy evidence, not only a non-NOMINAL state."""
    observer = SensorBenchmarkObserver('wheel')
    observer.observe_fault_status(1, 5.0)
    observer.observe_target_health(3, 6.0)
    observer.observe_fusion_status(
        2,
        6.0,
        accepted_measurements=('lidar_velocity', 'imu_yaw_rate'),
        rejected_measurements=('wheel_velocity', 'wheel_yaw_rate'),
    )
    observer.observe_lidar_velocity(6.1)
    observer.observe_fault_status(2, 15.0)
    observer.observe_target_health(1, 15.4)
    observer.observe_fusion_status(
        1,
        15.6,
        accepted_measurements=('wheel_velocity', 'imu_yaw_rate'),
    )
    observer.observe_metrics(metrics())
    observer.observe_metrics(metrics())

    assert observer.complete is True
    result = observer.result('phase8_wheel_freeze', 'PASS')
    assert result['events']['wheel_fault_response'] == 6.0
    assert result['events']['lidar_velocity_observed'] == 6.1
    assert result['fusion_behavior'] == {
        'wheel_velocity_rejected': True,
        'lidar_velocity_accepted': True,
        'imu_yaw_rate_accepted': True,
        'wheel_yaw_fallback_enabled': False,
    }


def test_wheel_observer_fails_closed_when_fusion_keeps_wheel_velocity():
    """A generic DEGRADED state alone cannot satisfy wheel-freeze evidence."""
    observer = SensorBenchmarkObserver('wheel')
    observer.observe_fault_status(1, 5.0)
    observer.observe_target_health(3, 6.0)
    observer.observe_fusion_status(
        2,
        6.0,
        accepted_measurements=('wheel_velocity', 'imu_yaw_rate'),
    )

    assert observer.complete is False
    assert 'wheel_fault_response' in observer.missing_requirements()


def test_wheel_observer_requires_lidar_translation_fallback():
    observer = SensorBenchmarkObserver('wheel')
    observer.observe_fault_status(1, 5.0)
    observer.observe_target_health(3, 6.0)
    observer.observe_fusion_status(
        2,
        6.0,
        accepted_measurements=('imu_yaw_rate',),
        rejected_measurements=(
            'wheel_velocity', 'lidar_velocity', 'wheel_yaw_rate'
        ),
    )

    assert observer.complete is False
    assert 'wheel_fault_response' in observer.missing_requirements()


def test_wheel_observer_requires_real_lidar_adapter_output():
    observer = SensorBenchmarkObserver('wheel')
    observer.observe_fault_status(1, 5.0)
    observer.observe_target_health(3, 6.0)
    observer.observe_fusion_status(
        2,
        6.0,
        accepted_measurements=('lidar_velocity', 'imu_yaw_rate'),
        rejected_measurements=('wheel_velocity', 'wheel_yaw_rate'),
    )

    assert 'wheel_fault_response' in observer.events
    assert 'lidar_velocity_observed' in observer.missing_requirements()


def test_confirmed_wheel_response_is_not_overwritten_by_shutdown_noise():
    """A later UNKNOWN sample cannot erase the first valid fault response."""
    observer = SensorBenchmarkObserver('wheel')
    observer.observe_fault_status(1, 5.0)
    observer.observe_fusion_status(
        2,
        7.0,
        accepted_measurements=('lidar_velocity', 'imu_yaw_rate'),
        rejected_measurements=('wheel_velocity',),
    )
    observer.observe_fusion_status(
        3,
        20.0,
        accepted_measurements=('wheel_yaw_rate',),
        rejected_measurements=('imu_yaw_rate',),
    )

    assert observer.events['wheel_fault_response'] == 7.0
    assert observer.fusion_behavior['wheel_velocity_rejected'] is True
    assert observer.fusion_behavior['lidar_velocity_accepted'] is True
