"""Unit tests for health evaluation against FaultStatus truth."""

import json

from builtin_interfaces.msg import Time
from resilient_nav_health_assessment.health_evaluator import (
    HealthEvaluationAccumulator,
)
from resilient_nav_interfaces.msg import FaultStatus, SensorHealth


def stamp(seconds):
    """Build a ROS Time message from integral or fractional seconds."""
    sec = int(seconds)
    return Time(sec=sec, nanosec=round((seconds - sec) * 1_000_000_000))


def fault_status(
    event_id='event_1', sensor='imu', model='bias', state=FaultStatus.SCHEDULED,
    at=0.0, start=10.0, end=20.0,
):
    """Build a compact FaultStatus test message."""
    msg = FaultStatus()
    msg.event_id = event_id
    msg.sensor = sensor
    msg.model = model
    msg.state = state
    msg.header.stamp = stamp(at)
    msg.start_time = stamp(start)
    msg.end_time = stamp(end)
    return msg


def health(sensor='imu', state=SensorHealth.HEALTHY, at=0.0, fault='none'):
    """Build a compact SensorHealth test message."""
    msg = SensorHealth()
    msg.sensor = sensor
    msg.state = state
    msg.detected_fault = fault
    msg.header.stamp = stamp(at)
    return msg


def activate(accumulator, **overrides):
    """Record scheduled and active truth messages for one event."""
    scheduled = fault_status(**overrides)
    accumulator.record_fault_status(scheduled)
    active_args = dict(overrides)
    active_args.update({'state': FaultStatus.ACTIVE, 'at': scheduled.start_time.sec})
    accumulator.record_fault_status(fault_status(**active_args))


def end(accumulator, **overrides):
    """Record an ENDED truth transition for one event."""
    event = fault_status(**overrides)
    accumulator.record_fault_status(fault_status(
        **{**overrides, 'state': FaultStatus.ENDED, 'at': event.end_time.sec},
    ))


def test_confusion_matrix_counts_tp_fp_fn_and_tn():
    accumulator = HealthEvaluationAccumulator()
    accumulator.record_health(health(at=1.0))
    accumulator.record_health(health(state=SensorHealth.FAULT, at=2.0))
    activate(accumulator)
    accumulator.record_health(health(at=11.0))
    accumulator.record_health(health(state=SensorHealth.FAULT, at=12.0, fault='bias'))

    metrics = accumulator.result()['sensors']['imu']

    assert metrics['tp'] == 1
    assert metrics['fp'] == 1
    assert metrics['fn'] == 1
    assert metrics['tn'] == 1


def test_unknown_is_counted_separately_and_excluded_from_matrix():
    accumulator = HealthEvaluationAccumulator()
    activate(accumulator)
    accumulator.record_health(health(state=SensorHealth.UNKNOWN, at=11.0))

    metrics = accumulator.result()['sensors']['imu']

    assert metrics['unknown'] == 1
    assert metrics['evaluated'] == 0
    assert metrics['tp'] == metrics['fp'] == metrics['fn'] == metrics['tn'] == 0


def test_precision_recall_f1_and_zero_denominators():
    empty = HealthEvaluationAccumulator().result()['total']
    assert empty['precision'] == empty['recall'] == empty['f1'] == 0.0

    accumulator = HealthEvaluationAccumulator()
    activate(accumulator)
    accumulator.record_health(health(state=SensorHealth.FAULT, at=11.0, fault='bias'))
    end(accumulator)
    accumulator.record_health(health(state=SensorHealth.FAULT, at=30.0))

    metrics = accumulator.result()['total']
    assert metrics['precision'] == 0.5
    assert metrics['recall'] == 1.0
    assert metrics['f1'] == 2.0 / 3.0


def test_first_alarm_time_and_detection_delay():
    accumulator = HealthEvaluationAccumulator()
    activate(accumulator, event_id='delayed', start=10.0, end=20.0)
    accumulator.record_health(health(state=SensorHealth.HEALTHY, at=10.5))
    accumulator.record_health(health(
        state=SensorHealth.DEGRADED, at=12.25, fault='bias',
    ))
    end(accumulator, event_id='delayed', start=10.0, end=20.0)

    event = accumulator.result()['events'][0]

    assert event['first_alarm_time_sec'] == 12.25
    assert event['detection_delay_sec'] == 2.25
    assert event['missed'] is False
    assert event['settled'] is True


def test_complete_missed_event_is_settled_on_ended():
    accumulator = HealthEvaluationAccumulator()
    activate(accumulator, event_id='missed', start=10.0, end=20.0)
    accumulator.record_health(health(at=11.0))
    end(accumulator, event_id='missed', start=10.0, end=20.0)

    event = accumulator.result()['events'][0]

    assert event['missed'] is True
    assert event['first_alarm_time_sec'] is None
    assert event['detection_delay_sec'] is None
    assert event['positive_health_samples'] == 1


def test_fault_classification_mapping_and_mismatch():
    accumulator = HealthEvaluationAccumulator()
    activate(accumulator, event_id='delay', model='fixed_delay')
    accumulator.record_health(health(
        state=SensorHealth.FAULT, at=11.0, fault='delay',
    ))
    activate(accumulator, event_id='drop', model='dropout', start=30.0, end=40.0)
    accumulator.record_health(health(
        state=SensorHealth.FAULT, at=31.0, fault='bias',
    ))

    events = {event['event_id']: event for event in accumulator.result()['events']}

    assert events['delay']['classification']['expected'] == 'delay'
    assert events['delay']['classification']['matched'] is True
    assert events['drop']['classification']['expected'] == 'stale'
    assert events['drop']['classification']['matched'] is False


def test_duplicate_statuses_do_not_duplicate_an_event():
    accumulator = HealthEvaluationAccumulator()
    scheduled = fault_status(event_id='once')
    active = fault_status(event_id='once', state=FaultStatus.ACTIVE, at=10.0)
    for msg in (scheduled, scheduled, active, active):
        accumulator.record_fault_status(msg)

    result = accumulator.result()

    assert result['event_count'] == 1
    assert len(result['events']) == 1


def test_passthrough_cancelled_events_are_excluded_from_evaluation():
    accumulator = HealthEvaluationAccumulator()
    for sensor in ('imu', 'wheel'):
        accumulator.record_fault_status(fault_status(
            event_id=f'{sensor}_passthrough', sensor=sensor,
        ))
        accumulator.record_fault_status(fault_status(
            event_id=f'{sensor}_passthrough', sensor=sensor,
            state=FaultStatus.CANCELLED, at=5.0,
        ))
    activate(accumulator, event_id='scan_fault', sensor='scan')

    result = accumulator.result()

    assert result['event_count'] == 1
    assert [event['event_id'] for event in result['events']] == ['scan_fault']
    assert result['events'][0]['sensor'] == 'scan'


def test_multiple_sensor_events_do_not_interfere():
    accumulator = HealthEvaluationAccumulator()
    activate(accumulator, event_id='imu_event', sensor='imu')
    activate(accumulator, event_id='scan_event', sensor='lidar')
    accumulator.record_health(health(
        sensor='imu', state=SensorHealth.FAULT, at=11.0, fault='bias',
    ))
    accumulator.record_health(health(
        sensor='scan', state=SensorHealth.HEALTHY, at=11.0,
    ))

    result = accumulator.result()

    assert result['sensors']['imu']['tp'] == 1
    assert result['sensors']['scan']['fn'] == 1
    assert result['sensors']['wheel']['evaluated'] == 0


def test_scheduled_active_ended_state_timeline():
    accumulator = HealthEvaluationAccumulator()
    accumulator.record_fault_status(fault_status(at=1.0, start=10.0, end=20.0))
    accumulator.record_health(health(at=2.0))
    accumulator.record_fault_status(fault_status(
        state=FaultStatus.ACTIVE, at=10.0, start=10.0, end=20.0,
    ))
    accumulator.record_health(health(state=SensorHealth.FAULT, at=11.0, fault='bias'))
    accumulator.record_fault_status(fault_status(
        state=FaultStatus.ENDED, at=20.0, start=10.0, end=20.0,
    ))
    accumulator.record_health(health(at=21.0))

    metrics = accumulator.result()['sensors']['imu']

    assert metrics['tn'] == 2
    assert metrics['tp'] == 1


def test_json_result_structure_is_serializable():
    accumulator = HealthEvaluationAccumulator()
    activate(accumulator, event_id='json_event')
    accumulator.record_health(health(
        state=SensorHealth.FAULT, at=11.0, fault='bias',
    ))

    result = accumulator.result()
    decoded = json.loads(json.dumps(result))

    assert decoded['schema_version'] == 1
    assert set(decoded) == {'schema_version', 'sensors', 'total', 'event_count', 'events'}
    assert set(decoded['sensors']) == {'imu', 'wheel', 'scan'}
    assert decoded['events'][0]['event_id'] == 'json_event'
