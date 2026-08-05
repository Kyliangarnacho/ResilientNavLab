"""Evaluate sensor-health outputs against FaultStatus simulation truth."""

from dataclasses import dataclass, field
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from resilient_nav_interfaces.msg import FaultStatus, SensorHealth


DEFAULT_HEALTH_TOPICS = {
    'imu': '/health/imu',
    'wheel': '/health/wheel',
    'scan': '/health/scan',
}
DEFAULT_FAULT_STATUS_TOPIC = '/fault_injection/status'

FAULT_CLASSIFICATIONS = {
    'bias': 'bias',
    'z_gyro_bias': 'bias',
    'fixed_delay': 'delay',
    'dropout': 'stale',
    'freeze': 'freeze',
    'sector_blindness': 'sector_blindness',
}
SENSOR_ALIASES = {
    'imu': 'imu',
    'wheel': 'wheel',
    'wheel_odometry': 'wheel',
    'lidar': 'scan',
    'laser_scan': 'scan',
    'scan': 'scan',
}


def seconds_from_stamp(stamp):
    """Convert a ROS Time message to floating-point seconds."""
    return stamp.sec + stamp.nanosec / 1_000_000_000.0


def canonical_sensor(sensor):
    """Map FaultStatus and SensorHealth sensor names to health topic names."""
    normalized = sensor.strip().lower()
    return SENSOR_ALIASES.get(normalized, normalized)


def safe_ratio(numerator, denominator):
    """Return a JSON-safe ratio, using zero for an undefined denominator."""
    return numerator / denominator if denominator else 0.0


@dataclass
class ConfusionCounts:
    """Counts for one sensor, excluding UNKNOWN predictions from the matrix."""

    tp: int = 0
    fp: int = 0
    fn: int = 0
    tn: int = 0
    unknown: int = 0

    def add(self, truth_positive, predicted_positive, unknown):
        """Add one aligned health observation."""
        if unknown:
            self.unknown += 1
        elif truth_positive and predicted_positive:
            self.tp += 1
        elif truth_positive:
            self.fn += 1
        elif predicted_positive:
            self.fp += 1
        else:
            self.tn += 1

    def as_dict(self):
        """Return counts and derived metrics in the report schema."""
        precision = safe_ratio(self.tp, self.tp + self.fp)
        recall = safe_ratio(self.tp, self.tp + self.fn)
        return {
            'tp': self.tp,
            'fp': self.fp,
            'fn': self.fn,
            'tn': self.tn,
            'unknown': self.unknown,
            'evaluated': self.tp + self.fp + self.fn + self.tn,
            'precision': precision,
            'recall': recall,
            'f1': safe_ratio(2.0 * precision * recall, precision + recall),
        }


@dataclass
class EvaluationEvent:
    """One FaultStatus event and the health messages aligned to it."""

    event_id: str
    sensor: str
    model: str
    start_time_sec: float
    end_time_sec: float
    transitions: list = field(default_factory=list)
    settled: bool = False
    first_alarm_time_sec: float = None
    positive_health_samples: int = 0
    alarm_count: int = 0
    matched_classification_count: int = 0
    mismatched_classification_count: int = 0
    observed_faults: set = field(default_factory=set)

    def record_status(self, state, stamp_sec):
        """Record a state transition once, while accepting repeated statuses."""
        transition = (stamp_sec, state)
        if transition not in self.transitions:
            self.transitions.append(transition)
            self.transitions.sort(key=lambda item: item[0])
        if state == FaultStatus.ENDED:
            self.settled = True

    def state_at(self, stamp_sec):
        """Return the latest known FaultStatus state at simulation time."""
        state = None
        for transition_time_sec, candidate in self.transitions:
            if transition_time_sec > stamp_sec:
                break
            state = candidate
        return state

    @property
    def expected_classification(self):
        """Return the expected health fault label for this truth model."""
        return FAULT_CLASSIFICATIONS.get(self.model)

    def record_alarm(self, stamp_sec, detected_fault):
        """Record an event-local positive prediction and its classification."""
        self.alarm_count += 1
        if self.first_alarm_time_sec is None:
            self.first_alarm_time_sec = stamp_sec
        self.observed_faults.add(detected_fault)
        expected = self.expected_classification
        if expected is None:
            return
        if detected_fault == expected:
            self.matched_classification_count += 1
        else:
            self.mismatched_classification_count += 1

    def as_dict(self):
        """Return a completed or in-progress event result."""
        expected = self.expected_classification
        if expected is None or self.alarm_count == 0:
            classification_matched = None
        else:
            classification_matched = self.matched_classification_count > 0
        delay_sec = None
        if self.first_alarm_time_sec is not None:
            delay_sec = self.first_alarm_time_sec - self.start_time_sec
        return {
            'event_id': self.event_id,
            'sensor': self.sensor,
            'model': self.model,
            'start_time_sec': self.start_time_sec,
            'end_time_sec': self.end_time_sec,
            'settled': self.settled,
            'positive_health_samples': self.positive_health_samples,
            'first_alarm_time_sec': self.first_alarm_time_sec,
            'detection_delay_sec': delay_sec,
            'missed': self.first_alarm_time_sec is None,
            'classification': {
                'expected': expected,
                'matched': classification_matched,
                'matched_alarm_count': self.matched_classification_count,
                'mismatched_alarm_count': self.mismatched_classification_count,
                'observed_faults': sorted(self.observed_faults),
            },
        }


class HealthEvaluationAccumulator:
    """Pure-Python accumulator used by the ROS node and unit tests."""

    def __init__(self, sensors=('imu', 'wheel', 'scan')):
        self._counts = {sensor: ConfusionCounts() for sensor in sensors}
        self._events = {}

    def record_fault_status(self, msg):
        """Ingest active FaultStatus events without creating duplicates."""
        sensor = canonical_sensor(msg.sensor)
        key = (sensor, msg.event_id)
        event = self._events.get(key)
        if event is None and msg.state != FaultStatus.ACTIVE:
            return
        if event is None:
            event = EvaluationEvent(
                event_id=msg.event_id,
                sensor=sensor,
                model=msg.model,
                start_time_sec=seconds_from_stamp(msg.start_time),
                end_time_sec=seconds_from_stamp(msg.end_time),
            )
            self._events[key] = event
        event.record_status(msg.state, seconds_from_stamp(msg.header.stamp))

    def record_health(self, msg):
        """Align one SensorHealth output with known same-sensor truth states."""
        sensor = canonical_sensor(msg.sensor)
        if sensor not in self._counts:
            self._counts[sensor] = ConfusionCounts()
        stamp_sec = seconds_from_stamp(msg.header.stamp)
        active_events = [
            event for event in self._events.values()
            if event.sensor == sensor
            and event.state_at(stamp_sec) == FaultStatus.ACTIVE
        ]
        truth_positive = bool(active_events)
        unknown = msg.state == SensorHealth.UNKNOWN
        predicted_positive = msg.state in (SensorHealth.DEGRADED, SensorHealth.FAULT)
        self._counts[sensor].add(truth_positive, predicted_positive, unknown)
        for event in active_events:
            event.positive_health_samples += 1
            if predicted_positive:
                event.record_alarm(stamp_sec, msg.detected_fault)

    def result(self):
        """Build the stable JSON-serializable evaluation result."""
        total = ConfusionCounts()
        sensors = {}
        for sensor in sorted(self._counts):
            counts = self._counts[sensor]
            sensors[sensor] = counts.as_dict()
            total.tp += counts.tp
            total.fp += counts.fp
            total.fn += counts.fn
            total.tn += counts.tn
            total.unknown += counts.unknown
        events = sorted(
            (event.as_dict() for event in self._events.values()),
            key=lambda event: (event['sensor'], event['event_id']),
        )
        return {
            'schema_version': 1,
            'sensors': sensors,
            'total': total.as_dict(),
            'event_count': len(events),
            'events': events,
        }


class HealthEvaluatorNode(Node):
    """Subscribe to health and truth topics, then report evaluation results."""

    def __init__(self):
        super().__init__('health_evaluator')
        self.declare_parameter('imu_health_topic', DEFAULT_HEALTH_TOPICS['imu'])
        self.declare_parameter('wheel_health_topic', DEFAULT_HEALTH_TOPICS['wheel'])
        self.declare_parameter('scan_health_topic', DEFAULT_HEALTH_TOPICS['scan'])
        self.declare_parameter('fault_status_topic', DEFAULT_FAULT_STATUS_TOPIC)
        self.declare_parameter('output_json_path', '')
        self._accumulator = HealthEvaluationAccumulator()
        self._result_emitted = False
        self.create_subscription(
            SensorHealth,
            self.get_parameter('imu_health_topic').value,
            self._on_health,
            10,
        )
        self.create_subscription(
            SensorHealth,
            self.get_parameter('wheel_health_topic').value,
            self._on_health,
            10,
        )
        self.create_subscription(
            SensorHealth,
            self.get_parameter('scan_health_topic').value,
            self._on_health,
            10,
        )
        self.create_subscription(
            FaultStatus,
            self.get_parameter('fault_status_topic').value,
            self._on_fault_status,
            10,
        )

    def _on_health(self, msg):
        """Accumulate one health prediction."""
        self._accumulator.record_health(msg)

    def _on_fault_status(self, msg):
        """Accumulate one truth state without changing the health monitor."""
        self._accumulator.record_fault_status(msg)

    def emit_result(self):
        """Print the complete JSON report and optionally persist it to a file."""
        if self._result_emitted:
            return
        self._result_emitted = True
        serialized = json.dumps(self._accumulator.result(), indent=2, sort_keys=True)
        print(serialized)
        output_json_path = self.get_parameter('output_json_path').value
        if output_json_path:
            Path(output_json_path).write_text(serialized + '\n', encoding='utf-8')


def main(args=None):
    """Run the health evaluator until ROS shutdown, then emit its report."""
    rclpy.init(args=args)
    node = HealthEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emit_result()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
