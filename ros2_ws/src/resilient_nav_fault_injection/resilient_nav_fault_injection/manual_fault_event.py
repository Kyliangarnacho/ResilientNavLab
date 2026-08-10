"""Publish a timed FaultStatus window for a manually applied fault."""

from dataclasses import dataclass
from math import isfinite

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from resilient_nav_fault_injection.imu_fault_models import seconds_to_stamp
from resilient_nav_interfaces.msg import FaultStatus


DEFAULT_STATUS_TOPIC = '/fault_injection/status'


@dataclass(frozen=True)
class ManualFaultConfiguration:
    """Validated metadata and timing for one manual truth event."""

    sensor: str
    model: str
    event_id: str
    severity: float
    start_delay_sec: float
    duration_sec: float

    def validated(self):
        """Return a normalized configuration or raise for unsafe input."""
        sensor = self.sensor.strip().lower()
        model = self.model.strip().lower()
        event_id = self.event_id.strip()
        if not sensor:
            raise ValueError('sensor must not be empty')
        if not model:
            raise ValueError('model must not be empty')
        if not event_id:
            raise ValueError('event_id must not be empty')
        if not isfinite(self.severity) or not 0.0 <= self.severity <= 1.0:
            raise ValueError('severity must be finite and within [0, 1]')
        if not isfinite(self.start_delay_sec) or self.start_delay_sec < 0.0:
            raise ValueError('start_delay_sec must be finite and non-negative')
        if not isfinite(self.duration_sec) or self.duration_sec <= 0.0:
            raise ValueError('duration_sec must be finite and greater than zero')
        return ManualFaultConfiguration(
            sensor=sensor,
            model=model,
            event_id=event_id,
            severity=float(self.severity),
            start_delay_sec=float(self.start_delay_sec),
            duration_sec=float(self.duration_sec),
        )


class ManualFaultTimeline:
    """Emit each scheduled truth transition exactly once, even after delay."""

    def __init__(self, created_sec, start_sec, end_sec):
        if start_sec < created_sec or end_sec <= start_sec:
            raise ValueError('manual fault timeline must be ordered')
        self._transitions = [
            (float(created_sec), FaultStatus.SCHEDULED),
            (float(start_sec), FaultStatus.ACTIVE),
            (float(end_sec), FaultStatus.ENDED),
        ]
        self._next_index = 0

    @property
    def complete(self):
        """Return whether the ENDED transition has been emitted."""
        return self._next_index == len(self._transitions)

    def states_due(self, now_sec):
        """Return every transition due by now without dropping late states."""
        due = []
        while (
            self._next_index < len(self._transitions)
            and self._transitions[self._next_index][0] <= now_sec
        ):
            due.append(self._transitions[self._next_index][1])
            self._next_index += 1
        return due


def make_manual_fault_status(
    configuration,
    *,
    state,
    stamp_sec,
    start_sec,
    end_sec,
):
    """Build one existing FaultStatus message without altering sensor data."""
    message = FaultStatus()
    message.header.stamp = seconds_to_stamp(stamp_sec)
    message.scenario_id = 'manual_fault_event'
    message.scenario_seed = 0
    message.event_id = configuration.event_id
    message.sensor = configuration.sensor
    message.model = configuration.model
    message.start_time = seconds_to_stamp(start_sec)
    message.end_time = seconds_to_stamp(end_sec)
    message.state = state
    message.severity = configuration.severity
    message.parameters_yaml = (
        'manual_event: true\n'
        f'start_delay_sec: {configuration.start_delay_sec}\n'
        f'duration_sec: {configuration.duration_sec}'
    )
    return message


class ManualFaultEvent(Node):
    """Publish truth labels for a real fault applied by an operator."""

    def __init__(self):
        super().__init__('manual_fault_event')
        self.declare_parameter('status_topic', DEFAULT_STATUS_TOPIC)
        self.declare_parameter('sensor', 'camera')
        self.declare_parameter('model', 'freeze')
        self.declare_parameter('event_id', 'manual_camera_fault_001')
        self.declare_parameter('severity', 1.0)
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('duration_sec', 5.0)

        status_topic = str(self.get_parameter('status_topic').value).strip()
        if not status_topic:
            raise ValueError('status_topic must not be empty')
        self._configuration = ManualFaultConfiguration(
            sensor=str(self.get_parameter('sensor').value),
            model=str(self.get_parameter('model').value),
            event_id=str(self.get_parameter('event_id').value),
            severity=float(self.get_parameter('severity').value),
            start_delay_sec=float(
                self.get_parameter('start_delay_sec').value
            ),
            duration_sec=float(self.get_parameter('duration_sec').value),
        ).validated()

        created_sec = self._now_sec()
        self._start_sec = (
            created_sec + self._configuration.start_delay_sec
        )
        self._end_sec = self._start_sec + self._configuration.duration_sec
        self._timeline = ManualFaultTimeline(
            created_sec, self._start_sec, self._end_sec
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            FaultStatus, status_topic, status_qos
        )
        self._timer = self.create_timer(0.02, self._publish_due_states)
        self._publish_due_states()

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_due_states(self):
        now_sec = self._now_sec()
        for state in self._timeline.states_due(now_sec):
            message = make_manual_fault_status(
                self._configuration,
                state=state,
                stamp_sec=now_sec,
                start_sec=self._start_sec,
                end_sec=self._end_sec,
            )
            self._publisher.publish(message)
            self.get_logger().info(
                f'Published manual FaultStatus state={state} '
                f'event_id={self._configuration.event_id}'
            )
        if self._timeline.complete:
            self._timer.cancel()


def main(args=None):
    """Run the manual fault truth event publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = ManualFaultEvent()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        rclpy.logging.get_logger('manual_fault_event').fatal(str(error))
        raise SystemExit(1) from error
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
