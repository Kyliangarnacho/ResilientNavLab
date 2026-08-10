"""Print concise camera SensorHealth changes and low-rate reminders."""

from math import isfinite
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from resilient_nav_interfaces.msg import SensorHealth


DEFAULT_HEALTH_TOPIC = '/health/camera'
DEFAULT_UNCHANGED_PRINT_PERIOD_SEC = 5.0

WATCH_METRICS = {
    'message_age': 'message_age_sec',
    'rolling_fps': 'rolling_observed_fps',
    'fingerprint_identical_duration': (
        'fingerprint_identical_duration_sec'
    ),
    'confirmation_elapsed': 'fault_confirmation_elapsed_sec',
    'recovery_elapsed': 'recovery_elapsed_sec',
}

STATE_NAMES = {
    SensorHealth.UNKNOWN: 'UNKNOWN',
    SensorHealth.HEALTHY: 'HEALTHY',
    SensorHealth.DEGRADED: 'DEGRADED',
    SensorHealth.FAULT: 'FAULT',
}


def format_camera_health_line(message):
    """Format one stable, single-line camera health observation."""
    metrics = dict(zip(message.metric_names, message.metric_values))

    def value(label):
        return float(metrics.get(WATCH_METRICS[label], float('nan')))

    state_name = STATE_NAMES.get(message.state, f'UNKNOWN_STATE_{message.state}')
    return (
        f'state={state_name} '
        f'health_score={message.health_score:.3f} '
        f'confidence={message.confidence:.3f} '
        f'detected_fault={message.detected_fault or "none"} '
        f'message_age={value("message_age"):.3f}s '
        f'rolling_fps={value("rolling_fps"):.3f}Hz '
        'fingerprint_identical_duration='
        f'{value("fingerprint_identical_duration"):.3f}s '
        f'confirmation_elapsed={value("confirmation_elapsed"):.3f}s '
        f'recovery_elapsed={value("recovery_elapsed"):.3f}s'
    )


class CameraHealthWatchGate:
    """Allow immediate state changes and throttle unchanged messages."""

    def __init__(self, unchanged_print_period_sec):
        if (
            not isfinite(unchanged_print_period_sec)
            or unchanged_print_period_sec <= 0.0
        ):
            raise ValueError(
                'unchanged_print_period_sec must be finite and greater than zero'
            )
        self._period_sec = float(unchanged_print_period_sec)
        self._last_signature = None
        self._last_printed_sec = None

    def should_print(self, message, now_sec):
        """Return true for first/change messages or elapsed reminder periods."""
        signature = (int(message.state), str(message.detected_fault))
        state_changed = signature != self._last_signature
        period_elapsed = (
            self._last_printed_sec is None
            or float(now_sec) - self._last_printed_sec >= self._period_sec
        )
        if not state_changed and not period_elapsed:
            return False
        self._last_signature = signature
        self._last_printed_sec = float(now_sec)
        return True


class CameraHealthWatch(Node):
    """Observe camera health without contributing any health decision."""

    def __init__(self):
        super().__init__('camera_health_watch')
        self.declare_parameter('health_topic', DEFAULT_HEALTH_TOPIC)
        self.declare_parameter(
            'unchanged_print_period_sec',
            DEFAULT_UNCHANGED_PRINT_PERIOD_SEC,
        )
        health_topic = str(self.get_parameter('health_topic').value)
        if not health_topic:
            raise ValueError('health_topic must not be empty')
        self._gate = CameraHealthWatchGate(float(
            self.get_parameter('unchanged_print_period_sec').value
        ))
        self._subscription = self.create_subscription(
            SensorHealth, health_topic, self._on_health, 10
        )

    def _on_health(self, message):
        if self._gate.should_print(message, time.monotonic()):
            self.get_logger().info(format_camera_health_line(message))


def main(args=None):
    """Run the camera health display-only watcher."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraHealthWatch()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
