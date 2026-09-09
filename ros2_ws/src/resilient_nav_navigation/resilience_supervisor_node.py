"""ROS adapter for tolerant navigation-level resilience supervision."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from resilience_supervisor import (
    ResilienceSupervisorPolicy,
    SupervisorConfig,
    SupervisorEvidence,
)

from resilient_nav_interfaces.msg import (
    FusionStatus,
    LocalizationQuality,
    ResilienceStatus,
    SensorHealth,
)


class ResilienceSupervisorNode(Node):
    """Aggregate existing monitors without inspecting benchmark truth."""

    def __init__(self) -> None:
        """Create subscriptions and the periodic supervisor output."""
        super().__init__('resilience_supervisor')
        self._declare_parameters()
        self._policy = ResilienceSupervisorPolicy(SupervisorConfig(
            startup_grace_sec=self._float_parameter('startup_grace_sec'),
            status_stale_sec=self._float_parameter('status_stale_sec'),
            hard_fault_confirmation_sec=self._float_parameter(
                'hard_fault_confirmation_sec'
            ),
        ))
        self._start_sec = self._now_sec()
        self._states = {
            'wheel': None,
            'imu': None,
            'scan': None,
            'fusion': None,
            'localization': None,
        }
        self._received = {name: None for name in self._states}
        self._localization_usable = None
        self._accepted_measurements = ()
        self._publisher = self.create_publisher(
            ResilienceStatus, '/resilience/status', 10
        )
        for sensor in ('wheel', 'imu', 'scan'):
            self.create_subscription(
                SensorHealth,
                f'/health/{sensor}',
                lambda message, name=sensor: self._on_sensor(name, message),
                10,
            )
        self.create_subscription(
            FusionStatus, '/fusion/status', self._on_fusion, 10
        )
        self.create_subscription(
            LocalizationQuality,
            '/localization/quality',
            self._on_localization,
            10,
        )
        publish_rate_hz = self._float_parameter('publish_rate_hz')
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.create_timer(1.0 / publish_rate_hz, self._publish)

    def _declare_parameters(self) -> None:
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('startup_grace_sec', 3.0)
        self.declare_parameter('status_stale_sec', 1.0)
        self.declare_parameter('hard_fault_confirmation_sec', 0.3)

    def _float_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        return value

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_sensor(self, name: str, message: SensorHealth) -> None:
        self._states[name] = int(message.state)
        self._received[name] = self._now_sec()

    def _on_fusion(self, message: FusionStatus) -> None:
        self._states['fusion'] = int(message.state)
        self._received['fusion'] = self._now_sec()
        self._accepted_measurements = tuple(message.accepted_measurements)

    def _on_localization(self, message: LocalizationQuality) -> None:
        self._states['localization'] = int(message.state)
        self._received['localization'] = self._now_sec()
        self._localization_usable = bool(message.localization_usable)

    @staticmethod
    def _age(now_sec, received_sec):
        if received_sec is None:
            return None
        return max(now_sec - received_sec, 0.0)

    def _publish(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1_000_000_000.0
        decision = self._policy.evaluate(
            SupervisorEvidence(
                wheel_state=self._states['wheel'],
                imu_state=self._states['imu'],
                scan_state=self._states['scan'],
                fusion_state=self._states['fusion'],
                localization_state=self._states['localization'],
                localization_usable=self._localization_usable,
                accepted_measurements=self._accepted_measurements,
                wheel_age_sec=self._age(now_sec, self._received['wheel']),
                imu_age_sec=self._age(now_sec, self._received['imu']),
                scan_age_sec=self._age(now_sec, self._received['scan']),
                fusion_age_sec=self._age(now_sec, self._received['fusion']),
                localization_age_sec=self._age(
                    now_sec, self._received['localization']
                ),
            ),
            now_sec,
            max(now_sec - self._start_sec, 0.0),
        )
        message = ResilienceStatus()
        message.header.stamp = now.to_msg()
        message.state = int(decision.state)
        message.navigation_allowed = decision.navigation_allowed
        message.confidence = decision.confidence
        message.hard_condition_elapsed_sec = (
            decision.hard_condition_elapsed_sec
        )
        message.reasons = list(decision.reasons)
        self._publisher.publish(message)


def main(args=None) -> None:
    """Run the Resilience Supervisor."""
    rclpy.init(args=args)
    node = ResilienceSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
