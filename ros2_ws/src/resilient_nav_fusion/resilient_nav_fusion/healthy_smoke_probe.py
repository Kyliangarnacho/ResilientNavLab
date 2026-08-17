"""One-shot ROS probe that records Phase 8 healthy-path runtime evidence."""

import json
from pathlib import Path
from time import monotonic

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from resilient_nav_interfaces.msg import FusionStatus
from sensor_msgs.msg import Imu

from .healthy_smoke import HeaderObservation, HealthySmokeObserver


class HealthySmokeProbe(Node):
    """Exit successfully only when all three healthy adaptive streams persist."""

    def __init__(self):
        super().__init__('phase8_healthy_smoke_probe')
        self.declare_parameter('required_samples', 3)
        self.declare_parameter('timeout_sec', 20.0)
        self.declare_parameter(
            'evidence_output', '/tmp/phase8_healthy_smoke_evidence.json'
        )
        self._observer = HealthySmokeObserver(
            required_samples=int(self.get_parameter('required_samples').value)
        )
        self._timeout_sec = float(self.get_parameter('timeout_sec').value)
        self._evidence_output = Path(
            str(self.get_parameter('evidence_output').value)
        )
        self._start_monotonic = monotonic()
        self._finished = False

        self.create_subscription(
            Odometry,
            '/fusion/input/wheel/odometry',
            self._on_wheel,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            '/fusion/input/imu/data',
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/odometry/adaptive',
            self._on_adaptive,
            qos_profile_sensor_data,
        )
        self.create_subscription(FusionStatus, '/fusion/status', self._on_status, 10)
        self.create_timer(0.1, self._check_completion)

    def _on_wheel(self, message: Odometry) -> None:
        self._observer.observe_wheel(_header_observation(message))

    def _on_imu(self, message: Imu) -> None:
        self._observer.observe_imu(_header_observation(message))

    def _on_adaptive(self, message: Odometry) -> None:
        self._observer.observe_adaptive(_header_observation(message))

    def _on_status(self, message: FusionStatus) -> None:
        self._observer.observe_fusion_state(message.state)

    def _check_completion(self) -> None:
        if self._finished:
            return
        if self._observer.passed:
            self._finish('PASS', exit_code=0)
            return
        elapsed_sec = monotonic() - self._start_monotonic
        if elapsed_sec >= self._timeout_sec:
            self._finish('FAIL', exit_code=1)

    def _finish(self, outcome: str, *, exit_code: int) -> None:
        self._finished = True
        evidence = {
            'outcome': outcome,
            'topics': {
                '/fusion/input/wheel/odometry': 'nav_msgs/msg/Odometry',
                '/fusion/input/imu/data': 'sensor_msgs/msg/Imu',
                '/odometry/adaptive': 'nav_msgs/msg/Odometry',
                '/fusion/status': 'resilient_nav_interfaces/msg/FusionStatus',
            },
            'observation': self._observer.summary(),
        }
        self._evidence_output.write_text(
            json.dumps(evidence, sort_keys=True) + '\n', encoding='utf-8'
        )
        self.get_logger().info(
            f'PHASE8_HEALTHY_SMOKE_{outcome} {json.dumps(evidence, sort_keys=True)}'
        )
        if exit_code:
            self.get_logger().error('phase8 healthy smoke requirements were not met')
            self.destroy_node()
            rclpy.shutdown()
            raise SystemExit(exit_code)
        self.destroy_node()
        rclpy.shutdown()


def _header_observation(message: Odometry | Imu) -> HeaderObservation:
    """Map only timestamp and frame values into the ROS-free smoke contract."""
    return HeaderObservation(
        frame_id=message.header.frame_id,
        stamp_sec=message.header.stamp.sec,
        stamp_nanosec=message.header.stamp.nanosec,
        child_frame_id=getattr(message, 'child_frame_id', ''),
    )


def main(args=None):
    """Run the one-shot healthy smoke probe."""
    rclpy.init(args=args)
    node = HealthySmokeProbe()
    try:
        rclpy.spin(node)
    finally:
        if not node._finished:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
