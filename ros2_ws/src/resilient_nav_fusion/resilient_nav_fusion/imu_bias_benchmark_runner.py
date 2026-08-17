"""Self-terminating, evaluation-only recorder for Phase 8 sensor benchmarks."""

import json
from math import isfinite
from pathlib import Path
from time import monotonic

from resilient_nav_interfaces.msg import FaultStatus, FusionStatus, SensorHealth
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String

from .benchmark_observer import SensorBenchmarkObserver


class ImuBenchmarkRunner(Node):
    """Observe evaluator-only evidence and stop the launch on a final result."""

    def __init__(self):
        super().__init__('phase8_imu_benchmark_runner')
        self.declare_parameter('benchmark_name', 'phase8_imu_bias')
        self.declare_parameter('fault_sensor', 'imu')
        self.declare_parameter('fault_status_sensor', '')
        self.declare_parameter('health_topic', '/health/imu')
        self.declare_parameter('require_alignment_sensitivity', False)
        self.declare_parameter('require_path_evidence', False)
        self.declare_parameter('expected_path_frame', 'odom')
        self.declare_parameter('min_complete_sim_time_sec', 0.0)
        self.declare_parameter(
            'output_json', '/tmp/phase8_imu_benchmark.json'
        )
        self.declare_parameter('timeout_wall_sec', 60.0)
        self._output_json = Path(str(self.get_parameter('output_json').value))
        self._benchmark_name = str(self.get_parameter('benchmark_name').value)
        self._fault_sensor = str(self.get_parameter('fault_sensor').value)
        configured_fault_status_sensor = str(
            self.get_parameter('fault_status_sensor').value
        )
        self._fault_status_sensor = (
            configured_fault_status_sensor or self._fault_sensor
        )
        self._health_topic = str(self.get_parameter('health_topic').value)
        if self._fault_sensor not in {'healthy', 'imu', 'wheel'}:
            raise ValueError('fault_sensor must be healthy, imu, or wheel')
        self._timeout_wall_sec = float(
            self.get_parameter('timeout_wall_sec').value
        )
        self._started_at = monotonic()
        self._observer = SensorBenchmarkObserver(self._fault_sensor)
        self._observation_counts = {
            'fault_status_messages': 0,
            'target_health_messages': 0,
            'fusion_status_messages': 0,
            'evaluator_metrics_messages': 0,
        }
        self._fault_status_observations: list[dict[str, object]] = []
        self._alignment_sensitivity: dict[str, object] | None = None
        self._require_alignment_sensitivity = bool(
            self.get_parameter('require_alignment_sensitivity').value
        )
        self._require_path_evidence = bool(
            self.get_parameter('require_path_evidence').value
        )
        self._expected_path_frame = str(
            self.get_parameter('expected_path_frame').value
        )
        self._min_complete_sim_time_sec = float(
            self.get_parameter('min_complete_sim_time_sec').value
        )
        self._latest_sim_time_sec = 0.0
        self._path_observations = {'ground_truth': 0, 'fixed': 0, 'adaptive': 0}
        self.exit_code: int | None = None
        self.get_logger().info(
            'benchmark target '
            f'sensor={self._fault_sensor} '
            f'fault_status_sensor={self._fault_status_sensor} '
            f'health_topic={self._health_topic}'
        )
        if self._fault_sensor != 'healthy':
            self.create_subscription(
                FaultStatus, '/fault_injection/status', self._on_fault, 10
            )
        if self._health_topic:
            self.create_subscription(
                SensorHealth, self._health_topic, self._on_target_health, 10
            )
        self.create_subscription(
            FusionStatus, '/fusion/status', self._on_fusion, 10
        )
        self.create_subscription(
            String,
            '/evaluation/localization_metrics',
            self._on_metrics,
            10,
        )
        self.create_subscription(
            String,
            '/evaluation/localization_alignment_sweep',
            self._on_alignment_sensitivity,
            10,
        )
        if self._require_path_evidence:
            for name in self._path_observations:
                self.create_subscription(
                    NavPath,
                    f'/evaluation/path/{name}',
                    lambda message, path_name=name: self._on_path(path_name, message),
                    10,
                )
        self.create_timer(
            0.1,
            self._check_completion,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _on_fault(self, message: FaultStatus) -> None:
        self._observation_counts['fault_status_messages'] += 1
        observation = {
            'sensor': message.sensor,
            'state': int(message.state),
            'stamp_sec': _stamp_sec(message.header.stamp),
        }
        if observation not in self._fault_status_observations:
            self._fault_status_observations.append(observation)
        if message.sensor != self._fault_status_sensor:
            return
        stamp_sec = _stamp_sec(message.header.stamp)
        if isfinite(stamp_sec):
            self._observer.observe_fault_status(message.state, stamp_sec)

    def _on_target_health(self, message: SensorHealth) -> None:
        self._observation_counts['target_health_messages'] += 1
        stamp_sec = _stamp_sec(message.header.stamp)
        if isfinite(stamp_sec):
            self._observer.observe_target_health(message.state, stamp_sec)

    def _on_fusion(self, message: FusionStatus) -> None:
        self._observation_counts['fusion_status_messages'] += 1
        stamp_sec = _stamp_sec(message.header.stamp)
        if isfinite(stamp_sec):
            self._latest_sim_time_sec = max(self._latest_sim_time_sec, stamp_sec)
            self._observer.observe_fusion_status(
                message.state,
                stamp_sec,
                accepted_measurements=tuple(message.accepted_measurements),
                rejected_measurements=tuple(message.rejected_measurements),
            )

    def _on_metrics(self, message: String) -> None:
        self._observation_counts['evaluator_metrics_messages'] += 1
        try:
            metrics = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warning('suppressed malformed evaluator metrics')
            return
        self._observer.observe_metrics(metrics)

    def _on_alignment_sensitivity(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warning('suppressed malformed alignment sensitivity')
            return
        windows = value.get('windows_sec') if isinstance(value, dict) else None
        if isinstance(windows, dict):
            self._alignment_sensitivity = windows

    def _on_path(self, name: str, message: NavPath) -> None:
        if message.header.frame_id == self._expected_path_frame and message.poses:
            self._path_observations[name] += 1

    def _check_completion(self) -> None:
        if self.exit_code is not None:
            return
        if self._complete:
            self._finish(0, 'PASS')
            return
        if monotonic() - self._started_at >= self._timeout_wall_sec:
            self._finish(1, 'FAIL', error='benchmark timeout before complete evidence')

    def _finish(self, exit_code: int, outcome: str, *, error: str | None = None) -> None:
        self.exit_code = exit_code
        result = self._observer.result(
            self._benchmark_name,
            outcome,
            error=error,
        )
        result['observation_counts'] = dict(self._observation_counts)
        result['fault_status_observations'] = list(self._fault_status_observations)
        result['alignment_sensitivity'] = self._alignment_sensitivity
        result['path_evidence'] = dict(self._path_observations)
        result['last_sim_time_sec'] = self._latest_sim_time_sec
        self._output_json.write_text(
            json.dumps(result, sort_keys=True) + '\n', encoding='utf-8'
        )
        self.get_logger().info(
            f'PHASE8_SENSOR_BENCHMARK_{outcome} '
            f'{json.dumps(result, sort_keys=True)}'
        )
        rclpy.shutdown()

    @property
    def _complete(self) -> bool:
        """Require requested evaluation-only evidence before declaring PASS."""
        if not self._observer.complete:
            return False
        if self._latest_sim_time_sec < self._min_complete_sim_time_sec:
            return False
        if self._require_alignment_sensitivity and self._alignment_sensitivity is None:
            return False
        return (
            not self._require_path_evidence
            or all(count > 0 for count in self._path_observations.values())
        )


def _stamp_sec(stamp) -> float:
    """Convert valid ROS time to seconds; invalid data cannot create an event."""
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return float('nan')
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def main(args=None):
    """Run the benchmark recorder until success, failure, or interruption."""
    rclpy.init(args=args)
    node = ImuBenchmarkRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.exit_code is None:
            node._finish(1, 'FAIL', error='benchmark interrupted')
    finally:
        code = node.exit_code if node.exit_code is not None else 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
