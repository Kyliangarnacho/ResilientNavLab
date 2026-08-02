import random

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)

from resilient_nav_fault_injection.imu_fault_models import (
    apply_imu_fault,
    BIAS_MODEL,
    DROPOUT_MODEL,
    FaultStatusChangeTracker,
    FIXED_DELAY_MODEL,
    GAUSSIAN_NOISE_MODEL,
    ImuFixedDelayQueue,
    make_imu_fault_status,
    validate_delay_sec,
    validate_dropout_probability,
    validate_model,
    validate_noise_sigma,
    validate_time_window,
)
from resilient_nav_interfaces.msg import FaultStatus
from sensor_msgs.msg import Imu


class ImuFaultInjector(Node):
    """Inject a configured fault model into IMU messages."""

    def __init__(
        self,
        *,
        node_name='imu_fault_injector',
        default_model=BIAS_MODEL,
    ):
        super().__init__(node_name)

        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/faulted/imu/data')
        self.declare_parameter('status_topic', '/fault_injection/status')
        self.declare_parameter('scenario_id', 'imu_bias_demo')
        self.declare_parameter('scenario_seed', 0)
        self.declare_parameter('event_id', 'imu_bias_001')
        self.declare_parameter('enabled', True)
        self.declare_parameter('model', default_model)
        self.declare_parameter('bias_rad_s', 0.15)
        self.declare_parameter('noise_sigma_rad_s', 0.05)
        self.declare_parameter('dropout_probability', 0.30)
        self.declare_parameter('delay_sec', 0.50)
        self.declare_parameter('start_time_sec', 0.0)
        self.declare_parameter('end_time_sec', 10.0)

        self._input_topic = (
            self.get_parameter('input_topic').get_parameter_value().string_value
        )
        self._output_topic = (
            self.get_parameter('output_topic').get_parameter_value().string_value
        )
        status_topic = (
            self.get_parameter('status_topic').get_parameter_value().string_value
        )
        self._scenario_id = (
            self.get_parameter('scenario_id').get_parameter_value().string_value
        )
        self._scenario_seed = (
            self.get_parameter('scenario_seed')
            .get_parameter_value()
            .integer_value
        )
        self._event_id = (
            self.get_parameter('event_id').get_parameter_value().string_value
        )
        self._start_time_sec = (
            self.get_parameter('start_time_sec')
            .get_parameter_value()
            .double_value
        )
        self._end_time_sec = (
            self.get_parameter('end_time_sec').get_parameter_value().double_value
        )

        model = self._get_model()
        validate_model(model)
        validate_time_window(self._start_time_sec, self._end_time_sec)
        validate_noise_sigma(self._get_noise_sigma_rad_s())
        validate_dropout_probability(self._get_dropout_probability())
        validate_delay_sec(self._get_delay_sec())

        self._fault_rng = random.Random(self._scenario_seed)
        self._delay_queue = ImuFixedDelayQueue(self._get_delay_sec())

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_tracker = FaultStatusChangeTracker()
        self._status_publisher = self.create_publisher(
            FaultStatus,
            status_topic,
            status_qos,
        )
        self._publisher = self.create_publisher(
            Imu,
            self._output_topic,
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            Imu,
            self._input_topic,
            self._on_imu,
            qos_profile_sensor_data,
        )
        self._release_timer = self.create_timer(
            0.01,
            self._publish_ready_delayed_messages,
        )

    def _on_imu(self, msg):
        model = self._get_model()
        enabled = self._get_enabled()
        bias_rad_s = self._get_bias_rad_s()
        noise_sigma_rad_s = self._get_noise_sigma_rad_s()
        dropout_probability = self._get_dropout_probability()
        delay_sec = self._get_delay_sec()

        status_msg = make_imu_fault_status(
            msg,
            model=model,
            enabled=enabled,
            bias_rad_s=bias_rad_s,
            noise_sigma_rad_s=noise_sigma_rad_s,
            dropout_probability=dropout_probability,
            delay_sec=delay_sec,
            start_time_sec=self._start_time_sec,
            end_time_sec=self._end_time_sec,
            scenario_id=self._scenario_id,
            scenario_seed=self._scenario_seed,
            event_id=self._event_id,
            source_topic=self._input_topic,
            faulted_topic=self._output_topic,
        )
        changed_status_msg = self._status_tracker.first_or_changed(status_msg)
        if changed_status_msg is not None:
            self._status_publisher.publish(changed_status_msg)

        if model == FIXED_DELAY_MODEL:
            immediate_msg = self._delay_queue.handle_message(
                msg,
                enabled=enabled,
                current_ros_time_sec=self._current_ros_time_sec(),
                start_time_sec=self._start_time_sec,
                end_time_sec=self._end_time_sec,
            )
            if immediate_msg is not None:
                self._publisher.publish(immediate_msg)
            self._publish_ready_delayed_messages()
            return

        faulted_msg = apply_imu_fault(
            msg,
            model=model,
            enabled=enabled,
            bias_rad_s=bias_rad_s,
            noise_sigma_rad_s=noise_sigma_rad_s,
            dropout_probability=dropout_probability,
            delay_sec=delay_sec,
            start_time_sec=self._start_time_sec,
            end_time_sec=self._end_time_sec,
            noise_rng=self._fault_rng,
        )
        if faulted_msg is not None:
            self._publisher.publish(faulted_msg)

    def _get_enabled(self):
        return self.get_parameter('enabled').get_parameter_value().bool_value

    def _get_model(self):
        return self.get_parameter('model').get_parameter_value().string_value

    def _get_bias_rad_s(self):
        return self.get_parameter('bias_rad_s').get_parameter_value().double_value

    def _get_noise_sigma_rad_s(self):
        return (
            self.get_parameter('noise_sigma_rad_s')
            .get_parameter_value()
            .double_value
        )

    def _get_dropout_probability(self):
        return (
            self.get_parameter('dropout_probability')
            .get_parameter_value()
            .double_value
        )

    def _get_delay_sec(self):
        return self.get_parameter('delay_sec').get_parameter_value().double_value

    def _current_ros_time_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_ready_delayed_messages(self):
        for ready_msg in self._delay_queue.pop_ready(self._current_ros_time_sec()):
            self._publisher.publish(ready_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImuFaultInjector()
    except ValueError as exc:
        logger = rclpy.logging.get_logger('imu_fault_injector')
        logger.fatal(f'Invalid IMU fault injector configuration: {exc}')
        rclpy.shutdown()
        raise SystemExit(1) from exc
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def bias_main(args=None):
    rclpy.init(args=args)
    try:
        node = ImuFaultInjector(
            node_name='imu_bias_injector',
            default_model=BIAS_MODEL,
        )
    except ValueError as exc:
        logger = rclpy.logging.get_logger('imu_bias_injector')
        logger.fatal(f'Invalid IMU bias injector configuration: {exc}')
        rclpy.shutdown()
        raise SystemExit(1) from exc
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = [
    'BIAS_MODEL',
    'DROPOUT_MODEL',
    'FIXED_DELAY_MODEL',
    'GAUSSIAN_NOISE_MODEL',
    'ImuFaultInjector',
    'bias_main',
    'main',
]
