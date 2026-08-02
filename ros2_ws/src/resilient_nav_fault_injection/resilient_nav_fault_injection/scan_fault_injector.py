import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)

from resilient_nav_fault_injection.imu_fault_models import (
    FaultStatusChangeTracker,
    validate_time_window,
)
from resilient_nav_fault_injection.scan_fault_models import (
    apply_scan_fault,
    make_scan_fault_status,
    SECTOR_BLINDNESS_MODEL,
    validate_model,
    validate_sector_width,
)
from resilient_nav_interfaces.msg import FaultStatus
from sensor_msgs.msg import LaserScan


class ScanFaultInjector(Node):
    """Inject a configured fault model into LaserScan messages."""

    def __init__(self):
        super().__init__('scan_fault_injector')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/faulted/scan')
        self.declare_parameter('status_topic', '/fault_injection/status')
        self.declare_parameter('scenario_id', 'scan_sector_blindness_demo')
        self.declare_parameter('scenario_seed', 0)
        self.declare_parameter('event_id', 'scan_sector_blindness_001')
        self.declare_parameter('enabled', True)
        self.declare_parameter('model', SECTOR_BLINDNESS_MODEL)
        self.declare_parameter('sector_center_rad', 0.0)
        self.declare_parameter('sector_width_rad', 1.0)
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

        validate_model(self._get_model())
        validate_sector_width(self._get_sector_width_rad())
        validate_time_window(self._start_time_sec, self._end_time_sec)

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
            LaserScan,
            self._output_topic,
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            LaserScan,
            self._input_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

    def _on_scan(self, msg):
        model = self._get_model()
        enabled = self._get_enabled()
        sector_center_rad = self._get_sector_center_rad()
        sector_width_rad = self._get_sector_width_rad()

        status_msg = make_scan_fault_status(
            msg,
            model=model,
            enabled=enabled,
            sector_center_rad=sector_center_rad,
            sector_width_rad=sector_width_rad,
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

        faulted_msg = apply_scan_fault(
            msg,
            model=model,
            enabled=enabled,
            sector_center_rad=sector_center_rad,
            sector_width_rad=sector_width_rad,
            start_time_sec=self._start_time_sec,
            end_time_sec=self._end_time_sec,
        )
        self._publisher.publish(faulted_msg)

    def _get_enabled(self):
        return self.get_parameter('enabled').get_parameter_value().bool_value

    def _get_model(self):
        return self.get_parameter('model').get_parameter_value().string_value

    def _get_sector_center_rad(self):
        return (
            self.get_parameter('sector_center_rad')
            .get_parameter_value()
            .double_value
        )

    def _get_sector_width_rad(self):
        return (
            self.get_parameter('sector_width_rad')
            .get_parameter_value()
            .double_value
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ScanFaultInjector()
    except ValueError as exc:
        logger = rclpy.logging.get_logger('scan_fault_injector')
        logger.fatal(f'Invalid scan fault injector configuration: {exc}')
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
    'SECTOR_BLINDNESS_MODEL',
    'ScanFaultInjector',
    'main',
]
