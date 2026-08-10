"""Runtime coverage for timed manual FaultStatus transitions."""

import os
import signal
import subprocess
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from resilient_nav_interfaces.msg import FaultStatus


def test_manual_fault_event_publishes_active_then_ended(tmp_path):
    """Verify the installed node automatically advances the truth window."""
    domain_id = 1 + (os.getpid() % 231)
    status_topic = f'/test/manual/run_{os.getpid()}/status'
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(domain_id)
    environment['ROS_LOG_DIR'] = str(tmp_path / 'ros_logs')
    context = Context()
    rclpy.init(context=context, domain_id=domain_id)
    probe = Node('manual_fault_event_runtime_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    status_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    statuses = []

    def on_status(message):
        statuses.append({
            'state': message.state,
            'sensor': message.sensor,
            'model': message.model,
            'event_id': message.event_id,
            'severity': message.severity,
            'start_sec': (
                message.start_time.sec + message.start_time.nanosec * 1e-9
            ),
            'end_sec': (
                message.end_time.sec + message.end_time.nanosec * 1e-9
            ),
        })

    probe.create_subscription(
        FaultStatus, status_topic, on_status, status_qos
    )
    log_path = tmp_path / 'manual_fault_event.log'
    log_stream = log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(
        [
            'ros2', 'run', 'resilient_nav_fault_injection',
            'manual_fault_event', '--ros-args',
            '-p', f'status_topic:={status_topic}',
            '-p', 'sensor:=camera',
            '-p', 'model:=freeze',
            '-p', 'event_id:=runtime_camera_freeze',
            '-p', 'severity:=0.75',
            '-p', 'start_delay_sec:=0.3',
            '-p', 'duration_sec:=0.3',
        ],
        env=environment,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
    )
    try:
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                log_stream.flush()
                raise AssertionError(log_path.read_text(encoding='utf-8'))
            executor.spin_once(timeout_sec=0.05)
            states = [status['state'] for status in statuses]
            if FaultStatus.ENDED in states:
                break

        states = [status['state'] for status in statuses]
        assert FaultStatus.ACTIVE in states
        assert FaultStatus.ENDED in states
        assert states.index(FaultStatus.ACTIVE) < states.index(FaultStatus.ENDED)
        active = next(
            status for status in statuses
            if status['state'] == FaultStatus.ACTIVE
        )
        assert active['sensor'] == 'camera'
        assert active['model'] == 'freeze'
        assert active['event_id'] == 'runtime_camera_freeze'
        assert abs(active['severity'] - 0.75) < 1e-6
        assert abs((active['end_sec'] - active['start_sec']) - 0.3) < 1e-6
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)
        log_stream.close()
        executor.remove_node(probe)
        probe.destroy_node()
        executor.shutdown()
        context.shutdown()
