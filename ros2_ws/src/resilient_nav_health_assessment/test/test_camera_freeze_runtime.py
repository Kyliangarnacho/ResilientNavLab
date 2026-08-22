"""Runtime integration for the isolated camera freeze test chain."""

import os
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from resilient_nav_interfaces.msg import SensorHealth
from sensor_msgs.msg import Image


def _start_process(command, environment, log_path):
    log_stream = log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(
        command,
        env=environment,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
        # Avoid fork() from pytest's multi-threaded DDS process.  This
        # isolated subprocess needs no descriptor-closing policy, permitting
        # Python to use posix_spawn.
        close_fds=False,
    )
    return process, log_stream


def _stop_process(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


def _assert_running(process, log_stream, log_path):
    if process.poll() is not None:
        log_stream.flush()
        raise AssertionError(log_path.read_text(encoding='utf-8'))


def _valid_image(node, pixels):
    message = Image()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = 'runtime_camera_optical_frame'
    message.height = pixels.shape[0]
    message.width = pixels.shape[1]
    message.encoding = 'rgb8'
    message.step = pixels.shape[1] * pixels.shape[2]
    message.data = pixels.tobytes()
    return message


def _has_subscription(node, topic, node_name):
    """Return whether the ROS graph contains the expected subscription."""
    return any(
        endpoint.node_name == node_name
        for endpoint in node.get_subscriptions_info_by_topic(topic)
    )


def test_freeze_source_drives_monitor_to_freeze_fault_without_stale(
    tmp_path, monkeypatch,
):
    """Exercise real ROS messages through source, frozen topic, and monitor."""
    domain_id = 1 + (os.getpid() % 231)
    suffix = f'run_{os.getpid()}'
    source_topic = f'/test/runtime/{suffix}/camera/source'
    frozen_topic = f'/test/runtime/{suffix}/camera/frozen'
    health_topic = f'/test/runtime/{suffix}/health/camera'
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(domain_id)
    environment['ROS_LOG_DIR'] = str(tmp_path / 'ros_logs')
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain_id))
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))

    context = Context()
    rclpy.init(context=context, domain_id=domain_id)
    probe = Node('camera_freeze_runtime_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    source_publisher = probe.create_publisher(
        Image, source_topic, qos_profile_sensor_data
    )
    frozen_samples = []
    health_samples = []

    def on_frozen(message):
        stamp_sec = (
            message.header.stamp.sec
            + message.header.stamp.nanosec * 1e-9
        )
        frozen_samples.append({
            'stamp_sec': stamp_sec,
            'data': bytes(message.data),
            'frame_id': message.header.frame_id,
            'encoding': message.encoding,
            'width': message.width,
            'height': message.height,
        })

    def on_health(message):
        health_samples.append((
            message.state,
            message.detected_fault,
            message.sample_count,
        ))

    probe.create_subscription(
        Image, frozen_topic, on_frozen, qos_profile_sensor_data
    )
    probe.create_subscription(
        SensorHealth, health_topic, on_health, 10
    )

    source_process = None
    monitor_process = None
    source_log = None
    monitor_log = None
    source_log_path = tmp_path / 'camera_freeze_source.log'
    monitor_log_path = tmp_path / 'camera_health_monitor.log'
    pixels = np.arange(16 * 24 * 3, dtype=np.uint8).reshape(16, 24, 3)
    try:
        source_process, source_log = _start_process(
            [
                'ros2', 'run', 'resilient_nav_health_assessment',
                'camera_freeze_source', '--ros-args',
                '-p', f'source_topic:={source_topic}',
                '-p', f'output_topic:={frozen_topic}',
                '-p', 'publish_rate_hz:=30.0',
            ],
            environment,
            source_log_path,
        )

        # A discovery graph query can remain empty during Fast DDS startup.
        # Repeated valid source frames and the first frozen output prove the
        # actual pub/sub path is ready without relying on that timing.
        frozen_deadline = time.monotonic() + 6.0
        while not frozen_samples and time.monotonic() < frozen_deadline:
            _assert_running(source_process, source_log, source_log_path)
            source_publisher.publish(_valid_image(probe, pixels))
            executor.spin_once(timeout_sec=0.05)
        assert frozen_samples, source_log_path.read_text(encoding='utf-8')

        monitor_process, monitor_log = _start_process(
            [
                'ros2', 'run', 'resilient_nav_health_assessment',
                'camera_health_monitor', '--ros-args',
                '-p', f'image_topic:={frozen_topic}',
                '-p', f'health_topic:={health_topic}',
                '-p', 'publish_rate_hz:=20.0',
                '-p', 'window_duration_sec:=1.0',
                '-p', 'min_samples:=3',
                '-p', 'stale_timeout_sec:=0.6',
                '-p', 'freeze_duration_sec:=0.25',
                '-p', 'fault_confirmation_sec:=0.15',
                '-p', 'recovery_confirmation_sec:=0.2',
            ],
            environment,
            monitor_log_path,
        )

        monitor_discovery_deadline = time.monotonic() + 6.0
        while (
            not _has_subscription(
                probe, frozen_topic, 'camera_health_monitor'
            )
            and time.monotonic() < monitor_discovery_deadline
        ):
            _assert_running(monitor_process, monitor_log, monitor_log_path)
            executor.spin_once(timeout_sec=0.05)
        assert _has_subscription(
            probe, frozen_topic, 'camera_health_monitor'
        ), monitor_log_path.read_text(encoding='utf-8')

        warmup_deadline = time.monotonic() + 4.0
        while (
            not any(
                fault != 'stale' and sample_count >= 3
                for _, fault, sample_count in health_samples
            )
            and time.monotonic() < warmup_deadline
        ):
            _assert_running(source_process, source_log, source_log_path)
            _assert_running(monitor_process, monitor_log, monitor_log_path)
            executor.spin_once(timeout_sec=0.05)
        assert any(
            fault != 'stale' and sample_count >= 3
            for _, fault, sample_count in health_samples
        ), monitor_log_path.read_text(encoding='utf-8')
        health_samples.clear()

        fault_deadline = time.monotonic() + 8.0
        while time.monotonic() < fault_deadline:
            _assert_running(source_process, source_log, source_log_path)
            _assert_running(monitor_process, monitor_log, monitor_log_path)
            executor.spin_once(timeout_sec=0.05)
            if any(
                state == SensorHealth.FAULT and fault == 'freeze'
                for state, fault, _ in health_samples
            ):
                break

        assert len(frozen_samples) >= 5
        expected_data = pixels.tobytes()
        assert all(sample['data'] == expected_data for sample in frozen_samples)
        assert all(
            later['stamp_sec'] > earlier['stamp_sec']
            for earlier, later in zip(frozen_samples, frozen_samples[1:])
        )
        assert all(
            sample['frame_id'] == 'runtime_camera_optical_frame'
            and sample['encoding'] == 'rgb8'
            and sample['width'] == 24
            and sample['height'] == 16
            for sample in frozen_samples
        )
        assert any(
            state == SensorHealth.FAULT and fault == 'freeze'
            for state, fault, _ in health_samples
        ), monitor_log_path.read_text(encoding='utf-8')
        assert all(fault != 'stale' for _, fault, _ in health_samples)
    finally:
        _stop_process(monitor_process)
        _stop_process(source_process)
        if monitor_log is not None:
            monitor_log.close()
        if source_log is not None:
            source_log.close()
        executor.remove_node(probe)
        probe.destroy_node()
        executor.shutdown()
        context.shutdown()
