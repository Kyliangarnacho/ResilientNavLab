"""Runtime checks for optional phase 7.2 camera health launch branches."""

import os
import signal
import subprocess
import time


def _environment(tmp_path, domain_id):
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(domain_id)
    environment['ROS2CLI_DISABLE_DAEMON'] = '1'
    environment['ROS_LOG_DIR'] = str(tmp_path / 'ros_logs')
    return environment


def _start_launch(tmp_path, environment, arguments):
    log_path = tmp_path / 'phase7_2_camera_health.launch.log'
    log_stream = log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(
        [
            'ros2', 'launch', 'resilient_nav_camera',
            'phase7_2_camera_health.launch.py',
            *arguments,
        ],
        env=environment,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
    )
    return process, log_stream, log_path


def _stop_launch(process, log_stream):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)
    log_stream.close()


def _node_names(environment):
    result = subprocess.run(
        ['ros2', 'node', 'list'],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    return set(result.stdout.splitlines()) if result.returncode == 0 else set()


def _wait_for_nodes(process, log_stream, log_path, environment, expected):
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            log_stream.flush()
            raise AssertionError(log_path.read_text(encoding='utf-8'))
        names = _node_names(environment)
        if expected <= names:
            return names
        time.sleep(0.2)
    log_stream.flush()
    raise AssertionError(log_path.read_text(encoding='utf-8'))


def _parameter(environment, node_name, parameter_name):
    result = subprocess.run(
        ['ros2', 'param', 'get', node_name, parameter_name],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    return result.stdout.strip()


def _health_config(tmp_path):
    config_path = tmp_path / 'runtime_camera_health.yaml'
    config_path.write_text(
        'camera_health_monitor:\n'
        '  ros__parameters:\n'
        '    publish_rate_hz: 10.0\n'
        '    stale_timeout_sec: 0.75\n',
        encoding='utf-8',
    )
    return config_path


def test_disabled_evaluator_and_watch_are_not_started(tmp_path):
    """Run monitor-only mode without opening the real C920 device."""
    environment = _environment(tmp_path, 1 + (os.getpid() % 230))
    source_topic = f'/test/run_{os.getpid()}/camera/source_disabled'
    health_config = _health_config(tmp_path)
    process, log_stream, log_path = _start_launch(
        tmp_path,
        environment,
        [
            'run_camera:=false',
            'run_evaluator:=false',
            'run_watch:=false',
            'run_image_view:=false',
            f'source_topic:={source_topic}',
            f'health_config:={health_config}',
        ],
    )
    try:
        names = _wait_for_nodes(
            process,
            log_stream,
            log_path,
            environment,
            {'/camera_health_monitor'},
        )
        time.sleep(0.5)
        names |= _node_names(environment)
        assert '/health_evaluator' not in names
        assert '/camera_health_watch' not in names
        assert '/camera_image_view' not in names
        assert _parameter(
            environment, '/camera_health_monitor', 'image_topic'
        ) == f'String value is: {source_topic}'
        assert _parameter(
            environment, '/camera_health_monitor', 'stale_timeout_sec'
        ) == 'Double value is: 0.75'
    finally:
        _stop_launch(process, log_stream)


def test_enabled_evaluator_and_watch_receive_combined_launch_parameters(tmp_path):
    """Run all health branches without starting the hardware camera branch."""
    environment = _environment(tmp_path, 2 + (os.getpid() % 229))
    source_topic = f'/test/run_{os.getpid()}/camera/source_enabled'
    output_json = tmp_path / 'camera_evaluation.json'
    health_config = _health_config(tmp_path)
    process, log_stream, log_path = _start_launch(
        tmp_path,
        environment,
        [
            'run_camera:=false',
            'run_evaluator:=true',
            'run_watch:=true',
            'run_image_view:=false',
            'use_sim_time:=false',
            f'source_topic:={source_topic}',
            f'health_config:={health_config}',
            f'evaluator_output_json:={output_json}',
        ],
    )
    try:
        _wait_for_nodes(
            process,
            log_stream,
            log_path,
            environment,
            {
                '/camera_health_monitor',
                '/health_evaluator',
                '/camera_health_watch',
            },
        )
        assert _parameter(
            environment, '/camera_health_monitor', 'image_topic'
        ) == f'String value is: {source_topic}'
        assert _parameter(
            environment, '/health_evaluator', 'camera_health_topic'
        ) == 'String value is: /health/camera'
        assert _parameter(
            environment, '/health_evaluator', 'output_json_path'
        ) == f'String value is: {output_json}'
        assert _parameter(
            environment, '/camera_health_watch', 'health_topic'
        ) == 'String value is: /health/camera'
        assert _parameter(
            environment, '/camera_health_monitor', 'use_sim_time'
        ) == 'Boolean value is: False'
    finally:
        _stop_launch(process, log_stream)

    assert output_json.exists()
