"""Runtime coverage for the evaluator-only launch entry point."""

import os
import signal
import subprocess
import time


def test_minimal_launch_sets_evaluator_output_json_parameter(tmp_path):
    """The running node must expose the non-empty launch output path."""
    requested_output = tmp_path / 'phase6_health_evaluation.json'
    requested_camera_topic = '/test/health/camera'
    environment = os.environ.copy()
    environment['ROS_LOG_DIR'] = str(tmp_path / 'ros_logs')
    environment['ROS_DOMAIN_ID'] = str(1 + (os.getpid() % 231))
    launch_log_path = tmp_path / 'health_evaluator_launch.log'

    with launch_log_path.open('w', encoding='utf-8') as launch_log:
        launch_process = subprocess.Popen(
            [
                'ros2',
                'launch',
                'resilient_nav_health_assessment',
                'health_evaluator_minimal.launch.py',
                f'evaluator_output_json:={requested_output}',
                f'camera_health_topic:={requested_camera_topic}',
            ],
            env=environment,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
        )
        parameter_result = None
        try:
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                parameter_result = subprocess.run(
                    [
                        'ros2', 'param', 'get', '/health_evaluator',
                        'output_json_path',
                    ],
                    env=environment,
                    capture_output=True,
                    text=True,
                    check=False,
                )
                if parameter_result.returncode == 0:
                    break
                time.sleep(0.25)

            assert parameter_result is not None
            assert parameter_result.returncode == 0, launch_log_path.read_text(
                encoding='utf-8'
            )
            assert parameter_result.stdout.strip() == (
                f'String value is: {requested_output}'
            )
            camera_parameter_result = subprocess.run(
                [
                    'ros2', 'param', 'get', '/health_evaluator',
                    'camera_health_topic',
                ],
                env=environment,
                capture_output=True,
                text=True,
                check=False,
            )
            assert camera_parameter_result.returncode == 0
            assert camera_parameter_result.stdout.strip() == (
                f'String value is: {requested_camera_topic}'
            )
        finally:
            if launch_process.poll() is None:
                launch_process.send_signal(signal.SIGINT)
                try:
                    launch_process.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    launch_process.kill()
                    launch_process.wait(timeout=10.0)
