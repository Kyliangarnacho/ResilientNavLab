"""Tests for the bounded stage 3 motion publisher."""

import importlib.util
import sys
from pathlib import Path

import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / 'scripts' / 'motion_test.py'
)
sys.path.insert(0, str(SCRIPT_PATH.parent))
SPEC = importlib.util.spec_from_file_location('motion_test', SCRIPT_PATH)
MOTION_TEST = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MOTION_TEST
SPEC.loader.exec_module(MOTION_TEST)


@pytest.mark.parametrize(
    ('mode', 'linear_speed', 'angular_speed', 'expected'),
    [
        ('straight', 0.25, 0.7, (0.25, 0.0)),
        ('spin', 0.25, -0.7, (0.0, -0.7)),
        ('arc', -0.25, 0.7, (-0.25, 0.7)),
    ],
)
def test_command_for_each_motion_mode(
    mode,
    linear_speed,
    angular_speed,
    expected,
):
    """Each mode should activate only its intended velocity components."""
    command = MOTION_TEST.command_for_mode(
        mode,
        linear_speed,
        angular_speed,
    )

    assert (command.linear_x, command.angular_z) == expected


def test_options_accept_configurable_speeds_and_duration():
    """The command line should expose both speeds and duration."""
    options = MOTION_TEST.parse_options([
        'arc',
        '--linear-speed',
        '0.3',
        '--angular-speed',
        '-0.4',
        '--duration',
        '1.25',
    ])

    assert options.mode == 'arc'
    assert options.linear_speed == 0.3
    assert options.angular_speed == -0.4
    assert options.duration == 1.25


@pytest.mark.parametrize(
    'arguments',
    [
        ['straight', '--linear-speed', '0'],
        ['spin', '--angular-speed', '0'],
        ['arc', '--linear-speed', '0'],
        ['arc', '--angular-speed', '0'],
        ['straight', '--duration', '0'],
        ['straight', '--duration', '-1'],
    ],
)
def test_options_reject_non_motion_and_non_positive_duration(arguments):
    """Invalid commands should fail before creating a ROS node."""
    with pytest.raises(SystemExit):
        MOTION_TEST.parse_options(arguments)


class RecordingPublisher:
    """Minimal publisher double for cleanup-path verification."""

    def __init__(self):
        """Initialize the recorded stop count."""
        self.stop_count = 0

    def publish_stop(self):
        """Record an automatic stop request."""
        self.stop_count += 1


def test_automatic_stop_runs_after_normal_completion():
    """The protected normal path should finish with a stop request."""
    publisher = RecordingPublisher()

    with MOTION_TEST.automatic_stop(publisher):
        pass

    assert publisher.stop_count == 1


def test_automatic_stop_runs_when_ctrl_c_interrupts_motion():
    """The Ctrl-C path should request stop before propagating interruption."""
    publisher = RecordingPublisher()

    with pytest.raises(KeyboardInterrupt):
        with MOTION_TEST.automatic_stop(publisher):
            raise KeyboardInterrupt

    assert publisher.stop_count == 1


ROUTE_PATH = (
    Path(__file__).resolve().parents[1] / 'scripts' / 'phase9_mapping_route.py'
)
ROUTE_SPEC = importlib.util.spec_from_file_location('phase9_mapping_route', ROUTE_PATH)
ROUTE = importlib.util.module_from_spec(ROUTE_SPEC)
sys.modules[ROUTE_SPEC.name] = ROUTE
ROUTE_SPEC.loader.exec_module(ROUTE)


def test_phase9_route_is_a_repeatable_four_corner_loop():
    """One lap contains four bounded straight commands and four left turns."""
    segments = ROUTE.route_segments(
        0.25, 0.5, 4.0, 1.57079632679, 1.14, 1,
    )

    assert len(segments) == 8
    assert segments[0][0] == 'straight'
    assert (segments[0][1].linear_x, segments[0][1].angular_z) == (0.25, 0.0)
    assert segments[1][0] == 'turn'
    assert (segments[1][1].linear_x, segments[1][1].angular_z) == (0.0, 0.5)
    assert segments[1][2] == pytest.approx(3.5814156246812)


def test_phase9_route_options_are_parameterized_and_reject_unsafe_values():
    """Route length and velocity remain explicit bounded experiment inputs."""
    options = ROUTE.parse_options([
        '--linear-speed', '0.2',
        '--angular-speed', '0.5',
        '--straight-duration', '5.0',
        '--turn-duration-scale', '1.2',
        '--laps', '2',
    ])

    assert options.linear_speed == 0.2
    assert options.angular_speed == 0.5
    assert options.straight_duration == 5.0
    assert options.turn_duration_scale == 1.2
    assert options.laps == 2

    with pytest.raises(SystemExit):
        ROUTE.parse_options(['--linear-speed', '0'])
    with pytest.raises(SystemExit):
        ROUTE.parse_options(['--laps', '0'])
    with pytest.raises(SystemExit):
        ROUTE.parse_options(['--turn-duration-scale', '0'])
