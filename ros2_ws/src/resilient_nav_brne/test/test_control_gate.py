"""Focused safety contracts for the manually armed BRNE control gate."""

import time
from types import SimpleNamespace

from geometry_msgs.msg import Twist

from resilient_nav_brne.brne_control_gate import BrneControlGate
from resilient_nav_brne.control_gate import endpoint_is_self, validated_diff_drive_command


LIMITS = {
    'maximum_linear_velocity': 0.30,
    'maximum_angular_velocity': 0.80,
}


def test_finite_bounded_diff_drive_command_is_accepted():
    assert validated_diff_drive_command(
        (0.2, 0.0, 0.0, 0.0, 0.0, -0.5), **LIMITS
    ) == (0.2, -0.5)


def test_nonfinite_out_of_bounds_reverse_and_unsupported_axes_fail_closed():
    invalid = [
        (float('nan'), 0.0, 0.0, 0.0, 0.0, 0.0),
        (0.31, 0.0, 0.0, 0.0, 0.0, 0.0),
        (-0.01, 0.0, 0.0, 0.0, 0.0, 0.0),
        (0.2, 0.01, 0.0, 0.0, 0.0, 0.0),
        (0.2, 0.0, 0.0, 0.0, 0.0, 0.81),
    ]
    assert all(
        validated_diff_drive_command(values, **LIMITS) is None
        for values in invalid
    )


def test_ownership_check_distinguishes_self_from_a_foreign_publisher():
    own = SimpleNamespace(node_name='brne_control_gate', node_namespace='/')
    foreign = SimpleNamespace(node_name='controller_server', node_namespace='/')
    assert endpoint_is_self(
        own, node_name='brne_control_gate', node_namespace='/'
    )
    assert not endpoint_is_self(
        foreign, node_name='brne_control_gate', node_namespace='/'
    )


def test_stale_raw_command_is_rejected_before_any_formal_publish():
    gate = object.__new__(BrneControlGate)
    gate._raw_command = Twist()
    gate._raw_command.linear.x = 0.2
    gate._raw_received_at = time.monotonic() - 1.0
    gate.input_timeout_sec = 0.35
    gate.maximum_linear_velocity = 0.30
    gate.maximum_angular_velocity = 0.80
    assert gate._validated_fresh_command() is None


def test_fresh_raw_angular_command_is_not_output_filtered():
    gate = object.__new__(BrneControlGate)
    gate._raw_command = Twist()
    gate._raw_command.linear.x = 0.2
    gate._raw_command.angular.z = -0.6
    gate._raw_received_at = time.monotonic()
    gate.input_timeout_sec = 0.35
    gate.maximum_linear_velocity = 0.30
    gate.maximum_angular_velocity = 0.80

    command = gate._validated_fresh_command()

    assert command.linear.x == 0.2
    assert command.angular.z == -0.6
