"""Regression for a calculation that outlives its BRNE input watchdog."""

import time

import numpy as np
import rclpy

from resilient_nav_interfaces.msg import PedestrianArray

from resilient_nav_brne.brne_shadow_node import BrneShadowNode
from resilient_nav_brne.shadow_planner import ShadowPlan


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _SlowPlanner:
    def __init__(self, config):
        self.config = config

    def plan(self, robot_pose, goal, pedestrians, **_kwargs):
        time.sleep(0.02)
        return ShadowPlan(
            linear_velocity=0.2,
            angular_velocity=0.1,
            trajectory=np.tile(robot_pose, (2, 1)),
            compute_ms=0.0,
        )


def test_shadow_drops_a_plan_when_compute_outlives_input_freshness():
    """A first JIT-like stall must publish only a zero raw Twist and no Path."""
    rclpy.init()
    node = BrneShadowNode()
    try:
        node.input_timeout_sec = 0.001
        node._robot_pose = np.array([0.0, 0.0, 0.0])
        node._goal = np.array([1.0, 0.0])
        node._pedestrians = [np.array([1.2, 0.5, 0.0, -0.1])]
        node._received_at = {
            'odom': time.monotonic(),
            'goal': time.monotonic(),
            'pedestrians': time.monotonic(),
        }
        node.planner = _SlowPlanner(node.planner.config)
        command_publisher = _Publisher()
        path_publisher = _Publisher()
        node.cmd_publisher = command_publisher
        node.path_publisher = path_publisher

        node._plan_timer()

        assert node.compute_count == 0
        assert len(command_publisher.messages) == 1
        assert command_publisher.messages[0].linear.x == 0.0
        assert command_publisher.messages[0].angular.z == 0.0
        assert not path_publisher.messages
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stale_inputs_reset_the_node_interaction_state():
    """A pedestrian watchdog expiry cannot retain stale event state."""
    rclpy.init()
    node = BrneShadowNode()
    try:
        node._warmup_complete = True
        node._interaction.update((0.0, 0.0), {7: (0.5, 0.0)})
        assert node._interaction.active
        node.cmd_publisher = _Publisher()

        node._plan_timer()

        assert not node._interaction.active
        assert len(node.cmd_publisher.messages) == 1
        assert node.cmd_publisher.messages[0].linear.x == 0.0
        assert node.cmd_publisher.messages[0].angular.z == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_fresh_empty_pedestrian_array_is_a_valid_no_agent_snapshot():
    """A processed empty sensor frame must not be treated as a stale sensor."""
    rclpy.init()
    node = BrneShadowNode()
    try:
        now = time.monotonic()
        node._robot_pose = np.array([0.0, 0.0, 0.0])
        node._goal = np.array([1.0, 0.0])
        node._received_at.update({'odom': now, 'goal': now})
        message = PedestrianArray()
        message.header.frame_id = 'odom'

        node._pedestrian_callback(message)
        snapshot = node._fresh_input_snapshot()

        assert snapshot is not None
        assert snapshot[2] == []
    finally:
        node.destroy_node()
        rclpy.shutdown()
