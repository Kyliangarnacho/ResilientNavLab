"""Supervisor-gated proxy for Nav2 NavigateToPose goals."""

from __future__ import annotations

import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from resilient_nav_interfaces.msg import ResilienceStatus


class ResilientGoalGate(Node):
    """Cancel Nav2 on HOLD and replay the same goal after recovery."""

    def __init__(self) -> None:
        """Create one upstream action server and one downstream client."""
        super().__init__('resilient_goal_gate')
        self.declare_parameter(
            'upstream_action_name', '/resilient_navigate_to_pose'
        )
        self.declare_parameter('downstream_action_name', '/navigate_to_pose')
        self.declare_parameter('navigation_scan_topic', '/faulted/scan')
        self.declare_parameter('replay_tf_target_frame', 'map')
        self.declare_parameter('replay_tf_confirmation_count', 2)
        upstream = str(self.get_parameter('upstream_action_name').value)
        downstream = str(self.get_parameter('downstream_action_name').value)
        if not upstream or not downstream or upstream == downstream:
            raise ValueError('goal-gate action names must be non-empty and distinct')
        navigation_scan_topic = str(
            self.get_parameter('navigation_scan_topic').value
        )
        self._replay_tf_target_frame = str(
            self.get_parameter('replay_tf_target_frame').value
        )
        self._replay_tf_confirmation_count = int(
            self.get_parameter('replay_tf_confirmation_count').value
        )
        if not navigation_scan_topic or not self._replay_tf_target_frame:
            raise ValueError('scan topic and replay TF target must be non-empty')
        if self._replay_tf_confirmation_count < 1:
            raise ValueError('replay_tf_confirmation_count must be positive')

        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._navigation_allowed = False
        self._goal_reserved = False
        self._downstream_goal = None
        self._hold_cancel_requested = False
        self._paused_by_hold = False
        self._replay_tf_confirmations = 0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self._client = ActionClient(
            self,
            NavigateToPose,
            downstream,
            callback_group=self._callback_group,
        )
        self._server = ActionServer(
            self,
            NavigateToPose,
            upstream,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self._goal_publisher = self.create_publisher(
            PoseStamped,
            '/resilience/active_goal',
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.create_subscription(
            ResilienceStatus,
            '/resilience/status',
            self._on_resilience,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            LaserScan,
            navigation_scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )

    def _on_goal(self, _request) -> GoalResponse:
        with self._lock:
            if self._goal_reserved:
                self.get_logger().warning('rejecting concurrent navigation goal')
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        self._wake.set()
        return CancelResponse.ACCEPT

    def _on_resilience(self, message: ResilienceStatus) -> None:
        child = None
        with self._lock:
            was_allowed = self._navigation_allowed
            self._navigation_allowed = bool(message.navigation_allowed)
            if (
                not self._navigation_allowed
                and self._downstream_goal is not None
                and not self._hold_cancel_requested
            ):
                self._hold_cancel_requested = True
                self._paused_by_hold = True
                child = self._downstream_goal
            if (
                self._navigation_allowed
                and not was_allowed
                and self._paused_by_hold
            ):
                self._replay_tf_confirmations = 0
        if child is not None:
            self.get_logger().warning(
                'Supervisor HOLD: canceling the active Nav2 child goal'
            )
            child.cancel_goal_async()
        self._wake.set()

    def _on_scan(self, message: LaserScan) -> None:
        """Count fresh scans whose stamped pose is available in map TF."""
        with self._lock:
            needs_readiness = (
                self._paused_by_hold and self._navigation_allowed
            )
        if not needs_readiness:
            return
        source_frame = message.header.frame_id.lstrip('/')
        transform_ready = bool(source_frame) and self._tf_buffer.can_transform(
            self._replay_tf_target_frame,
            source_frame,
            Time.from_msg(message.header.stamp),
            timeout=Duration(seconds=0.0),
        )
        with self._lock:
            if not (self._paused_by_hold and self._navigation_allowed):
                return
            if transform_ready:
                self._replay_tf_confirmations = min(
                    self._replay_tf_confirmations + 1,
                    self._replay_tf_confirmation_count,
                )
            else:
                self._replay_tf_confirmations = 0
        self._wake.set()

    def _execute(self, upstream_goal):
        self._goal_publisher.publish(upstream_goal.request.pose)
        try:
            return self._run_goal(upstream_goal)
        finally:
            with self._lock:
                self._goal_reserved = False
                self._downstream_goal = None
                self._hold_cancel_requested = False
                self._paused_by_hold = False
                self._replay_tf_confirmations = 0
            self._wake.set()

    def _run_goal(self, upstream_goal):
        while rclpy.ok():
            if upstream_goal.is_cancel_requested:
                self._cancel_child()
                upstream_goal.canceled()
                return NavigateToPose.Result()
            if not self._is_navigation_allowed():
                self._wait_briefly()
                continue
            if not self._is_replay_ready():
                self._wait_briefly()
                continue
            if not self._client.wait_for_server(timeout_sec=0.2):
                self._wait_briefly()
                continue

            send_future = self._client.send_goal_async(
                upstream_goal.request,
                feedback_callback=lambda feedback: self._forward_feedback(
                    upstream_goal, feedback
                ),
            )
            child = self._wait_future(send_future, upstream_goal, 5.0)
            if child is None:
                if upstream_goal.is_cancel_requested:
                    upstream_goal.canceled()
                else:
                    upstream_goal.abort()
                return NavigateToPose.Result()
            if not child.accepted:
                self.get_logger().error('Nav2 rejected the forwarded goal')
                upstream_goal.abort()
                return NavigateToPose.Result()
            with self._lock:
                self._downstream_goal = child
                self._paused_by_hold = False
                self._replay_tf_confirmations = 0
                should_cancel = not self._navigation_allowed
                if should_cancel:
                    self._hold_cancel_requested = True
                    self._paused_by_hold = True
            if should_cancel:
                child.cancel_goal_async()

            result_future = child.get_result_async()
            while rclpy.ok() and not result_future.done():
                if upstream_goal.is_cancel_requested:
                    self._cancel_child()
                elif not self._is_navigation_allowed():
                    self._cancel_child(due_to_hold=True)
                self._wait_briefly()
            if not result_future.done() or result_future.result() is None:
                upstream_goal.abort()
                return NavigateToPose.Result()

            wrapped = result_future.result()
            with self._lock:
                canceled_for_hold = self._hold_cancel_requested
                self._downstream_goal = None
                self._hold_cancel_requested = False
            if upstream_goal.is_cancel_requested:
                upstream_goal.canceled()
                return wrapped.result
            if (
                wrapped.status == GoalStatus.STATUS_CANCELED
                and canceled_for_hold
            ):
                self.get_logger().info(
                    'Nav2 child canceled for HOLD; retaining the saved goal'
                )
                continue
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                upstream_goal.succeed()
            elif wrapped.status == GoalStatus.STATUS_CANCELED:
                upstream_goal.canceled()
            else:
                upstream_goal.abort()
            return wrapped.result

        upstream_goal.abort()
        return NavigateToPose.Result()

    def _cancel_child(self, due_to_hold: bool = False) -> None:
        with self._lock:
            child = self._downstream_goal
            already_requested = self._hold_cancel_requested
            if due_to_hold and child is not None:
                self._hold_cancel_requested = True
        if child is not None and (not due_to_hold or not already_requested):
            child.cancel_goal_async()

    def _forward_feedback(self, upstream_goal, feedback) -> None:
        if not upstream_goal.is_active:
            return
        try:
            upstream_goal.publish_feedback(feedback.feedback)
        except RuntimeError:
            return

    def _is_navigation_allowed(self) -> bool:
        with self._lock:
            return self._navigation_allowed

    def _is_replay_ready(self) -> bool:
        with self._lock:
            return (
                not self._paused_by_hold
                or self._replay_tf_confirmations
                >= self._replay_tf_confirmation_count
            )

    def _wait_briefly(self) -> None:
        self._wake.wait(timeout=0.05)
        self._wake.clear()

    def _wait_future(self, future, upstream_goal, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return future.result()
            if upstream_goal.is_cancel_requested:
                return None
            self._wait_briefly()
        return None

    def destroy_node(self):
        """Destroy action endpoints before the ROS node."""
        self._server.destroy()
        self._client.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the goal gate with concurrent action and status callbacks."""
    rclpy.init(args=args)
    node = ResilientGoalGate()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
