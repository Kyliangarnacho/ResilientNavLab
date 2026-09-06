"""Send the fixed Scene 1 RPP goal after Nav2 is genuinely active."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


READY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class RppScene1GoalCoordinator(Node):
    """Own one NavigateToPose request and the matching pedestrian-ready fact."""

    def __init__(self) -> None:
        super().__init__('rpp_scene1_goal_coordinator')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter(
            'navigation_ready_service', '/lifecycle_manager_navigation/is_active'
        )
        self.declare_parameter('ready_topic', '/rpp/ready')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('poll_period_sec', 0.1)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.goal = (
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
            float(self.get_parameter('goal_yaw').value),
        )
        poll_period_sec = float(self.get_parameter('poll_period_sec').value)
        if (
            not self.map_frame
            or not all(math.isfinite(value) for value in self.goal)
            or not math.isfinite(poll_period_sec)
            or poll_period_sec <= 0.0
        ):
            raise ValueError('RPP Scene 1 coordinator parameters are invalid')

        self.ready_publisher = self.create_publisher(
            Bool, str(self.get_parameter('ready_topic').value), READY_QOS
        )
        self._navigation_ready_client = self.create_client(
            Trigger, str(self.get_parameter('navigation_ready_service').value)
        )
        self._action_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter('action_name').value)
        )
        self._readiness_request_in_flight = False
        self._goal_request_in_flight = False
        self._goal_accepted = False
        # Make the fail-closed state explicit for an already-running driver.
        self._publish_ready(False)
        self.create_timer(poll_period_sec, self._advance)

    def _publish_ready(self, ready: bool) -> None:
        self.ready_publisher.publish(Bool(data=ready))

    def _advance(self) -> None:
        if self._goal_accepted or self._goal_request_in_flight:
            return
        if not self._navigation_ready_client.wait_for_service(timeout_sec=0.0):
            return
        if self._readiness_request_in_flight:
            return
        self._readiness_request_in_flight = True
        future = self._navigation_ready_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_navigation_ready)

    def _on_navigation_ready(self, future) -> None:
        self._readiness_request_in_flight = False
        try:
            response = future.result()
        except Exception as error:  # Transport failures are retried by _advance.
            self.get_logger().warning(
                'navigation readiness query failed: ' + type(error).__name__
            )
            return
        if not response.success:
            return
        if not self._action_client.wait_for_server(timeout_sec=0.0):
            return
        self._goal_request_in_flight = True
        future = self._action_client.send_goal_async(self._goal_message())
        future.add_done_callback(self._on_goal_response)

    def _goal_message(self) -> NavigateToPose.Goal:
        request = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.goal[0]
        pose.pose.position.y = self.goal[1]
        pose.pose.orientation.z = math.sin(self.goal[2] / 2.0)
        pose.pose.orientation.w = math.cos(self.goal[2] / 2.0)
        request.pose = pose
        # Empty requests use the frozen server-side BT XML.
        request.behavior_tree = ''
        return request

    def _on_goal_response(self, future) -> None:
        self._goal_request_in_flight = False
        try:
            goal_handle = future.result()
        except Exception as error:  # Transport failures are retried by _advance.
            self.get_logger().warning(
                'NavigateToPose request failed: ' + type(error).__name__
            )
            return
        if not goal_handle.accepted:
            self.get_logger().warning('Scene 1 NavigateToPose goal was rejected')
            return
        self._goal_accepted = True
        self._publish_ready(True)
        self.get_logger().info(
            'Scene 1 NavigateToPose accepted; waiting for driver plan/odometry gate'
        )
        goal_handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        try:
            result = future.result()
            status = int(result.status)
        except Exception as error:
            self.get_logger().warning(
                'Scene 1 NavigateToPose result failed: ' + type(error).__name__
            )
            return
        self.get_logger().info(f'Scene 1 NavigateToPose terminal status: {status}')
        # Keep ready latched for the accepted run. The unchanged driver still
        # fail-closes immediately if /plan or pedestrian odometry becomes stale.

    def publish_shutdown_stop(self) -> None:
        if not rclpy.ok():
            return
        try:
            self._publish_ready(False)
        except Exception:  # Context can be invalidated before SIGINT teardown.
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RppScene1GoalCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
