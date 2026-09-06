"""Periodically request Phase 10 Navfn plans for the BRNE demos."""

from __future__ import annotations

import math
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class BrnePeriodicPlanner(Node):
    """Request only ComputePathToPose; never start or emulate a controller."""

    def __init__(self):
        super().__init__('brne_periodic_planner')
        self.declare_parameter('action_name', '/compute_path_to_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('replan_frequency_hz', 1.0)
        self.declare_parameter('goal_tolerance', 0.20)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.planner_id = str(self.get_parameter('planner_id').value)
        self.goal = (
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
            float(self.get_parameter('goal_yaw').value),
        )
        frequency = float(self.get_parameter('replan_frequency_hz').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        if (
            not self.map_frame or not self.planner_id
            or not all(math.isfinite(value) for value in self.goal)
            or frequency <= 0.0 or self.goal_tolerance <= 0.0
        ):
            raise ValueError('periodic planner parameters are invalid')

        self._action_client = ActionClient(
            self,
            ComputePathToPose,
            str(self.get_parameter('action_name').value),
        )
        self._request_in_flight = False
        self._goal_reached = False
        self.successful_plans = 0
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._on_amcl_pose,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / frequency, self._request_plan)

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != self.map_frame:
            return
        position = message.pose.pose.position
        if not all(math.isfinite(value) for value in (position.x, position.y)):
            return
        if math.dist((position.x, position.y), self.goal[:2]) <= self.goal_tolerance:
            if not self._goal_reached:
                self.get_logger().info(
                    'BRNE demo goal tolerance reached; periodic planning stopped'
                )
            self._goal_reached = True

    def _request_plan(self) -> None:
        if self._goal_reached:
            return
        if self._request_in_flight:
            return
        if not self._action_client.wait_for_server(timeout_sec=0.0):
            return
        request = ComputePathToPose.Goal()
        request.goal = self._goal_message()
        request.planner_id = self.planner_id
        request.use_start = False
        self._request_in_flight = True
        future = self._action_client.send_goal_async(request)
        future.add_done_callback(self._on_goal_response)

    def _goal_message(self) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = self.goal[0]
        message.pose.position.y = self.goal[1]
        message.pose.orientation.z = math.sin(self.goal[2] / 2.0)
        message.pose.orientation.w = math.cos(self.goal[2] / 2.0)
        return message

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # rclpy action errors vary by transport state.
            self._request_in_flight = False
            self.get_logger().warning(
                f'ComputePathToPose request failed: {type(error).__name__}'
            )
            return
        if not goal_handle.accepted:
            self._request_in_flight = False
            self.get_logger().warning('ComputePathToPose request was rejected')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future) -> None:
        self._request_in_flight = False
        try:
            wrapped = future.result()
        except Exception as error:  # rclpy action errors vary by transport state.
            self.get_logger().warning(
                f'ComputePathToPose result failed: {type(error).__name__}'
            )
            return
        result = wrapped.result
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or result.error_code != ComputePathToPose.Result.NONE
            or not result.path.poses
        ):
            self.get_logger().warning(
                'ComputePathToPose produced no usable /plan: '
                f'status={wrapped.status}, error={result.error_code}'
            )
            return
        self.successful_plans += 1
        self.get_logger().info(
            f'ComputePathToPose refresh {self.successful_plans}: '
            f'{len(result.path.poses)} poses'
        )

def main(args=None):
    rclpy.init(args=args)
    node = BrnePeriodicPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
