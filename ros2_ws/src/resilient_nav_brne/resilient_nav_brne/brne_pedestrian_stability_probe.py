"""Bounded pedestrian-only runtime verifier for the Scene 1 constrained actor."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


def _roll_pitch_yaw(quaternion) -> tuple[float, float, float] | None:
    """Return finite intrinsic RPY from one normalized quaternion."""
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        return None
    x_value, y_value, z_value, w_value = (value / norm for value in values)
    roll = math.atan2(
        2.0 * (w_value * x_value + y_value * z_value),
        1.0 - 2.0 * (x_value * x_value + y_value * y_value),
    )
    pitch_sine = 2.0 * (w_value * y_value - z_value * x_value)
    pitch = math.asin(max(-1.0, min(1.0, pitch_sine)))
    yaw = math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )
    return roll, pitch, yaw


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class PedestrianStabilityProbe(Node):
    """Drive the joint once and measure all Scene 1 upright-state contracts."""

    def __init__(self):
        super().__init__('brne_pedestrian_stability_probe')
        self.declare_parameter('command_topic', '/brne/pedestrian/joint_velocity')
        self.declare_parameter('odometry_topic', '/brne/pedestrian/odometry')
        self.declare_parameter('speed', 0.25)
        self.declare_parameter('target_world_y', -2.50)
        self.declare_parameter('observation_duration_sec', 60.0)
        self.declare_parameter('expected_frame', 'gazebo_world')
        self.declare_parameter('expected_child_frame', 'brne_pedestrian')
        self.declare_parameter('orientation_tolerance_rad', 1e-3)
        self.declare_parameter('position_tolerance_m', 0.02)
        self.declare_parameter(
            'result_path', '/tmp/brne_pedestrian_stability_result.json'
        )

        self.speed = float(self.get_parameter('speed').value)
        self.target_world_y = float(self.get_parameter('target_world_y').value)
        self.observation_duration_sec = float(
            self.get_parameter('observation_duration_sec').value
        )
        self.expected_frame = str(self.get_parameter('expected_frame').value)
        self.expected_child_frame = str(
            self.get_parameter('expected_child_frame').value
        )
        self.orientation_tolerance_rad = float(
            self.get_parameter('orientation_tolerance_rad').value
        )
        self.position_tolerance_m = float(
            self.get_parameter('position_tolerance_m').value
        )
        self.result_path = Path(str(self.get_parameter('result_path').value))
        if (
            self.speed <= 0.0
            or self.observation_duration_sec <= 0.0
            or self.orientation_tolerance_rad <= 0.0
            or self.position_tolerance_m <= 0.0
        ):
            raise ValueError('pedestrian stability probe parameters are invalid')

        self.publisher = self.create_publisher(
            Float64, str(self.get_parameter('command_topic').value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odometry_topic').value),
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self._wall_started = time.monotonic()
        self._first_sim_time: float | None = None
        self._crossing_complete_time: float | None = None
        self._initial: dict | None = None
        self._latest: dict | None = None
        self._max_abs_roll = 0.0
        self._max_abs_pitch = 0.0
        self._max_abs_yaw_drift = 0.0
        self._max_body_twist_x = 0.0
        self._max_abs_body_twist_y = 0.0
        self._max_abs_body_twist_z = 0.0
        self._contract_errors: list[str] = []
        self._done = False
        self._result: dict | None = None
        self.create_timer(0.05, self._timer)

    @property
    def done(self) -> bool:
        return self._done

    @property
    def result(self) -> dict | None:
        return self._result

    def _on_odometry(self, message: Odometry) -> None:
        if message.header.frame_id != self.expected_frame:
            self._record_contract_error('unexpected odometry frame')
            return
        if message.child_frame_id != self.expected_child_frame:
            self._record_contract_error('unexpected odometry child frame')
            return
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        if stamp_ns <= 0:
            self._record_contract_error('missing odometry timestamp')
            return
        pose = message.pose.pose.position
        twist = message.twist.twist.linear
        values = (pose.x, pose.y, pose.z, twist.x, twist.y, twist.z)
        rpy = _roll_pitch_yaw(message.pose.pose.orientation)
        if rpy is None or not all(math.isfinite(value) for value in values):
            self._record_contract_error('non-finite odometry')
            return
        sample = {
            'time_sec': stamp_ns / 1e9,
            'x': float(pose.x),
            'y': float(pose.y),
            'z': float(pose.z),
            'roll': float(rpy[0]),
            'pitch': float(rpy[1]),
            'yaw': float(rpy[2]),
            'body_twist_x': float(twist.x),
            'body_twist_y': float(twist.y),
            'body_twist_z': float(twist.z),
        }
        if self._initial is None:
            self._initial = sample
            self._first_sim_time = sample['time_sec']
        self._latest = sample
        self._max_abs_roll = max(self._max_abs_roll, abs(sample['roll']))
        self._max_abs_pitch = max(self._max_abs_pitch, abs(sample['pitch']))
        self._max_abs_yaw_drift = max(
            self._max_abs_yaw_drift,
            abs(_normalize_angle(sample['yaw'] - self._initial['yaw'])),
        )
        self._max_body_twist_x = max(self._max_body_twist_x, sample['body_twist_x'])
        self._max_abs_body_twist_y = max(
            self._max_abs_body_twist_y, abs(sample['body_twist_y'])
        )
        self._max_abs_body_twist_z = max(
            self._max_abs_body_twist_z, abs(sample['body_twist_z'])
        )

    def _timer(self) -> None:
        if self._done:
            return
        if self._latest is None or self._first_sim_time is None:
            if time.monotonic() - self._wall_started > 15.0:
                self._finish('no valid pedestrian odometry within 15 wall seconds')
            return
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self._first_sim_time
        command = 0.0
        if self._crossing_complete_time is None:
            if self._latest['y'] >= self.target_world_y - self.position_tolerance_m:
                self._crossing_complete_time = elapsed
            else:
                command = self.speed
        self.publisher.publish(Float64(data=command))
        # Require an actual odometry sample at or beyond the duration.  A
        # timer-only condition could otherwise finish at 60 s while the latest
        # 20 Hz odometry evidence still represented 59.95 s.
        observed_duration = self._latest['time_sec'] - self._first_sim_time
        if observed_duration >= self.observation_duration_sec:
            self._finish(None)

    def _finish(self, infrastructure_error: str | None) -> None:
        if self._done:
            return
        self.publisher.publish(Float64())
        self._done = True
        initial = self._initial
        latest = self._latest
        errors = list(self._contract_errors)
        if infrastructure_error is not None:
            errors.append(infrastructure_error)
        if initial is None or latest is None:
            self._result = {'pass': False, 'errors': errors}
            self.get_logger().error(
                'BRNE_PEDESTRIAN_STABILITY_RESULT ' + json.dumps(self._result, sort_keys=True)
            )
            self._write_result()
            return
        crossing_velocity = None
        if self._crossing_complete_time is not None and self._crossing_complete_time > 0.0:
            crossing_velocity = (latest['y'] - initial['y']) / self._crossing_complete_time
        if self._crossing_complete_time is None:
            errors.append('crossing did not reach target')
        if self._max_abs_roll > self.orientation_tolerance_rad:
            errors.append('roll exceeds tolerance')
        if self._max_abs_pitch > self.orientation_tolerance_rad:
            errors.append('pitch exceeds tolerance')
        if abs(latest['x'] - initial['x']) > self.position_tolerance_m:
            errors.append('world X drift exceeds tolerance')
        if abs(latest['z'] - initial['z']) > self.position_tolerance_m:
            errors.append('world Z drift exceeds tolerance')
        if self._max_abs_yaw_drift > self.orientation_tolerance_rad:
            errors.append('yaw drift exceeds tolerance')
        if crossing_velocity is None or abs(crossing_velocity - self.speed) > 0.02:
            errors.append('crossing velocity differs from command')
        if abs(latest['body_twist_x']) > 0.02:
            errors.append('final body-frame velocity is nonzero')
        self._result = {
            'pass': not errors,
            'errors': errors,
            'start_world_x': initial['x'],
            'start_world_y': initial['y'],
            'start_world_z': initial['z'],
            'end_world_x': latest['x'],
            'end_world_y': latest['y'],
            'end_world_z': latest['z'],
            'x_drift_m': latest['x'] - initial['x'],
            'z_drift_m': latest['z'] - initial['z'],
            'max_abs_roll_rad': self._max_abs_roll,
            'max_abs_pitch_rad': self._max_abs_pitch,
            'max_abs_yaw_drift_rad': self._max_abs_yaw_drift,
            'crossing_velocity_mps': crossing_velocity,
            'max_body_twist_x_mps': self._max_body_twist_x,
            'max_abs_body_twist_y_mps': self._max_abs_body_twist_y,
            'max_abs_body_twist_z_mps': self._max_abs_body_twist_z,
            'final_body_twist_x_mps': latest['body_twist_x'],
            'crossing_complete_after_sec': self._crossing_complete_time,
            'observation_duration_sec': latest['time_sec'] - initial['time_sec'],
            'odometry_frame': self.expected_frame,
            'odometry_child_frame': self.expected_child_frame,
        }
        logger = self.get_logger().info if self._result['pass'] else self.get_logger().error
        logger('BRNE_PEDESTRIAN_STABILITY_RESULT ' + json.dumps(self._result, sort_keys=True))
        self._write_result()

    def _record_contract_error(self, error: str) -> None:
        if error not in self._contract_errors:
            self._contract_errors.append(error)

    def _write_result(self) -> None:
        """Persist exactly the emitted verification record outside the workspace."""
        if self._result is None:
            return
        self.result_path.write_text(
            json.dumps(self._result, sort_keys=True) + '\n', encoding='utf-8'
        )

    def publish_shutdown_stop(self) -> None:
        self.publisher.publish(Float64())


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianStabilityProbe()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
        return 0 if node.result is not None and node.result.get('pass') else 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.publish_shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
