import argparse
import json
import math
import statistics

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from resilient_nav_interfaces.msg import FaultStatus
from sensor_msgs.msg import Imu, LaserScan


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(angle_a, angle_b):
    return math.atan2(math.sin(angle_a - angle_b), math.cos(angle_a - angle_b))


def mean(values):
    return statistics.fmean(values) if values else None


def stdev(values):
    return statistics.pstdev(values) if len(values) > 1 else 0.0


class FaultProbe(Node):
    """Collect compact fault-injection evidence and print JSON on exit."""

    def __init__(self, duration_sec):
        super().__init__('fault_probe')
        self._duration_sec = duration_sec
        self._raw_imu = {}
        self._faulted_imu = {}
        self._raw_wheel = {}
        self._faulted_wheel = {}
        self._healthy_ekf = {}
        self._faulted_ekf = {}
        self._statuses = []
        self._raw_scan_count = 0
        self._faulted_scan_count = 0
        self._faulted_scan_nan_counts = []
        self._faulted_scan_beam_counts = []
        self._faulted_delay_values = []
        self._finished = False

        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            FaultStatus,
            '/fault_injection/status',
            self._on_status,
            status_qos,
        )
        self.create_subscription(
            Imu,
            '/imu/data',
            lambda msg: self._store_imu(self._raw_imu, msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            '/faulted/imu/data',
            self._on_faulted_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/wheel/odometry',
            lambda msg: self._store_odom(self._raw_wheel, msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/faulted/wheel/odometry',
            lambda msg: self._store_odom(self._faulted_wheel, msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self._on_raw_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            '/faulted/scan',
            self._on_faulted_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            lambda msg: self._store_odom(self._healthy_ekf, msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/odometry/faulted',
            lambda msg: self._store_odom(self._faulted_ekf, msg),
            qos_profile_sensor_data,
        )
        self.create_timer(duration_sec, self._finish)

    @property
    def finished(self):
        return self._finished

    def _finish(self):
        self._finished = True

    def _on_status(self, msg):
        self._statuses.append({
            'stamp': stamp_to_seconds(msg.header.stamp),
            'scenario_id': msg.scenario_id,
            'event_id': msg.event_id,
            'sensor': msg.sensor,
            'model': msg.model,
            'state': int(msg.state),
            'state_name': self._state_name(msg.state),
            'severity': float(msg.severity),
            'start': stamp_to_seconds(msg.start_time),
            'end': stamp_to_seconds(msg.end_time),
        })

    def _store_imu(self, store, msg):
        store[self._stamp_key(msg)] = {
            'stamp': stamp_to_seconds(msg.header.stamp),
            'z': float(msg.angular_velocity.z),
        }

    def _on_faulted_imu(self, msg):
        self._store_imu(self._faulted_imu, msg)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._faulted_delay_values.append(
            max(0.0, now_sec - stamp_to_seconds(msg.header.stamp))
        )

    def _store_odom(self, store, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        twist = msg.twist.twist
        store[self._stamp_key(msg)] = {
            'stamp': stamp_to_seconds(msg.header.stamp),
            'x': float(position.x),
            'y': float(position.y),
            'yaw': yaw_from_quaternion(orientation),
            'vx': float(twist.linear.x),
            'wz': float(twist.angular.z),
        }

    def _on_raw_scan(self, _msg):
        self._raw_scan_count += 1

    def _on_faulted_scan(self, msg):
        self._faulted_scan_count += 1
        beam_count = len(msg.ranges)
        nan_count = sum(1 for value in msg.ranges if math.isnan(value))
        self._faulted_scan_beam_counts.append(beam_count)
        self._faulted_scan_nan_counts.append(nan_count)

    def report(self):
        return {
            'duration_sec': self._duration_sec,
            'fault_status': self._status_report(),
            'imu': self._imu_report(),
            'wheel': self._wheel_report(),
            'lidar': self._lidar_report(),
            'ekf': self._ekf_report(),
        }

    def _status_report(self):
        state_names = sorted({status['state_name'] for status in self._statuses})
        scenario_ids = sorted(
            {status['scenario_id'] for status in self._statuses}
        )
        models = sorted({status['model'] for status in self._statuses})
        return {
            'count': len(self._statuses),
            'states': state_names,
            'scenario_ids': scenario_ids,
            'models': models,
        }

    def _imu_report(self):
        diffs = []
        active_windows = self._active_windows(sensor='imu')
        raw_count = 0
        faulted_count = 0
        for key, raw in self._raw_imu.items():
            if not self._in_windows(raw['stamp'], active_windows):
                continue
            raw_count += 1
            faulted = self._faulted_imu.get(key)
            if faulted is not None:
                faulted_count += 1
                diffs.append(faulted['z'] - raw['z'])

        return {
            'raw_count': len(self._raw_imu),
            'faulted_count': len(self._faulted_imu),
            'active_raw_count': raw_count,
            'active_faulted_count': faulted_count,
            'drop_rate': self._drop_rate(
                raw_count,
                faulted_count,
            ),
            'angular_velocity_z_diff_mean': mean(diffs),
            'angular_velocity_z_diff_stddev': stdev(diffs),
            'delay_sec_mean': mean(self._faulted_delay_values),
            'delay_sec_max': max(self._faulted_delay_values, default=None),
        }

    def _wheel_report(self):
        active_windows = self._active_windows(sensor='wheel_odometry')
        raw_values = [
            value for _, value in sorted(
                self._raw_wheel.items(),
                key=lambda item: item[1]['stamp'],
            )
            if self._in_windows(value['stamp'], active_windows)
        ]
        faulted_values = [
            value for _, value in sorted(
                self._faulted_wheel.items(),
                key=lambda item: item[1]['stamp'],
            )
            if self._in_windows(value['stamp'], active_windows)
        ]
        raw_motion = self._total_motion(raw_values)
        faulted_motion = self._total_motion(faulted_values)
        return {
            'raw_count': len(raw_values),
            'faulted_count': len(faulted_values),
            'raw_position_change_m': raw_motion,
            'faulted_position_change_m': faulted_motion,
            'freeze_detected': (
                raw_motion is not None
                and faulted_motion is not None
                and raw_motion > 0.01
                and faulted_motion < 0.001
            ),
        }

    def _lidar_report(self):
        total_beams = sum(self._faulted_scan_beam_counts)
        total_nan = sum(self._faulted_scan_nan_counts)
        return {
            'raw_count': self._raw_scan_count,
            'faulted_count': self._faulted_scan_count,
            'nan_beams': total_nan,
            'total_beams': total_beams,
            'nan_ratio': (
                float(total_nan) / float(total_beams) if total_beams else None
            ),
            'max_nan_beams_per_scan': max(
                self._faulted_scan_nan_counts,
                default=0,
            ),
        }

    def _ekf_report(self):
        yaw_diffs = []
        position_diffs = []
        for key, healthy in self._healthy_ekf.items():
            faulted = self._faulted_ekf.get(key)
            if faulted is None:
                faulted = self._nearest_by_stamp(
                    healthy['stamp'],
                    self._faulted_ekf.values(),
                )
            if faulted is None:
                continue
            yaw_diffs.append(abs(angle_delta(faulted['yaw'], healthy['yaw'])))
            position_diffs.append(
                math.hypot(faulted['x'] - healthy['x'],
                           faulted['y'] - healthy['y'])
            )
        return {
            'healthy_count': len(self._healthy_ekf),
            'faulted_count': len(self._faulted_ekf),
            'yaw_abs_diff_mean_rad': mean(yaw_diffs),
            'yaw_abs_diff_max_rad': max(yaw_diffs, default=None),
            'position_diff_mean_m': mean(position_diffs),
            'position_diff_max_m': max(position_diffs, default=None),
        }

    def _active_windows(self, sensor=None):
        windows = []
        for status in self._statuses:
            if status['state_name'] == 'CANCELLED':
                continue
            if sensor is not None and status['sensor'] != sensor:
                continue
            windows.append((status['start'], status['end']))
        if windows:
            return windows
        return [(-math.inf, math.inf)]

    @staticmethod
    def _in_windows(stamp, windows):
        return any(start <= stamp < end for start, end in windows)

    @staticmethod
    def _drop_rate(raw_count, faulted_count):
        if raw_count <= 0:
            return None
        return max(0.0, float(raw_count - faulted_count) / float(raw_count))

    @staticmethod
    def _total_motion(values):
        if len(values) < 2:
            return None
        first = values[0]
        last = values[-1]
        return math.hypot(last['x'] - first['x'], last['y'] - first['y'])

    @staticmethod
    def _nearest_by_stamp(stamp, values):
        candidates = list(values)
        if not candidates:
            return None
        return min(candidates, key=lambda value: abs(value['stamp'] - stamp))

    @staticmethod
    def _stamp_key(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    @staticmethod
    def _state_name(state):
        names = {
            FaultStatus.SCHEDULED: 'SCHEDULED',
            FaultStatus.ACTIVE: 'ACTIVE',
            FaultStatus.ENDED: 'ENDED',
            FaultStatus.CANCELLED: 'CANCELLED',
        }
        return names.get(int(state), f'UNKNOWN_{int(state)}')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=18.0)
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', *ros_args])
    node = FaultProbe(parsed.duration)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(json.dumps(node.report(), sort_keys=True), flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['FaultProbe', 'main']
