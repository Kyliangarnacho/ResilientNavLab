"""Read-only Phase 10 Local Costmap readiness, rolling-window, and ROI probe."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from geometry_msgs.msg import PolygonStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from costmap_contract import (
    LETHAL_COST_THRESHOLD,
    close,
    cost_distribution,
    grid_cell_index,
    rolling_window_center,
)
from global_costmap_probe import VOLATILE_RELIABLE_QOS, validate_footprint


LOCAL_COSTMAP_SERVICE = '/local_costmap/local_costmap/get_state'
LOCAL_PARAMETER_SERVICE = '/local_costmap/local_costmap/get_parameters'
LOCAL_COSTMAP_TOPIC = '/local_costmap/costmap'
LOCAL_FOOTPRINT_TOPIC = '/local_costmap/published_footprint'

EXPECTED_LOCAL_PARAMETERS = {
    'global_frame': 'odom',
    'robot_base_frame': 'base_footprint',
    'rolling_window': True,
    'width': 6,
    'height': 6,
    'resolution': 0.05,
    'track_unknown_space': False,
    'update_frequency': 5.0,
    'publish_frequency': 2.0,
}
PARAMETER_NAMES = (
    'global_frame',
    'robot_base_frame',
    'rolling_window',
    'width',
    'height',
    'resolution',
    'track_unknown_space',
    'update_frequency',
    'publish_frequency',
    'inflation_layer.inflation_radius',
    'inflation_layer.cost_scaling_factor',
    'obstacle_layer.scan.expected_update_rate',
    'obstacle_layer.scan.sensor_frame',
)


class LocalCostmapProbe(Node):
    """Observe Local Costmap state without publishing ROS data or parameters."""

    def __init__(self) -> None:
        super().__init__('phase10_local_costmap_probe')
        self.costmap_message: OccupancyGrid | None = None
        self.scan_message: LaserScan | None = None
        self.footprint_message: PolygonStamped | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lifecycle_client = self.create_client(GetState, LOCAL_COSTMAP_SERVICE)
        self.parameter_client = self.create_client(GetParameters, LOCAL_PARAMETER_SERVICE)
        self.create_subscription(
            OccupancyGrid, LOCAL_COSTMAP_TOPIC, self._on_costmap, VOLATILE_RELIABLE_QOS
        )
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            PolygonStamped, LOCAL_FOOTPRINT_TOPIC, self._on_footprint,
            VOLATILE_RELIABLE_QOS,
        )

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self.costmap_message = message

    def _on_scan(self, message: LaserScan) -> None:
        self.scan_message = message

    def _on_footprint(self, message: PolygonStamped) -> None:
        self.footprint_message = message

    def lifecycle_active(self) -> bool:
        if not self.lifecycle_client.service_is_ready():
            return False
        future = self.lifecycle_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        return bool(
            future.done()
            and future.result() is not None
            and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        )

    def odom_to_base_ready(self) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', Time(), timeout=Duration(seconds=0.1)
            )
        except Exception:
            return False
        return True

    def effective_parameters(self) -> dict[str, object] | None:
        """Read effective Costmap parameters through the standard read-only service."""
        if not self.parameter_client.service_is_ready():
            return None
        request = GetParameters.Request(names=list(PARAMETER_NAMES))
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if not future.done() or future.result() is None:
            return None
        return {
            name: parameter_value(value)
            for name, value in zip(PARAMETER_NAMES, future.result().values)
        }


def parameter_value(value) -> object:
    """Convert the small, fixed parameter set to JSON-safe primitive values."""
    if value.type == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if value.type == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    if value.type == ParameterType.PARAMETER_STRING:
        return value.string_value
    raise ValueError(f'unsupported or unset local Costmap parameter type {value.type}')


def validate_local_costmap(costmap: OccupancyGrid) -> dict[str, object]:
    """Validate Local Costmap metadata without imposing Global-map geometry."""
    origin = costmap.info.origin.position
    checks = {
        'frame_id': costmap.header.frame_id == 'odom',
        'resolution': close(costmap.info.resolution, 0.05),
        'width': costmap.info.width == 120,
        'height': costmap.info.height == 120,
        'data_length': len(costmap.data) == costmap.info.width * costmap.info.height,
        'occupancy_values': all(-1 <= value <= 100 for value in costmap.data),
    }
    if not all(checks.values()):
        failed = ', '.join(name for name, passed in checks.items() if not passed)
        raise ValueError(f'{LOCAL_COSTMAP_TOPIC} failed checks: {failed}')
    return {
        'frame_id': costmap.header.frame_id,
        'resolution': costmap.info.resolution,
        'width': costmap.info.width,
        'height': costmap.info.height,
        'origin': {'x': origin.x, 'y': origin.y},
        'window_size_m': {
            'width': costmap.info.width * costmap.info.resolution,
            'height': costmap.info.height * costmap.info.resolution,
        },
        'cost_distribution': cost_distribution(costmap.data),
    }


def validate_effective_parameters(parameters: dict[str, object]) -> dict[str, object]:
    """Freeze frame/window contract while allowing explicit experiment overrides."""
    for name, expected in EXPECTED_LOCAL_PARAMETERS.items():
        actual = parameters.get(name)
        if isinstance(expected, float):
            if not isinstance(actual, float) or not close(actual, expected):
                raise ValueError(f'local Costmap parameter {name} is not {expected}')
        elif actual != expected:
            raise ValueError(f'local Costmap parameter {name} is not {expected!r}')
    for name in (
        'inflation_layer.inflation_radius',
        'inflation_layer.cost_scaling_factor',
        'obstacle_layer.scan.expected_update_rate',
    ):
        actual = parameters.get(name)
        if not isinstance(actual, float) or not math.isfinite(actual) or actual <= 0.0:
            raise ValueError(f'local Costmap parameter {name} must be finite and positive')
    if parameters.get('obstacle_layer.scan.sensor_frame') != 'lidar_link':
        raise ValueError('local Costmap scan sensor frame is not lidar_link')
    return parameters


def robot_record(probe: LocalCostmapProbe, costmap: OccupancyGrid) -> dict[str, float]:
    """Pair one received Costmap publication with the current odom pose.

    Standalone ``nav2_costmap_2d`` can republish an OccupancyGrid carrying a
    stale header timestamp while it updates a rolling window.  The sampling
    instant is therefore the probe receipt time, not the grid header stamp.
    This proves the moving-window relation without treating the published
    timestamp as an authoritative TF query time.
    """
    transform = probe.tf_buffer.lookup_transform(
        'odom',
        'base_footprint',
        Time(),
        timeout=Duration(seconds=0.3),
    ).transform
    origin = costmap.info.origin.position
    centre_x, centre_y = rolling_window_center(
        origin.x,
        origin.y,
        costmap.info.width,
        costmap.info.height,
        costmap.info.resolution,
    )
    return {
        'costmap_stamp_sec': costmap.header.stamp.sec + costmap.header.stamp.nanosec / 1e9,
        'origin_x': origin.x,
        'origin_y': origin.y,
        'window_center_x': centre_x,
        'window_center_y': centre_y,
        'robot_x': transform.translation.x,
        'robot_y': transform.translation.y,
    }


def rolling_summary(
    records: list[dict[str, float]], min_motion_m: float, max_error_m: float
) -> dict[str, object]:
    """Prove a rolling grid follows odom translation with cell-size tolerance."""
    if len(records) < 2:
        raise ValueError('too few distinct Local Costmap publications for rolling check')
    start = records[0]
    end = records[-1]
    robot_dx = end['robot_x'] - start['robot_x']
    robot_dy = end['robot_y'] - start['robot_y']
    origin_dx = end['origin_x'] - start['origin_x']
    origin_dy = end['origin_y'] - start['origin_y']
    motion = math.hypot(robot_dx, robot_dy)
    if motion < min_motion_m:
        raise ValueError(
            f'robot motion {motion:.3f} m is below required {min_motion_m:.3f} m'
        )
    centre_errors = [
        math.hypot(
            record['window_center_x'] - record['robot_x'],
            record['window_center_y'] - record['robot_y'],
        )
        for record in records
    ]
    origin_delta_error = math.hypot(origin_dx - robot_dx, origin_dy - robot_dy)
    if max(centre_errors) > max_error_m or origin_delta_error > max_error_m:
        raise ValueError(
            'rolling window does not follow odom within tolerance '
            f'(max centre error={max(centre_errors):.3f} m, '
            f'origin delta error={origin_delta_error:.3f} m)'
        )
    return {
        'publication_count': len(records),
        'robot_delta_m': {'x': robot_dx, 'y': robot_dy, 'norm': motion},
        'origin_delta_m': {'x': origin_dx, 'y': origin_dy},
        'max_window_center_error_m': max(centre_errors),
        'origin_delta_error_m': origin_delta_error,
        'records': records,
    }


def roi_snapshot(
    costmap: OccupancyGrid, center_x: float, center_y: float, radius: float
) -> dict[str, object]:
    """Capture a compact, world-addressable circular ROI for offline comparison."""
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError('ROI radius must be finite and positive')
    origin = costmap.info.origin.position
    resolution = costmap.info.resolution
    cells = []
    for row in range(costmap.info.height):
        y = origin.y + (row + 0.5) * resolution
        if abs(y - center_y) > radius:
            continue
        for column in range(costmap.info.width):
            x = origin.x + (column + 0.5) * resolution
            if math.hypot(x - center_x, y - center_y) <= radius:
                cells.append([x, y, costmap.data[row * costmap.info.width + column]])
    if not cells:
        raise ValueError('ROI lies outside Local Costmap window')
    return {
        'frame_id': costmap.header.frame_id,
        'center': {'x': center_x, 'y': center_y},
        'radius_m': radius,
        'resolution': resolution,
        'cells': cells,
    }


def cost_at(costmap: OccupancyGrid, x: float, y: float) -> int:
    """Read a single odom-frame Local Costmap cell without mutating it."""
    origin = costmap.info.origin.position
    index = grid_cell_index(
        origin.x,
        origin.y,
        costmap.info.resolution,
        costmap.info.width,
        costmap.info.height,
        x,
        y,
    )
    return costmap.data[index]


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    parser.add_argument('--observe-sec', type=float, default=0.0)
    parser.add_argument('--require-rolling', action='store_true')
    parser.add_argument('--min-motion-m', type=float, default=0.25)
    # 0.125 m bounds 5 Hz update / 2 Hz publication latency at the bounded
    # smoke speed plus one 0.05 m cell quantization.  It is an a-priori
    # interface tolerance, not a threshold fitted after an experiment.
    parser.add_argument('--max-rolling-error-m', type=float, default=0.125)
    parser.add_argument('--result-path', required=True)
    parser.add_argument('--watch-x', type=float)
    parser.add_argument('--watch-y', type=float)
    parser.add_argument('--roi-center-x', type=float)
    parser.add_argument('--roi-center-y', type=float)
    parser.add_argument('--roi-radius', type=float)
    return parser.parse_args(argv)


def _require_pair(args, first: str, second: str) -> None:
    if (getattr(args, first) is None) != (getattr(args, second) is None):
        raise SystemExit(f'--{first.replace("_", "-")} and --{second.replace("_", "-")} must be paired')


def _record_if_new(
    probe: LocalCostmapProbe, records: list[dict[str, float]], last_stamp: tuple[int, int] | None
) -> tuple[int, int] | None:
    costmap = probe.costmap_message
    if costmap is None:
        return last_stamp
    stamp = (costmap.header.stamp.sec, costmap.header.stamp.nanosec)
    if stamp == last_stamp:
        return last_stamp
    try:
        records.append(robot_record(probe, costmap))
    except Exception:
        return last_stamp
    return stamp


def main(argv: list[str] | None = None) -> int:
    args = parse_arguments(argv)
    for name in ('timeout_sec', 'observe_sec', 'min_motion_m', 'max_rolling_error_m'):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0.0:
            raise SystemExit(f'--{name.replace("_", "-")} must be finite and non-negative')
    if args.timeout_sec <= 0.0 or args.min_motion_m <= 0.0 or args.max_rolling_error_m <= 0.0:
        raise SystemExit('timeout, minimum motion, and rolling error must be positive')
    if args.require_rolling and args.observe_sec <= 0.0:
        raise SystemExit('--require-rolling requires --observe-sec > 0')
    _require_pair(args, 'watch_x', 'watch_y')
    _require_pair(args, 'roi_center_x', 'roi_center_y')
    if (args.roi_center_x is None) != (args.roi_radius is None):
        raise SystemExit('ROI center and --roi-radius must be supplied together')

    result_path = Path(args.result_path)
    result: dict[str, object] = {'outcome': 'FAIL', 'probe': 'phase10_local_costmap_probe'}
    rclpy.init(args=argv)
    probe = LocalCostmapProbe()
    try:
        deadline = time.monotonic() + args.timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.2)
            if (
                probe.lifecycle_active()
                and probe.costmap_message is not None
                and probe.scan_message is not None
                and probe.footprint_message is not None
                and probe.odom_to_base_ready()
                and probe.effective_parameters() is not None
            ):
                break
        if not probe.lifecycle_active():
            raise RuntimeError('local_costmap is not ACTIVE')
        if probe.costmap_message is None:
            raise RuntimeError(f'no {LOCAL_COSTMAP_TOPIC} received')
        if probe.scan_message is None or probe.scan_message.header.frame_id != 'lidar_link':
            raise RuntimeError('no valid lidar_link /scan received')
        if probe.footprint_message is None:
            raise RuntimeError(f'no {LOCAL_FOOTPRINT_TOPIC} received')
        if not probe.odom_to_base_ready():
            raise RuntimeError('odom -> base_footprint was not queryable by the probe')
        parameters = probe.effective_parameters()
        if parameters is None:
            raise RuntimeError('unable to query effective Local Costmap parameters')

        runtime_costmap = validate_local_costmap(probe.costmap_message)
        runtime_parameters = validate_effective_parameters(parameters)
        runtime_footprint = validate_footprint(
            probe, probe.footprint_message, target_frame='odom'
        )
        records: list[dict[str, float]] = []
        last_stamp = _record_if_new(probe, records, None)
        if args.observe_sec > 0.0:
            observation_deadline = time.monotonic() + args.observe_sec
            while time.monotonic() < observation_deadline:
                rclpy.spin_once(probe, timeout_sec=0.1)
                last_stamp = _record_if_new(probe, records, last_stamp)

        result.update({
            'outcome': 'PASS',
            'lifecycle': {'local_costmap': 'active'},
            'runtime_costmap': runtime_costmap,
            'effective_parameters': runtime_parameters,
            'scan_frame_id': probe.scan_message.header.frame_id,
            'published_footprint': runtime_footprint,
        })
        if args.require_rolling:
            result['rolling_window'] = rolling_summary(
                records, args.min_motion_m, args.max_rolling_error_m
            )
        if args.watch_x is not None:
            result['watch_cell'] = {
                'x': args.watch_x,
                'y': args.watch_y,
                'occupancy': cost_at(probe.costmap_message, args.watch_x, args.watch_y),
                'lethal_or_higher': cost_at(probe.costmap_message, args.watch_x, args.watch_y)
                >= LETHAL_COST_THRESHOLD,
            }
        if args.roi_center_x is not None:
            result['roi'] = roi_snapshot(
                probe.costmap_message,
                args.roi_center_x,
                args.roi_center_y,
                args.roi_radius,
            )
    except Exception as error:
        result['outcome'] = 'FAIL'
        result['error'] = str(error)
    finally:
        result_path.parent.mkdir(parents=True, exist_ok=True)
        result_path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
        probe.destroy_node()
        rclpy.shutdown()
    if result['outcome'] != 'PASS':
        print(json.dumps(result, indent=2), file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
