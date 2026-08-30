"""Read-only Phase 10 Global Costmap readiness and footprint probe."""

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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from costmap_contract import (
    FOOTPRINT_PADDING,
    PHYSICAL_FOOTPRINT,
    close,
    cost_distribution,
    grid_cell_index,
    padded_footprint,
    transform_polygon,
    yaw_from_quaternion,
)
from map_server_probe import validate_map


MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
VOLATILE_RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

class GlobalCostmapProbe(Node):
    """Observe the Costmap without publishing motion, TF, or parameters."""

    def __init__(self) -> None:
        super().__init__('phase10_global_costmap_probe')
        self.map_message: OccupancyGrid | None = None
        self.costmap_message: OccupancyGrid | None = None
        self.scan_message: LaserScan | None = None
        self.footprint_message: PolygonStamped | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lifecycle_client = self.create_client(
            GetState, '/global_costmap/global_costmap/get_state'
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, MAP_QOS)
        self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self._on_costmap,
            VOLATILE_RELIABLE_QOS,
        )
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            PolygonStamped,
            '/global_costmap/published_footprint',
            self._on_footprint,
            VOLATILE_RELIABLE_QOS,
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_message = message

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

    def map_to_base_ready(self) -> bool:
        """Wait until this probe has a coherent sim-time TF sample."""
        try:
            self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=0.1)
            )
        except Exception:
            return False
        return True


def _transform_footprint(
    probe: GlobalCostmapProbe, target_frame: str = 'map'
) -> tuple[tuple[float, float], ...]:
    """Transform the frozen padded footprint into a requested Costmap frame."""
    transform = probe.tf_buffer.lookup_transform(
        target_frame, 'base_footprint', Time(), timeout=Duration(seconds=0.5)
    ).transform
    return transform_polygon(padded_footprint(), transform)


def validate_costmap(
    costmap: OccupancyGrid, runtime_map: OccupancyGrid
) -> dict[str, object]:
    """Check a non-rolling global grid still has the frozen map geometry."""
    origin = costmap.info.origin
    map_origin = runtime_map.info.origin
    checks = {
        'frame_id': costmap.header.frame_id == 'map',
        'resolution': close(costmap.info.resolution, runtime_map.info.resolution),
        'width': costmap.info.width == runtime_map.info.width,
        'height': costmap.info.height == runtime_map.info.height,
        'origin_x': close(origin.position.x, map_origin.position.x),
        'origin_y': close(origin.position.y, map_origin.position.y),
        'data_length': len(costmap.data) == costmap.info.width * costmap.info.height,
        'occupancy_values': all(-1 <= value <= 100 for value in costmap.data),
    }
    if not all(checks.values()):
        failed = ', '.join(name for name, passed in checks.items() if not passed)
        raise ValueError(f'/global_costmap/costmap failed checks: {failed}')
    distribution = cost_distribution(costmap.data)
    if distribution['inflated'] == 0 or distribution['lethal_like'] == 0:
        raise ValueError('costmap has no observable lethal or inflated geometry')
    return {
        'frame_id': costmap.header.frame_id,
        'resolution': costmap.info.resolution,
        'width': costmap.info.width,
        'height': costmap.info.height,
        'origin': {'x': origin.position.x, 'y': origin.position.y},
        'data_length': len(costmap.data),
        'cost_distribution': distribution,
    }


def validate_footprint(
    probe: GlobalCostmapProbe, message: PolygonStamped, target_frame: str = 'map'
) -> dict[str, object]:
    """Compare Nav2's published padded polygon with the TF-transformed contract."""
    if message.header.frame_id != target_frame:
        raise ValueError(f'published footprint is not in the {target_frame} frame')
    observed = tuple((point.x, point.y) for point in message.polygon.points)
    expected = _transform_footprint(probe, target_frame)
    if len(observed) != len(expected):
        raise ValueError(
            f'published footprint has {len(observed)} points, expected {len(expected)}'
        )
    if not all(
        close(actual_x, expected_x, 0.02) and close(actual_y, expected_y, 0.02)
        for (actual_x, actual_y), (expected_x, expected_y) in zip(observed, expected)
    ):
        raise ValueError('published footprint does not match the padded base_footprint polygon')
    return {
        'frame_id': message.header.frame_id,
        'point_count': len(observed),
        'padding_m': FOOTPRINT_PADDING,
        'physical_polygon_base_footprint': PHYSICAL_FOOTPRINT,
    }


def cost_at(costmap: OccupancyGrid, x: float, y: float) -> int:
    """Read one map-frame OccupancyGrid cell without mutating the Costmap."""
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


def _percentile(sorted_values: list[float], fraction: float) -> float:
    index = min(len(sorted_values) - 1, math.ceil(fraction * len(sorted_values)) - 1)
    return sorted_values[index]


def scan_map_sanity(probe: GlobalCostmapProbe, runtime_map: OccupancyGrid) -> dict[str, float | int]:
    """Reject a coherent scan/map displacement at the scan's own timestamp."""
    scan = probe.scan_message
    if scan is None:
        raise ValueError('no scan available for map-alignment sanity check')
    transform = probe.tf_buffer.lookup_transform(
        'map',
        'lidar_link',
        Time.from_msg(scan.header.stamp),
        timeout=Duration(seconds=0.5),
    ).transform
    q = transform.rotation
    yaw = yaw_from_quaternion(q)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    resolution = runtime_map.info.resolution
    origin = runtime_map.info.origin.position
    occupied = [
        (
            origin.x + (index % runtime_map.info.width + 0.5) * resolution,
            origin.y + (index // runtime_map.info.width + 0.5) * resolution,
        )
        for index, value in enumerate(runtime_map.data)
        if value >= 65
    ]
    if not occupied:
        raise ValueError('frozen map has no occupied cells for scan sanity check')
    distances = []
    upper_range = min(scan.range_max, 2.5)
    for index, scan_range in enumerate(scan.ranges[::4]):
        if not math.isfinite(scan_range) or not scan.range_min <= scan_range <= upper_range:
            continue
        angle = scan.angle_min + (index * 4) * scan.angle_increment
        lidar_x = scan_range * math.cos(angle)
        lidar_y = scan_range * math.sin(angle)
        endpoint_x = transform.translation.x + lidar_x * cosine - lidar_y * sine
        endpoint_y = transform.translation.y + lidar_x * sine + lidar_y * cosine
        distances.append(min(
            math.hypot(endpoint_x - occupied_x, endpoint_y - occupied_y)
            for occupied_x, occupied_y in occupied
        ))
    if len(distances) < 20:
        raise ValueError('too few finite scan endpoints for map-alignment sanity check')
    distances.sort()
    median = _percentile(distances, 0.5)
    p90 = _percentile(distances, 0.9)
    if median > 0.10 or p90 > 0.20:
        raise ValueError(
            'scan/map residual exceeds the cell-level sanity limits '
            f'(median={median:.3f} m, p90={p90:.3f} m)'
        )
    return {
        'scan_stamp_sec': scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9,
        'finite_endpoint_samples': len(distances),
        'median_nearest_static_occupied_m': median,
        'p90_nearest_static_occupied_m': p90,
    }


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    parser.add_argument('--result-path', required=True)
    parser.add_argument('--watch-x', type=float)
    parser.add_argument('--watch-y', type=float)
    parser.add_argument(
        '--skip-scan-map-sanity',
        action='store_true',
        help=(
            'Use only while a deliberately non-map temporary obstacle is in '
            'view; the baseline smoke always runs the saved-map sanity check.'
        ),
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_arguments(argv)
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        raise SystemExit('--timeout-sec must be finite and positive')
    if (args.watch_x is None) != (args.watch_y is None):
        raise SystemExit('--watch-x and --watch-y must be provided together')

    result_path = Path(args.result_path)
    result: dict[str, object] = {'outcome': 'FAIL', 'probe': 'phase10_global_costmap_probe'}
    rclpy.init(args=argv)
    probe = GlobalCostmapProbe()
    try:
        deadline = time.monotonic() + args.timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.2)
            if (
                probe.lifecycle_active()
                and probe.map_message is not None
                and probe.costmap_message is not None
                and probe.scan_message is not None
                and probe.footprint_message is not None
                and probe.map_to_base_ready()
            ):
                break
        if not probe.lifecycle_active():
            raise RuntimeError('global_costmap is not ACTIVE')
        if probe.map_message is None:
            raise RuntimeError('no /map received')
        if probe.costmap_message is None:
            raise RuntimeError('no /global_costmap/costmap received')
        if probe.scan_message is None or probe.scan_message.header.frame_id != 'lidar_link':
            raise RuntimeError('no valid lidar_link /scan received')
        if probe.footprint_message is None:
            raise RuntimeError('no /global_costmap/published_footprint received')
        if not probe.map_to_base_ready():
            raise RuntimeError('map -> base_footprint was not queryable by the probe')

        runtime_map = validate_map(probe.map_message)
        runtime_costmap = validate_costmap(probe.costmap_message, probe.map_message)
        runtime_footprint = validate_footprint(probe, probe.footprint_message)
        scan_sanity = (
            {'skipped_for_temporary_non_map_obstacle': True}
            if args.skip_scan_map_sanity
            else scan_map_sanity(probe, probe.map_message)
        )
        result.update({
            'outcome': 'PASS',
            'lifecycle': {'global_costmap': 'active'},
            'runtime_map': runtime_map,
            'runtime_costmap': runtime_costmap,
            'scan_frame_id': probe.scan_message.header.frame_id,
            'scan_map_sanity': scan_sanity,
            'published_footprint': runtime_footprint,
        })
        if args.watch_x is not None:
            result['watch_cell'] = {
                'x': args.watch_x,
                'y': args.watch_y,
                'occupancy': cost_at(probe.costmap_message, args.watch_x, args.watch_y),
            }
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
