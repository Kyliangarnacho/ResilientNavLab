"""Read-only readiness probe for the Phase 10 Global and Local Costmap pair."""

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
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from costmap_contract import rolling_window_center
from global_costmap_probe import (
    MAP_QOS,
    VOLATILE_RELIABLE_QOS,
    validate_costmap,
    validate_footprint,
)
from local_costmap_probe import (
    LOCAL_COSTMAP_SERVICE,
    LOCAL_COSTMAP_TOPIC,
    LOCAL_FOOTPRINT_TOPIC,
    LOCAL_PARAMETER_SERVICE,
    PARAMETER_NAMES,
    parameter_value,
    validate_effective_parameters,
    validate_local_costmap,
)
from map_server_probe import validate_map


class CostmapJointProbe(Node):
    """Observe both Costmaps and their shared healthy localization prerequisites."""

    def __init__(self) -> None:
        super().__init__('phase10_costmap_joint_probe')
        self.map_message: OccupancyGrid | None = None
        self.global_costmap_message: OccupancyGrid | None = None
        self.local_costmap_message: OccupancyGrid | None = None
        self.scan_message: LaserScan | None = None
        self.global_footprint_message: PolygonStamped | None = None
        self.local_footprint_message: PolygonStamped | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.global_lifecycle_client = self.create_client(
            GetState, '/global_costmap/global_costmap/get_state'
        )
        self.local_lifecycle_client = self.create_client(GetState, LOCAL_COSTMAP_SERVICE)
        self.local_parameter_client = self.create_client(
            GetParameters, LOCAL_PARAMETER_SERVICE
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, MAP_QOS)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._on_global_costmap,
            VOLATILE_RELIABLE_QOS,
        )
        self.create_subscription(
            OccupancyGrid, LOCAL_COSTMAP_TOPIC, self._on_local_costmap,
            VOLATILE_RELIABLE_QOS,
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            PolygonStamped, '/global_costmap/published_footprint',
            self._on_global_footprint, VOLATILE_RELIABLE_QOS,
        )
        self.create_subscription(
            PolygonStamped, LOCAL_FOOTPRINT_TOPIC, self._on_local_footprint,
            VOLATILE_RELIABLE_QOS,
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_message = message

    def _on_global_costmap(self, message: OccupancyGrid) -> None:
        self.global_costmap_message = message

    def _on_local_costmap(self, message: OccupancyGrid) -> None:
        self.local_costmap_message = message

    def _on_scan(self, message: LaserScan) -> None:
        self.scan_message = message

    def _on_global_footprint(self, message: PolygonStamped) -> None:
        self.global_footprint_message = message

    def _on_local_footprint(self, message: PolygonStamped) -> None:
        self.local_footprint_message = message

    def lifecycle_active(self, client) -> bool:
        if not client.service_is_ready():
            return False
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        return bool(
            future.done()
            and future.result() is not None
            and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        )

    def tf_chain_ready(self) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=0.1)
            )
            self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', Time(), timeout=Duration(seconds=0.1)
            )
        except Exception:
            return False
        return True

    def local_effective_parameters(self) -> dict[str, object] | None:
        if not self.local_parameter_client.service_is_ready():
            return None
        future = self.local_parameter_client.call_async(
            GetParameters.Request(names=list(PARAMETER_NAMES))
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if not future.done() or future.result() is None:
            return None
        return {
            name: parameter_value(value)
            for name, value in zip(PARAMETER_NAMES, future.result().values)
        }


def local_window_alignment(probe: CostmapJointProbe, costmap: OccupancyGrid) -> dict[str, float]:
    """Report the instantaneous odom-centred rolling-window geometry."""
    transform = probe.tf_buffer.lookup_transform(
        'odom', 'base_footprint', Time(), timeout=Duration(seconds=0.5)
    ).transform
    origin = costmap.info.origin.position
    centre_x, centre_y = rolling_window_center(
        origin.x,
        origin.y,
        costmap.info.width,
        costmap.info.height,
        costmap.info.resolution,
    )
    error = math.hypot(centre_x - transform.translation.x, centre_y - transform.translation.y)
    if error > 0.075:
        raise ValueError(f'Local Costmap centre error is {error:.3f} m')
    return {
        'window_center_x': centre_x,
        'window_center_y': centre_y,
        'robot_x': transform.translation.x,
        'robot_y': transform.translation.y,
        'center_error_m': error,
    }


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    parser.add_argument('--result-path', required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_arguments(argv)
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        raise SystemExit('--timeout-sec must be finite and positive')
    result_path = Path(args.result_path)
    result: dict[str, object] = {'outcome': 'FAIL', 'probe': 'phase10_costmap_joint_probe'}
    rclpy.init(args=argv)
    probe = CostmapJointProbe()
    try:
        deadline = time.monotonic() + args.timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.2)
            if (
                probe.lifecycle_active(probe.global_lifecycle_client)
                and probe.lifecycle_active(probe.local_lifecycle_client)
                and probe.map_message is not None
                and probe.global_costmap_message is not None
                and probe.local_costmap_message is not None
                and probe.scan_message is not None
                and probe.global_footprint_message is not None
                and probe.local_footprint_message is not None
                and probe.tf_chain_ready()
                and probe.local_effective_parameters() is not None
            ):
                break
        if not probe.lifecycle_active(probe.global_lifecycle_client):
            raise RuntimeError('global_costmap is not ACTIVE')
        if not probe.lifecycle_active(probe.local_lifecycle_client):
            raise RuntimeError('local_costmap is not ACTIVE')
        if probe.map_message is None or probe.global_costmap_message is None:
            raise RuntimeError('no /map or /global_costmap/costmap received')
        if probe.local_costmap_message is None:
            raise RuntimeError(f'no {LOCAL_COSTMAP_TOPIC} received')
        if probe.scan_message is None or probe.scan_message.header.frame_id != 'lidar_link':
            raise RuntimeError('no valid lidar_link /scan received')
        if probe.global_footprint_message is None or probe.local_footprint_message is None:
            raise RuntimeError('one or both Costmap footprints are missing')
        if not probe.tf_chain_ready():
            raise RuntimeError('map -> odom -> base_footprint chain was not queryable')
        parameters = probe.local_effective_parameters()
        if parameters is None:
            raise RuntimeError('unable to query effective Local Costmap parameters')

        runtime_map = validate_map(probe.map_message)
        runtime_global = validate_costmap(probe.global_costmap_message, probe.map_message)
        runtime_local = validate_local_costmap(probe.local_costmap_message)
        runtime_parameters = validate_effective_parameters(parameters)
        result.update({
            'outcome': 'PASS',
            'lifecycle': {'global_costmap': 'active', 'local_costmap': 'active'},
            'runtime_map': runtime_map,
            'global_costmap': runtime_global,
            'local_costmap': runtime_local,
            'local_effective_parameters': runtime_parameters,
            'scan_frame_id': probe.scan_message.header.frame_id,
            'global_footprint': validate_footprint(
                probe, probe.global_footprint_message, target_frame='map'
            ),
            'local_footprint': validate_footprint(
                probe, probe.local_footprint_message, target_frame='odom'
            ),
            'local_window_alignment': local_window_alignment(
                probe, probe.local_costmap_message
            ),
            'tf_chain': 'map -> odom -> base_footprint',
        })
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
