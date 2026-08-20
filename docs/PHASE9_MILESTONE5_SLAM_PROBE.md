# Phase 9 Milestone 5 — Read-only SLAM Probe

## Scope

`resilient_nav_slam/slam_probe` is a diagnostic observer for the frozen
healthy mapping baseline.  It subscribes only to `/scan` and `/map`, queries
the existing TF buffer, and writes periodic JSON snapshots through the ROS
logger.  It creates no application publisher, publishes no TF, changes no
Slam Toolbox/EKF parameter, and has no evaluator, pose-graph analysis,
Ground Truth, health, fault, or control path.

The console entry point is:

```bash
ros2 run resilient_nav_slam slam_probe
```

The node defaults to `use_sim_time=true`, matching the Phase 9 Gazebo smoke.
It reports `scan_rate_hz`, `map_received`, map resolution/width/height,
unknown/free/occupied ratios, `map_to_odom_freshness_sec`,
`odom_to_base_footprint_freshness_sec`, each frozen TF edge's connectivity,
and aggregate `tf_chain_connected`.

## Dynamic smoke evidence (2026-08-20)

The existing `phase9_healthy_mapping_smoke.launch.py` was run with RViz
disabled only for this short observer run; the robot remained stationary.  In
an isolated ROS domain and Gazebo partition, the probe's first periodic report
was empty while subscriptions and the TF buffer were filling.  All following
reports were healthy:

| Field | Observed value |
| --- | --- |
| `scan_frame` / `scan_rate_hz` | `lidar_link`; approximately `12.2–13.7 Hz` |
| `map_received` | `true` |
| map geometry | `0.05 m` resolution, `73 × 30` cells |
| occupancy ratios | unknown `0.7863`, free `0.2014`, occupied `0.0123` |
| `map_to_odom_freshness_sec` | `0.0 s` |
| `odom_to_base_footprint_freshness_sec` | `0.05 s` |
| `tf_chain_connected` | `true`, with every required edge connected |

These are interface observations from a stationary smoke, not map quality,
pose-graph, localization accuracy, or navigation performance claims.

## Debug record

| Phenomenon | Locate | Modification | Verify |
| --- | --- | --- | --- |
| Initial standalone probe reported multi-year TF freshness despite connected TF. | `ros2 run` defaults to system time, while the smoke's transform stamps use Gazebo `/clock`. | Passed a local `use_sim_time=true` parameter override when the probe node is constructed. | Corrected run reported `0.0 s` map-to-odom and `0.05 s` odom-to-base freshness. |
| Unit test expected a binary floating-point scan-rate calculation to equal exactly `5.0`. | The metric was `4.999999999999996` because timestamps are floats. | Changed the test to use pytest approximate comparison; the metric calculation is unchanged. | Targeted pure-metric and package tests passed. |
