# Phase 9 Milestone 4 — Healthy Mapping Smoke

## Scope and composition

The smoke starts a **stationary** healthy robot only.  It has no
`motion_test`, route, probe, evaluator, fault input, health input, or Robot
Agent.

`resilient_nav_slam/launch/phase9_healthy_mapping_smoke.launch.py` composes:

1. the existing Phase 4 IMU/LiDAR Gazebo chain (world, spawn, sensor bridge,
   and `robot_state_publisher`), with its RViz disabled and its legacy
   `odom_tf_broadcaster` explicitly disabled;
2. the existing healthy EKF, consuming `/wheel/odometry` and publishing the
   sole `odom -> base_footprint` transform;
3. the package's Slam Toolbox `online_async` mapping launch, publishing the
   sole `map -> odom` transform and `/map`; and
4. the Phase 9 RViz configuration.

No production estimator, controller, or SLAM parameter was tuned for this
smoke.

## Dynamic evidence (2026-08-20)

The smoke was run in an isolated ROS domain and Gazebo partition after
sourcing Jazzy and the workspace.  The robot remained stationary.

| Check | Observed result |
| --- | --- |
| Scan path | `/scan` was published by `sensor_bridge`; its received `header.frame_id` was `lidar_link`; Slam Toolbox subscribed to it. |
| Map path | `/map` was published by `slam_toolbox`; its received `header.frame_id` was `map`. |
| Required TF | `map -> odom`, `odom -> base_footprint`, and `base_footprint -> lidar_link` each resolved repeatedly.  The static LiDAR offset was `[-0.100, 0.000, 0.350]`. |
| TF owners | `/tf` publishers were `robot_state_publisher`, `ekf_filter_node`, and `slam_toolbox`; the node list contained no `odom_tf_broadcaster`. |
| Phase 9 RViz | Both `/phase9_mapping_rviz` and its `rviz2` process were present. |
| TF diagnostics | No `extrapolat`, `error`, or `failed` entry was found in the final launch log tree.  `tf2_echo` first reported a missing frame while its listener was warming up, then resolved the requested transform; this is not a runtime extrapolation failure. |

Slam Toolbox registered the LiDAR successfully.  It also emitted its
non-blocking warning that the conservatively inherited `max_laser_range`
(`20.0 m`) exceeds the simulated LiDAR capability (`12.0 m`).  It was not
changed: this milestone explicitly excludes SLAM performance tuning, and the
mapping/TF smoke completed with the physical sensor stream.

## Phenomenon → locate → modify → verify

| Phenomenon | Locate | Modification | Verification |
| --- | --- | --- | --- |
| The first verification runner could see neither `/scan` nor `/map`, even though the launch log showed the sensor bridge and Slam sensor registration. | The launch list had been backgrounded together with its setup commands, so later probe commands did not inherit the isolated ROS environment. | Corrected the smoke runner to source/setup/export in the same shell before launching and querying. | `/scan`=`lidar_link`, `/map`=`map`, and all required transforms resolved in the corrected isolated run. |
| The initial smoke started no Phase 9 RViz process. | The included Phase 4 launch used the same `use_rviz` configuration name and set it to `false`, overwriting the parent launch configuration. | Wrapped the Phase 4 include in a scoped `GroupAction`; its internal RViz stays disabled without affecting the Phase 9 RViz condition. | Final node list contained `/phase9_mapping_rviz`, and the corresponding `rviz2` process was running. |
