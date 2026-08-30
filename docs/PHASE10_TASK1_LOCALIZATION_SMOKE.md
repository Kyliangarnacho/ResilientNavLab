# Phase 10 Task 1 — Healthy Nav2 Localization Smoke

## Scope and ownership

Task 1 adds healthy saved-map localization only.  It does not start a Nav2
planner, controller, costmap, behavior tree, recovery server, or autonomous
navigation action.

The runtime TF contract is fixed as follows:

- `nav2_amcl`: `map -> odom`;
- existing healthy `robot_localization` EKF: `odom -> base_footprint`;
- existing `robot_state_publisher`: robot body and sensor fixed transforms.

Ground Truth remains outside the primary localization launch.  The optional
`phase10_amcl_evaluation.launch.py` is evaluator-only and uses the Phase 9
predeclared `T_map_odom`; it never supplies a runtime input to AMCL, Map Server,
EKF, or the localization wrapper.

## Reproducible entry points

```bash
source /opt/ros/jazzy/setup.bash
cd ~/projects/resilient_nav_lab/ros2_ws
source install/setup.bash

# Task 1.3: no Gazebo and no AMCL.
ros2 launch resilient_nav_navigation phase10_map_server_smoke.launch.py use_rviz:=false

# Task 1.4–1.5: healthy Gazebo, sensors, healthy EKF, Map Server, and AMCL.
ros2 launch resilient_nav_navigation phase10_localization_smoke.launch.py \
  use_rviz:=false auto_initial_pose:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

The localization smoke always starts from the explicit Phase 9 baseline spawn
and does not load an AMCL saved pose.  `phase10_initial_pose_helper` waits for
AMCL to be active, a nonzero simulation clock, and an `/initialpose` subscriber
before sending its one map-frame pose.  RViz remains optional and contains map,
scan, AMCL pose, particle-cloud, and explicit initial-pose views only.

For the motion observation, start the read-only probe first, then use the
existing bounded simulation command (it sends an explicit final stop):

```bash
ros2 run resilient_nav_navigation phase10_localization_probe \
  --observation-sec 20 --result-path /tmp/phase10_task1_motion.json
ros2 run resilient_nav_simulation motion_test arc \
  --linear-speed 0.12 --angular-speed 0.35 --duration 3.0
```

## Recorded dynamic evidence

Task 1.3 Map-Server-only probe passed against the installed Phase 9 asset:

- lifecycle state: `active`;
- `/map`: frame `map`, resolution `0.05000000074505806`, `227 x 226`,
  origin `(-2.22269738693, -2.16105234952, 0)`;
- installed PGM/YAML/posegraph-data/posegraph hashes matched the frozen Phase 9
  identities in `PHASE10_NAV2_JAZZY_INTERFACE.md`.

Task 1.4–1.5 fresh-process motion probe passed after explicit initialization:

- both `map_server` and `amcl` were `active`;
- `/scan` frame was `lidar_link`; `/clock` advanced;
- AMCL published four particle-cloud messages and map-frame pose covariance
  diagonal `x=0.02186149`, `y=0.03976580`, `yaw=0.01485930`;
- direct `/tf` observation recorded `map -> odom` 271 times and
  `odom -> base_footprint` 358 times, composing the required
  `map -> odom -> base_footprint` chain.

The Map Server proof was run separately before AMCL.  The AMCL motion proof was
run in a fresh process after the static initialization proof.  No claim is made
here about absolute localization accuracy or autonomous navigation performance.

## Checks and known issue

The new package static suite passed `12 passed`; `colcon build --symlink-install
--packages-select resilient_nav_navigation` passed.  An ordinary launch shutdown
still exposes an existing `resilient_nav_monitor/system_heartbeat` double
`rclpy.shutdown()` error after Ctrl-C.  It occurs during teardown, is outside
this package, and did not invalidate the active-state or dynamic localization
evidence; it remains a Phase 4 technical debt.
