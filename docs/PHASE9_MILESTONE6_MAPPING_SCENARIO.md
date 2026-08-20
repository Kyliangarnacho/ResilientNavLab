# Phase 9 Milestone 6 — Healthy Mapping World and Route

## Reproducible scenario

`resilient_nav_simulation/worlds/phase9_slam_world.sdf` is an independent,
static 12 m room.  It leaves `phase2_world.sdf` unchanged.  The new world has
four perimeter walls, asymmetric L and T structures, rectangular obstacles of
different sizes, an occluding block, and a cylinder.  These provide distinct
corner and occlusion geometry without creating a maze.

The default Phase 9 smoke now selects this world and spawns the robot at
`(-3.5, -3.5, 0)`.  It can still select another SDF explicitly:

```bash
ros2 launch resilient_nav_slam phase9_healthy_mapping_smoke.launch.py \
  world:=/absolute/path/to/another_world.sdf
```

The older Phase 2/3/4 launch defaults remain `phase2_world.sdf`.

The repeatable Phase 9 route is:

```bash
ros2 run resilient_nav_simulation phase9_mapping_route
```

It publishes four straight segments and four positive 90-degree turns per
lap, returning to the start position and heading.  Defaults are `0.25 m/s`,
`16 s` per side (a nominal 4 m square), `0.6 rad/s`, and one lap.  The route
uses the extracted shared `motion_safety` primitives from `motion_test`, so
normal completion and Ctrl-C both execute the same repeated zero-`/cmd_vel`
stop.  `--straight-duration`, `--turn-angle`, `--turn-duration-scale`, and
`--laps` are explicit experiment parameters.

No Nav2, adaptive EKF, SLAM algorithm parameter, TF owner, evaluator, or
pose-graph feature is added.

## Dynamic smoke (2026-08-20)

The final smoke used the new default world, healthy mapping launch, read-only
`slam_probe`, and one intentionally short route lap (`0.25 m/s`, `5 s` per
side, `0.6 rad/s`, scale `1.14`).  The robot completed all eight segments and
the final repeated `/cmd_vel` samples were zero.

| Observation | Before route | After route |
| --- | --- | --- |
| map geometry | `227 × 226` at `0.05 m` | `227 × 290` at `0.05 m` |
| unknown/free/occupied | `0.8860 / 0.1113 / 0.0027` | `0.5528 / 0.4376 / 0.0097` |
| scan rate | `13.84 Hz` | `13.32 Hz` |
| `map -> odom` freshness | `0.0 s` | `0.0 s` |
| `odom -> base_footprint` freshness | `0.05 s` | `0.05 s` |
| TF chain | connected | connected |

The final wheel-odometry position was `(0.134, -0.043) m`, a displacement of
approximately `0.141 m` from its zeroed spawn-relative odometry origin.  The
route terminated normally, the Gazebo/launch log contained no collision,
contact, TF extrapolation, error, or failure diagnostic, and no old
`odom_tf_broadcaster` appeared in the TF-owner inspection.  The observed
dynamic owners remained `robot_state_publisher`, `ekf_filter_node`, and
`slam_toolbox` under the frozen ownership contract.

These are short smoke observations only.  They do not measure mapping
accuracy, loop closure quality, localization accuracy, collision safety under
arbitrary routes, or navigation performance.

## Phenomenon → locate → modify → verify

| Phenomenon | Locate | Modification | Verify |
| --- | --- | --- | --- |
| Initial open-loop square route ended about `0.78 m` from its origin. | Actual Gazebo turn response did not produce the nominal 90-degree heading change from the uncorrected duration. | Added explicit, bounded `turn_duration_scale`; a first `1.25` trial over-compensated, so the final default was calibrated to `1.14`. | Final short lap ended about `0.141 m` from its odometry origin. |
| `slam_probe` emitted a shutdown traceback when a bounded smoke sent SIGINT. | `rclpy.shutdown()` was called after the context had already been interrupted. | Treat `KeyboardInterrupt` as normal probe termination and only shut down an active context. | Final bounded probe logs ended without the prior traceback. |
| `ros2 launch --show-args` initially failed in the sandbox. | ROS launch attempted to create its default read-only `~/.ros/log` directory. | Directed this verification's `ROS_LOG_DIR` to `/tmp`; no launch code changed. | Phase 9 arguments displayed with the new world as default. |
