# Phase 8 Results

Data flow: `/faulted/*` -> fixed EKF; `/faulted/*` + `SensorHealth` -> Measurement Adapter -> `/fusion/input/*` -> adaptive EKF; Gazebo pose -> `/evaluation/ground_truth_pose`; evaluator/Path read only truth, fixed and adaptive outputs.

| scenario | outcome | adaptive_improved | note |
| --- | --- | --- | --- |
| Healthy | PASS | false | 0.05 s: fixed/adaptive position RMSE `0.0703/0.0831 m`, yaw RMSE `0.0740/0.1234 rad`, 904 samples. |
| IMU bias | PASS | true | 0.05 s replay: position RMSE `0.4318/0.2124 m`, yaw RMSE `0.6999/0.1890 rad`, 753 samples. |
| IMU fixed delay | PASS | false on final replay | 0.05 s replay: `0.0745/0.0901 m`, `0.0736/0.1308 rad`, 799 samples; earlier formal run improved, so this is run-to-run-sensitive. |
| IMU dropout | SKIPPED | n/a | Existing 100 Hz, `p=0.30` scene does not reliably meet the `0.5 s` stale evidence condition; unchanged. |
| Wheel freeze | PASS | false | 0.05 s replay: `0.0733/0.0861 m`, `0.0733/0.1165 rad`, 712 samples; rejects bad wheel velocity, not a replacement translation source. |

Alignment sweep uses fixed `0.02/0.03/0.05 s` windows on unchanged trajectories. All four runs had sufficient common samples; sample count grows with the window. Direction stayed false for Healthy, Delay and Wheel and true for Bias, so the principal IMU-bias and wheel fail-closed conclusions are not window-direction artifacts. Raw sweep records are in the four `PHASE8_*_BENCHMARK.json` files.

Recovery evidence: bias `Health/Fusion=15.201/15.4 s`; delay `15.4/15.6 s`; wheel `15.2/15.401 s`, after fault end `15.0 s`. Post-recovery evaluator metrics were required before PASS; hysteresis prevents a single healthy sample from immediate recovery.

`trajectory_path_adapter` publishes `/evaluation/path/ground_truth`, `/evaluation/path/fixed`, `/evaluation/path/adaptive` in `odom`; `phase8_trajectory_overlay.launch.py use_rviz:=true` opens the minimal RViz overlay. It is evaluation-only.

Reproduce with `ROS_DOMAIN_ID=<isolated> GZ_PARTITION=<isolated> ros2 launch resilient_nav_fusion phase8_imu_bias_benchmark.launch.py ...`. Known shutdown noise: `system_heartbeat` may call shutdown twice; Gazebo/motion may require SIGTERM after recorder-led shutdown.
