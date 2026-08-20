# Phase 9 健康二维建图基线

Phase 9 使用本机 ROS 2 Jazzy `slam_toolbox` 的 `online_async` 作为 Borrow runtime。健康 EKF 是 `odom -> base_footprint` 的唯一 owner，Slam Toolbox 是 `map -> odom` 与 `/map` 的唯一 owner；`robot_state_publisher` 提供固定机器人/lidar 链。

正式 Run B 使用 `phase9_slam_world.sdf`、固定 `mapper_params_online_async.yaml`、完整 `phase9_mapping_route` 与只读 evaluation overlay。路线实际持续约 `73.25 s`，结束后由既有安全逻辑发送 zero `/cmd_vel`。结果见 `resilient_nav_slam/results/phase9/run_b_healthy_mapping.json`。

Run B 地图为 `227 x 226`、`0.05 m`，unknown/free/occupied 比例为 `0.212682 / 0.762407 / 0.024911`。经一次明确 SE(2) 初始对齐后的 SLAM-vs-Gazebo Ground Truth 指标：position RMSE `0.323735 m`，yaw RMSE `0.120474 rad`，最大位置/yaw 误差 `1.252462 m / 0.465403 rad`，最终漂移 `1.246515 m / 0.455149 rad`，return-to-start 误差 `1.246515 m / 0.455149 rad`，样本数 `3027`。

这些是健康仿真 baseline，不代表生产精度、Nav2 能力、fault-aware SLAM 或 Adaptive EKF 对比。
