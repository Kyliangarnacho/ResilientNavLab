# Phase 9 评价边界

`phase9_evaluation.launch.py` 才 include Phase 8 已有 Ground Truth bridge/adapter。mapping 与 localization core launch 不依赖 `/evaluation/*`。`slam_pose_adapter` 只读查询 `map -> base_footprint` 并发布 `/evaluation/slam_pose`；`slam_map_adapter` 只读转换 `/map` metadata；`slam_evaluator` 只订阅 evaluation topics，使用 Phase 8 timestamp pairing/yaw wrap/RMSE 工具和一次显式 SE(2) 初始对齐。

Ground Truth、Gazebo exact pose、GT error 与 evaluator JSON 永不进入 Slam Toolbox、Health/Fusion policy 或 Agent。Run JSON 保存在 `resilient_nav_slam/results/phase9/`。

Run B rosbag 保存在临时目录 `/tmp/phase9_run_b_mapping_bag`，未提交；MCAP 19.0 MiB、118.219 s、125341 messages，包含 `/clock`、`/cmd_vel`、`/scan`、`/odometry/filtered`、`/map`、`/tf`、`/tf_static`、Ground Truth 与 SLAM pose。

本机 Jazzy 可提供 graph visualization 等公开话题，但在本轮运行中没有稳定、可区分“普通 graph update”和“loop closure”的公开证据。因此 loop closure **未确认**；不得把闭环路线返回起点等同于 loop closure。

## 最小参数对比

在同一 `phase9_slam_world`、同一建图起点、相同约 24 秒的正方形短路线和相同其余配置下，仅比较 `minimum_travel_distance`。`0.1` run 的地图为 `228 x 307`、unknown/free/occupied `0.521158 / 0.465984 / 0.012858`、position/yaw RMSE `0.109931 m / 0.209194 rad`、样本 `1416`；`0.5` run 的地图为 `227 x 346`、unknown/free/occupied `0.775954 / 0.218571 / 0.005475`、position/yaw RMSE `0.064098 m / 0.141079 rad`、样本 `1010`。结果 JSON 分别为 `run_parameter_min_travel_0_1.json` 和 `run_parameter_min_travel_0_5.json`。

正式 baseline 的当前值正是 `0.5`，故任务书中的“当前 baseline”与显式 `0.5` 并非三种不同参数。对同值 `0.5` 的两次额外隔离重复启动均在 Gazebo `/world/resilient_lab/create` service 未就绪时停止，未生成 evaluator JSON；该负结果保留，未伪造第三组数据。较小阈值理论上会处理更多 scan，但更多 scan 不必然降低误差；过大阈值会减少约束与地图更新。任何参数结论都必须在同路线下比较；本轮不因此改变冻结的正式 baseline。
