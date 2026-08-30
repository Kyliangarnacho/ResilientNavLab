# Phase 10 Task 3.1：Planner Server + ComputePathToPose

记录日期：2026-08-28

## 范围与结论

Task 3.1 已通过健康 saved-map 链上的官方 Nav2 Jazzy 1.3.12
`planner_server` + `nav2_navfn_planner::NavfnPlanner` 动态验收。它只提供
`/compute_path_to_pose`、`/plan` 与 `/is_path_valid` 的全局路径计算；机器人不运动。
没有启动 Controller Server、BT Navigator、Behavior Server、NavigateToPose、recovery 或任何
`/cmd_vel` 发布者。

Task 2 的 standalone Global/Local Costmap smoke 仍原样保留为独立回归入口。正式 Planner
入口**不**启动第二个 standalone Global Costmap：`planner_server` 自己拥有唯一
`/global_costmap/global_costmap`，并复用同一份冻结的 Task 2 Global Costmap 参数。

## 正式数据链与 ownership

```text
Phase 9 frozen occupancy map -> Map Server -> /map -> Planner Global StaticLayer
healthy /scan -> Planner Global ObstacleLayer -> InflationLayer
AMCL map -> odom -> healthy EKF odom -> base_footprint -> footprint
Planner Global Costmap -> NavfnPlanner -> ComputePathToPose -> /plan + action result
```

`phase10_planner_smoke.launch.py` scoped-include 既有
`phase10_localization_smoke.launch.py`，然后仅启动官方 `nav2_planner/planner_server` 与它的
Lifecycle Manager。`nav2_costmaps.yaml` 中的 `global_costmap` 保持 Task 2 contract：`map`、
`base_footprint`、0.05 m、Static/Obstacle/Inflation、原 8 点 footprint、1/1 Hz。

`nav2_planner.yaml` 的唯一 planner plugin 是 `GridBased`：

```yaml
plugin: nav2_navfn_planner::NavfnPlanner
tolerance: 0.0
use_astar: false
allow_unknown: false
```

`tolerance=0.0` 和 `allow_unknown=false` 使占用目标验证不能被目标附近替代 cell 或 unknown
space 掩盖；这不是 planner 算法比较或性能调参。

## 可复现验收

启动（可选 RViz 使用 `phase10_planner.rviz`，其仅显示 Map、Scan、Global Costmap、footprint
与 `/plan`）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/resilient_nav_phase10_task3_logs \
  ros2 launch resilient_nav_navigation phase10_planner_smoke.launch.py use_rviz:=false
```

另一个已 source 的终端运行只读 probe：

```bash
ros2 run resilient_nav_navigation phase10_planner_probe \
  --timeout-sec 60 \
  --result-path /tmp/resilient_nav_phase10_task3_planner_result.json
```

probe 先验证 Planner lifecycle active、唯一 `/global_costmap`、冻结 map/costmap metadata、
Task 2 footprint、`map -> base_footprint`、scan，再读取实际 planner 参数。它只发送
`ComputePathToPose` action request，且在每轮前后检查机器人 pose；对成功结果同时验证：

1. action 成功、path 与每个 pose 均在 `map` frame；
2. `/plan` 收到 map-frame path；
3. Nav2 `/is_path_valid` 返回 true；
4. 把每段路径按 0.025 m 加密采样，published Global Costmap 中 lethal sample 为零。

Task 3.2 开始前还把此 probe 扩展为 raw Costmap full-footprint gate：同一非对称 padded footprint
按最多半 cell 的平移与外顶点最多半 cell 的旋转间距扫过全 Path，`254` lethal、`255` unknown 或
越界均拒绝。`simple_reachable` 的 85 个 sweep pose 与 `static_obstacle_detour` 的 810 个 sweep
pose 均通过；详见 `PHASE10_TASK3_CONTROLLER_SMOKE.md`。这不改变 Task 3.1 的 Planner-only action
及“机器人不动”结论。

三个冻结场景来自 `config/planner_smoke_scenarios.yaml`：

| 场景 | 目标 / 预期 | 实测 |
| --- | --- | --- |
| `simple_reachable` | `(1.0, 0.0)`，成功 | 38 poses，长度 `0.9960 m`，77 samples、0 lethal、`IsPathValid=true` |
| `static_obstacle_detour` | `(4.0, 4.0)`，成功且绕障 | 直线有 37 lethal samples；返回 240 poses、`6.0399 m`，363 samples、0 lethal、`IsPathValid=true` |
| `occupied_goal` | `(5.05, 2.15)`，冻结中央矩形障碍的已观测边界 | action accepted 后 `ABORTED`，`GOAL_OCCUPIED=206`，goal cell cost `100`，无 path |

central obstacle 的几何中心在 occupancy map 中不是保证 lethal 的 fill：保存图记录的是被 LiDAR
观测到的表面。因此失败场景固定使用经 Global Costmap 动态确认的左边界 cell `(5.05, 2.15)`，
不是把 collision interior 误当作 map obstacle。

首轮结果中起止位姿 delta 为 `0.000000 m` / `0.00000274 rad`；完整 fresh-process restart
结果为 `0.000000 m` / `0.000000 rad`。两轮均通过同一三场景，因此 action 计算没有触发机器人
运动，也不依赖上一轮 Planner 状态。

## 自动检查与边界

`test/test_planner_resources.py` 静态冻结 Navfn-only 参数、三种场景、scoped launch、无第二张
Costmap、无控制/BT/导航执行符号、RViz 只读显示和 probe 无 publisher/`cmd_vel`。
本轮相关静态测试为 29 passed；`colcon build --packages-select resilient_nav_navigation
--symlink-install` 通过。

本任务不验证 Controller、轨迹跟踪、速度限制、行为树、recovery、可达性以外的全局算法比较、
长期性能或 fault-aware navigation。后续任何让路径变成运动的工作必须单独授权并先建立
Controller 与 Safety Gate 验收。
