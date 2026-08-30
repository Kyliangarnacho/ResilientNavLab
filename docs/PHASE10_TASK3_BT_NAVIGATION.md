# Phase 10 Task 3.3：BT Navigator + NavigateToPose

Task 3.3 在健康 saved-map 基线上把 Task 3.1 的 Navfn Planner 和 Task 3.2 的 RPP Controller
交给官方 Nav2 Jazzy 1.3.12 `bt_navigator` 编排。正式入口为
`phase10_bt_navigation_smoke.launch.py`；它不创建新的 Costmap：Planner 独占 Global Costmap，
Controller 独占 Local Costmap。

BT 使用本机上游 `navigate_w_replanning_time.xml`（SHA-256
`1e5c47087a7f728e46e7bb25b00784477d83962e2094e9d5c3e94f4b0c3fb782`）：
`PipelineSequence -> RateController(1 Hz) -> ComputePathToPose -> FollowPath`。
它不包含 Recovery、Behavior Server、Clear Costmap、Spin、Wait 或 BackUp。`bt_loop_duration=10 ms`
只控制 tick，不是重规划频率；`expected_planner_frequency=20 Hz` 仍只是 Planner 执行时间合同。

新 probe 只向 `/navigate_to_pose` 发送固定 `simple_reachable` 或
`static_obstacle_detour` goal，且保持空 `behavior_tree` 以强制使用服务端冻结 XML；它不调用
`ComputePathToPose` 或 `FollowPath`。它记录 action feedback/result、`/behavior_tree_log`、`/plan`、
`/received_global_plan`、`/cmd_vel`、TF、最终误差和停车。每个观察到的 Global Path 都在正式 BT
之外用既有 full-footprint raw-Costmap sweep 验收；这不是运行时 BT 节点。

三个导航节点由单一 ordered Lifecycle Manager 按 Planner、Controller、BT Navigator 管理。旧的
Task 3.1/3.2 wrapper 保持默认 lifecycle manager；只在 Task 3.3 include 时显式关闭内部 manager。
RViz 配置增加官方 `nav2_rviz_plugins/GoalTool`，仅作人工健康 goal 测试。

probe 的运行期 path safety evidence 使用 `NavigateToPose` feedback 的 `current_pose`（frame、
数值和 timestamp 均验证），把每条 `/plan` 与最近的 feedback 以 ROS stamp 优先、同一 executor
接收 monotonic time 兜底的 0.50 s 窗口关联。任何 1.0 s 后仍没有关联 feedback 的 Path 均判失败，
不会静默跳过 sweep。这样不再因 probe 自己晚启动的短 TF buffer 产生 past-extrapolation 后误取消
健康 action。TF 仍用于 readiness 和 action 后的独立 `map -> base_footprint` / feedback 交叉检查；
probe 保留 60 s TF history，最终差异上限为 `0.10 m` / `0.10 rad`。开始命令前还要求恰好一个、
单调的 `/clock` publisher；若 Gazebo 残留或未发布时钟，则拒绝验收而不发送目标。

动态验收的 machine-readable JSON 保存在 `/tmp/phase10_task33_*.json`。Task 3.3 两个独立 fresh
process 均 PASS：

- `static_obstacle_detour`：`NavigateToPose.Result.NONE=0`、17 次 Path/`ComputePathToPose`
  SUCCESS、564 条 feedback、0 recovery、149 条 cmd；所有观察到的 Path 均通过 full-footprint
  sweep（zero lethal/unknown）。终点 feedback XY/yaw 误差为 `0.0647 m` / `0.1390 rad`，最终
  TF/feedback 差为 `0.0250 m` / `0.0176 rad`，并观察到停车。
- fresh restart `simple_reachable`：`Result.NONE=0`、9 次 Path/`ComputePathToPose` SUCCESS、
  825 条 feedback、0 recovery、174 条 cmd；9 次 sweep 均 zero lethal/unknown，终点 XY/yaw
  误差 `0.0967 m` / `0.1435 rad`，最终 TF/feedback 差 `0.000002 m` / `0.0126 rad`，并观察到停车。

Task 3.1/3.2 的独立 fresh 回归也通过：Planner 的可达/绕障 Path 均通过 full-footprint sweep、
占用 goal 以 `GOAL_OCCUPIED=206` 拒绝且机器人位移为零；RPP `FollowPath` simple/detour 均完成，
最终 detour 的 XY/yaw 误差 `0.0915 m` / `0.1516 rad`、6316 个轨迹样本和 146 次运行期 footprint
sweep 均无 lethal/unknown，且 Controller 输出零速停车。一次随后重启的 Gazebo 无 `/clock`，导致
Planner 永不 active；该实例在发送 action 前被时钟检查拒绝，清理精确残留进程后 fresh 重跑通过。

未覆盖动态障碍、Recovery、Behavior Server、fault-aware navigation、Agent、benchmark 或长期性能。
