# Phase 10 Task 3.2：Controller Server + FollowPath

记录日期：2026-08-28

## 范围与结论

Task 3.2 已在健康 saved-map 链上通过官方 Nav2 Jazzy 1.3.12 的
`controller_server` 与
`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`（RPP）动态验收。
它只接受已由 `ComputePathToPose` 生成的固定 `FollowPath` action；没有启动 BT Navigator、
`NavigateToPose`、recovery、周期重规划、动态障碍测试或 benchmark。

`phase10_controller_smoke.launch.py` scoped-include Task 3.1 planner wrapper。因此正式链中只有
Planner Server 拥有一个 Global Costmap，Controller Server 拥有一个 Local Costmap；Task 2 standalone
Costmap smoke 保留为独立回归入口。

```text
frozen Phase 9 map -> Map Server -> Planner Global Costmap -> Navfn -> Path
healthy /scan -> Global + Controller Local ObstacleLayer -> InflationLayer
AMCL map -> odom -> healthy EKF odom -> base_footprint
Path -> FollowPath(RPP) -> /cmd_vel Twist -> Gazebo bridge -> differential robot
```

TF ownership 不变：AMCL 是 `map -> odom` 唯一 owner，healthy EKF 是
`odom -> base_footprint` 唯一 owner，robot_state_publisher 管理其后的 robot links。

## Full-footprint 预检

Navfn 的 path 表达 robot origin，不保证非对称八边形 footprint 的逐姿态安全。新增的纯 Python
`path_safety.py` 对 `/global_costmap/costmap_raw` 做独立 gate：使用同一份 collision-derived
八点 footprint 和 `0.01 m` padding，平移间距不超过半 cell（`0.025 m`），转角间距令最外顶点
移动不超过半 cell。每个姿态以 SAT 检查完整凸 footprint 和 raw cell 的交叠；`254` lethal、`255`
unknown 和越界都拒绝，`253` inscribed 只记录。

在启动 Controller 前，fresh Planner-only run 的结果为：

| 场景 | Path | full-footprint sweep | 结论 |
| --- | --- | --- | --- |
| `simple_reachable` | 38 poses，0.996 m | 85 poses，lethal/unknown 皆 0 | PASS |
| `static_obstacle_detour` | 240 poses，6.040 m | 810 poses，lethal/unknown 皆 0 | PASS |

这不是 Planner 的替代算法，也没有修改 Navfn；它是复用的项目侧验收门。其合成栅格单测还覆盖：
origin centerline 安全但完整 footprint 撞 lethal、终端转向撞到非对称尾部、unknown 与越界拒绝。

## Controller 合同

`config/nav2_controller.yaml` 只定义 Controller Server 与 RPP；Local Costmap 仍直接复用
`config/nav2_costmaps.yaml` 的 Task 2 contract（`odom`、rolling `6x6 m`、scan-only
ObstacleLayer + InflationLayer、同一 footprint）。关键运行期参数实测为：

- `controller_plugins=[FollowPath]`，RPP plugin 为官方 Jazzy 类；
- `odom_topic=/odometry/filtered`；
- `enable_stamped_cmd_vel=false`，与 Gazebo bridge 的 `geometry_msgs/msg/Twist` 合同一致；
- `publish_zero_velocity=true`；20 Hz Controller；
- `PoseProgressChecker`（0.05 m / 0.10 rad / 10 s）与 stateful `SimpleGoalChecker`
  （0.10 m / 0.15 rad）；
- RPP 以 `desired_linear_vel=0.20 m/s`、fixed 0.40 m lookahead、
  `use_collision_detection=true`、`allow_reversing=false` 运行。

probe 只允许 YAML 中的 `simple_reachable` 和 `static_obstacle_detour`。它先从 Planner 取得 Path，
再次完成 full-footprint gate，随后显式指定 `FollowPath`、`goal_checker`、`progress_checker` 发送
action。运行中只观察 `/cmd_vel`、feedback、map-frame trajectory、`/received_global_plan`、
`/lookahead_collision_arc` 与 Local raw Costmap。速度若超过固定 smoke bound（线速度 `0.22 m/s`、
角速度 `0.75 rad/s`）或运行期 footprint 检查失败，probe cancel action；无论结果如何仅额外发布
5 条全零 Twist 作为退出安全停，绝不自行发布非零速度。

Costmap/footprint 是 lifecycle 后的状态快照，probe 对这些四个 topic 请求 reliable
transient-local QoS，以便晚于 lifecycle activation 启动时仍取得当前 snapshot；scan、action 和
velocity 仍按各自运行期 QoS 观察。这是 probe 的证据可靠性修正，不改变 Nav2 Costmap 参数或数据面。

## 动态证据

两条路径均从独立 fresh process 启动，初始 AMCL pose 为 `(0,0,0)`：

| 场景 | action / 控制证据 | 跟踪与终点 | 碰撞证据 | 结论 |
| --- | --- | --- | --- | --- |
| `simple_reachable` | 164 commands、165 feedback；最大 `0.20 m/s` / `0.376 rad/s` | cross-track `0.0154 m`；XY `0.0973 m`、yaw `0.1406 rad` | 85 preflight + 30 runtime sweeps 均 zero lethal/unknown；collision arc 6 samples 均安全 | PASS |
| `static_obstacle_detour` | 805 commands、806 feedback；最大 `0.20 m/s` / `0.600 rad/s` | cross-track `0.0532 m`；XY `0.1139 m`、yaw `0.1270 rad` | 810 preflight + 147 runtime sweeps 均 zero lethal/unknown；collision arc 9 samples 均安全 | PASS |

两轮均为 `FollowPath.Result.NONE=0`，并观察到 Controller completion 的全零速度输出，随后 probe
额外写入 5 条全零 stop。此证据证明 Path → Controller → `/cmd_vel` → Gazebo robot 的健康闭环，
不代表动态障碍、recovery、长期性能或 autonomous navigation 已验证。

## 复现与自动检查

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/resilient_nav_lab/ros2_ws/install/setup.bash
ros2 launch resilient_nav_navigation phase10_controller_smoke.launch.py use_rviz:=false

# 在另一个终端；每个场景均应从 fresh launch 开始。
ros2 run resilient_nav_navigation phase10_controller_probe \
  --scenario simple_reachable \
  --result-path /tmp/phase10_task3_controller_simple.json
```

对 detour 将 `--scenario` 改为 `static_obstacle_detour`。`use_rviz:=true` 使用只读
`phase10_controller.rviz` 显示 Map、Scan、两张 Costmap、Path、RPP received plan/collision arc、
footprint、robot 与 TF；没有 SetGoal tool。

静态测试覆盖 RPP/`Twist` 参数合同、受限场景、costmap reuse、无 BT/第二 Costmap、probe 的
full-footprint/collision-arc/zero-only stop 边界和只读 RViz。完整包的 build/test 结果另见本次
Task 3.2 收口记录。

## 未覆盖边界

未测试周期 replan、动态障碍行为、主动碰撞拒绝分支、recovery、BT、NavigateToPose、速度平滑、
长距离性能、fault-aware navigation 或 Agent 控制。Ground Truth 不进入任何 Nav2 runtime 或 probe
输入。
