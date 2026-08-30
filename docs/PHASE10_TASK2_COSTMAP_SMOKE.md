# Phase 10 Task 2.1–2.4：Footprint、Global/Local Costmap 与参数验收

记录日期：2026-08-27

## 1. 范围与结论

Phase 10 Task 2 已完成健康 saved-map 定位链上的二维 footprint 合同冻结、`map` frame
Global Costmap、`odom` frame Local Rolling Costmap，以及 inflation 参数实验和联合验收。实现只使用官方 Jazzy
`nav2_costmap_2d` 的 StaticLayer、ObstacleLayer 和 InflationLayer；没有复制地图
融合、ray tracing 或 inflation 实现。

本任务不创建 planner、controller、behavior tree、recovery server、导航目标或自主
执行链。因此它证明的是静态地图与局部实时障碍可分别进入两张 Costmap，而不是自主导航能力。

## 2. Footprint 合同

`base_footprint -> base_link` 的既有静态平移为 `(-0.10, 0, 0.15) m`。根据当前
Xacro 的车体 box collision、左右轮 cylinder collision 与 caster collision 在地面
的投影，采用逆时针的物理 footprint：

```text
[[-0.35, -0.175], [-0.10, -0.215], [0.10, -0.215], [0.15, -0.175],
 [0.15,  0.175], [ 0.10,  0.215], [-0.10,  0.215], [-0.35,  0.175]]
```

`footprint_padding=0.01 m` 是 Costmap runtime 的安全外扩；它不改写 URDF collision。
轮子是 y 方向最宽边界（`±0.215 m`），车体则决定前后范围（`x=-0.35..0.15 m`）。
最终配置在 `resilient_nav_navigation/config/nav2_costmaps.yaml`，并由静态测试同时
比对 Xacro contract、参数、launch 和 RViz 资源。

## 3. 运行接口与复用关系

`phase10_global_costmap_smoke.launch.py` 以受作用域隔离的 Include 启动既有
`phase10_localization_smoke.launch.py`（Gazebo healthy baseline、sensors、healthy EKF、
Map Server、AMCL、显式 initial pose），且对子 launch 固定 `use_rviz=false`。外层
`use_rviz` 只控制新的 `phase10_global_costmap.rviz`，不会重现此前的 LaunchConfiguration
污染问题。

随后 launch 在命名空间 `/global_costmap` 启动官方 `nav2_costmap_2d` executable 和
官方 Lifecycle Manager。全局代价地图节点名为 `/global_costmap/global_costmap`，关键
数据链如下：

```text
Phase 9 frozen occupancy map -> Map Server -> /map -> StaticLayer
healthy LiDAR -> /scan -> ObstacleLayer (marking + clearing)
AMCL map -> odom -> healthy EKF odom -> base_footprint -> footprint
StaticLayer + ObstacleLayer + InflationLayer -> /global_costmap/costmap
```

参数固定 `global_frame=map`、`robot_base_frame=base_footprint`、`resolution=0.05`、
`rolling_window=false`、`update/publish=1/1 Hz`、`map_subscribe_transient_local=true`。本机 `nav2_costmap_2d`
为官方 Jazzy `1.3.12-1noble.20260615.154707`，前缀 `/opt/ros/jazzy`；实际 plugin
class 名为 `nav2_costmap_2d::StaticLayer`、`nav2_costmap_2d::ObstacleLayer`、
`nav2_costmap_2d::InflationLayer`。

独立 executable 会接受 lifecycle transition，但本机运行时没有建立 Lifecycle Manager
所期待的 managed-node bond。因此 manager 设为 `bond_timeout: 0.0`，避免把已 active
的 Costmap 误报为 bond failure；最终 readiness 仍直接查询
`/global_costmap/global_costmap/get_state`，必须为 `active`，而非仅凭 node 存在判定。

Global、Local 与联合入口均使用标准 Lifecycle Manager `autostart=true`。此前为排查
rotation ghost 引入的 startup gate 已删除：最终 endpoint 证据证明根因是 LiDAR 固定 FOV
边界 beam，而不是启动顺序。standalone executable 的 `bond_timeout=0.0` 仍独立保留；旋转
ghost 的最终证据和最小 source 修复见
[`PHASE10_COSTMAP_ROTATION_GHOST_REPAIR.md`](PHASE10_COSTMAP_ROTATION_GHOST_REPAIR.md)。

`/scan` 运行时 publisher 与 Costmap/AMCL subscriber 均为 Best Effort、volatile，输入
QoS 相容。`phase10_global_costmap_probe` 是只读订阅/TF/lifecycle probe，不发布 ROS
消息、不写参数、不发送控制命令。

## 4. 动态验收证据

隔离 fresh-process run 使用 Phase 9 唯一 occupancy map，启动后 Lifecycle Manager
报告 managed node active。probe 得到：

| 检查 | 结果 |
| --- | --- |
| lifecycle | `/global_costmap/global_costmap` 为 `active` |
| map/costmap metadata | `frame_id=map`，resolution `0.05 m`，`227x226`，origin `(-2.22269738693, -2.16105234952)`；与冻结 Phase 9 map 一致 |
| footprint | 8 个点及 `0.01 m` padding 与 Xacro-derived contract 一致 |
| 图层输出 | full costmap 有 `34598` free、`15764` inflated、`940` lethal cells |
| TF/scan | `map -> base_footprint` 可用，scan frame 为 `lidar_link` |
| scan-map sanity | 38 个有限 scan endpoint：到 frozen static occupied cell 的 median `0.014594 m`，P90 `0.026839 m`，低于 `0.10/0.20 m` sanity 阈值 |

为独立验证 ObstacleLayer marking、clearing 和 InflationLayer，临时在 Gazebo 世界
`(-2.0, -3.5)` 创建一个 `0.3 x 0.3 x 1.0 m` 静态 box，位于机器人正前方、未占用的
地图区域。动态观测为：

| 阶段 | `(map x=1.5, y=0.0)` lethal cell | `(map x=1.9, y=0.0)` inflation cell |
| --- | ---: | ---: |
| 临时障碍前 | `0` | `0` |
| scan 观测到障碍后 | `99` | `35` |
| 删除障碍并等待 clearing 后 | `0` | `0` |

临时模型在验收结束时已通过 Gazebo remove service 删除；创建时必须通过
`ros_gz_sim create -x -2.0 -y -3.5 -z 0.5` 显式传入 pose：该工具会覆盖 SDF 内 model
pose，省略 flags 会把模型置于原点而使这项动态证据失效。Phase 9 map、world asset 和
ROS package 资产均未被修改。带有该临时非地图障碍时，probe 明确跳过 scan-map static
sanity，删除后再以完整 sanity 通过，避免把预期动态障碍误判为静态地图/定位失配。

## 5. Local Rolling Costmap（Task 2.3）

`phase10_local_costmap_smoke.launch.py` 以受作用域隔离的 Include 复用 Task 1
localization smoke，并在独立 `/local_costmap` namespace 启动官方 standalone
`nav2_costmap_2d` 和 Lifecycle Manager。它不加载 StaticLayer：冻结 `/map` 只属于
Global Costmap；Local Costmap 是 `odom` 下的 scan-only rolling world model，避免把
AMCL/map 的定位修正混入实时局部窗口。

```text
healthy LiDAR -> /scan -> Local ObstacleLayer (marking + clearing)
healthy EKF odom -> base_footprint -> Local footprint
ObstacleLayer + InflationLayer -> /local_costmap/costmap (rolling, odom)
```

固定 contract 为 `global_frame=odom`、`rolling_window=true`、`width=height=6 m`、
`resolution=0.05 m`（`120x120` cells）、`update/publish=5/2 Hz`、`track_unknown_space=false`，
并复用第 2 节的 8 点 footprint、`0.01 m` padding、`/scan` 的 `2.5/3.0 m`
obstacle/raytrace range。它只加载 `ObstacleLayer` 与 `InflationLayer`。

动态 probe 直接确认 `/local_costmap/local_costmap` 为 active、scan frame 为
`lidar_link`、published footprint 在 `odom`、参数与上述 contract 一致。4 秒有界直行中，
机器人平移 `0.4171 m`、窗口 origin 平移 `0.4000 m`，最大窗口中心误差 `0.0599 m`、
origin-delta error `0.0171 m`。联合 launch 的重复直行确认 `0.2400 m` robot motion、
`0.0201 m` origin-delta error。滚动判定的固定 `0.125 m` tolerance 由 5 Hz 更新、2 Hz
发布、受限 smoke 速度和 `0.05 m` cell quantization 的上界导出，不依据实验后调阈值。

同一可删除 box 在 Local Costmap 的 `(odom x=1.5, y=0.0)` watch cell 使值从 `0` 变为
`99`，删除并等待 clearing 后回到 `0`。这直接验证 Local ObstacleLayer marking、
ray-traced clearing 和 InflationLayer 输入，而不改写 frozen map。

## 6. Inflation 参数实验与联合验收（Task 2.4）

实验在 fresh process 中以 launch-time 参数覆盖运行，绝不使用 `ros2 param set`。三个
profile、ROI、阈值和方向性判定在运行前冻结于
`config/costmap_experiment_profiles.yaml`；probe 导出以世界坐标索引的 `1.0 m` ROI，
`phase10_costmap_experiment_evaluator` 只比较 before/marked/cleared 的共同 cells。

| profile | inflation radius | cost scaling | 受控障碍结果 |
| --- | ---: | ---: | --- |
| baseline | 0.55 m | 3.0 | extent `0.4000 m`，inflated area `1.0900 m²`，annulus median `38`，clearing ratio `1.0` |
| wider | 0.75 m | 3.0 | extent `0.6021 m`，inflated area `2.0400 m²`，clearing ratio `1.0` |
| steeper | 0.55 m | 8.0 | extent `0.4000 m`，annulus median `8`，inflated cost sum `9744`，clearing ratio `1.0` |

机器可读 evaluator PASS：wider 相对 baseline 的 extent 增加 `0.2021 m`、area 增加
`0.9500 m²`；steeper 的 extent difference 为 `0`，固定 annulus median cost 下降 `30`，
inflated cost sum 下降 `12740`。这证明 radius 改变空间范围，scaling 在同一 radius 下改变
cost 衰减，而不是仅凭 RViz 视觉判断。

保留的紧凑机器可读结论见
[`PHASE10_TASK2_COSTMAP_EXPERIMENT.json`](PHASE10_TASK2_COSTMAP_EXPERIMENT.json) 和
[`PHASE10_TASK2_COSTMAP_ACCEPTANCE.json`](PHASE10_TASK2_COSTMAP_ACCEPTANCE.json)；逐 cell
ROI snapshot 是本轮临时实验输出，不纳入仓库。

`phase10_costmaps_smoke.launch.py` 组合既有 Global wrapper 与 Local Costmap，不启动
任何 navigation server。fresh startup 和一次 complete restart 均由 joint probe PASS：两
个 lifecycle 都为 active；Global 仍是 frozen `map`、`227x226`、origin
`(-2.22269738693,-2.16105234952)`，Local 仍是 `odom`、`120x120`、`6 m` rolling window；
TF 链唯一为 `map -> odom -> base_footprint`。因此 Global StaticLayer 没有随局部运动滚动，
Local Window 也没有错误消费 frozen map。

在有可用显示的主机，可将 local 或 joint launch 的 `use_rviz:=true`，使用对应 RViz
文件人工确认 Map、Scan、RobotModel、两张 costmap 和 footprints 的空间关系；本次
headless 验收不把该人工视觉检查伪称为已完成。

## 7. 复现与静态回归

在已 source 的 Jazzy 与 workspace install 环境运行：

```bash
export ROS_LOG_DIR=/tmp/resilient_nav_phase10_ros_logs
ros2 launch resilient_nav_navigation phase10_global_costmap_smoke.launch.py \
  gz_args:='-s' use_rviz:=false
ros2 run resilient_nav_navigation phase10_global_costmap_probe \
  --result-path /tmp/phase10_global_costmap_probe.json \
  --watch-x 1.5 --watch-y 0.0
```

Local-only 与 joint 入口分别为：

```bash
ros2 launch resilient_nav_navigation phase10_local_costmap_smoke.launch.py use_rviz:=false
ros2 launch resilient_nav_navigation phase10_costmaps_smoke.launch.py use_rviz:=false
ros2 run resilient_nav_navigation phase10_local_costmap_probe \
  --result-path /tmp/phase10_local_costmap_probe.json
ros2 run resilient_nav_navigation phase10_costmap_joint_probe \
  --result-path /tmp/phase10_costmap_joint_probe.json
```

在有可用显示的主机，可将 `use_rviz:=true`，用 `phase10_global_costmap.rviz` 人工查看
Map、Scan、RobotModel、TF、published footprint 和 `/global_costmap/costmap` 是否空间一致。
本次受限 headless run 没有把该人工视觉检查伪称为已完成。

完成代码后的静态回归为：

- `python3 -m pytest src/resilient_nav_navigation/test -q`：24 passed；
- `colcon build --symlink-install --packages-select resilient_nav_navigation`：通过；
- `colcon test --packages-select resilient_nav_description resilient_nav_navigation`：615 tests、0 errors、0 failures、1 skipped。

## 8. 已知边界

- Ctrl-C teardown 时既有 `system_heartbeat` 会重复调用 `rclpy.shutdown()`；这是 Phase 4
  已知无关技术债。
- 本机 standalone `nav2_costmap_2d` 在 Ctrl-C teardown 曾以 `-11` 退出，但 active-state、
  layer data、动态 marking/inflation/clearing 和 fresh restart 都在 teardown 前已验证。
  本任务未修改上游 binary；后续若把该 standalone 模式用于长期运行，应单独最小复现并
  评估该退出行为。
- rotation ghost 的最终根因是旧 `-135°` FOV 边界的 `beam_index=0` 异常短距返回。该 ray
  已由 samples `640→639` 与向内移动 `min_angle` 排除；CW/CCW 修复后均为零 marking-range
  候选，用户确认无 persistent ghost。此前 startup gate、15/5 Hz hardening 与专用 runtime
  diagnostic 已从正式运行链删除。
- 本任务不消除 cell-level scan/map 差异；上述 headless quantitative sanity 仅确认没有
  明显双墙、整体平移或旋转错位。若后续出现超阈值残差，应依次检查 frozen map、AMCL
  `map -> odom`、EKF `odom -> base_footprint`、lidar TF/timing 与 scan QoS。
