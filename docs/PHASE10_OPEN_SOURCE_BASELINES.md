# Phase 10 开源项目基线（健康导航技术雷达）

记录日期：2026-08-26

## 1. 目的与授权边界

本文是 Phase 10 Milestone 0 的外部项目决策快照。它只定义后续 healthy navigation baseline 应借鉴的职责边界、运行时候选和许可证风险；不是安装清单、实现授权、性能结论或控制授权。

本 Milestone 不安装 Nav2 或其他依赖，不创建导航包，不修改 ROS 代码，不启动规划、控制、恢复或机器人状态变更。Phase 9 已完成的健康二维 LiDAR SLAM 继续提供既有 `/map`、`map -> odom` 与健康 EKF 的 `odom -> base_footprint` ownership；Ground Truth、`FaultStatus`、scenario/seed、fault model truth、`parameters_yaml` 和 `/faulted/*` 命名不得进入任何未来导航 runtime。Robot Agent 仍为离线、只读 Diagnosis，不能成为 Planner、Recovery 或控制入口。

> **2026-08-28 implementation update.** 上段是 2026-08-26 Milestone 0 的授权前快照。
> 后续已按独立 Task 1–3.2 授权采用官方 Jazzy Nav2 1.3.12 binary：Map Server、AMCL、
> Costmap、`planner_server`/Navfn 的 `ComputePathToPose`，以及 `controller_server`/RPP 的固定
> `FollowPath` 已完成健康基线验收。后续 Task 3.3 已采用官方 `bt_navigator` 的
> `NavigateToPose` 和上游无 Recovery 的固定周期重规划 XML，完成两个 frozen 健康场景的验收；
> 仍不含 Behavior Server、Recovery、动态障碍、fault-aware navigation 或 Agent navigation。

> **2026-08-30 closure update.** Phase 10 后续按 Task 5 授权继续 Borrow
> 官方 `ros_gz` SpawnEntity/DeleteEntity、Nav2 官方 recovery XML、Behavior Server 和
> Clear Costmap / Spin / Wait / BackUp behaviours；这些只服务于冻结的 Gazebo 环境
> robustness benchmark。Task 5.1–5.4 已收口，且没有复制 Nav2/ros_gz 源码、没有将
> GT 回流导航，也没有引入 fault-aware navigation 或 Agent control。历史表中对
> Behavior Server/Recovery/dynamic obstacle 的 Reject/defer 仅是 Milestone 0 快照，
> 不再代表当前实现状态。

**Borrow** 表示仅在后续明确授权时直接采用上游已发布 runtime，而不复制源码；**Adapt** 表示独立重述其架构/验收思想到本项目 schema、TF 与安全合同；**Reject** 表示不引入其代码、运行时或依赖。任何实际引入前必须重新核验精确版本、Jazzy 兼容性、传递依赖和许可证。

## 2. 决策总表

| 项目 | Borrow / Adapt / Reject 决策 | Phase 10 runtime 决策 | 许可证决策 |
| --- | --- | --- | --- |
| [Navigation2](https://github.com/ros-navigation/navigation2) | **Borrow（按 Task 1–3.3 已授权）**官方 Jazzy binary 的 Map Server、AMCL、Costmap、Planner Server/Navfn、Controller Server/RPP 与 BT Navigator/`NavigateToPose`；**Adapt** lifecycle、planner/controller/costmap 的显式 ownership、full-footprint gate 与可观测性；**Reject/defer** Behavior Server、Recovery、docking、源码复制、动态障碍与 fault-aware autonomous execution。 | 健康 2D navigation 的唯一 runtime；当前运行 saved-map localization、Costmap 和固定场景的 `NavigateToPose`（BT 以 1 Hz 重规划）。非零 `/cmd_vel` 只由 Controller 在两条 YAML 白名单健康目标下输出，probe 有速度上限、取消、时钟唯一性与零停证据；BT 不创建新的 Costmap。 | 上游仓库为混合许可，必须逐 package/file 核验；未特别标注文件默认为 Apache-2.0，不能把该默认扩展为全仓库或传递依赖。 |
| [Apollo](https://github.com/ApolloAuto/apollo) | **Adapt** routing、planning、control 与安全监督的分层，以及 scenario/evaluation 与 runtime 分离；**Reject** Apollo/Cyber RT、HD map、车辆模型、planner/controller、配置和源码。 | 无 runtime 候选；不引入车辆级栈或 ROS bridge。 | Apache-2.0；只保留独立表述的架构启发，未来复制或引入时仍须核验 notices 与依赖。 |
| [Autoware Universe](https://github.com/autowarefoundation/autoware_universe) | **Adapt** component diagnostics、map/planning/control 分层和可机器读取的状态报告；**Reject** 全栈、自动驾驶消息、地图/感知/规划/控制模块、launch 与源码。 | 无 runtime 候选；不把差速机器人伪装为道路车辆，也不接入其大型依赖图。 | Apache-2.0；未来若采用任一子包，须重新核验其 package 与传递依赖许可证。 |
| [Grid Map](https://github.com/ANYbotics/grid_map) | **Adapt** 多层地图中“层名、来源、时间与语义必须显式”的数据合同；**Reject** 其 C++/ROS runtime、`GridMap` 消息、PCL/OpenCV/terrain/elevation/PointCloud2 管线。 | 无 runtime 候选；Phase 9 的二维 `OccupancyGrid` 和 Slam Toolbox 地图保持不变。 | BSD-3-Clause；未复制实现，未来若采用须保留通知并复核所有依赖。 |
| [ros_motion_planning](https://github.com/ai-winter/ros_motion_planning) | **Adapt** 仅限算法比较维度（global path、local tracking、约束和指标）；**Reject** 其 ROS 1/Noetic 代码、plugins、controllers、配置、仿真资源与任何源码移植。 | 无 runtime 候选；不作为 Jazzy 导航方案，也不提供 `/cmd_vel`。 | GPL-3.0；禁止复制、移植、fork 或派生其代码/配置。任何未来集成须先取得独立 GPL 合规审查与项目授权。 |

许可证信息用于工程风险识别，不构成法律意见。

## 3. 运行时与安全边界

若后续单独授权 healthy Nav2 baseline，运行时必须保持以下约束：

1. Phase 9 mapping runtime 中 Slam Toolbox 是 `map -> odom` 与 `/map` 的唯一 owner；Phase 10 saved-map localization runtime 中 AMCL 是 `map -> odom` 的唯一 owner，Map Server 是 `/map` 的唯一 owner，healthy EKF 始终是 `odom -> base_footprint` 的唯一 owner；不得新增重复 TF publisher。
2. 导航输入只可使用合法的地图、TF、健康传感器观测和明确 schema；benchmark/evaluator 真值永不回流到 navigation、costmap、planner、controller、缓存、日志或 Agent。
3. Planner、controller、recovery 和发布 `/cmd_vel` 是不同授权面。Milestone 0 没有任何执行授权；未来动作必须通过显式白名单 Tool 与独立 Safety Gate，不能由 Nav2、Agent 或通用 ROS 接口绕过。
4. 评价应分别报告定位/地图前提、全局路径、局部控制、碰撞/停止、安全降级和恢复；不能以到达目标替代各层正确性，也不能把 simulator truth 当 runtime input。

## 4. 后续门槛（非实现授权）

任何 Phase 10 实现提案至少应说明：精确的 Nav2/Jazzy 发布版本和依赖来源、最小 bringup 范围、TF/topic ownership、是否与已完成 Phase 9 map 兼容、`/cmd_vel` 的 Safety Gate、正常/地图缺失/TF 失效/传感器退化/停止的可回放验收，以及所有新增组件的许可证与 notice 复核。本文件不构成这些实现的授权。
