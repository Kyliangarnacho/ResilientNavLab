# Phase 9 架构：健康二维 SLAM baseline

记录日期：2026-08-20

## 1. 状态、目标与非目标

本文冻结 Phase 9 的健康二维 SLAM baseline 架构。它约束已集成的 Jazzy Slam Toolbox `online_async` mapping、pose-graph persistence/reload、localization 和 evaluator-only Ground Truth overlay 的数据流、TF ownership 与真值隔离；不改变既有健康参考定位、Phase 8 adaptive 对照链或 Robot Agent 边界。

本文是运行架构合同，而非性能或产品化结论。已完成的集成不复制或修改 Slam Toolbox 源码，也不授权 Nav2、规划、恢复、控制、真实硬件定位、fault-aware SLAM 或 TF owner 切换。

## 2. 冻结的数据链

```text
/wheel/odometry + /imu/data
        │
        ▼
Healthy EKF ── /odometry/filtered
        │  (only TF publisher)
        ▼
odom ──► base_footprint ──► base_link ──► lidar_link
               ▲                 robot_state_publisher fixed TF chain
               │
/scan (sensor_msgs/msg/LaserScan; frame resolves to lidar_link)
        │
        ▼
Slam Toolbox online_async (Borrow runtime)
        │
        ├── /map (nav_msgs/msg/OccupancyGrid)
        └── map ──► odom (only map -> odom TF publisher)
```

运行时合同如下：

1. Slam Toolbox 只消费健康 `/scan`，其 `header.frame_id` 必须通过固定 lidar TF 链和健康 `odom -> base_footprint` 在 scan 时间戳解析。它不订阅 `/odometry/filtered` 作为普通 topic；其 odometry 先验来自 TF lookup。
2. Healthy EKF 继续从健康 wheel/IMU 输入计算并独占 `odom -> base_footprint`。`/odometry/faulted` 和 `/odometry/adaptive` 均继续 `publish_tf=false`，不能成为此 baseline 的 odom TF 来源。
3. `robot_state_publisher` 继续提供从 `base_footprint`、`base_link` 到 `lidar_link` 的固定机器人描述链。它不发布 `map -> odom`，也不提供估计结果。
4. Slam Toolbox 独占 `map -> odom` 并发布 `/map`。`map -> odom` 可以反映 SLAM 的全局校正；它不得覆盖、重发布或改写 `odom -> base_footprint`。

因此完整可查询树为 `map -> odom -> base_footprint -> base_link -> lidar_link`。一个 frame 的同一边只能有一个发布者；任何未来 Launch 都必须先停用冲突的 `map -> odom` 发布者，而不能通过静态 TF、第二个 EKF 或重映射掩盖冲突。

## 3. TF ownership 表

| Transform | 唯一 owner | 输入/职责 | 禁止的 owner 或行为 |
| --- | --- | --- | --- |
| `odom -> base_footprint` | Healthy EKF | 健康 wheel/IMU 定位参考；继续发布 `/odometry/filtered`。 | Slam Toolbox、adaptive EKF、faulted EKF、`odom_tf_broadcaster`（在此完整链中）均不得并发发布。 |
| `base_footprint -> base_link -> lidar_link` | `robot_state_publisher` | Xacro 固定关节与传感器外参。 | EKF、Slam Toolbox、手写 static transform publisher 不得重复发布。 |
| `map -> odom` | Slam Toolbox `online_async` | 由 2D SLAM 把地图坐标与健康 odometry 连起来。 | Healthy/faulted/adaptive EKF、Gazebo pose、AMCL、Nav2、手写 static TF 或任何 evaluator 不得发布。 |

`/map` 的唯一 owner 同样是 Slam Toolbox；它是 `nav_msgs/msg/OccupancyGrid`，用于地图消费/可视化，不是 Ground Truth，也不得反馈为 EKF、Health Monitor 或 Agent 输入。

## 4. Slam Toolbox ROS 2 `online_async` 接口参考

本节以本机已核验的 Jazzy Slam Toolbox `online_async_launch.py` 与 `mapper_params_online_async.yaml` 接口为准。项目仅保守派生自有 YAML/launch 集成层，不复制 Slam Toolbox 源码；精确运行结果和保存产物见 Phase 9 的 mapping、persistence、localization 与 evaluation 技术文档。

| 接口 | 上游名称 | 本 baseline 的冻结语义 |
| --- | --- | --- |
| Launch argument | `autostart` | 未来 Launch 可使用其 lifecycle 自动 configure/activate 语义；本 Milestone 不创建 Launch。 |
| Launch argument | `use_lifecycle_manager` | 生命周期管理选择仅属于未来 Slam Toolbox Launch；不接入 Robot Agent。 |
| Launch argument | `use_sim_time` | 未来仿真运行必须与既有 `/clock` 语义一致；不引入第二时钟。 |
| Launch argument | `slam_params_file` | 上游用于加载 ROS 2 参数文件；项目 mapping launch 用它加载经审查的本包 YAML。 |
| Node parameter | `odom_frame=odom`、`map_frame=map`、`base_frame=base_footprint` | 固定 frame 语义，分别对应上节的 TF tree。 |
| Node parameter | `scan_topic=/scan` | 只接入健康二维 `LaserScan`；不接入 `/faulted/*`、PointCloud2、RGB-D 或匿名化 Phase 8 fusion input。 |
| Node parameter | `mode=mapping` | 仅为未来健康建图 baseline 的候选模式接口；未在本 Milestone 运行或验证。 |
| Node parameter | `transform_publish_period` | 这是上游发布 `map -> odom` 的控制接口；当前 baseline 采用经 smoke 验证的非零值以保持 Slam Toolbox 的唯一 owner 地位。 |
| Node parameter | `map_update_interval`、`resolution` | `/map` 的更新/栅格接口；未来独立实验再选择数值和评价方式。 |
| Node parameter | `restamp_tf`、`transform_timeout`、`tf_buffer_duration` | 时间戳和 TF 查询行为接口；必须与仿真时间和 `/scan` 时间戳一并验收，不在本文调参。 |

上游 `online_async` 是 lifecycle node；未来验收必须记录其 configure/activate 状态、TF 可查询性和 `/map` 发布，而不是仅以进程存在判定成功。上游 README 将 `scan_queue_size` 说明为 async 的单扫描队列约束，但本节引用的上游 `mapper_params_online_async.yaml` 未显式给出该字段；本项目不会据此手写参数，而将在实际 Jazzy 版本核验时确认其可用性和最终值。

## 5. 当前 estimator 的禁止输入

下列对象均不进入当前健康二维 SLAM estimator（Slam Toolbox node、其参数、TF buffer 输入选择、缓存、日志或 `/map` 生成路径）：

| 对象 | 合法位置 | 禁止原因 |
| --- | --- | --- |
| Ground Truth（Gazebo pose、`/evaluation/*`） | evaluator/benchmark only | 不能为 SLAM 提供地图、位姿初值、回环答案、参数或评分反馈。 |
| `FaultStatus`、`/fault_injection/status`、scenario/seed/model/parameters truth | 故障注入与 evaluator only | 是实验真值；不得触发 SLAM 模式、TF 行为或任何参数分支。 |
| `SensorHealth` | Health Monitor、Phase 8 独立 adaptive 对照链、未来经授权的策略层 | Milestone 2 的 healthy SLAM baseline 不以健康状态 gate、重配或修改 Slam Toolbox；不得把检测当作估计器真值。 |
| Robot Agent / `agent-core` | 当前仅离线、只读诊断 | 不订阅 Live ROS、不可调用 Slam Toolbox、不可读取/写入参数、不可发布 TF、地图或控制。 |
| `/faulted/*`、`/fusion/input/*`、`/odometry/faulted`、`/odometry/adaptive` | 既有故障或 Phase 8 对照链 | 不属于健康 SLAM 输入；尤其不得把 adaptive/faulted 输出伪装成 healthy odometry TF。 |

无法证明输入及其 metadata 不含真值、故障分支名称或 benchmark answer 时，未来 SLAM adapter/Launch 必须 fail closed，拒绝连接该输入。

## 6. 后续验收门槛（非实现授权）

未来单独实现任务至少应验证：

1. `/scan` 类型、时间戳、`frame_id` 与 `lidar_link` 外参完整，且能够在同一时间戳查询 `odom -> base_footprint -> base_link -> lidar_link`。
2. Healthy EKF 是 `odom -> base_footprint` 的唯一 TF owner；Slam Toolbox 是 `map -> odom` 和 `/map` 的唯一 owner；没有静态或重复发布者。
3. `online_async` 仅以经核验的 Jazzy 参数接口启动并完成 lifecycle configure/activate；`map_frame`、`odom_frame`、`base_frame`、`scan_topic`、时间接口与本合同一致。
4. Ground Truth、FaultStatus、SensorHealth、Robot Agent、`/faulted/*`、`/fusion/input/*` 和 evaluator 输出均不在 SLAM node 的订阅、参数、缓存、日志或发布路径中。
5. 正常、无 scan、TF 缺失/过期、时钟暂停与恢复必须分别记录；这些是接口完整性验收，不等同于地图精度、回环、重定位、容错导航或真实硬件性能结论。
