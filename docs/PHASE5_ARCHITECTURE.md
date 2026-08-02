# 阶段 5 故障注入架构设计

> 状态更新：本文是 2026-08-02 的阶段 5 开工前架构设计稿。阶段 5 已在 2026-08-03 完成可复现实验闭环，最终状态、证据和未完成内容以 `docs/PHASE5_SUMMARY.md` 为准。

## 1. 范围与边界

- 记录日期：2026-08-02
- 当前状态：架构设计；未实现节点、消息包、Launch、YAML 或测试
- 基线来源：阶段 4 的 IMU、二维 Lidar、RGB-D 和 `/wheel/odometry` + `/imu/data` EKF

阶段 5 的目标是建立可控、可复现的传感器故障注入链路、故障真值标签和实验记录边界。本文只定义接口和职责，不实现 ROS 2 包，不改变阶段 4 话题、frame、EKF 配置或 TF 发布行为。

明确不在本文范围内：

- 不修改阶段 4 核心代码或现有 Launch 默认行为。
- 不安装软件，不创建功能包，不启动 Gazebo、RViz 或 ROS 图。
- 不实现健康评估、异常检测、自适应融合、Nav2、SLAM 或容错策略。
- 不把本文设计描述为已经完成或验证的能力。

## 2. 阶段 4 保持不变的接口

阶段 4 完整 EKF 链路保持为当前基线：

```text
/wheel/odometry  nav_msgs/msg/Odometry   frame_id=odom, child_frame_id=base_footprint
/imu/data         sensor_msgs/msg/Imu     frame_id=imu_link
    -> robot_localization ekf_node
/odometry/filtered nav_msgs/msg/Odometry  frame_id=odom, child_frame_id=base_footprint
    -> odom -> base_footprint TF
```

阶段 4 已有传感器接口保持原名、原类型和原 frame：

| Topic | 类型 | 消息 frame |
| --- | --- | --- |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | `odom` / `base_footprint` |
| `/imu/data` | `sensor_msgs/msg/Imu` | `imu_link` |
| `/scan` | `sensor_msgs/msg/LaserScan` | `lidar_link` |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `camera_optical_frame` |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_optical_frame` |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | `camera_optical_frame` |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_optical_frame` |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | `odom` / `base_footprint` |

阶段 5 注入节点只能订阅这些基线 topic 并发布新的 `/faulted/*` 传感器数据 topic。它不得重映射、覆盖或停止阶段 4 的原始输入和健康 EKF 输出。Faulted EKF 的对照输出不采用 `/faulted/*` 规则，固定为 `/odometry/faulted`。

## 3. 包职责与依赖方向

计划新增两个包，但本文不创建它们：

| 包 | 类型 | 职责 | 不负责 |
| --- | --- | --- | --- |
| `resilient_nav_interfaces` | ROS 2 interface package | 定义阶段 5 及后续阶段共享的消息、常量和接口语义，首个消息为 `FaultStatus.msg` | 不包含节点、故障模型、算法或 Launch |
| `resilient_nav_fault_injection` | ROS 2 runtime package | 加载场景 YAML，订阅基线输入 topic，按仿真时间和固定随机种子生成 `/faulted/*` 传感器数据，并发布 `FaultStatus` 真值标签 | 不实现健康评估、自适应融合、EKF 或导航决策 |

依赖方向必须单向：

```text
resilient_nav_fault_injection
  -> resilient_nav_interfaces
  -> builtin_interfaces, std_msgs

resilient_nav_localization
  -> robot_localization
```

阶段 5 不要求 `resilient_nav_localization` 依赖 `resilient_nav_fault_injection`。如果后续为 faulted EKF 增加单独配置或 Launch，定位包仍应只依赖 `robot_localization` 和自身配置所需的常规 ROS 2 依赖；除非节点实际订阅或发布 `FaultStatus`，否则不应增加对 `resilient_nav_interfaces` 或 `resilient_nav_fault_injection` 的构建依赖。

## 4. Topic 接口

### 4.0 健康基线 topic

本文中的“健康”只表示未注入故障的 clean/reference 输入与估计输出，不表示健康评估算法或检测结果。阶段 5 不新增 `/health/*` 在线评估 topic。

| 健康基线 topic | 类型 | 阶段 5 用途 |
| --- | --- | --- |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | wheel odometry 注入源、健康参考 EKF 输入 |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU 注入源、健康参考 EKF 输入 |
| `/scan` | `sensor_msgs/msg/LaserScan` | LaserScan 注入源、感知故障对照 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 健康参考 EKF 输出 |

### 4.1 `/faulted/*` 传感器数据 topic

注入节点按“原始传感器输入 topic 去掉前导斜杠后接到 `/faulted/` 下”的规则发布故障传感器数据：

| 原始输入 | Faulted 输出 | 类型 | frame_id 规则 |
| --- | --- | --- | --- |
| `/wheel/odometry` | `/faulted/wheel/odometry` | `nav_msgs/msg/Odometry` | 保持 `odom` / `base_footprint` |
| `/imu/data` | `/faulted/imu/data` | `sensor_msgs/msg/Imu` | 保持 `imu_link` |
| `/scan` | `/faulted/scan` | `sensor_msgs/msg/LaserScan` | 保持 `lidar_link` |

首批故障模型只覆盖 IMU、wheel odometry 和 LaserScan。RGB-D 图像接口在阶段 5 设计中保留原始基线，不进入首批 `/faulted/*` 输出，避免同时引入高带宽图像故障、同步和视觉指标。

除非故障模型明确模拟延迟或乱序，faulted 消息必须保留原始 `header.stamp`。延迟模型只改变发布时间，不改写 stamp；乱序模型发布较早 stamp 的缓存消息。frame_id 不得被故障模型篡改，否则会把传感器数据故障和 TF 外参故障混在一起。

Faulted EKF 输出是定位对照结果，不属于 `/faulted/*` 传感器数据命名规则；其 topic 固定为 `/odometry/faulted`，类型为 `nav_msgs/msg/Odometry`，消息 frame 保持 `frame_id=odom`、`child_frame_id=base_footprint`。

### 4.2 故障真值与状态 topic

计划由故障注入包发布：

| Topic | 类型 | 语义 |
| --- | --- | --- |
| `/fault_status` | `resilient_nav_interfaces/msg/FaultStatus` | 所有当前激活、即将激活或刚结束的故障真值标签 |
| `/faulted/clock_window` | `resilient_nav_interfaces/msg/FaultStatus` | 可选调试输出；只在需要逐事件回放时发布同一消息类型 |

`/fault_status` 是真值标签，不是健康评估输出。后续阶段 6 的健康评估可以订阅 `/faulted/*` 和 `/fault_status` 做离线评分，但在线检测算法不得把 `/fault_status` 当作输入特征。

## 5. `FaultStatus.msg` 设计

计划消息定义如下：

```text
std_msgs/Header header
string scenario_id
uint32 scenario_seed
string event_id
string source_topic
string faulted_topic
string sensor
string model
builtin_interfaces/Time start_time
builtin_interfaces/Time end_time
uint8 state
float32 severity
string[] affected_fields
string parameters_yaml

uint8 SCHEDULED=0
uint8 ACTIVE=1
uint8 ENDED=2
uint8 CANCELLED=3
```

字段语义：

| 字段 | 语义 |
| --- | --- |
| `header.stamp` | 发布该状态的仿真时间；`header.frame_id` 为空字符串 |
| `scenario_id` | 场景 YAML 中的稳定 ID，用于 rosbag 和指标归档 |
| `scenario_seed` | 场景固定随机种子；同一场景、同一输入 bag 应生成相同故障序列 |
| `event_id` | 单个故障事件的稳定 ID，必须在场景内唯一 |
| `source_topic` | 被注入的原始 topic，例如 `/imu/data` |
| `faulted_topic` | 对应输出 topic，例如 `/faulted/imu/data` |
| `sensor` | 逻辑传感器名：`imu`、`wheel_odometry` 或 `laser_scan` |
| `model` | 故障模型名，例如 `bias_step`、`dropout`、`range_saturation` |
| `start_time` / `end_time` | 事件在仿真时间中的闭开窗口 `[start_time, end_time)` |
| `state` | 当前事件状态；枚举值见消息常量 |
| `severity` | 归一化严重度，范围 `[0.0, 1.0]`；具体换算由模型文档定义 |
| `affected_fields` | 受影响字段路径，例如 `angular_velocity.z`、`twist.twist.linear.x`、`ranges` |
| `parameters_yaml` | 该事件实际生效参数的 YAML 字符串，用于记录随机采样后的具体值 |

`FaultStatus` 只表达故障真值和注入配置，不表达检测置信度、健康评分、报警等级或恢复决策。

## 6. 场景 YAML 结构

场景文件应完全由仿真时间驱动，并显式记录随机种子。建议结构：

```yaml
schema_version: 1
scenario_id: imu_yaw_bias_001
description: IMU yaw-rate bias during a short spin segment
random_seed: 424242
time_base: sim_time

inputs:
  wheel_odometry:
    source_topic: /wheel/odometry
    faulted_topic: /faulted/wheel/odometry
  imu:
    source_topic: /imu/data
    faulted_topic: /faulted/imu/data
  laser_scan:
    source_topic: /scan
    faulted_topic: /faulted/scan

defaults:
  pass_through_before_first_message: true
  publish_fault_status_hz: 10.0
  preserve_header_stamp: true

events:
  - event_id: imu_bias_01
    sensor: imu
    model: bias_step
    window:
      start: 12.0
      duration: 8.0
    affected_fields:
      - angular_velocity.z
    parameters:
      bias_mean: 0.08
      bias_stddev: 0.0
      units: rad/s
```

时间窗口规则：

- `time_base` 必须为 `sim_time`，窗口单位为秒。
- 故障注入节点和任何阶段 5 对照 EKF 都必须设置 `use_sim_time=true`，并以 ROS 2 `/clock` 推进窗口和状态发布时间。
- 每个事件必须指定 `window.start` 和且仅指定 `window.duration` 或 `window.end`。
- 窗口采用 `[start, end)` 语义，`stamp == end` 的消息不再注入该事件。
- `window.start`、`window.end` 和 `duration` 必须非负，且 `end > start`。
- 事件可以重叠，但同一 sensor 同一字段的重叠事件必须按 YAML 顺序串联应用，并在 `FaultStatus.parameters_yaml` 中记录最终实际参数。
- 固定 `random_seed` 是必填项，并应映射到 `FaultStatus.scenario_seed`；所有随机采样必须来自该场景 seed 派生出的事件级随机流，不能使用系统时间、墙钟到达顺序或未固定的全局随机源。

## 7. EKF 对照链路与 TF 发布权

阶段 5 需要区分“干净参考估计”和“故障输入估计”，但不改变阶段 4 接口。

| EKF | 输入 | 输出 | TF 发布权 |
| --- | --- | --- | --- |
| 健康参考 EKF | `/wheel/odometry`、`/imu/data` | `/odometry/filtered` | 沿用阶段 4；在完整阶段 4 链中唯一发布 `odom -> base_footprint` |
| Faulted EKF | `/faulted/wheel/odometry`、`/faulted/imu/data` | `/odometry/faulted` | 必须设置 `publish_tf=false`，不得发布 `odom -> base_footprint` |

原因：

- `/odometry/filtered` 和 `odom -> base_footprint` 已由阶段 4 定义为当前干净基线的融合结果。
- `/odometry/faulted` 是实验对照输出，不应在首批架构中驱动 RobotModel 或导航 TF。
- 如果后续任务明确要求“用故障输入驱动导航”，应在单独 Launch 中显式选择唯一 TF owner，并记录该运行不再是干净参考链。

Faulted EKF 的 frame 仍使用 `world_frame=odom`、`odom_frame=odom`、`base_link_frame=base_footprint`，输出消息的 frame 与阶段 4 一致；区别只在 topic 命名和 TF 开关。健康参考 EKF 可按阶段 4 发布 `odom -> base_footprint`；Faulted EKF 必须显式 `publish_tf=false`，不能成为该 TF 的备用发布者。

## 8. 首批故障模型

### 8.1 IMU：`sensor_msgs/msg/Imu`

首批覆盖字段为 `angular_velocity.z`，可选扩展到 `angular_velocity.x/y` 和 `linear_acceleration.x/y/z`，但不修改 `orientation`。

| 模型 | 作用 | 关键参数 | 指标重点 |
| --- | --- | --- | --- |
| `bias_step` | 在窗口内给字段加固定偏置 | `bias_mean`、`bias_stddev`、`units` | yaw drift、faulted EKF yaw error |
| `bias_ramp` | 偏置随仿真时间线性增长 | `start_bias`、`end_bias` | 漂移斜率、累计定位误差 |
| `noise_scale` | 放大原有噪声或叠加零均值噪声 | `stddev`、`scale` | 输出方差、EKF 平滑程度 |
| `dropout` | 按概率丢弃消息，不发布 faulted 样本 | `drop_probability` | 消息间隔、EKF 超时行为 |
| `freeze` | 重复窗口开始前最后一条消息 | `hold_last_sample` | stamp/值冻结识别、yaw rate 残差 |

IMU orientation covariance 当前为 Gazebo 原生零矩阵，阶段 5 不通过故障模型伪造 orientation 或 covariance。

### 8.2 Wheel odometry：`nav_msgs/msg/Odometry`

首批覆盖 `twist.twist.linear.x` 和 `twist.twist.angular.z`。不改写 `header.frame_id=odom` 或 `child_frame_id=base_footprint`。

| 模型 | 作用 | 关键参数 | 指标重点 |
| --- | --- | --- | --- |
| `scale` | 模拟轮径或编码比例错误 | `linear_scale`、`angular_scale` | 直行距离误差、旋转角误差 |
| `bias_step` | 给速度加常值偏置 | `linear_bias`、`angular_bias` | filtered 速度偏差、最终 pose 误差 |
| `freeze` | 重复最后一条 odometry | `hold_last_sample` | 停滞窗口、EKF 预测漂移 |
| `dropout` | 随机或连续丢弃 wheel odometry | `drop_probability`、`burst_length` | 输入缺测率、输出连续性 |
| `delay` | 缓存后延迟发布 | `delay_seconds` | 时序滞后、轨迹相位差 |

如果同时改 pose 和 twist，必须在模型说明中保持二者物理一致；首批建议只改 EKF 实际使用的 twist 字段，避免制造与阶段 4 无关的重复信息冲突。

### 8.3 LaserScan：`sensor_msgs/msg/LaserScan`

首批覆盖 `ranges` 和可选 `intensities`，不修改 `angle_min`、`angle_increment`、`range_min`、`range_max` 或 `frame_id=lidar_link`。

| 模型 | 作用 | 关键参数 | 指标重点 |
| --- | --- | --- | --- |
| `range_bias` | 对有效 range 加固定偏置并裁剪到量程 | `bias` | 平均距离误差、障碍物边界偏移 |
| `noise_scale` | 增加有效 range 的随机噪声 | `stddev`、`seed_stream` | range RMSE、异常点比例 |
| `dropout_beams` | 把部分 beam 置为 `inf` 或 `nan` | `beam_probability`、`mode` | 有效 beam 比例、连续缺口长度 |
| `sector_blind` | 固定角度扇区失明 | `angle_min`、`angle_max`、`replacement` | 扇区覆盖率、最近障碍丢失 |
| `range_saturation` | 把超阈值或随机 beam 推到 `range_max` | `threshold`、`probability` | 饱和比例、最近障碍误差 |

LaserScan 首批不参与阶段 4 EKF，但必须记录，因为它是后续健康评估和导航感知故障的主要观测。

## 9. Rosbag 记录范围

阶段 5 每次故障实验应至少记录：

```text
/clock
/cmd_vel
/joint_states
/tf
/tf_static
/wheel/odometry
/imu/data
/scan
/odometry/filtered
/faulted/wheel/odometry
/faulted/imu/data
/faulted/scan
/odometry/faulted
/fault_status
```

可选记录四个 RGB-D topic，用于保留完整阶段 4 上下文；默认不纳入首批指标，避免 bag 体积和图像吞吐掩盖故障注入验证。

记录文件命名建议包含 `scenario_id`、`random_seed`、日期和运行序号，例如：

```text
bags/phase5/imu_yaw_bias_001_seed424242_20260802_run01/
```

bag 元数据旁应保存原始场景 YAML、展开后的实际事件参数和阶段 4/5 Launch 参数。

## 10. 验证指标

### 10.1 通用指标

| 指标 | 语义 |
| --- | --- |
| 注入窗口命中率 | 在 `[start, end)` 内被修改、丢弃或延迟的样本比例 |
| 窗口外透明性 | 窗口外 faulted topic 与原始 topic 的一致性 |
| 固定 seed 复现性 | 同一输入和同一 seed 下 faulted 数据、`FaultStatus` 和指标一致 |
| 时间戳一致性 | 非延迟/乱序模型保持原始 stamp；延迟模型只改变到达时间 |
| frame 一致性 | faulted 消息保持阶段 4 frame 语义 |
| 真值标签完整性 | 每个事件有 SCHEDULED、ACTIVE、ENDED 或 CANCELLED 状态记录 |

### 10.2 各类故障指标

| 传感器 | 模型类别 | 验证指标 |
| --- | --- | --- |
| IMU | bias / ramp | `angular_velocity.z` 均值偏移、yaw 积分误差、`/odometry/faulted` yaw 与健康参考 yaw 差 |
| IMU | noise | 窗口内标准差倍率、异常样本比例、filtered yaw 抖动 |
| IMU | dropout / freeze | 消息间隔分布、冻结值持续时间、EKF 输出连续性 |
| Wheel odometry | scale / bias | 直行 x 误差、旋转 yaw 误差、速度均值偏差 |
| Wheel odometry | dropout / delay / freeze | 输入缺测率、延迟秒数误差、filtered pose 最大相邻跳变 |
| LaserScan | range bias / noise | 有效 beam 的 MAE/RMSE、最近障碍距离误差 |
| LaserScan | dropout / sector blind / saturation | 有效 beam 比例、最长连续缺口、扇区覆盖率、最近有效距离变化 |

首批验证只评价故障注入是否按配置生效以及对固定 EKF 对照输出的影响，不评价检测延迟、误报率、健康评分或导航任务成功率。

## 11. 一致性检查清单

本文设计采用以下一致性约束：

- 原始阶段 4 topic 不改名：`/wheel/odometry`、`/imu/data`、`/scan`、`/odometry/filtered` 保持不变。
- 健康参考 EKF 输出固定为 `/odometry/filtered`；Faulted EKF 输出固定为 `/odometry/faulted`。
- Faulted 传感器数据统一放在 `/faulted/*`，消息类型与对应原始传感器 topic 相同。
- `FaultStatus` 位于 `resilient_nav_interfaces/msg/FaultStatus`，由 `resilient_nav_fault_injection` 发布。
- `odom`、`base_footprint`、`imu_link`、`lidar_link` 和 `camera_optical_frame` 继续沿用阶段 4 frame 语义。
- 只有健康参考 EKF 发布 `odom -> base_footprint`；Faulted EKF 必须 `publish_tf=false`，避免与阶段 4 EKF 的 TF 冲突。
- 依赖方向从运行包指向接口包，不让接口包依赖任何运行逻辑。
- `/fault_status` 是故障真值，不是健康评估结果。
- 所有阶段 5 窗口调度、`FaultStatus.header.stamp` 和对照 EKF 都使用仿真时间；固定 `random_seed` 不得被系统时间或墙钟顺序替代。

## 12. 发现的冲突与未解决问题

已发现并在设计中规避的冲突：

- 阶段 4 健康 EKF 已独占 `odom -> base_footprint`；因此 faulted EKF 必须设置 `publish_tf=false`，不能发布同一 TF。
- 阶段 3 默认仍使用 `/odom` 和 `odom_tf_broadcaster`；阶段 5 必须基于阶段 4 完整链的 `/wheel/odometry`，不能把阶段 3 `/odom` 当作故障注入输入。
- LaserScan 不参与阶段 4 EKF；它的故障指标只能覆盖感知数据质量，不能直接解释 `/odometry/filtered`。

未解决问题，留待实现任务明确：

- `FaultStatus.parameters_yaml` 的 YAML 子集和转义规则需要在接口实现时用测试固定。
- 同一字段多事件重叠时是否允许非交换模型组合，需要在首个运行节点实现前进一步收紧。
- Faulted EKF 的 Launch 命名、参数文件位置和是否运行健康参考 EKF 并行对照，需在实现任务中授权。
- 是否将 RGB-D 图像故障纳入阶段 5 后续批次，需单独评估 bag 体积、同步和指标成本。
