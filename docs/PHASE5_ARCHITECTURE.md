# 阶段 5 多传感器故障注入架构

## 1. 文档状态与范围

- 设计日期：2026-08-02
- 可信起点：`cb3b9a7142129a4517180b254a11b0f88c65452a`
- 当前 ROS 2：Jazzy
- 当前仿真：Gazebo Harmonic
- 当前定位基线：`/wheel/odometry` 与 `/imu/data` 输入 `robot_localization`
- 本文状态：阶段 5 架构设计，不代表故障注入代码已经实现或动态验收通过

阶段 5 的目标是保留阶段 4 健康数据链，同时增加可配置、可复现、带故障真值标签的独立故障数据链。阶段结束时应能够用场景 YAML 启动健康与故障对照实验，记录原始数据、故障数据、故障真值和两个 EKF 输出，并通过固定随机种子复现实验。

本阶段只覆盖：

- IMU、轮式里程计和二维 Lidar 的故障注入；
- 故障场景配置与仿真时间窗口；
- 故障真值状态消息；
- 健康 EKF 与 faulted EKF 对照；
- 定量检查、rosbag 记录和回放。

本阶段不覆盖：

- 故障自动检测、健康评分或故障分类；
- 自适应协方差、自适应融合或传感器剔除；
- SLAM、Nav2、MoveIt 或容错路径规划；
- RGB-D 故障模型、PointCloud2 或实物传感器驱动。

`/fault_injection/status` 是注入器掌握的故障真值，不是健康评估算法的输出。后续检测算法不得把该话题作为输入。

## 2. 总体数据链

```text
Gazebo DiffDrive / IMU / GPU Lidar
  -> ros_gz_bridge
  -> 健康原始话题
       ├── /wheel/odometry ─┐
       ├── /imu/data ───────┼-> 健康 EKF
       │                    │     ├-> /odometry/filtered
       │                    │     └-> odom -> base_footprint TF
       │                    │
       ├── /wheel/odometry -> wheel_fault_injector
       │                         └-> /faulted/wheel/odometry ─┐
       ├── /imu/data -> imu_fault_injector                    ├-> faulted EKF
       │                  └-> /faulted/imu/data ──────────────┘     └-> /odometry/faulted
       └── /scan -> scan_fault_injector
                         └-> /faulted/scan

三个 injector
  -> /fault_injection/status
       ├-> 终端与 RViz 对照
       ├-> fault_probe 定量检查
       └-> rosbag 记录与回放
```

健康原始话题必须保持阶段 4 的名称、消息类型、frame 语义和发布来源。injector 只能订阅它们并发布新的 `/faulted/*` 话题，不能重映射、覆盖或替代健康话题。

## 3. 软件包职责与依赖方向

阶段 5 计划新增两个 ROS 2 软件包。

### 3.1 `resilient_nav_interfaces`

建议类型：`ament_cmake` 接口包。

职责：

- 定义 `msg/FaultStatus.msg`；
- 导出消息生成所需依赖；
- 不包含节点、Launch、场景、故障算法或实验逻辑。

该包应保持稳定和轻量，供故障注入、后续健康评估和实验工具共同依赖。

### 3.2 `resilient_nav_fault_injection`

建议类型：`ament_python`。

职责：

- 纯故障模型和确定性随机数逻辑；
- IMU、wheel odometry、LaserScan 三个 injector 节点；
- 场景 YAML、统一 Launch 和阶段 5 RViz；
- faulted EKF 的启动与话题重映射；
- `fault_probe`、rosbag 辅助工具和自动测试；
- 实验输出目录与元数据约定。

推荐依赖方向：

```text
resilient_nav_fault_injection
  ├-> resilient_nav_interfaces
  ├-> resilient_nav_localization
  ├-> resilient_nav_simulation
  └-> 标准 ROS 2 消息与运行库
```

`resilient_nav_localization` 不反向依赖故障注入包。阶段 4 的包和启动入口保持独立可运行，避免阶段 5 破坏已验收基线。

## 4. Topic 与节点接口

### 4.1 健康与故障话题

| 数据 | 健康输入 | 故障输出 | 消息类型 |
| --- | --- | --- | --- |
| IMU | `/imu/data` | `/faulted/imu/data` | `sensor_msgs/msg/Imu` |
| 轮式里程计 | `/wheel/odometry` | `/faulted/wheel/odometry` | `nav_msgs/msg/Odometry` |
| 二维 Lidar | `/scan` | `/faulted/scan` | `sensor_msgs/msg/LaserScan` |
| 健康融合 | `/odometry/filtered` | 不适用 | `nav_msgs/msg/Odometry` |
| 故障融合 | 不适用 | `/odometry/faulted` | `nav_msgs/msg/Odometry` |
| 故障真值 | 不适用 | `/fault_injection/status` | `resilient_nav_interfaces/msg/FaultStatus` |

### 4.2 建议节点名

```text
/imu_fault_injector
/wheel_fault_injector
/scan_fault_injector
/faulted_ekf_filter_node
```

节点名、输入话题、输出话题和场景文件均应通过参数或 Launch 参数配置，但默认值使用上表中的稳定接口。

### 4.3 QoS 原则

- faulted 传感器话题应与对应健康源使用兼容的 sensor-data QoS，避免 injector 引入新的可靠性语义；
- `/fault_injection/status` 使用 reliable QoS；
- 状态话题建议使用 transient-local，以便晚加入的检查工具立即获得最近状态；
- injector 不得通过 QoS 缓冲伪造额外延迟，固定延迟只能由显式故障模型产生。

## 5. `FaultStatus` 消息契约

文件：

```text
resilient_nav_interfaces/msg/FaultStatus.msg
```

字段：

```text
builtin_interfaces/Time stamp
string scenario_id
string injector_name
string source_topic
string output_topic
string fault_type
bool active
float64 severity
uint32 seed
uint64 input_count
uint64 output_count
string details
```

字段语义：

| 字段 | 语义 |
| --- | --- |
| `stamp` | 状态发布时的 ROS 仿真时间，不使用墙钟时间 |
| `scenario_id` | 当前场景的稳定标识，用于关联 YAML、rosbag 和实验目录 |
| `injector_name` | 产生该状态的 injector 节点名 |
| `source_topic` | 健康输入话题 |
| `output_topic` | 独立故障输出话题 |
| `fault_type` | 当前配置的故障模型；健康旁路使用 `passthrough` |
| `active` | 当前仿真时间是否位于故障生效窗口内 |
| `severity` | 当前激活强度摘要；未激活时为 `0.0` |
| `seed` | 该 injector 实际使用的确定性随机种子 |
| `input_count` | 节点启动后累计收到的输入消息数 |
| `output_count` | 节点启动后累计发布的输出消息数 |
| `details` | UTF-8 紧凑 JSON，记录模型参数、窗口和必要运行状态 |

`severity` 只用于同一种 `fault_type` 内快速比较，不是跨故障类型统一的健康分数。模型的完整参数仍以场景 YAML 和 `details` 为准。

状态发布时机：

- 节点获得有效配置并进入运行状态时发布一次；
- `active` 状态切换时立即发布；
- 运行中按固定仿真时间周期发布，建议默认 `1.0 s`；
- 输入、输出计数变化时不要求逐消息发布，避免状态话题放大负载。

三个 injector 共用 `/fault_injection/status`，依靠 `injector_name`、`source_topic` 和 `output_topic` 区分来源。

## 6. 场景 YAML 结构

场景文件建议存放于：

```text
resilient_nav_fault_injection/config/scenarios/
```

每个场景至少包含：

- `scenario_id` 和说明；
- 全局 `seed`；
- 仿真时间窗口；
- 三个 injector 的输入、输出和故障类型；
- 模型参数；
- 期望验证指标。

建议结构：

```yaml
scenario:
  scenario_id: imu_yaw_bias_demo
  description: Inject +0.15 rad/s yaw-rate bias into IMU
  seed: 20260802
  time_reference: simulation_time
  status_period_sec: 1.0

injectors:
  imu:
    enabled: true
    source_topic: /imu/data
    output_topic: /faulted/imu/data
    seed_offset: 101
    fault:
      type: imu_yaw_rate_bias
      start_delay_sec: 5.0
      duration_sec: 10.0
      severity: 0.15
      parameters:
        bias_rad_s: 0.15

  wheel:
    enabled: true
    source_topic: /wheel/odometry
    output_topic: /faulted/wheel/odometry
    seed_offset: 202
    fault:
      type: passthrough
      start_delay_sec: 0.0
      duration_sec: 0.0
      severity: 0.0
      parameters: {}

  scan:
    enabled: true
    source_topic: /scan
    output_topic: /faulted/scan
    seed_offset: 303
    fault:
      type: passthrough
      start_delay_sec: 0.0
      duration_sec: 0.0
      severity: 0.0
      parameters: {}

expected_metrics:
  imu_yaw_rate_mean_delta_rad_s: 0.15
  healthy_topics_unchanged: true
  faulted_ekf_tf_publishers: 0
```

### 6.1 时间基准

阶段 5 场景使用 Gazebo 仿真时间。统一启动入口会从仿真时间接近零的世界启动完整链路，因此故障窗口按消息时间戳判断：

```text
active when:
start_delay_sec <= message_stamp_sec < start_delay_sec + duration_sec
```

`duration_sec <= 0` 对 `passthrough` 表示没有故障窗口。暂停 Gazebo 时，故障窗口、延迟队列和状态周期都必须暂停；恢复后继续推进，不能改用墙钟补偿。

### 6.2 随机种子

每个 injector 的有效种子定义为：

```text
effective_seed = (scenario.seed + injector.seed_offset) mod 2^32
```

禁止使用 Python 进程随机化的 `hash()` 派生种子。随机模型只在故障激活窗口内消耗随机数，从而保证相同场景、相同输入序列和相同种子产生相同故障序列。`FaultStatus.seed` 记录有效种子。

### 6.3 配置校验

启动前应拒绝以下配置，而不是静默纠正：

- 缺少 `scenario_id`、`seed`、输入或输出话题；
- 源话题与输出话题相同；
- 不支持的 `fault.type`；
- 负 `start_delay_sec`；
- dropout 概率不在 `[0, 1]`；
- 延迟为负；
- Lidar 扇区角度或单位不明确；
- 某故障缺少必需参数。

配置错误时 injector 应以非零退出，并指出 YAML 路径、字段和非法值。

## 7. 健康 EKF、faulted EKF 与 TF 发布权

### 7.1 健康 EKF

阶段 4 行为保持不变：

```text
/wheel/odometry + /imu/data
  -> robot_localization
  -> /odometry/filtered
  -> odom -> base_footprint TF
```

健康 EKF 保持 `publish_tf=true`，继续作为完整阶段 5 链路中唯一的 `odom -> base_footprint` 动态 TF 发布者。

### 7.2 faulted EKF

```text
/faulted/wheel/odometry + /faulted/imu/data
  -> 第二个 robot_localization ekf_node
  -> /odometry/faulted
```

faulted EKF 必须：

- 复用阶段 4 的二维模式、频率、frame 和融合字段；
- 通过话题重映射使用 faulted 输入；
- 把默认 filtered 输出重映射为 `/odometry/faulted`；
- 设置 `publish_tf=false`；
- 使用独立节点名，避免参数和日志混淆。

优先复用 `resilient_nav_localization/config/ekf.yaml`，并在 Launch 中覆盖输入、输出和 `publish_tf`，避免复制后产生调参漂移。若 ROS 2 参数覆盖限制迫使新增 overlay 文件，overlay 只能包含 faulted 链差异。

两个 EKF 输出都保持：

```text
header.frame_id = odom
child_frame_id = base_footprint
```

但 RViz 中机器人的实际 TF 姿态只由健康 EKF 驱动。`/odometry/faulted` 用 Odometry 或 Path 显示对比，不建立第二棵同名 TF 树。

### 7.3 TF 验收条件

完整阶段 5 启动时：

- `/tf` 中只有健康 EKF 发布 `odom -> base_footprint`；
- faulted EKF 不发布 `/tf`；
- `robot_state_publisher` 继续负责 `base_footprint` 以下机器人树；
- 不新增 `faulted_odom -> base_footprint` 等替代树，避免扩大本阶段范围。

## 8. 故障模型语义

所有模型都遵守：

- 不修改健康输入消息；
- 输出使用深拷贝或新消息对象；
- 默认保留 `frame_id`、`child_frame_id`、协方差和原始时间戳；
- 只有场景明确要求的字段可以变化；
- 非激活窗口执行 passthrough；
- 所有 NaN、Inf 和越界处理必须可测试且有明确状态记录。

### 8.1 健康旁路 `passthrough`

输入消息按原值发布到 faulted 话题。除传输到达时间外，序列化后的有效字段应与输入一致：

```text
input_count == output_count
raw payload == faulted payload
```

该场景用于证明 injector 本身没有改变健康数据语义。

### 8.2 IMU yaw-rate 常值偏置 `imu_yaw_rate_bias`

仅修改：

```text
angular_velocity.z += bias_rad_s
```

保持 orientation、其余角速度、线加速度、协方差、frame 和 header stamp 不变。默认不修改协方差，因为本阶段需要观察“测量已坏但下游仍相信原协方差”的效果。

验证指标：

```text
mean(faulted.angular_velocity.z - raw.angular_velocity.z)
  ≈ bias_rad_s
```

### 8.3 高斯噪声增强 `gaussian_noise`

对场景指定字段叠加：

```text
noise ~ Normal(mean, standard_deviation)
faulted_value = raw_value + noise
```

第一批至少支持 IMU yaw rate；扩展到 wheel 或 scan 时必须显式列出字段和单位。默认不改协方差。随机数来自该 injector 的有效种子。

验证指标：

- 差值均值接近配置均值；
- 差值标准差接近配置标准差；
- 同一输入与 seed 的差值序列完全可复现。

### 8.4 随机丢帧 `random_dropout`

在激活窗口内，对每条输入执行一次确定性 Bernoulli 判断：

```text
drop when random() < probability
```

被丢弃的消息不发布。只在激活窗口内消耗随机数。计数满足：

```text
input_count - output_count = cumulative_dropped_count
```

验证指标：

```text
observed_dropout_ratio
  = dropped_messages / active_window_input_messages
```

固定样本数下不要求与理论概率完全相等，但相同输入和 seed 必须得到相同丢帧位置。

### 8.5 固定仿真时间延迟 `fixed_delay`

激活窗口内的消息进入按释放时间排序的队列：

```text
release_time = original_header_stamp + delay_sec
```

输出时保留原始 `header.stamp`，使下游能够观察真实测量年龄。Gazebo 暂停时队列不释放。为避免时序倒置，故障窗口结束后，新消息不得越过仍在队列中的旧消息。

验证指标：

```text
publish_sim_time - message.header.stamp
  ≈ delay_sec
```

队列必须有显式上限。超限时应报告错误或使用配置指定的丢弃策略，不能静默无限增长。

### 8.6 wheel odometry 数值冻结 `wheel_freeze`

故障激活前保存最近一条健康 wheel odometry。激活期间，每收到一条新输入，就发布一条消息：

- `header.stamp` 使用当前输入消息时间戳；
- `pose.pose` 和 `twist.twist` 保持为冻结样本的数值；
- frame、child frame 和协方差保持对应冻结样本语义；
- 输入与输出频率保持一致。

这样模拟“通信仍活跃、时间戳仍推进，但测量数值卡死”，与随机丢帧和机器人真实停止区分开。若故障开始时尚无基准样本，节点应保持 passthrough 并在状态中报告 `waiting_for_baseline=true`。

验证指标：

- raw pose 或 twist 在运动中变化；
- faulted pose 和 twist 在激活窗口内方差接近零；
- faulted header stamp 持续推进；
- `input_count == output_count`。

### 8.7 Lidar 扇区失明 `lidar_sector_blackout`

扇区角度在 `LaserScan` 自身坐标系中定义。第一批正式场景使用前方：

```text
[-30 deg, +30 deg]
```

对扇区内索引：

```text
ranges[i] = +inf
```

若 `intensities` 长度与 `ranges` 对齐，则对应强度设为 `0.0`。扇区外 `ranges`、角度元数据、range_min、range_max、frame 和 header stamp 保持不变。实现必须正确处理扫描角度跨越 `-pi/pi` 的情况。

验证指标：

- 扇区内所有有效索引为 `+inf`；
- 扇区外数据与 raw 一致；
- raw `/scan` 不受影响。

## 9. 定量验证指标

阶段 5 的 `fault_probe` 应按场景输出机器可读摘要，至少覆盖：

| 模型 | 核心指标 |
| --- | --- |
| passthrough | 字段一致性、输入/输出计数差 |
| bias | faulted 与 raw 的均值差 |
| noise | 差值均值、标准差、固定 seed 序列一致性 |
| dropout | 活跃窗口输入数、输出数、丢帧率和丢帧索引摘要 |
| delay | 仿真发布时间减原始 header stamp 的均值、最大值和抖动 |
| freeze | raw 变化量、faulted 变化量、faulted stamp 单调性 |
| scan blackout | 扇区内失明比例、扇区外一致性 |
| EKF effect | 健康/故障 EKF 的位置差、yaw 差和是否出现非有限值 |

所有比较必须按消息时间戳或可解释的最近邻规则对齐，不能仅比较终端中碰巧相邻的两条输出。

## 10. rosbag 记录范围与实验目录

### 10.1 默认核心记录集

```text
/clock
/cmd_vel
/wheel/odometry
/imu/data
/scan
/faulted/wheel/odometry
/faulted/imu/data
/faulted/scan
/fault_injection/status
/odometry/filtered
/odometry/faulted
/tf
/tf_static
```

RGB-D 图像不是阶段 5 首批故障对象，默认核心 bag 不记录图像，避免无关数据显著增大文件。需要完整视觉上下文时可提供可选 full-sensor 录制配置，但不得改变默认验收集。

### 10.2 实验目录

建议结构：

```text
experiments/
└── <scenario_id>/
    └── <run_id>/
        ├── bag/
        ├── scenario.yaml
        ├── run_metadata.yaml
        └── probe_summary.yaml
```

`run_metadata.yaml` 至少记录：

- `scenario_id`、run ID 和 UTC 启动时间；
- 场景文件副本及 SHA-256；
- Git commit 和工作树是否 clean；
- ROS 2、Gazebo 和关键包版本；
- 实际启动命令；
- 全局 seed 与各 injector 有效 seed；
- bag topic 列表；
- 正常结束、异常退出或用户中断状态。

终端日志不能替代上述元数据，因为日志可能截断、顺序受并发影响、难以机器解析，也不能保证与具体 bag 一一对应。

### 10.3 回放语义

回放时不启动 Gazebo 传感器源和在线 injector，避免同名 topic 出现重复发布者。允许启动：

- RViz；
- `fault_probe` 的离线/回放模式；
- 只读分析工具。

回放验收至少确认 raw、faulted、status 和两个 EKF 输出可以按记录时间恢复，并且 `/clock` 驱动的可视化与分析不会改用墙钟。

## 11. 统一 Launch 设计

建议入口：

```text
resilient_nav_fault_injection/launch/phase5_fault_demo.launch.py
```

职责：

1. Include `resilient_nav_localization/launch/phase4_ekf_demo.launch.py`；
2. 保留健康 EKF 与阶段 4 所有健康话题；
3. 加载并校验 `scenario_file`；
4. 启动三个 injector；
5. 可选启动 faulted EKF；
6. 可选加载阶段 5 RViz；
7. 把所有节点设置为 `use_sim_time=true`。

建议 Launch 参数：

```text
scenario_file
use_sensor_rviz
start_faulted_ekf
start_fault_probe
```

Launch 不负责修改系统依赖、安装软件、清理实验目录或自动提交 Git。

## 12. RViz 对照设计

阶段 5 RViz 在阶段 4 配置基础上增加：

- raw `/scan` 与 faulted `/faulted/scan` 两个 LaserScan 显示；
- `/odometry/filtered` 与 `/odometry/faulted` 两个 Odometry 或 Path 显示；
- Fixed Frame 继续使用 `odom`；
- RobotModel 继续由健康 TF 驱动。

RViz 只用于直观对照，不能替代定量验证。颜色、大小和显示开关属于可视化配置，不属于故障判定依据。

## 13. 计划目录

```text
ros2_ws/src/
├── resilient_nav_interfaces/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── msg/
│       └── FaultStatus.msg
└── resilient_nav_fault_injection/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    ├── resilient_nav_fault_injection/
    │   ├── fault_models.py
    │   ├── scenario.py
    │   ├── imu_fault_injector.py
    │   ├── wheel_fault_injector.py
    │   ├── scan_fault_injector.py
    │   ├── fault_probe.py
    │   └── experiment_tools.py
    ├── config/
    │   └── scenarios/
    ├── launch/
    │   └── phase5_fault_demo.launch.py
    ├── rviz/
    │   └── phase5_fault_comparison.rviz
    └── test/
```

纯故障模型应与 ROS 节点回调分离，使 bias、noise、dropout、delay、freeze 和 blackout 可以在不启动 ROS 图的情况下单元测试。

## 14. 故障隔离与错误处理

- injector 崩溃不得影响健康原始话题和健康 EKF；
- 场景无效时应在启动阶段失败，不允许自动回退到未知配置；
- 输入长时间缺失时状态应保留计数并报告输入年龄，但不能伪造输出；
- faulted EKF 输入缺失时可产生诊断或停止更新，但不得接管健康 TF；
- 任一输出出现非有限值时，probe 必须报告，不能只在 RViz 中观察；
- Ctrl-C 后节点应正常退出，延迟队列不需要在关闭时强制发布；
- 不删除、重命名或覆盖阶段 4 Launch、配置和健康接口。

## 15. 第一批正式场景

```text
healthy_passthrough.yaml
imu_yaw_bias.yaml
imu_noise.yaml
wheel_freeze.yaml
imu_dropout.yaml
wheel_delay.yaml
scan_sector_blackout.yaml
combined_fault_demo.yaml
```

最小参数：

| 场景 | 关键参数 |
| --- | --- |
| 健康旁路 | 三个 injector 均为 `passthrough` |
| IMU yaw 偏置 | `bias_rad_s: 0.15` |
| IMU 高斯噪声 | 明确 `mean`、`standard_deviation` 和 seed |
| wheel freeze | 运动中冻结 pose 与 twist |
| IMU dropout | `probability: 0.30`，固定 seed |
| wheel delay | `delay_sec: 0.20` |
| Lidar 前方失明 | `min_angle_deg: -30`，`max_angle_deg: 30` |
| 组合故障 | 至少两个 injector 使用明确、可分辨的窗口 |

## 16. 静态方案验收

架构进入代码实现前，应能确认：

- raw、faulted、status 和两个 EKF 输出命名没有冲突；
- 健康 EKF 是唯一 `odom -> base_footprint` 发布者；
- `FaultStatus` 字段能表达场景、来源、故障类型、窗口状态、seed 和计数；
- 场景 YAML 包含稳定 ID、仿真时间、故障参数和期望指标；
- 随机 seed 派生不依赖进程随机 hash；
- fixed delay 与 freeze 的时间戳语义明确且不同；
- 默认 rosbag 同时记录 raw、faulted、status 和两个 EKF 输出；
- 故障真值与未来健康评估输入边界明确；
- 阶段 4 健康接口和独立启动能力不被改变；
- 文档没有把计划功能写成已经动态验证的能力。
