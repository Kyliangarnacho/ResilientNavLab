# Phase 8 架构：健康感知融合里程碑 1–4

记录日期：2026-08-16

## 1. 目标、状态与非目标

Phase 8 里程碑 1 冻结机器人主线的四条实验链、共享 `FusionStatus` 接口和未来自适应融合的安全边界。里程碑 2 新增 ROS-free 的 `FusionPolicy`；里程碑 3 新增 `measurement_adapter` ROS 2 Node，将策略允许的 wheel/IMU 测量发布到匿名化 `/fusion/input/*`。里程碑 4 新增独立的 `adaptive_ekf.yaml` 和最小 Launch，将这些匿名化测量接入未修改的 `robot_localization` EKF，并固定输出 `/odometry/adaptive`。它不修改 existing healthy/faulted EKF、传感器、故障注入或健康评估运行逻辑。

本文不涉及 `resilient_nav_agent` 或独立 `agent-core`。它也不授权 Nav2、SLAM、真实硬件定位、任意 EKF 参数写入、规划、恢复或控制输出。

当前不变的定位基线为 `robot_localization` 二维 EKF：`/wheel/odometry` 只融合线速度 `vx`，`/imu/data` 只融合 yaw rate，以 20 Hz 发布 `/odometry/filtered`。该健康参考 EKF 是 `odom -> base_footprint` 的唯一 TF owner。

## 2. 共享接口：`FusionStatus`

`resilient_nav_interfaces/msg/FusionStatus.msg` 只表示融合运行时观察到的链路状态和测量接纳结果。它不表达故障注入真值、场景身份、故障模型、随机种子、参数、期望答案或 evaluator 结论。

```text
std_msgs/Header header
string chain_id
uint8 state
string[] accepted_measurements
string[] rejected_measurements
string[] reasons
float32 confidence
builtin_interfaces/Time window_start
builtin_interfaces/Time window_end
uint32 sample_count

uint8 UNKNOWN=0
uint8 NOMINAL=1
uint8 DEGRADED=2
uint8 HOLD=3
```

字段合同：

| 字段 | 语义 |
| --- | --- |
| `header` | 发布时刻及发布者 frame；不是测量真值时间窗的替代。 |
| `chain_id` | 产生状态的运行链稳定标识；Milestone 1 为未来 adaptive 发布者预留值 `adaptive`。 |
| `state` | `UNKNOWN`（信息不足）、`NOMINAL`（所有配置测量正常接纳）、`DEGRADED`（至少一项受限但仍有接纳测量）、`HOLD`（本周期没有允许的新测量）。它不是 `FaultStatus` 状态。 |
| `accepted_measurements` / `rejected_measurements` | 本窗口中被接纳/拒绝的逻辑测量名，例如 `wheel_velocity`、`imu_yaw_rate`；不得填入 `/faulted/*`、场景名或故障模型名。 |
| `reasons` | 可审计的运行时理由，例如 `imu_health_unknown`、`wheel_health_fault` 或 `insufficient_samples`；不得带入注入参数或 evaluator 答案。 |
| `confidence` | 发布者对本状态完整性的 `[0, 1]` 置信度；`UNKNOWN` 可用 `0.0`，不使用负值表达真值。 |
| `window_start` / `window_end` / `sample_count` | 本次判定使用的 ROS 时间窗口与样本数；两时间必须满足 `end >= start`。 |

接口包继续以 `builtin_interfaces` 和 `std_msgs` 作为 rosidl 消息依赖；没有新增运行时依赖。`FusionStatus` 不替代现有 `SensorHealth`：前者汇总融合决策，后者保留单传感器健康证据。

## 3. 四条冻结链与 topic 命名

| 链 | 输入 | 输出 | 状态 | TF |
| --- | --- | --- | --- | --- |
| healthy reference | `/wheel/odometry`、`/imu/data`；`/scan` 仅作感知参考 | `/odometry/filtered` | 既有、不得改动 | 健康 EKF 唯一发布 `odom -> base_footprint` |
| fixed faulted | `/faulted/wheel/odometry`、`/faulted/imu/data`；`/faulted/scan` 仅作感知对照 | `/odometry/faulted` | 既有、不得改动 | `publish_tf=false` |
| adaptive | `measurement_adapter` 订阅 faulted wheel/IMU 与 `/health/wheel`、`/health/imu`；adaptive EKF 只订阅 `/fusion/input/*` | `/odometry/adaptive`、`/fusion/status`（`FusionStatus`） | Policy、Adapter、独立 EKF YAML 与最小 Launch 已实现 | `publish_tf=false` |
| ground truth | 场景 YAML、`/fault_injection/status` 与 `FaultStatus` | evaluator JSON / benchmark 记录 | evaluator-only | 无 |

命名规则如下：

- `/odometry/filtered` 始终表示健康参考；`/odometry/faulted` 始终表示固定故障输入对照；预留的 `/odometry/adaptive` 只表示未来健康感知策略输出。三者不得相互重映射或覆盖。
- `/fusion/input/*` 是未来 adaptive 发布者的逻辑入口，禁止直接以 `/faulted/*` 作为其稳定接口名。实验 Launch 可以在边界处将实际数据重映射到匿名化入口，但 adaptive 节点、`FusionStatus` 和其缓存不得保留 injected branch 名称。
- `/fusion/status` 是唯一预留融合状态 topic，类型固定为 `resilient_nav_interfaces/msg/FusionStatus`。单传感器健康仍沿用 `/health/imu`、`/health/wheel`、`/health/scan` 与可选 `/health/camera`。
- `/fault_injection/status`、`FaultStatus`、scenario YAML 和 `/faulted/*` 只服务故障注入、记录、对照与 evaluator；它们不是 adaptive 的状态、配置或输出接口。

## 4. Measurement policy（冻结）

Milestone 1–2 不会向现有 EKF 写入或重加载噪声、协方差、topic 或滤波字段。里程碑 2 的纯 Python policy 已把 wheel/IMU 的 `DEGRADED` 映射为配置显式给出的协方差倍率；里程碑 4 的新 EKF 只消费 Adapter 已发布的消息，仍不会向任何既有 EKF 写入或重加载参数。未明确定义的传感器或字段一律拒绝。

| 逻辑测量 | 当前健康证据 | `HEALTHY` | `DEGRADED` | `FAULT` / `UNKNOWN` |
| --- | --- | --- | --- | --- |
| `wheel_velocity` | `/health/wheel` | 接纳为正常 wheel `vx`，scale `1.0` | 仅 `FusionPolicy` 内以显式 `wheel_degraded_covariance_scale` 接纳；未接入 EKF | 立即拒绝 |
| `imu_yaw_rate` | `/health/imu` | 接纳为正常 yaw rate，scale `1.0` | 仅 `FusionPolicy` 内以显式 `imu_degraded_covariance_scale` 接纳；未接入 EKF | 立即拒绝 |
| `laser_scan` | `/health/scan` | 当前不进入二维 EKF，只记录可用性 | 当前不进入二维 EKF | 当前不进入二维 EKF |
| `camera_image` | `/health/camera`（可选） | 当前不进入二维 EKF，只记录可用性 | 当前不进入二维 EKF | 当前不进入二维 EKF |

含义与约束：

1. `SensorHealth.DEGRADED` 不等于可随意 down-weight。里程碑 2 只允许通过 `FusionPolicyConfig` 提供的、最小值为 `1.0` 的明确倍率表达候选策略；要将它写入实际滤波器，仍须另行定义每个字段的噪声映射、边界、回放证据和回归测试。
2. wheel 与 IMU 同时不可用时，policy 报告 `HOLD`；不得把历史速度、注入真值或某个估计输出伪装为新测量。
3. `accepted_measurements` 和 `rejected_measurements` 必须使用逻辑测量名，而非 topic、fault model 或场景标识；`reasons` 必须能由允许的 `SensorHealth` 字段与本地窗口计算得到。
4. Phase 8 后续若需要改写现有 `robot_localization` 配置、切换 source 或发布控制命令，均需单独任务授权。本表不是对现有 EKF 的动态配置授权。

### 4.1 `FusionPolicy` 状态转移

`FusionPolicy` 只接收已 Sanitizer 的 `MeasurementHealth(state, confidence)` 和 `FusionPolicyConfig`，不导入 ROS 消息，也不读取 topic、`FaultStatus` 或场景数据。它输出 `FusionDecision(state, accepted_measurements, rejected_measurements, covariance_scales, wheel_yaw_fallback_enabled, reasons, confidence)`；`reasons` 是稳定、可审计的字符串元组，并一对一映射到 `FusionStatus.reasons`。

- 两项 `HEALTHY`：`NOMINAL`，接纳 `wheel_velocity` 与 `imu_yaw_rate`，scale 均为 `1.0`。
- 任一 `DEGRADED`：`DEGRADED`，接纳该测量但使用对应显式倍率。
- IMU 为 `FAULT`/`UNKNOWN` 且 wheel 可用：立即拒绝 `imu_yaw_rate`，接纳 `wheel_velocity` 并启用 `wheel_yaw_rate` fallback；fallback scale 为 wheel scale 乘以 `wheel_yaw_fallback_covariance_scale`。
- wheel 为 `FAULT`/`UNKNOWN` 且 IMU 可用：立即拒绝 wheel 测量，仅接纳 `imu_yaw_rate`，不启用 fallback。
- 两项 `FAULT`：`HOLD`；两项 `UNKNOWN`：`UNKNOWN`；均无接纳测量，scale 均为 `None`。
- `FAULT`/`UNKNOWN` 的拒绝立即生效。由拒绝恢复为接纳，或由较大 scale 恢复到较小 scale，须达到 `recovery_confirmation_cycles`（默认连续 2 次）才生效；一次健康抖动不会解除 hold 或关闭 fallback。

### 4.2 Measurement Adapter（里程碑 3）

`resilient_nav_fusion.measurement_adapter` 是唯一的 ROS 2 边界适配器，入口为 `ros2 run resilient_nav_fusion measurement_adapter`。默认 topic 如下；全部可由节点参数覆盖，但覆盖值不得成为 `FusionStatus.reasons` 或策略输入字段。

| 方向 | Topic | 类型 | 规则 |
| --- | --- | --- | --- |
| 订阅 | `/faulted/wheel/odometry` | `nav_msgs/msg/Odometry` | 仅作为 adapter 输入；从不覆盖或修改原消息。 |
| 订阅 | `/faulted/imu/data` | `sensor_msgs/msg/Imu` | 仅作为 adapter 输入；从不覆盖或修改原消息。 |
| 订阅 | `/health/wheel`、`/health/imu` | `resilient_nav_interfaces/msg/SensorHealth` | allowlist 仅取 `state`、`confidence`；丢弃 `source_topic` 和其余 metadata。 |
| 发布 | `/fusion/input/wheel/odometry` | `nav_msgs/msg/Odometry` | policy 接纳时深拷贝发布；仅缩放 `twist.covariance[0]`。 |
| 发布 | `/fusion/input/imu/data` | `sensor_msgs/msg/Imu` | policy 接纳时深拷贝发布；仅缩放 `angular_velocity_covariance[8]`。 |
| 发布 | `/fusion/input/wheel/yaw_rate` | `geometry_msgs/msg/TwistWithCovarianceStamped` | 仅 IMU 不可用、wheel 可用时发布；深拷贝 wheel twist，header stamp 保留，frame 设为 wheel `child_frame_id`。 |
| 发布 | `/fusion/status` | `resilient_nav_interfaces/msg/FusionStatus` | `FusionDecision.reasons` 逐项复制到结构化 `reasons`。 |

协方差索引来自本机 ROS 2 Jazzy 消息定义，而不是猜测：`TwistWithCovariance` 是按 `(x, y, z, roll, pitch, yaw)` 排列的 6×6 row-major 矩阵，故 wheel `linear.x` 为 index `0`、wheel `angular.z` 为 index `35`；`Imu.angular_velocity_covariance` 是 x/y/z 的 3×3 row-major 矩阵，故 angular `z` 为 index `8`。

`HEALTHY` 深拷贝且 scale 为 `1.0`；`DEGRADED` 只对相应方差乘以 policy scale，不改线速度、角速度、时间戳或 frame；`FAULT`/`UNKNOWN` 不发布该测量。若目标方差为负、非有限、索引不可用、scale 非法，或 fallback 缺少 `child_frame_id`，adapter fail closed 并 suppress 输出。`sensor_msgs/msg/Imu` 还规定 angular-velocity 协方差全零为未知、index `0=-1` 为该测量不可用；两者也会 suppress IMU。`Odometry` 没有对应 sentinel，故其目标方差为零时保留，不由 adapter 猜测新的方差。

measurement_adapter 的 `SensorHealth` 适配器只读取 `state` 与 `confidence`；它丢弃 `sensor`、`source_topic`、候选故障、原因、指标、窗口和样本数。当前 Phase 6 monitor 的默认来源可以是 `/faulted/*`，因此直接保留 `source_topic` 会泄露实验分支命名；该字段不会进入 `MeasurementHealth`、`FusionDecision`、`FusionStatus` 或发布测量。

### 4.3 Adaptive EKF（里程碑 4）

`resilient_nav_fusion/config/adaptive_ekf.yaml` 是新的、静态的 `robot_localization` 配置，而不是对算法源码或运行中参数的修改。它在二维、20 Hz 的既有契约下只配置三项测量：

- `odom0: /fusion/input/wheel/odometry`，仅融合 `vx`（配置向量 index 6）。
- `imu0: /fusion/input/imu/data`，仅融合 IMU yaw-rate（index 11）。
- `twist0: /fusion/input/wheel/yaw_rate`，仅融合独立 wheel yaw-rate fallback（index 11）。

配置中没有 `/faulted/*`、`FaultStatus`、`SensorHealth`、场景字段或健康判定。Adapter 是否发布上述每一种测量已完整承载 `FusionPolicy` 的接纳、suppress、协方差倍率和 fallback 决策；EKF 不重复实现这些规则。`phase8_adaptive_ekf.launch.py` 只启动 adapter 与 `robot_localization/ekf_node`，把默认 `odometry/filtered` 重映射为固定的 `/odometry/adaptive`。

## 5. TF ownership

| 输出 | `odom -> base_footprint` 权限 | 原因 |
| --- | --- | --- |
| `/odometry/filtered` | 唯一允许发布者 | 当前健康参考显示、RobotModel 和导航语义均以它为准。 |
| `/odometry/faulted` | 永不发布 | 仅为固定故障实验对照，维持 `publish_tf=false`，避免重复 TF。 |
| `/odometry/adaptive` | 不发布 | 里程碑 4 已发布 odometry，但 `adaptive_ekf.yaml` 固定 `publish_tf=false`；未来如需驱动 TF，必须在独立 Launch 中显式切换唯一 owner，并停止健康参考 owner。 |

`FusionStatus` 自身不拥有或推导 TF；它只能解释测量接纳状态，不能宣称 `/odometry/adaptive` 已成为机器人位姿真值。

## 6. 实验真值隔离

运行时适配链、`FusionStatus`、自适应算法输入、参数、日志、缓存和 rosbag topic 清单不得含以下任何内容：

- `FaultStatus`、`/fault_injection/status`、`scenario_id`、`scenario_seed`；
- 故障模型名、注入参数、`parameters_yaml`、事件窗口或预期分类；
- `/faulted/*` 形式的内部 source topic 名、可反推出测试分支的文件名或 metadata；
- evaluator JSON、真值对齐结果、检测分数或 benchmark answer。

只有独立的 evaluator/benchmark 链可以读取这些信息，并且只能在执行结束后将运行时输出与真值比对。实验运行链要测试故障数据时，应在 Launch 或回放边界匿名化为 `/fusion/input/*`，并确保 adaptive 侧的消息、状态与缓存不携带原始 branch 标识。无法证明字段安全时，必须拒绝输入（fail closed）。

### 6.1 Evaluation-only Gazebo pose channel

实际 Gazebo Transport 检查确认 `/world/resilient_lab/dynamic_pose/info` 为 `gz.msgs.Pose_V`，其中包含具名实体；用于本项目的模型专属 source 为 `/model/resilient_nav_robot/tf`，同样为 `gz.msgs.Pose_V`。实际消息包含 `frame_id=odom`、`child_frame_id=base_footprint` 与 Gazebo 时间戳。

`phase8_ground_truth.launch.py` 以 **GZ→ROS 单向** `ros_gz_bridge` 将该模型专属 source 桥接为 `/evaluation/gazebo_model_tf`（`tf2_msgs/msg/TFMessage`）。`ground_truth_pose_adapter` 只接受其中唯一的 `odom -> base_footprint` transform，保留其 header 时间戳和 pose，发布 `/evaluation/ground_truth_pose`（`geometry_msgs/msg/PoseStamped`）。未匹配、重复或非法时间戳一律 suppress。

该 Launch 和 adapter 仅供 evaluator/benchmark 使用。`FusionPolicy`、`measurement_adapter`、health 和 adaptive EKF 不导入、订阅、发布或引用 `/evaluation/*`；Ground Truth 也不发布 TF 或改变既有定位链。

### 6.2 Localization Evaluator

`resilient_nav_fusion.localization_evaluator` 是 ROS-free 的轨迹对齐与指标核心；`localization_evaluator_node` 只是只读 ROS 边界。Node 只订阅 `/evaluation/ground_truth_pose`（`geometry_msgs/msg/PoseStamped`）、`/odometry/faulted` 与 `/odometry/adaptive`（均为 `nav_msgs/msg/Odometry`），并且只向 `/evaluation/localization_metrics` 发布 JSON 结构化结果（`std_msgs/msg/String`）。它不发布测量、TF 或任何会影响 estimator、health、fusion 的 topic。

三个轨迹以每个 Ground Truth sample 的最近时间戳对齐；fixed 与 adaptive 均必须在显式 `max_alignment_delta_sec` 内。缺失/非有限时间或位姿、非单调时间、frame 不匹配、无效四元数、样本不足或共同对齐样本不足都会 suppress/withhold，绝不产生部分指标。每一组输出 `position_rmse`、`yaw_rmse`、`max_position_error`、`max_yaw_error`、`final_position_drift`、`final_yaw_drift` 与 `sample_count`；结果另输出绝对 `position_rmse_benefit = fixed - adaptive`，以及 fixed RMSE 非零时的相对改善率。yaw error 固定为 `[-pi, pi)` 的最短角距离。

## 7. 本里程碑验收与已知问题

本里程碑的验收范围是：

1. `FusionStatus.msg` 被 rosidl 注册并可生成；接口测试锁定字段、枚举、依赖声明及无真值字段的约束。
2. `FusionPolicy` 与 Measurement Adapter 的 targeted tests 覆盖策略原因结构化复制、协方差放大、suppress、fallback、深拷贝、非法协方差 fail-closed 和 faulted sensor-data QoS 兼容性。
3. Adaptive 静态资源测试锁定三个 `/fusion/input/*` 输入、三组观测向量、`publish_tf=false`、固定 `/odometry/adaptive` 重映射，以及 Launch/config 安装。
4. 现有 healthy/faulted EKF 配置和所有既有运行 Launch 保持未修改；adaptive EKF 只在新的最小 Launch 中运行，且没有新的 TF owner。
5. Ground Truth runtime 验收确认 `/evaluation/ground_truth_pose` 类型为 `PoseStamped`，持续约 108 Hz；采样为 `frame_id=odom`、有效 Gazebo 时间戳和接近初始原点的合理位姿。Localization Evaluator 的纯数学与 ROS 消息边界测试覆盖重合、固定偏移、yaw wrap-around、样本不足、时间错位、frame、时间戳和隔离。
6. IMU benchmark launch/observer/runner 已参数化，但只复用既有 Phase 5/6 场景；fixed 与 adaptive 均以同一 `/faulted/wheel/odometry`、`/faulted/imu/data` 对照。IMU bias 与 fixed-delay 的独立 ROS/Gazebo domain 运行均成功；原始结果分别见 `docs/PHASE8_IMU_BIAS_BENCHMARK.json`、`docs/PHASE8_IMU_DELAY_BENCHMARK.json`。当前 recorder 记录同时明确 `benchmark_outcome` 与 `adaptive_improved`，后者只有 position 和 yaw RMSE 都严格低于 fixed 时才为 true；因此不把实验完整性误写成性能改善。Benchmark recorder 只在 evaluator 侧读取 truth status，用于记录 fault/health/fusion/recovery 时间线；运行链保持不读取该信息。
7. 既有 `imu_dropout_demo.yaml` 保持未改动，但不作为本轮正式 Phase 8 benchmark：它在 100 Hz IMU 上仅配置随机 `dropout_probability=0.30`，而 Phase 6 的 stale 阈值为 `0.5 s`，无法可靠生成 Health 首次异常和完整恢复证据。此处不通过调高概率或改变检测阈值来制造改善；待有单独授权的可靠现成场景后再纳入。
8. Wheel-freeze benchmark 复用既有 `wheel_freeze_ekf_comparison.yaml` 和 Phase 6 wheel-freeze Health 判定；固定与 adaptive 均消费同一组 faulted wheel/IMU 数据。观察器以逻辑目标 `wheel` 记录 Health/Fusion，但显式将该目标映射到既有 `FaultStatus.sensor=wheel_odometry`，只用于 evaluator 侧时间线。正式独立运行得到 `benchmark_outcome=PASS`：冻结 `5.0–15.0 s`，wheel Health 和 Fusion 均在 `7.0 s` 首次异常，Health 于 `15.2 s`、Fusion 于 `15.4 s` 恢复。故障期间证实 `wheel_velocity` 被拒绝、`imu_yaw_rate` 被接纳、wheel yaw fallback 未启用，符合 policy；原始结果见 `docs/PHASE8_WHEEL_FREEZE_BENCHMARK.json`。本系统没有独立 translation redundancy，因此这项实验不构造替代线速度，也不把 fail-closed 的 wheel 抑制解释为可以恢复准确前进位移；本次 `adaptive_improved=false`，因为虽位置 RMSE 较低，yaw RMSE 并未低于 fixed。

仍待后续授权解决的问题：`DEGRADED` 倍率的经验证噪声映射、adapter 与实际回放/仿真时序的扩展验收、adaptive 仅剩单测量时的状态估计语义，以及未来是否以独立 Launch 将 TF owner 从健康参考切到 adaptive。它们均不在里程碑 4 的 EKF 实现范围内。
