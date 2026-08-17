# Phase 8 开源项目基线（机器人主线）

记录日期：2026-08-16

## 1. 目的与授权边界

本文是 Phase 8 机器人主线的外部参考快照，用于约束后续的健康感知融合、定位降级与容错导航设计取舍；它不是实现计划、依赖清单或性能结论。

本次只记录设计借鉴：不 clone 外部仓库、不安装依赖、不复制代码、不修改 ROS 2 机器人代码，也不授权 Nav2、SLAM、真实硬件定位、控制策略或机器人状态变更。`resilient_nav_agent` 和独立 `agent-core` 不在本文范围内。

### 当前本地基线

- ROS 2 Jazzy + Gazebo Harmonic；差速机器人已完成短时运动和安全停车基线。
- 定位仍是 `robot_localization` 的二维、odom-frame EKF：`/wheel/odometry` 只融合 `vx`，`/imu/data` 只融合 `yaw rate`，20 Hz，输出 `/odometry/filtered` 和唯一的 `odom -> base_footprint` TF。
- Phase 5 的故障输入发布在 `/faulted/*`，faulted EKF 输出 `/odometry/faulted` 且不发布 TF；健康原始 topic 不被覆盖。
- `SensorHealth` 是运行时健康接口，提供传感器、状态、分数、置信度、候选类别、原因、指标、时间窗和样本数。
- `FaultStatus` 含 `scenario_id`、`scenario_seed`、模型、参数和故障时间窗，是故障注入与 evaluator 的真值接口；它不得成为 Phase 8 运行时融合、决策或控制输入。

因此，Phase 8 后续如获授权，首先应把 `SensorHealth` 与可审计的估计器观测质量关联；不得以 `FaultStatus`、场景名或 faulted topic 命名替代检测或切换逻辑。

## 2. 总体取舍

| 项目 | Phase 8 参考定位 | 依赖决策 | 许可证意识 |
| --- | --- | --- | --- |
| [PX4](https://github.com/PX4/PX4-Autopilot) | 估计器健康、观测拒绝与状态可观测性 | 不引入；只借鉴设计 | BSD-3-Clause；若将来复制代码或派生实现，须单独做归因与许可证复核 |
| [ArduPilot](https://github.com/ArduPilot/ardupilot) | 多源估计器 source set / lane 切换的安全语义 | 不引入；只借鉴设计 | GPL-3.0；禁止复制、移植或派生代码，未来若考虑集成必须先做独立许可证评审 |
| [OpenVINS](https://github.com/rpng/open_vins) | 视觉—IMU 时间、外参和不确定性的定位前置条件 | 不引入；只借鉴设计与评测问题 | GPL-3.0；禁止复制、移植或派生代码，未来若考虑集成必须先做独立许可证评审 |
| [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) | LiDAR—IMU 紧耦合与退化质量输出的长期参考 | 不引入；只借鉴设计 | BSD-3-Clause；若将来复制代码或派生实现，须单独做归因与许可证复核 |

“只借鉴设计”表示可独立重新表述问题、接口边界和验收指标；不表示可以复制源码、配置、注释、测试数据或算法实现。许可证说明是工程风险提示，不构成法律意见。

## 3. PX4

- **解决的问题：** PX4 是模块化飞控栈；其 EKF2 为姿态和位置提供扩展卡尔曼滤波估计。它体现了把传感器观测有效性、创新量/估计器状态与上层状态机分离的工程模式，而不是把异常直接等同于控制动作。[PX4 仓库](https://github.com/PX4/PX4-Autopilot)；[EKF2 源码入口](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/EKF2.cpp)。
- **borrow：**
  - 以独立、结构化的估计器质量/状态输出承载“观测是否可用于融合”的理由、时间与置信度；后续可与 `SensorHealth` 对齐，而不是从自由文本日志推断。
  - 将观测质量检查、观测接纳/拒绝、估计器状态报告和上层降级策略分成可测边界；每层保留可复现实验指标。
  - 为任何未来的源选择保留“观察—判断—建议/批准—执行后验证”的闭环，而非根据单个健康字段直接切换。
- **do_not_borrow：** 不接入 PX4、uORB、NuttX、MAVLink、飞行器参数系统、飞控 failsafe 或执行器控制；不复制 EKF2 源码、参数表或调参值；不把飞行器状态位原样映射为地面差速机器人的安全策略。
- **依赖决策：** **Reject-or-Defer。** 当前固定字段 `robot_localization` EKF 保持不变；不新增 PX4 依赖、桥接或进程。只有在一个单独授权的、可回放的健康感知融合实验显示现有估计器接口不足时，才可提出最小的本地接口方案。
- **license awareness：** PX4 主仓库为 [BSD 3-Clause](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE)。本阶段没有形成其派生作品；将来若复制任何受版权保护的实现或资源，须保留通知、满足再分发条件，并在合入前复核依赖树中可能存在的不同许可证。

## 4. ArduPilot

- **解决的问题：** ArduPilot 的 EKF3 支持按位置、速度、高度和航向等字段配置 source set，并支持不同 EKF lane 的亲和与切换；这是“冗余来源切换必须有明确输入语义和连续性处理”的成熟参考。[EKF source selection and switching](https://ardupilot.org/plane/docs/common-ekf-sources.html)；[EKF3 lane switching](https://ardupilot.org/copter/docs/common-ek3-affinity-lane-switching.html)。
- **borrow：**
  - 将未来的降级候选明确为有限的、版本化的 source/fusion mode，而非任意参数修改。
  - 对每个候选模式定义可用传感器、进入条件、退出条件、保持时间、估计连续性检查和回放验收指标；`SensorHealth` 只提供观测证据，不是故障真值开关。
  - 先在不改动机器人状态的 replay/对照实验中验证选择逻辑，再讨论在线、确定性且受安全门约束的切换。
- **do_not_borrow：** 不接入 ArduPilot、MAVLink、飞行模式、RC 切换、自动 failsafe 或其参数体系；不实现自动在线 lane/source 切换；不复制 EKF3、冗余管理或地面站代码，也不将飞行器级容错动作迁移为本项目行为。
- **依赖决策：** **Reject。** 项目当前为地面差速机器人仿真，且尚未授权自适应融合或容错导航；不引入 ArduPilot 运行时、库、配置或 ROS bridge。
- **license awareness：** ArduPilot 项目采用 [GPL-3.0](https://github.com/ArduPilot/ardupilot)。因此本项目仅保留独立表述的架构启发，禁止复制、移植、fork 或派生其代码和受保护配置；任何未来集成或派生工作必须先经单独 GPL 合规与项目授权审查。

## 5. OpenVINS

- **解决的问题：** OpenVINS 是基于 MSCKF 滑动窗口的视觉—惯性估计平台，覆盖相机—IMU 外参、时间偏移和内参等标定问题。它强调相对时间错误与外参错误会显著影响动态轨迹上的估计质量。[OpenVINS features](https://docs.openvins.com/)；[sensor calibration guidance](https://docs.openvins.com/gs-calibration.html)。
- **borrow：**
  - 在未来视觉参与定位前，把相机—IMU 时间同步、外参、图像质量、观测数量和标定质量设为显式的准入/不确定性问题，而不是只检查“相机 topic 存在”。
  - 为未来实验分别记录 calibration、time alignment、视觉 tracking/信息质量和估计输出质量；与现有 camera `SensorHealth` 的 stale/freeze/exposure/blur/low-information 证据保持区分但可关联。
  - 先用可回放数据验证任何视觉辅助定位的可观测性与失败模式，再评估是否需要新的估计器。
- **do_not_borrow：** 当前不接入 VIO、MSCKF、OpenVINS simulator、在线标定、视觉特征追踪或其 ROS 接口；不把 C920 的健康阈值宣称为 VIO 标定或定位质量结论；不复制源码、配置、标定脚本或数据集。
- **依赖决策：** **Reject。** 当前 C920 尚未接入机器人 TF、相机—IMU 同步或真实硬件定位，项目也没有 PointCloud2/视觉定位授权；不新增 OpenVINS、OpenCV 扩展、Ceres 或标定工具依赖。
- **license awareness：** OpenVINS 代码与文档采用 [GPL-3.0](https://github.com/rpng/open_vins)。本阶段仅借鉴问题分解和测试维度；禁止复制、移植、fork 或派生实现。任何未来集成必须先进行独立 GPL 合规审查并获得明确授权。

## 6. LIO-SAM

- **解决的问题：** LIO-SAM 面向实时 LiDAR—IMU 里程计与建图，以因子图结合 LiDAR、IMU 预积分及可选 GPS；其资料特别暴露了对点云格式、IMU 频率、时间戳和外参的前置要求。[LIO-SAM repository and architecture](https://github.com/TixiaoShan/LIO-SAM)。
- **borrow：**
  - 把“快速局部里程计”和“较慢的全局一致性/建图”视为不同职责与不同质量指标，避免未来功能增长时把实时路径和离线/低频优化混为一层。
  - 在未来 3D LiDAR 或 LiDAR—IMU 融合前，显式验收时间同步、坐标外参、测量频率、观测退化和输出协方差/质量，而非仅验证 topic 连通。
  - 为潜在 LiDAR 退化建立不泄露真值的质量证据（例如几何约束不足、时间异常、残差/协方差变化）；保持由 `SensorHealth` 描述运行时判定、由 `FaultStatus` 仅用于 evaluator 的双通道边界。
- **do_not_borrow：** 不接入 LIO-SAM、ROS 1/catkin、GTSAM、LOAM 特征管线、GPS 因子图、机械多线 LiDAR 假设或其参数；不把现有二维 `LaserScan` 伪装为 PointCloud2；不复制代码、launch、配置、数据集或调参值。
- **依赖决策：** **Reject-or-Defer。** 当前项目只有二维 Lidar、无 PointCloud2 bridge、无 3D LiDAR、无 SLAM 授权；不新增 ROS 1、GTSAM 或 LIO-SAM 依赖。若未来单独授权 3D LiDAR 阶段，应先完成接口、标定与回放验收提案。
- **license awareness：** LIO-SAM 主仓库标示为 [BSD-3-Clause](https://github.com/TixiaoShan/LIO-SAM)。本阶段没有复制实现；未来若复用实现或资源，须逐文件确认版权与许可证，并完成 BSD 通知及依赖许可证复核。

## 7. Phase 8 后续门槛（非实现授权）

任何后续机器人主线实现提案至少应先说明：

1. 运行时输入只使用合法的观测、`SensorHealth` 和估计器质量，不读取 `FaultStatus` 或任何场景/注入真值。
2. 新模式或降级策略是有限、Schema 化、可回放和可审计的；不开放任意 EKF 参数写入或任意 ROS 控制。
3. 正常、单传感器退化、恢复、误报和无信息五类对照都具有明确指标；通过 evaluator 的结果不得反向喂给运行时逻辑。
4. 任何影响机器人状态的实际切换、规划或控制都须另有授权和独立 Safety Gate；本文件不构成该授权。

