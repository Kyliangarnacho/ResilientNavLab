# ResilientNavLab 项目范围

## 项目目标

ResilientNavLab 用 ROS 2 和 Gazebo 建立可复现的移动机器人实验平台，研究：

- 多传感器接入与统一状态表达；
- 可控故障注入与 Ground Truth 隔离；
- wheel longitudinal-speed bias 与真正停止发布消息的 LiDAR dropout，供 fault-aware navigation 的暂停/恢复验收；
- 传感器健康评估和自适应融合；
- SLAM、Nav2 与动态行人交互；
- 面向故障的诊断、降级、恢复与效果评价。

## 已实现范围

- 仿真机器人、IMU/LiDAR/RGB-D/C920 数据链和 EKF 定位；
- IMU、wheel、LiDAR 的首批故障模型与健康评价；
- health-aware measurement policy、独立 adaptive EKF，以及 wheel fault 下已通过富几何场景动态回归的
  scan-matching LiDAR 平移 fallback；
- Gazebo DiffDrive 轮关节运动学 odometry 的 uncertainty wrapper：该 wheel odometry 是 runtime 测量而非
  Ground Truth，wrapper 统一补充量化、噪声与 covariance，再按 wheel yaw pose、wheel `vx` 和 IMU
  yaw-rate 的明确合同送入 EKF；
- 基于 Phase 9 建图世界、独立世界位姿 GT 和混合运动路线的 5 类可调 physical disturbance 采数骨架；
- physical disturbance 离线 dataset builder、Robust point-to-line ICP、44 维传感器/ICP feature、4 个
  独立 GT reliability label 和隔离的 GT label audit；12 个有效 run 已形成 1019 个 schema-v2 窗口，旧
  point-to-point 窗口集已作废；
- 基于完整 run 分组的 physical disturbance RF V1 离线训练：三个 `RandomForestClassifier` 分别输出
  wheel translation、wheel rotation、IMU yaw-rate 的原始 reliability probability；LiDAR 不训练，继续
  使用 ICP quality/observability gate；
- wheel translation 补强 tranche 已整理为独立 schema-v2 训练主池（17 runs / 1495 windows / 145 个
  translation-unreliable label）和禁止训练的 OOD 验收集（1 run / 118 windows / 37 个
  translation-unreliable label）；RF V2 已在三组受限参数中仅按 validation 选择原 V1 参数，并保存
  三目标模型、逐窗概率、固定 test 与 OOD 结果；
- RF V2 三模型已封装进 Fusion runtime：在线 0.4 s 窗口严格复用 44 维 allowlist，输出未经阈值离散化的
  `predict_proba`；Fusion Supervisor 将 SensorHealth 硬边界、RF 连续权重和 LiDAR ICP gate 汇总到同一
  Measurement Adapter，对 wheel 平移/旋转和 IMU yaw-rate 分量分别调 covariance；
- 薄封装 map-frame Localization Quality Monitor：只消费 `/health/scan`、AMCL pose 协方差/时效、
  `map -> odom` TF 时效和明显 pose jump，输出显式 provisional/ok/degraded/lost 与可用性；不读取实验真值，
  不执行 Nav2 控制；
- 薄封装 Resilience Supervisor 与 Nav2 goal gate：聚合 Health、Fusion 和 Localization runtime 状态，
  允许单源或短暂降级继续导航；定位不可用或关键量测组合持续失效时取消 Nav2 子 goal，恢复后重发同一 goal；
- 定位与导航 TF/感知职责已明确：Adaptive EKF 独占 `odom -> base_footprint`，AMCL 独占 `map -> odom`；
  Global Costmap 使用保存地图的 StaticLayer 与 Inflation，BRNE 不再通过特殊静态 scan 链改写全局障碍层，
  只负责动态行人信息；
- healthy 2D LiDAR SLAM 与 saved-map Nav2 baseline；
- BRNE V1 的 LiDAR dynamic-agent Scene 1/2/3 人工闭环；
- RA-1A 离线只读 Robot Diagnostic Agent。

## 强制边界

- Ground Truth 只允许 Benchmark/Evaluator 使用，不得进入 Agent、估计器或控制决策输入。
- LLM 永远不得进入实时控制闭环。
- RA-1A 不得发布 `/cmd_vel`、修改参数、管理节点或执行 Recovery。
- 外部框架只按 [开源项目基线](OPEN_SOURCE_BASELINES.md) 中明确 Adopt 的部分使用。
- 当前人工 Demo、单次 smoke 和统计 benchmark 必须明确区分。

## 尚未实现

- fault-aware Nav2 在更多 seed、扰动强度和真实硬件上的泛化验收；当前 severe wheel+IMU 与综合物理扰动
  仍未通过冻结终点质量合同；
- Localization Quality Monitor 与 Resilience Supervisor 的系统性门限泛化验收，以及 physical disturbance
  RF 概率校准；
- Live ROS Robot Agent，以及受控 Planner/Recovery 权限升级；
- 通用人群感知、遮挡续接、长期 tracking 和真实人体实验；
- 真实机器人部署、长期运动性能和完整硬件安全认证。
