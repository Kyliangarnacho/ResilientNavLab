# 当前状态

更新日期：2026-09-09。

## 阶段状态

| 阶段 | 状态 | 当前可用能力 | 主要限制 |
| --- | --- | --- | --- |
| 0–3 | 完成 | ROS/Gazebo 基础设施、机器人模型、差速运动、odom TF | 只验证低速平地基础运动 |
| 4 | 完成 | IMU、二维 LiDAR、RGB-D、wheel+IMU EKF | 无 PointCloud2 和真实硬件定位 |
| 5 | 完成 | 可复现故障注入、`FaultStatus`、faulted EKF、bag | 真值只供 evaluator，不能进入 Agent |
| 6 | 完成 | IMU/wheel/scan 健康判定与评价 | 不执行控制或恢复 |
| 7 | 完成 | C920、CameraInfo、去畸变、camera health v1 | WSL USB/IP 帧率和偏暗限制仍在 |
| 8 | 完成 | 健康感知融合、启动合法数据准入、异常预降级、Robust ICP wheel-fault LiDAR 平移 fallback、GT 隔离评价 | Phase 9 富几何世界 wheel-freeze V3 与 healthy parity 均 PASS；healthy 单次评价仍有 wheel/IMU 各 1 个 transient false positive |
| 9 | 完成 | 二维 SLAM、地图保存/重载、loop closure | endpoint 误差仍明显，不称为高精度定位 |
| 10 | 已收口 | Nav2 localization、Costmap、Navfn、RPP、BT、Recovery、Cancel | 8/8 valid PASS；严格自动 9/9 未达到 |
| BRNE V1 | 已收口 | sensor-input Scene 1/2/3、横穿/迎面交互、armed control | 人工 baseline；非统计 benchmark |
| Physical Disturbance V2 | 已收口 | RF V2 三个固定模型已封装进 runtime，并完成 physical disturbance full-chain benchmark 与上游调权到 Adaptive EKF 的链路审计 | low-friction OOD 仍有负优化，概率未校准，不能宣称跨扰动普遍优于 fixed |
| Fault-aware Navigation | 首轮动态 benchmark 完成 | Localization Quality Monitor、容错型 Resilience Supervisor、Adaptive EKF TF 单一所有权及 Nav2 goal cancel/replay gate 已接成 runtime 链；健康 3/3 严格 PASS，故障 5/7 严格 PASS、7/7 Nav2 goal 成功 | severe wheel+IMU 与综合物理扰动未满足冻结终点质量合同；收口后 TF 证据重发逻辑尚未做 Gazebo 回归 |
| RA-1A | 完成 | Offline Diagnosis、只读 Tools、strict result、scorer | 无 Live ROS、Planner、Recovery 或控制权 |

## 当前关键合同

- 主定位 TF 始终保持单 owner；评价 Ground Truth 不回流估计、健康、融合、导航或 Agent。
- Phase 10 是健康 Nav2 baseline，不为后续实验静默改写其参数。
- BRNE Scene 1/2/3 共用 `resilient_nav_brne/config/brne_v1_runtime.yaml`。正式 pedestrian input
  来自 LiDAR tracker；Gazebo odometry adapter 仅用于隔离验证。
- Robot Agent 仍是离线只读 Diagnosis。LLM 不进入实时控制闭环。
- 所有修改必须保留用户未提交变更，测试结论必须区分静态、隔离 runtime、人工验收和 benchmark。

## 本阶段系统收口

本阶段把此前分散的几何、测量、融合和导航恢复工作整理为一条有明确职责边界的 runtime 链：

1. **LiDAR / TF 几何闭环**：旋转时墙体拖影和 ghosting 的主要根因不是 LiDAR 固定延迟，而是旧 EKF
   yaw 缺少绝对姿态锚。当前 EKF 使用 wheel yaw pose 与 wheel `vx`，IMU yaw-rate 提供动态角速度观测。
   Global Costmap 已脱离 `/brne/static_scan` 一类特殊输入，恢复为保存地图、StaticLayer 与 Inflation 的标准
   静态环境链；BRNE 只负责动态行人信息。
2. **Wheel Odometry 测量合同**：wheel odometry 来自 Gazebo DiffDrive 的轮关节运动学积分，属于合法
   runtime 传感器测量而非 Ground Truth。`wheel odom -> uncertainty wrapper -> EKF` 链统一补充有限量化、
   噪声和 covariance，明确了测量来源、使用字段与不确定性归属。
3. **Adaptive Fusion / RF V2**：physical disturbance 数据集、44 维在线 feature 和三个 measurement-specific
   Random Forest classifier 已完成；旧 ICP 的近零平移局部极小已由 robust point-to-line ICP 修复。正式链为
   `Sensor Health + RF reliability + LiDAR quality/observability -> Fusion Supervisor`，再经
   `covariance/reject/fallback -> Adaptive EKF`。Fixed/Adaptive A/B 已确认调权链真正生效，同时保留部分
   扰动仅持平或轻微负优化的结论。
4. **Localization 与任务级韧性**：Adaptive EKF 独占 `odom -> base_footprint`，AMCL 独占 `map -> odom`；
   healthy/crowd 探索未支持把主要偏差归因于 crowd-specific AMCL robustness。Localization Quality Monitor
   负责汇总 scan、AMCL covariance/freshness、TF freshness 和 pose jump。Resilience Supervisor 区分正常、
   降级继续和必须 HOLD；上游 `/resilient_navigate_to_pose` 保存任务，在 HOLD 时取消 Nav2 child goal，恢复
   readiness 后重发同一 goal，并处理 scan dropout 锁存、TF buffer 恢复和 benchmark path sweep 越权终止等竞态。

这条闭环不使用 FaultStatus、场景参数或 GT 做在线决策；GT 仍只进入离线 label 与 Benchmark/Evaluator。

## Benchmark 收口摘要

指定运动 Fixed/Adaptive 对照使用相同 GT 评分，负的变化率表示 Adaptive 更好。完整原始汇总见
[`benchmark_summary.json`](../data/physical_disturbance_v2/rf_full_chain_benchmark_20260908/benchmark_summary.json)。

| 指定运动场景 | Fixed 位置 RMSE (m) | Adaptive 位置 RMSE (m) | 位置变化 | 结论 |
| --- | ---: | ---: | ---: | --- |
| Healthy | 0.0434 | 0.0284 | -34.5% | 健康不退化，位置改善；yaw 基本持平 |
| Low friction OOD (`mu=0.055`) | 0.3578 | 0.3637 | +1.7% | RF 已降权，但 fallback 不足，轻微负优化 |
| 左右轮附着不一致 | 0.0704 | 0.0716 | +1.7% | 近似持平 |
| 粗糙路面 | 0.0261 | 0.0269 | +3.2% | 近似持平，小幅负优化 |
| 外部冲击 | 0.0361 | 0.0318 | -11.8% | 位置明显改善 |
| 轻度轮阻 | 0.0317 | 0.0322 | +1.6% | 近似持平 |
| Wheel freeze + LiDAR fallback（Phase 8） | 1.2512 | 0.5809 | -53.6% | 位置显著改善；yaw RMSE 改善 66.6% |

Fault-aware Navigation 最终选择遵循“每个场景保留最后一次有效 retry”；完整选择和综合结论见
[`selection_manifest.json`](../data/fault_aware_navigation/final_benchmark_20260909/selection_manifest.json) 与
[`adaptive_effectiveness_summary.json`](../data/fault_aware_navigation/final_benchmark_20260909/adaptive_effectiveness_summary.json)。

| 导航场景 | 严格 benchmark | Nav2 goal | Supervisor 行为 | 终点/结论 |
| --- | --- | --- | --- | --- |
| Healthy simple | PASS | 成功 | 0 HOLD | 终点误差约 0.0965 m |
| Healthy detour | PASS | 成功 | 0 HOLD | 终点误差约 0.0937 m |
| Healthy multi-turn | PASS | 成功 | 0 HOLD | 终点误差约 0.0983 m |
| Wheel freeze | PASS | 成功 | 降级运行、0 HOLD | 终点误差约 0.0927 m |
| Wheel bias | PASS | 成功 | 降级运行、0 HOLD | 终点误差约 0.0974 m |
| LiDAR dropout | PASS | 成功 | 1 次 HOLD，约 9.305 s，随后恢复 | 终点误差约 0.0907 m |
| Wheel+IMU mild | PASS | 成功 | 降级运行、0 HOLD | 终点误差约 0.0944 m |
| Wheel+IMU moderate | PASS | 成功 | 降级运行、0 HOLD | 终点误差约 0.0965 m |
| Wheel+IMU severe | FAIL | 成功 | HOLD 后重发成功 | 最终定位误差超过冻结合同 |
| 综合物理扰动 | FAIL | 成功 | 无 HOLD | 最终定位误差超过冻结合同 |

以上导航表对应收口前已采集 runtime。收口后恢复了 `0.3 s` Costmap timeout，并把 goal 重发从固定墙钟
延时改为 scan 时间戳 TF readiness；该小改动仅有 targeted tests，不能倒写成已完成动态 benchmark。

## 已知问题

- Phase 10 有一次 goal 前 TF readiness infrastructure-invalid trial；AMCL/scan 仍有有限偏差。
- BRNE `close_stop_threshold=0.20 m` 是 point-agent 实验门限，不是 footprint 几何安全证明。
- LiDAR dynamic-agent V1 不做人体分类、遮挡续接或长期 ID 恢复。
- C920 在 WSL USB/IP 下仍可能出现闪帧、FPS 波动和偏暗。
- fault-aware navigation 首轮最终集为健康 3/3 严格 PASS、故障 5/7 严格 PASS；7 个故障场景的 Nav2 goal
  均成功。severe wheel+IMU 和综合物理扰动仍超过冻结终点误差门限，因此只证明已验证场景中的任务维持/
  暂停/恢复能力，不宣称恶劣扰动下定位质量普遍可靠。
- Physical Disturbance V2 的旧 schema-v1 窗口集不得训练；当前有效入口是 schema-v2 point-to-line 数据集。
  RF V1 已固定整 run 的 7/2/3 train/validation/test 分组，test 留出 Healthy、severe friction 与增强 push；
  后续不得通过查看 test 后调门限来掩盖 wheel translation 的强度分布漂移。
- wheel translation booster 的有效训练入口是
  `data/physical_disturbance_v2/rf_dataset_v2_wheel_translation_boost_20260908`；独立最终验收入口是
  `data/physical_disturbance_v2/rf_dataset_v2_wheel_translation_ood_20260908`。后者 manifest 标记
  `training_prohibited=true`，不得用于训练、门限选择或 feature selection。
- 当前 RF V2 训练证据入口是 `data/physical_disturbance_v2/rf_v2_20260908`，runtime 固定模型副本安装自
  `resilient_nav_fusion/models/physical_reliability_rf_v2/`。两组废案产物已从项目数据目录清理，三组
  validation 对比仍保存在选中结果的 `hyperparameter_comparison.{json,csv}`。
- 在线 RF 预热窗口不得阻塞合法 provisional 数据：预热时 wheel/IMU reliability 取中性 `1.0`；运行时
  不使用训练阶段的 classification threshold，也不对连续概率增加 recovery confirmation。SensorHealth
  仍保留故障安全边界，但 Fusion 默认不再重复其恢复确认。
- RF V2 首轮 full-chain benchmark 归档于
  `data/physical_disturbance_v2/rf_full_chain_benchmark_20260908/benchmark_summary.json`。补跑后 6 个计划
  run 均有有效数据；healthy 的 adaptive/fixed 位置 RMSE 为 `0.0284/0.0434 m`，yaw RMSE 基本相同；external
  impact 的位置 RMSE 改善 `11.8%`。asymmetric adhesion、rough road 与 mild wheel obstruction 的位置
  RMSE 相对 fixed 分别变化 `+1.7%/+3.2%/+1.6%`，只能判为近似持平。有效 low-friction OOD 中 RF
  明显降权 wheel，但 LiDAR fallback 稀疏，adaptive 位置 RMSE 仍退化 `1.7%`；此前两个启动失败留下的
  小 bag 已按 infrastructure-invalid 排除。
- Physical benchmark 已增加待动态验证的长暴露压力入口：`stress_loops` 在正常起步后沿同一安全圆轨迹
  正向/反向运行约 50 s，`stress_shuttle` 为非对称附着保留固定左右轮轨迹；场景可扩大 disturbance zone，
  动态外力与轮阻使用可配置的短脉冲次数/间隔而非持续拖拽；benchmark 可显式传入 LiDAR fallback
  reliability 门限。该能力当前只有静态测试，不写成动态 PASS。
