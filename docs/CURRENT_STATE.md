# 当前状态

更新日期：2026-09-08。

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
| Physical Disturbance V2 | 持续压力验收中 | RF V2 三个固定模型已封装进 runtime；6 组有效首轮 full-chain 对照中 healthy 不劣于 fixed，external impact 的 adaptive 位置 RMSE 改善 11.8%；已增加大覆盖扰动区和持续压力路线 | low-friction OOD 虽触发强降权但位置 RMSE 退化 1.7%；概率未校准，尚不能宣称跨扰动普遍优于 fixed |
| RA-1A | 完成 | Offline Diagnosis、只读 Tools、strict result、scorer | 无 Live ROS、Planner、Recovery 或控制权 |

## 当前关键合同

- 主定位 TF 始终保持单 owner；评价 Ground Truth 不回流估计、健康、融合、导航或 Agent。
- Phase 10 是健康 Nav2 baseline，不为后续实验静默改写其参数。
- BRNE Scene 1/2/3 共用 `resilient_nav_brne/config/brne_v1_runtime.yaml`。正式 pedestrian input
  来自 LiDAR tracker；Gazebo odometry adapter 仅用于隔离验证。
- Robot Agent 仍是离线只读 Diagnosis。LLM 不进入实时控制闭环。
- 所有修改必须保留用户未提交变更，测试结论必须区分静态、隔离 runtime、人工验收和 benchmark。

## 已知问题

- Phase 10 有一次 goal 前 TF readiness infrastructure-invalid trial；AMCL/scan 仍有有限偏差。
- BRNE `close_stop_threshold=0.20 m` 是 point-agent 实验门限，不是 footprint 几何安全证明。
- LiDAR dynamic-agent V1 不做人体分类、遮挡续接或长期 ID 恢复。
- C920 在 WSL USB/IP 下仍可能出现闪帧、FPS 波动和偏暗。
- fault-aware navigation、Live Robot Agent 和真实机器人部署尚未完成。
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
