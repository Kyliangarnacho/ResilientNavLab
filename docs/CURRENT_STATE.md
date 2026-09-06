# 当前状态

更新日期：2026-09-06。

## 阶段状态

| 阶段 | 状态 | 当前可用能力 | 主要限制 |
| --- | --- | --- | --- |
| 0–3 | 完成 | ROS/Gazebo 基础设施、机器人模型、差速运动、odom TF | 只验证低速平地基础运动 |
| 4 | 完成 | IMU、二维 LiDAR、RGB-D、wheel+IMU EKF | 无 PointCloud2 和真实硬件定位 |
| 5 | 完成 | 可复现故障注入、`FaultStatus`、faulted EKF、bag | 真值只供 evaluator，不能进入 Agent |
| 6 | 完成 | IMU/wheel/scan 健康判定与评价 | 不执行控制或恢复 |
| 7 | 完成 | C920、CameraInfo、去畸变、camera health v1 | WSL USB/IP 帧率和偏暗限制仍在 |
| 8 | 完成 | 健康感知融合、adaptive EKF、GT 隔离评价 | 无独立平移冗余；adaptive 不保证总是更优 |
| 9 | 完成 | 二维 SLAM、地图保存/重载、loop closure | endpoint 误差仍明显，不称为高精度定位 |
| 10 | 已收口 | Nav2 localization、Costmap、Navfn、RPP、BT、Recovery、Cancel | 8/8 valid PASS；严格自动 9/9 未达到 |
| BRNE V1 | 已收口 | sensor-input Scene 1/2/3、横穿/迎面交互、armed control | 人工 baseline；非统计 benchmark |
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
