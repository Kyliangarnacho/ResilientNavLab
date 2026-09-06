# ResilientNavLab 项目范围

## 项目目标

ResilientNavLab 用 ROS 2 和 Gazebo 建立可复现的移动机器人实验平台，研究：

- 多传感器接入与统一状态表达；
- 可控故障注入与 Ground Truth 隔离；
- 传感器健康评估和自适应融合；
- SLAM、Nav2 与动态行人交互；
- 面向故障的诊断、降级、恢复与效果评价。

## 已实现范围

- 仿真机器人、IMU/LiDAR/RGB-D/C920 数据链和 EKF 定位；
- IMU、wheel、LiDAR 的首批故障模型与健康评价；
- health-aware measurement policy 和独立 adaptive EKF；
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

- fault-aware localization/SLAM/Nav2 的正式闭环与统一 benchmark；
- Live ROS Robot Agent，以及受控 Planner/Recovery 权限升级；
- 通用人群感知、遮挡续接、长期 tracking 和真实人体实验；
- 真实机器人部署、长期运动性能和完整硬件安全认证。
