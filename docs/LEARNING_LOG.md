# 学习与决策日志

本文件只保留会影响后续设计或防止重复踩坑的调试结论。阶段成果见各阶段唯一总结，不在这里重复
堆叠逐次命令、完整终端输出或普通参数调整。

## 基础 ROS / Gazebo

- Gazebo Transport 与 ROS 2 Topic 名称相同也不会自动互通；必须核对 bridge 的方向与消息类型。
- 安装 ROS/Gazebo 后，旧 shell 可能因未重新 source 而找不到 `gz`，这不等于安装失败。
- `base_footprint` 必须统一 Gazebo DiffDrive、odometry 和 TF 的参考点，否则转向时会出现系统偏差。
- Gazebo 原生 TF 未直接桥接；ROS 侧动态 TF 保持单 owner，避免多来源竞争。
- 受限 sandbox 中 DDS 可能报 `getifaddrs: Operation not permitted`。该问题属于环境权限，不能据此
  修改算法或伪造动态验证结果。

## 故障、健康与融合

- 故障注入不能覆盖健康 topic；`/faulted/*`、fixed EKF 和 healthy baseline 必须并存，才能公平对照。
- probe 统计必须按 `FaultStatus` 活动窗口过滤并使用仿真时钟，否则会把健康窗口混入故障指标。
- Health/Fusion 与 Benchmark Truth 必须分通道：检测和策略不读取真值，Evaluator 才能读取。
- wheel freeze 时拒绝错误 wheel velocity 是正确的 fail-closed 行为，但系统没有独立平移冗余，不能
  宣称恢复了真实前进速度。
- Adaptive EKF 不必在每个场景优于 fixed EKF；实验完整性和性能改善必须分别报告。

## SLAM 与 Nav2

- persisted-map localization 评价必须使用实验前声明的固定 frame transform，不能从被评价轨迹反向拟合。
- loop closure 的直接证据应包含候选接受、约束链接和位姿校正，普通 scan edge 不足以证明闭环。
- Navfn centerline free 不代表完整非对称 footprint 安全；Planner Path 必须用 raw Costmap 做 footprint sweep。
- lifecycle/TF readiness 在发 goal 前失败属于 infrastructure-invalid，不能计为导航算法失败。
- fully blocked baseline 的关键事实是 Planner 无有效路径、BT abort 和 Controller stop，不应靠缩短观察
  窗口美化终止延迟。

## BRNE 接入与控制语义

- 早期 smoke 将 `num_samples` 降到 16 后，行为不可代表官方 runtime。最终恢复 pinned commit
  `633a5cdcb39ab27f18b596cb8cb1968644f82391` 的 `196×25`、kernel `0.2/0.2`、cost
  `15/3/20` 和 ped scale `0.1`。
- wrapper 曾把 `(plan_steps, samples, 2)` 的 control ensemble 同时沿时间和 sample 求和，造成即时命令
  被 25 个时间步叠加。修复后只沿 sample axis 加权，发布第 0 步，并用完整 weighted sequence 重模拟
  `/brne/optimal_path`。
- pinned angular sampler 的 support 会随 nominal angular 收缩；极端 `±0.8 rad/s` 时可能只剩一个方向。
  曾用 passing-side commitment 验证 `0.25` proposal scaling 能恢复候选支持。该机制改善过 Scene 1，
  但随着 crossing/head-on event 统一后成为重复状态，最终已删除；当前由 event owner 的 proposal
  protection 承担同一职责。
- global-path freeze 没有改善左右摆动，因为机器人 origin 持续移动时，固定 waypoint 的相对航向仍会变。
  nominal freeze 会导致长期偏向一侧，output low-pass 也只掩盖策略切换；三者均已删除。
- 旧 safety mask 用 robot future 对行人当前冻结中心，误删本来会随时间分离的轨迹。当前使用
  robot candidate 与 pedestrian CV mean 的对应时间步距离；横穿偏好侧且末端明确分离的 candidate
  使用 `0.10` soft factor，而不是一律硬置零。
- crossing side bias 必须先在机器人坐标系判断完整速度方向、`t_cross` 和 forward intersection。只看
  odom 的 x/y 或只看一个速度分量会在机器人转向、高速斜穿时误分类。
- 迎面与横穿 event 必须互斥；45° 附近优先归入宽一些的 head-on approach cone，避免一个行人同时
  获得两套规则。

## BRNE 感知与规划职责

- Gazebo pedestrian odometry 直供 BRNE 是“上帝视角”，只能做隔离验证。正式 Scene 使用 scan timestamp
  的 TF、LiDAR clustering、common-drift correction、tracking 和 velocity EMA。
- 机器人转向时 scan/TF 共模漂移会污染速度；应从多个背景 cluster 的鲁棒中值估计公共位移，并拒绝
  突然的大方向变化，而不是单纯不断加重滤波。
- 同一行人不能同时作为 Navfn 的静态障碍和 BRNE dynamic agent。新 cluster 在确认身份前先从
  `/brne/static_scan` 隔离；确认静态后恢复，确认动态后继续隔离，停止足够久再降级为静态。
- 新鲜的空 `PedestrianArray` 表示当前观测无人，应使用确定性 Navfn waypoint fallback；消息缺失或 stale
  则必须 fail closed。二者不能混为一谈。

## Robot Agent

- Agent Input 必须从 allowlist 构造，不能先读取完整 Ground Truth 再靠提示词要求模型忽略。
- Fake 8/8 只证明 pipeline，不代表真实模型诊断能力；真实 Qwen baseline 的 fault-label 失败不能通过
  修改 fixture、truth 或 Scorer 美化。
- 通用 Runtime/Tool/Trace 问题应回到独立 `agent-core` 最小复现和修复，Robot Domain 不维护影子 Core。
