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

- LiDAR “完整 dropout”必须在故障窗口内停止发布 `/faulted/scan`，不能用全 NaN scan 代替；后者仍有消息
  freshness，只是在字段层损坏，考验的 Health/AMCL 路径不同。wheel bias 当前只改
  `twist.twist.linear.x`，保留 pose 与 yaw-rate，以隔离检验 wheel translation reliability。
- 故障注入不能覆盖健康 topic；`/faulted/*`、fixed EKF 和 healthy baseline 必须并存，才能公平对照。
- 后续故障研究分为 sensor fault 与 physical disturbance 两路：前者描述消息/测量失效，后者描述物理世界
  变化引起的运动模型失效。两路允许独立注入与检测，但最终统一进入 Fusion/Adapter 决策，不维护两套
  Adaptive。当前 sensor fault 链已实现；physical disturbance 已有五类场景、独立世界位姿 GT、混合运动
  采数入口和 RF V2 在线 reliability，二者已在统一 Fusion/Adapter 汇总；首轮动态 benchmark 的 6 个
  计划 run 经补跑后均已有有效结果。
- Physical Disturbance 首轮直线 pilot 暴露出三类实验债：DiffDrive `/tf` 不能充当独立 GT；直线走廊中
  point-to-point ICP 即使 RMSE/inlier 良好也可能低估纵向速度；wheel-link 轴向大 torque 会把短时轮阻
  变成翻车冲击。V2 因此改用 PosePublisher 世界模型位姿并首帧重基准，所有对照共享近场非对称
  landmarks 和加速/停车/S 弯/双向 yaw 路线；动态扰动从首个非零运动命令相对触发，轮阻用基座反向
  力与小 yaw 力矩近似。后续动态数据证明这些链路能够完成采集，但各扰动的定位收益仍须逐项报告，
  不能把场景可运行直接写成 adaptive 性能 PASS。
- Physical Disturbance reliability 的最终监督器路线已决定使用小型 Random Forest：输入为 0.3–0.5 s
  窗口的一致性残差、IMU 动力学统计、ICP 质量/可观测性和时序持续性，输出 measurement-specific
  reliability probability。dataset builder、标签合同、RF V2 与 Fusion 接入已实现。重叠窗口只增加
  时序样本，不增加独立实验数；训练/验证必须按完整 run 分组，并留出未参与调参的扰动强度，禁止随机
  拆窗口造成泄漏。独立 GT 误差只进入 label 与单独的 `label_audit_gt_only.csv`，训练 feature 由显式
  allowlist 限定。
- 六组 V2 首轮窗口暴露出 point-to-point ICP 的系统性零位移局部极小：相邻 scan 的真实位移约 2 cm，
  但离散点最近邻目标在零位移附近更小，导致其以低 RMSE、高 inlier 输出约 `0.04 m/s`，而 wheel/GT
  约为 `0.22 m/s`。这不是场景几何数量能根治的问题。当前已改为带 surface normal、可观测性门控及
  reciprocal/MAD/trimmed outlier rejection 的 point-to-line ICP；同六组 bag 离线回放的 LiDAR 中位速度
  恢复到 `0.22–0.24 m/s`，独立正常场景动态验证的 LiDAR/GT 绝对误差中位数约 `0.018 m/s`、P90 约
  `0.045 m/s`。旧 schema-v1 窗口集已作废；扩充后的 12 个有效 run 已重建为 1019 个 schema-v2 窗口，
  其中一次会碰撞的高速 Healthy 轨迹被保存在 raw data 的 `rejected/` 下且不进入 builder。该 physical
  tranche 的 LiDAR translation 标签 1019/1019 均可靠，说明修复有效，但也意味着它不能单独训练 LiDAR
  reliability 的负类；后续必须合并 sensor-fault 负例，不能用类别重采样伪造观测故障。
- RF V1 使用 7/2/3 个完整 run 形成 538/169/312 个 train/validation/test 窗口，采用 sklearn
  `RandomForestClassifier`、`balanced_subsample` 和只由 validation 选择的 measurement-specific 门限。
  留出测试中 wheel rotation 的 balanced accuracy / unreliable recall 为 `0.974/0.974`，IMU yaw-rate 为
  `0.992/1.000`；wheel translation 虽有 `0.980` ROC-AUC，但 balanced accuracy / unreliable recall 仅
  `0.555/0.118`，表现为未见 severe slip 下的概率分布漂移。不得利用 test 回调门限美化该结果；在补充
  强度覆盖或做概率校准前，wheel translation RF 不应接入 Fusion。当前 `predict_proba` 是未校准原始概率。
- wheel translation booster 将旧 12 runs 与 5 个训练候选重建为 17 runs / 1495 windows，translation
  unreliable 从 62 增至 145；新的 `mu=0.055` 长摩擦区 run 独立封存为 118-window OOD，其中 37 个
  translation unreliable。OOD 必须在目录和 manifest 两层排除，不能在后续重划 split 时回流训练。
- `motion_test` 按 wall clock 限时；Gazebo real-time factor 明显小于 1 时，实际路程会短于运动学预估。
  physical disturbance 采集必须用独立 GT 复核扰动区占用，不能仅凭命令时长断言穿越。该批 Run 14
  只短暂进入低摩擦区，因此其主要价值是高速动态样本；Run 12/13 和隔离 OOD 均有足够扰动区窗口。
- RF V2 只比较三组预先受限的 `RandomForestClassifier` 参数，并按三个目标的 validation mean balanced
  accuracy 选型：原 V1 参数 `0.94944`、depth-8/leaf-3 正则化组 `0.94894`、depth-16/leaf-1/50%-feature
  组 `0.88222`，因此保留原参数。固定 test 中 wheel translation balanced accuracy / unreliable recall
  从 V1 的 `0.555/0.118` 提升为 `0.912/0.882`；模型锁定后首次读取的独立 `mu=0.055` OOD 为
  `0.969/1.000`。OOD 未参与训练、门限或参数选择；单个 OOD run 不能外推为全面泛化证明。
- RF V2 runtime 使用三个 joblib 的 class-1 原始 `predict_proba` 连续调 covariance，不复用 validation
  classification threshold。0.4 s 窗口尚未填满时给 wheel/IMU 中性权重 `1.0`，避免 adaptive 启动先天
  弱于 fixed；SensorHealth 负责离散故障安全边界，Fusion 默认不再重复 recovery confirmation。wheel
  translation/rotation 分量分别调权，只有极低 reliability 才启用 LiDAR translation 或 wheel yaw fallback；
  LiDAR 不训练，继续由 Robust ICP quality/observability gate 约束。
- 一次独立 runtime 审计同时记录了 `/faulted/*` 与 `/fusion/input/*`：857 个同时间戳 wheel 样本的测量值
  完全不变，但 translation/rotation covariance 平均分别放大 `13.8×/20.0×`，P90 分别为 `60.9×/100×`；
  ROS graph 也确认 Adaptive EKF 只订阅 Adapter 输出的 wheel、IMU 和两个 fallback topic。该轮低摩擦
  position RMSE 为 Adaptive `0.291 m`、Fixed `0.442 m`。因此“订错 topic、没有写入 covariance、倍率始终
  太小”均被排除；某些长压力 run 输出接近，原因应从当轮 RF 概率和 fallback 占用率解释，不能归咎于
  Adapter→EKF 链路未生效。
- RF V2 首轮 full-chain 对照验证了启动中性权重设计：healthy run 的 adaptive 位置 RMSE 比 fixed 低
  `34.5%`，yaw RMSE 差异小于 `0.1%`，没有重现 adaptive 在健康启动阶段先天偏弱的问题。external impact
  的位置 RMSE 改善 `11.8%`，但 asymmetric adhesion、rough road 和 mild wheel obstruction 只达到约
  `1.6%–3.2%` 的轻微退化/近似持平，不能宣称不同物理扰动下均明显优于 fixed。low-friction OOD 的
  有效补跑显示 RF 强降权但 adaptive 位置 RMSE 仍退化 `1.7%`；此前只有 `/cmd_vel` 的小 bag 必须按
  infrastructure-invalid 排除。启动失败留下同名输出目录时，后续成功启动不等于 recorder 已成功覆写，
  验收必须检查必需 topic、时长和 evaluator sample count。
- Physical disturbance 压力测试不能只把瞬时参数推到极端；若故障有效窗口很短，fixed EKF 本身误差低，
  covariance 降权的代价会掩盖收益。静态地面扰动采用短暂正常起步、大覆盖区域和有界循环/往返路线，
  让策略持续工作数十秒；外力与轮阻仍保持现实的短持续时间，改用间隔清除后的 2–3 次脉冲，禁止用
  数十秒持续 wrench 把机器人拖到墙边。fallback 门限作为 benchmark 参数显式记录，默认仍为 `0.10`。
- probe 统计必须按 `FaultStatus` 活动窗口过滤并使用仿真时钟，否则会把健康窗口混入故障指标。
- Health/Fusion 与 Benchmark Truth 必须分通道：检测和策略不读取真值，Evaluator 才能读取。
- Health 的初始 `UNKNOWN` 不能和故障后的 recovery 混为一谈。wheel/IMU 原始消息通过时间戳、frame、
  有限值、四元数和使用中协方差检查后，可作为 `PROVISIONAL` 以原协方差首次接入；一旦观察到真实
  `FAULT`，恢复确认由 Health Monitor 单层负责，Fusion 不再叠加同类等待。delay/bias 的确认前异常证据应先降为 `DEGRADED`，wheel freeze 则用
  短时无进展预降级和完整窗口确认，避免在确认窗口中继续按健康权重接纳可疑数据。
- wheel fault 时，scan-to-scan LiDAR odometry 只提供独立前向速度，IMU 继续提供 yaw-rate。LiDAR
  measurement 由 Robust ICP 输出质量、本地消息合法性检查和统一 FusionPolicy 控制，不依赖
  `/health/scan`，wheel healthy 时不得送入 adaptive EKF；
  Phase 9 富几何世界的 wheel-freeze 回归已验证该组合，但不能外推到几何稀疏、动态障碍或复合故障。
- 突然出现的行人或障碍物属于 ICP correspondence outlier，不应作为 sensor fault 送入 Health Monitor。
  LiDAR odometry 在 FusionPolicy 之前先做 reciprocal matching 和 median/MAD residual rejection，再执行
  trimmed ICP；动态点占比过高或剩余稳定点不足时直接不发布速度。
- IMU bias detector 依赖 wheel yaw-rate 交叉校验；wheel fault 时的 `wheel_reference_unavailable` 表示
  bias 暂时不可观测，并不表示 IMU 传输或 yaw-rate 本身失效。Health 保留 `UNKNOWN` 诊断结论，Adapter
  仅对这一精确原因将 IMU 作为 `DEGRADED` 测量接纳；其他 `UNKNOWN` 和 IMU 自身故障继续 fail closed。
- Adaptive EKF 不必在每个场景优于 fixed EKF；实验完整性和性能改善必须分别报告。

## SLAM 与 Nav2

- Fault-aware Nav2 出现 `map` frame 长期不存在时，应先沿 TF 上游检查 Adaptive EKF 的输入生产者。
  本次实际故障是裸 `colcon build` 把 Fusion Measurement Adapter 的 shebang 重写为系统 Python，
  该解释器缺少 `joblib/scikit-learn`，Adapter 启动即退出；Adaptive EKF 无量测便不发布
  `odom -> base_footprint`，AMCL 无法消费 scan，最终也不会发布 `map -> odom`。修复合同是所有
  `resilient_nav_fusion` 重建均由仓库 `.venv/bin/python -m colcon` 驱动，而不是调整 AMCL/Planner 门限。
- 导航级 Supervisor 不应把任一连续调权或单个 monitor 的短暂降级直接升级成停车。当前合同把
  `Fusion DEGRADED`（只要仍有 accepted measurement）、单独 wheel/IMU 异常和单 health topic stale
  视为可继续运动的 `DEGRADED`；定位明确不可用立即 `HOLD`，Fusion 无有效量测、关键传感器组合失效或
  Nav2 障碍感知 scan 持续 `FAULT` 则经 0.3 s 确认后 `HOLD`。benchmark 曾临时把 fault-aware Controller
  的 Costmap timeout 放宽到 `2.0 s`，收口时已恢复基线 `0.3 s`，避免把实验便利参数固化为运行合同。
  Nav2 BT 保持不变，由上游 action gate 在 `HOLD` 时取消子 goal，并在恢复后重发保存的同一 goal。
  Adaptive EKF 是正式链路唯一 `odom -> base_footprint` TF owner。
- 完整 scan dropout 持续足够久后，Health 的有限历史可能从 `FAULT` 变回 `UNKNOWN/no_messages_received`；
  Supervisor 必须在 `UNKNOWN` 或未收到新消息时保留已确认的 scan fault，防止 dropout 期间提前重发；
  一旦出现真实 `DEGRADED` 证据即可解除硬锁存并进入可运行恢复态，不强求一步跳到 `HEALTHY`。长时间
  scan 中断还会让 TF buffer 出现历史缺口，因此 goal gate 不再使用固定 `1.5 s` 墙钟延时，而是在恢复后
  等待连续两帧 `/faulted/scan` 的消息时间戳都可查询到 `map` TF 后重发。这是数据链 readiness 证据，
  不是第二层健康恢复确认窗口；该收口修改尚未做 Gazebo 动态回归。
- 首轮 fault-aware navigation 最终集的健康场景严格 3/3 PASS 且无 HOLD；7 个故障场景的 Nav2 goal 均
  成功，其中严格 endpoint 合同 5/7 PASS。LiDAR dropout 出现一次约 9.305 s HOLD 后恢复到达，wheel
  freeze/bias 与 mild/moderate wheel+IMU 在无多余停车下到达。severe wheel+IMU 和综合物理扰动虽到达，
  但终点定位误差超限；任务完成与定位质量必须继续分开报告。
- map-frame 定位质量不能把传感器监测器的启动 `UNKNOWN` 直接等同于不可用。当前用显式
  `LOCALIZATION_PROVISIONAL + localization_usable` 区分“证据仍在到齐”和“已通过 pose/TF 基础检查”；
  启动时完整健康证据可直接进入 `OK`；从 `LOST` 恢复则先进入可用的 `DEGRADED`，默认连续两次新的
  `map -> odom` 更新均通过完整检查后进入 `OK`。重复 timer tick 不计数，中途异常会清零，因此保留短确认
  而不重新引入按秒等待的长 recovery window。
  `/health/scan` 单独故障先降级；考虑 AMCL 静止时 pose 可能低频，仅 pose stale 且 TF 连续也保持可用降级，
  只有关键 `map -> odom` TF 失效、严重协方差或明显 pose jump 才声明 `LOST`。
- 使用仿真时间时，AMCL/TF 回调可能先于对应 `/clock` 回调被 executor 派发；不能据“header stamp 暂时领先
  node clock”永久判定 localization invalid。结构合法性只检查有限正时间戳，freshness 同时使用本地接收年龄
  和 header 年龄，仍能在消息停止后 fail closed。fault-aware 策略 benchmark 的 footprint sweep 保留为记录
  证据，不得在运行中因单个栅格审计结果取消 Supervisor 正在管理的 goal；正式路径安全结论仍应离线报告。
- persisted-map localization 评价必须使用实验前声明的固定 frame transform，不能从被评价轨迹反向拟合。
- AMCL crowd robustness 探索中，clean baseline 用同一初始位姿做五圈 `0.6 m` 半径运动后，EKF/GT
  位置 RMSE 与终点误差分别约 `0.002/0.004 m`，而 AMCL/GT 分别约 `0.196/0.408 m`，终点航向
  误差约 `0.646 rad`；无新增物体时已出现与 crowd/fixed 探索轮次同量级的 `map->odom` 偏移。
  crowd 与 fixed 轮次未严格控制圈数，不能排序其影响。当前没有证据把偏移归因于行人，也不足以支持
  引入 beam skipping；应先把保存地图相对 GT 的几何/航向误差作为 baseline 问题复核。
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
- 机器人转向时 scan/TF 共模漂移包含旋转分量；只减统一平移会让不同方位的静态障碍产生不同残差。
  应从多个背景 cluster 鲁棒估计二维刚体运动，再做 tracker residual，而不是单纯不断加重滤波。
- `LaserScan.ranges` 是 float32；对 Python double 做 `nextafter(range_max, 0)` 后写回消息会舍入成
  `range_max`，被要求 `range < range_max` 的投影/Costmap 路径丢弃。曾用明确 float32 裕量修复该
  clearing 编码，但这不能修复 tracker 误分类和错误几何留下的占据；BRNE 后续已移除 filtered-scan
  global-costmap 路径。
- CW/CCW 墙面诊断显示：只用 IMU yaw-rate 积分的 EKF 在 `0.6 rad/s` 原地旋转时相对 GT 约有
  `0.15 rad` 中位 yaw 误差，停止后仍保留约 `0.28 rad`，不能解释成可由时间戳插值消除的纯传输延迟。
  本地 odom 应由 wheel odometry 的 yaw pose 锚定，并用独立 IMU yaw-rate 提供动态观测。Gazebo
  DiffDrive 的 wheel odometry 是合法 runtime 观测，不是 GT 泄漏，但无打滑且 covariance 全零的模型
  过于理想。当前用 Gazebo WheelSlip 制造真实运动与轮速积分之间的小幅物理偏差，并在 bridge 后只为
  零值 `vx/yaw/yaw-rate` covariance 填入 nominal 方差；wheel yaw 另使用 `0.002 rad` 量化和 fixed-seed
  `0.0005 rad` measurement noise。fixed、faulted 和 adaptive EKF 均融合 wheel yaw，adaptive degraded
  policy 同时膨胀 wheel `vx` 与 yaw pose covariance，而不是污染 EKF 输出。
- 用实时 scan 改写出“静态 scan”要求 tracker 在每帧正确区分机器人自运动、行人和固定物；误分类会
  直接污染 Global Costmap，并在动态对象停止后制造错误静态占据。BRNE demo 因而改为成熟的分层职责：
  Navfn 只消费保存的静态地图，tracker 的 `/scan` 只产生 `/brne/pedestrians`。新固定障碍的局部安全
  约束另行实现，不重新引入 V1 candidate/filtered-scan heuristic。
- 新鲜的空 `PedestrianArray` 表示当前观测无人，应使用确定性 Navfn waypoint fallback；消息缺失或 stale
  则必须 fail closed。二者不能混为一谈。

## Robot Agent

- Agent Input 必须从 allowlist 构造，不能先读取完整 Ground Truth 再靠提示词要求模型忽略。
- Fake 8/8 只证明 pipeline，不代表真实模型诊断能力；真实 Qwen baseline 的 fault-label 失败不能通过
  修改 fixture、truth 或 Scorer 美化。
- 通用 Runtime/Tool/Trace 问题应回到独立 `agent-core` 最小复现和修复，Robot Domain 不维护影子 Core。
