# ResilientNavLab 学习与决策日志

本日志按日期记录项目中的事实、判断、经验和后续问题。尚未实施或验证的内容应标记为计划或待办。

## 2026-09-06 — BRNE V1 Scene 1/2/3 正式收口

- Scene 3 人工验证后的算法与数值冻结为 Scene 1/2/3 统一 profile：pinned BRNE `196×25` core、
  `3.20 m` interaction entry、`0.20 m + 4 outputs` separation release、互斥 crossing/head-on event、
  crossing `10×` side bias 与 5-output initial direction mask、head-on fixed passing side、`0.25`
  proposal support scale、time-aligned safety factors、`0.20 m` point-agent close-stop，以及 no-agent
  deterministic Navfn waypoint fallback。三套 sensor Demo 不再各自在 launch 中暴露或复制算法参数。
- 新增唯一 `config/brne_v1_runtime.yaml`，同时承载 shadow planner 与 LiDAR tracker 的当前冻结数值；
  Scene 1/2/3 的两个节点都从这份文件取值。Scene launch 只保留 `arm_brne`、`use_rviz`、`log_level`
  以及各自场景几何/编排，减少以后只改一个场景而产生参数漂移的可能。
- 删除旧 passing-side commitment 实现、ROS diagnostics topic、参数与专用 probe；它曾用于证明
  pinned angular sampler 在极端 path nominal 下会丢失一侧 support，但已被当前 event-owned proposal
  protection 取代。另删除已证伪的 global-path freeze / nominal freeze 生产分支；历史实验结论保留在
  本日志，不继续作为 runtime 兼容面。
- 正式动态行人输入仍是 timestamped LiDAR cluster/track；Gazebo odometry adapter 和 GT closed-loop
  launch 只保留为感知/控制隔离验证，不作为公平 benchmark。Navfn 继续只消费 tracker 输出的
  `/brne/static_scan`，RViz 显示 Global Costmap。
- 本条记录建立的是人工 Scene baseline，不是统计 benchmark，也不把 `0.20 m` point-agent mask
  宣称为完整 footprint 安全证明。最终 package pytest 为 `123 passed, 1 xfailed`；colcon result 为
  `124 tests, 0 errors, 0 failures, 1 skipped`（同一 strict xfail 在 JUnit 中记为 skipped）。根 `.venv`
  驱动的 scoped build 成功，shadow/tracker 两个 install console script shebang 均指向根 `.venv`；
  source/install 的统一 YAML 与三套正式 launch 逐文件一致。当前机器的 `196×25` isolated probe 为
  cold warm-up `4752.483 ms`，随后 30 次 warm planning mean/P95/max 为
  `11.315/12.706/15.292 ms`，低于 5 Hz 的 `200 ms` 周期。接口与 Navfn-remap 边界 targeted
  tests 另为 `7 passed`，`resilient_nav_interfaces + resilient_nav_navigation + resilient_nav_brne`
  三包联合构建成功。本轮未重复运行 Gazebo。

## 2026-09-05 — Sensor Scene 1 interaction-time global-path retention

- 为隔离 BRNE 连续周期方向变化是否由 Navfn Path 微变触发，仅在 sensor-input Scene 1 增加 `/plan`
  出口冻结：LiDAR tracker 首次确认 dynamic track 后，periodic planner 停止发起新 Navfn request，并以
  原 1 Hz 频率刷新 timestamp、重发最近一条成功 Path。Planner Server 的原始 plan 仅在该 overlay
  remap 到 `/brne/navfn_plan_raw`，因此在途 action result 也不能覆盖冻结 Path；Phase 10 默认仍发布
  `/plan`，参数与规划算法未改。
- freeze owner 使用 sensor track ID；release 与既有 commitment 一致，采用 `min_distance_seen + 0.20 m`
  separation gain 和最近 4 个 5 Hz 距离样本的正趋势。已完成的同一可见 track 不会在下一帧立即重新
  acquire，track 消失后才结束其 interaction lifecycle。BRNE commitment 在该 Demo 仍为 disabled。
- 删除上一轮实验性 angular output low-pass，control gate 再次直接发布通过 freshness、ownership、有限值
  和速度边界校验的 BRNE raw command。本条记录只描述实现与自动测试，不把最终 armed Gazebo 行为写成
  已验收结论。
- 静态 `/plan` 仍不能保证 path nominal 固定：local waypoint 虽不变，robot origin 在移动，二者连线的
  heading error 仍会变化。为做单变量验证，sensor Scene 1 在 dynamic track 出现前保存最近 angular
  nominal，并在同一 interaction lifecycle 内把它作为 BRNE sampler proposal nominal；release 后恢复
  实时 nominal。该实验不恢复 passing-side commitment，也不干预 BRNE weighted command 的方向。
- 人工检查确认 global-path freeze 没有改善初始左右摆动，sensor Scene 1 随后解除该 overlay：Navfn
  再次以原 1 Hz 直接发布 `/plan`。interaction angular nominal freeze 保留为当前唯一实验变量；通用
  periodic planner 中默认关闭的 path-freeze 支持未影响其他入口。
- 当前 nominal-freeze 实验最终保持原起点 odom `y=-1.0` 和 `0.25 m/s`，只将终点从
  `y=+1.0` 延长到 `y=+2.0`；3 m crossing 为 12 s。共享 prismatic joint 的允许行程扩到
  3 m，其他 Scene 的 driver 仍在原 2 m 目标停车。
- 人工观察到机器人退出 all-masked stop 后，LiDAR dynamic-agent velocity arrow 出现剧烈方向波动；为做
  最小因果对照，当前 sensor Scene 1 暂时恢复 Gazebo odometry adapter 作为
  `/brne/pedestrians` 唯一 writer。LiDAR tracker 继续只为 Navfn 提供 `/brne/static_scan`，其 agent
  输出隔离到 `/brne/sensor_pedestrians_unused`。BRNE core、mask、nominal freeze 和控制未改；该 GT
  输入只用于诊断，不代表最终 benchmark 感知合同。
- GT 对照确认后，`/brne/pedestrians` 已恢复由 LiDAR tracker 发布，Gazebo odometry adapter 再次从
  sensor Scene 1 graph 移除。interaction nominal freeze 同时关闭；commitment 仍关闭。替代实验只在
  单次 interaction 的前 5 个输出（约 1 s）内，以第一次明显非零 BRNE angular output 的方向为锚，
  对超过 `0.2 rad/s` 的反向 path nominal 沿用 `0.25` 缩放；第 6 个输出恢复 raw nominal。该状态不
  强制 command 方向，也不修改 BRNE core、weights 或 mask。
- 当前 sensor Scene 1 crossing X 调整为 odom `0.8`（world `-2.7`），proposal protection 窗口恢复为
  5 outputs；12 s 行程、`0.25 m/s`、真实 LiDAR 输入与 commitment disabled 均不变。
- wrapper close-stop mask 从“完整 robot future 对冻结 pedestrian 当前中心”改为逐时刻比较
  `robot_candidate[t]` 与 pedestrian CV mean `[x+vx*t, y+vy*t]`，继续使用原 `0.58 m` 几何阈值；
  因而仅空间上经过行人旧位置、但相同时间步已安全分离的候选不再被误删。pinned BRNE core、随机
  pedestrian ensemble、proposal protection 和 weighted control 均未改变。
- LiDAR tracker 的 6 帧 common-drift-corrected 速度拟合与 dynamic promotion 判据保持不变；只在
  已确认 track 的发布速度上按 ID 增加 `alpha=0.3` EMA：首个输出沿用拟合值，后续为
  `0.3 * v_fit + 0.7 * v_prev`。本条只记录实现与自动回归，尚未替代 armed Gazebo 人工验收。
- 对“`close_stop_threshold=0.01` 仍停车”的复核发现，最近六次实际 launch 生成的 shadow-node 参数
  文件均仍为 `0.58`，没有一次进入 `0.01`；对应 shadow 进程在观察段持续完成 plan，未发生进程退出或
  ownership revoke。为避免继续从 RViz 猜测，shadow node 现在启动时打印实际生效阈值，并只在零速原因
  发生变化时报告 all-masked、输入 missing/stale、planning rejection、goal tolerance 或仍有 safe
  candidates 的 weighted-zero。该诊断不修改 mask、BRNE control 或 gate 行为。
- 一次正式未 armed、无 RViz 的 Gazebo Scene 1 参数探针确认，单行 launch override 能使 shadow node
  实际打印 `close_stop_threshold=0.010 m`，并在 35 s 观察段持续规划且未报告 all-masked。用户侧
  `close_stop_threshold:=0.01: command not found` 是该 token 被 shell 换行后作为独立命令执行，ROS 未收到
  它。为让当前因果实验不再依赖尾部参数，sensor Scene 1 overlay 临时默认改为 `0.01`；通用 planner 与
  其他 Demo 的 `0.58` 默认未改。
- `0.01` 参数链验证完成后，sensor Scene 1 的可传 `close_stop_threshold` 默认值调整为 `0.50 m`；同一行
  launch override 仍可选择其他值。mask 的 time-aligned CV 语义、通用 planner 的 `0.58 m` 默认值及
  其他控制逻辑均未改变。
- sensor Scene 1 随后将可传 mask 阈值初定为 `0.40 m`，并仅在 wrapper 的 pinned BRNE robot weights
  之后增加近期 crossing side bias：所有状态先投影到当前 robot frame，以
  `t_cross=-p_lateral/v_lateral` 和前向交点门控，再对行人运动反侧的 candidate future lateral
  displacement 乘 `1.25`。默认门为 lateral speed `0.12 m/s`、现有 `2.5 s` planning horizon 和前向
  `[0.20,1.50] m`；这些项目级参数在 sensor launch 可覆盖。commitment、proposal protection、mask、
  BRNE core 与最终 weighted-control extraction 均未改。
- 人工 Demo 在 `crossing_lateral_speed_threshold=0.10`、forward gate `[0.20,1.50] m`、side-bias
  multiplier `1.5` 与 close-stop `0.30 m` 下表现良好，这组数值现成为 sensor Scene 1 overlay 默认值，
  且仍可由 launch 覆盖。既有 finite-output proposal protection 保持默认 5 outputs，并由同名 launch
  参数 `proposal_protection_window_outputs` 提供覆盖；窗口内部的反向判定和 `0.25` 缩放未改。
- `brne_scene2_demo.launch.py` 已对齐上述 sensor profile：BRNE 输入来自 LiDAR tracker，Navfn 使用
  `/brne/static_scan`，commitment 关闭，mask/crossing bias/proposal window 的默认值与 Scene 1 相同。
  两个 prismatic pedestrian 的原几何、速度和顺序保留；因为不再存在 commitment release 事件，Ped2
  场景门仅在 robot map `x > 0.8` 后等待原有 `0.4 s` 再启动，不改变 BRNE 控制逻辑。
- 为观察双行人场景中的主动反侧绕行，sensor Scene 1/Scene 2 overlay 默认值调整为 crossing-side
  multiplier `2.0`、close-stop `0.27 m`、finite proposal window `5`；窗口内反向 nominal 的保护触发
  阈值从 `0.20` 提高到 `0.35 rad/s`。mask 与 side-bias 的 launch 覆盖保留，BRNE core 和 mask 语义未改。
- Scene 2 极端参数诊断把 crossing `t_cross` 门从 `2.5 s` 放宽到 `4.0 s`。有效 runtime 中 Ped2 启动后
  约 `0.74 s` 才完成 LiDAR dynamic-track 确认；确认后 `v_lateral≈-0.24`、preferred side 为 robot-left，
  证明方向判定正确。`1000×` bias 可令目标侧最终权重占比达到约 `0.79–1.00`，但部分周期 proposal
  中目标侧 candidate 数直接为零，此时任何倍率均无效。诊断因此保留 per-track gate、bias 前后权重占比、
  mask 后 candidate 数、proposal support 和最终 command；没有据此修改 BRNE core 或 temporal policy。
- crossing/interaction 语义随后收敛：LiDAR track 在 6 帧 dynamic confirmation 后再完成 3 次
  `alpha=0.3` EMA 更新才发布；已发布 agent 进入 `2.20 m` 欧氏门后才建立 interaction。soft crossing
  event 只评估 interaction owner，门调整为 lateral `0.08 m/s`、`t_cross<=4.0 s`、forward
  `[0.20,2.00] m`，默认 multiplier 为 `5.0`；event 过 robot centerline 后退出，而 interaction 仍按
  `0.20 m + 4-frame` 持续分离释放。event 最初 3 个输出清零与 pedestrian lateral velocity 同向的 angular
  candidates，窗口可由 launch 覆盖；原 5-output proposal support 保留。低 lateral 的迎面 interaction
  不触发硬侧向 mask，BRNE core、safety mask 与 commitment-disabled 状态未改。
- Scene 1/2 人工调参随后冻结 overlay 默认值为 forward max `2.00 m`、crossing multiplier `10.0`、
  crossing 初始反向窗口 `5` outputs、point-agent close-stop `0.20 m`。为避免 time-aligned mask 把
  active crossing owner 的偏好侧候选全部消掉，仅当候选在 horizon 末端已比最近点分离超过既有
  `0.20 m` margin 时，将其 mask factor 从 `0` 改为 `0.1`；其他 pedestrian、非偏好侧和未分离候选仍
  硬置零。`10.0 × 0.1 = 1.0`，所以门限内候选只恢复到 BRNE core 原权重，不获得净 crossing 奖励。
  当前 footprint 外接半径约 `0.391 m`，加 actor `0.18 m` 半径得到保守相切中心距约 `0.571 m`；
  `0.20 m` 仅为实验 point-agent threshold，不是实体碰撞安全保证。
- Scene 2 的第二个行人在 interaction 首次输出仍曾出现与其横向速度同向的控制，故 sensor track 发布前的
  velocity EMA warm-up 从 `3` 次增加到 `5` 次；15 Hz 下只增加约 `0.13 s` 观察时间。dynamic promotion、
  `alpha=0.3` EMA、interaction/event、5-output 初始反向 mask、proposal protection 和 BRNE core 均未改。
- 同一人工现象在 5 次 EMA warm-up 下仍不足够稳定，因此冻结为 8 次；相对 5 次再增加约 `0.20 s`
  观测。Scene 2 同时冻结 forward max `2.00 m`、crossing multiplier `10.0`、初始反向窗口 `5`
  outputs 和 point-agent close-stop `0.20 m`，这些 runtime 行为本轮均未再调整。
- 固定 8 次随后被统一的实际稳定判据替代，而不是继续按场景增加等待：dynamic track 最近 4 次
  `alpha=0.3` EMA 速度必须都超过 `0.08 m/s`，且每个方向相对最新方向的夹角不超过 `0.35 rad`
  （约 20°）才允许发布。横穿与迎面使用同一组参数；方向仍抖动时继续等待，稳定时不承担固定 8 次
  的额外延迟。首次通过后稳定状态按 track 锁存，EMA 继续更新，避免单帧波动造成 agent 断流和
  interaction reset。interaction/event、mask、BRNE core 与已冻结 Scene 参数不变。
- 新增最薄 `brne_scene3_head_on_demo.launch.py`：只生成一名从 odom `(3.0,0)` 以 `0.25 m/s` 沿 `-X`
  迎面运动的 constrained pedestrian，robot goal 与 Scene 2 同为 `(2.8,0)`，BRNE/sensor 参数完整复用
  当前冻结 profile。既有 driver 只增加默认保持 `y` 的 `motion_axis` 选择；Scene 3 用 spawn yaw `pi`
  把已验证的 model `+X` prismatic axis 映射到 world `-X`，Gazebo odometry 仍只用于 actuator 终点而不进入
  BRNE。迎面低 lateral velocity 不触发 crossing side rule；最终行为留给人工 Demo 验收。
- Scene 3 首次人工观察表明纯 BRNE 在迎面对称选择中会左右翻转并原地滞留。interaction owner 的 event
  分类因此改为单一合速度方向分类器与互斥优先池：head-on 要求 approach speed `>=0.12 m/s` 且合速度
  位于 robot `-forward` 的 `0.52 rad`（约 30 度）窄锥角内；其余运动仅在
  `|v_lateral|>=0.08 m/s` 且 lateral alignment `|v_lateral|/|v|>=0.50` 时归入 crossing。纵向锥角严格
  小于 45 度，所以 45 度斜穿优先进入 crossing；单一返回值和 active-event 锁存保证一个行人同一时刻
  只能进入一种 event，避免高速斜向运动同时命中两套独立分量门限。完成标志只在速度仍属于同一类别
  时防止立即重复进入；方向类别改变后允许同一 interaction 进入另一 event。该规则继续避免
  传感器 lateral 噪声误触发既有 crossing bias、初始
  direction mask 和 `0.1` safety softening。head-on 第一条明显非零 BRNE angular output 定侧，之后硬
  清零另一侧即时 angular candidates，并持续保护 proposal support；机器人参考点距 entry 时冻结的
  pedestrian CV 直线达到 footprint-derived `0.58 m` 后退出 event。既有 interaction release、crossing
  event、BRNE core 和 Scene 参数未改。
- Scene 3 的斜迎面定侧增加一个只作用于 head-on event entry 的方向层：合速度仍在既有约 30 度纵向
  锥内，但 lateral 偏角超过 `0.17 rad`（约 10 度）时，机器人从第一条 BRNE 输出起固定选择 pedestrian
  lateral 运动的反侧；死区内继续由第一条明显非零 BRNE angular output 定侧。LiDAR dynamic-agent
  发布范围由 `2.5 m` 前移到 `3.5 m`，通用 interaction 欧氏入口由 `2.20 m` 前移到 `2.80 m` 并重命名为
  `interaction_entry_distance`，使 15 Hz tracking 与 4-frame EMA 方向稳定发生在更充分的距离上；行人
  和机器人速度均未改。
- 修复 sensor-input Demo 在尚无 dynamic agent 时无法起步的问题：正确 frame 且持续由新鲜 scan 产生的
  空 `PedestrianArray` 现在表示“观测无人”，ROS-free shadow planner 直接按 Navfn local waypoint 输出
  确定性 no-agent unicycle fallback。它使用既有 `0.20 m/s` nominal、`0.80 rad/s` 角速度边界，按
  `cos²(heading_error)` 对大转角降速并在临近目标时减速，不创建 BRNE candidates；agent 出现即恢复完整
  BRNE。没有消息、stale 或 frame 错误仍然 fail closed，避免把感知链失效当作无人。
- `/scan` 与 TF 的静态复核确认 tracker 已用 scan 自身 timestamp 查询 `lidar_link -> odom`，odom TF
  broadcaster 也原样继承 odometry timestamp，相关 launch 统一使用 simulation time，因此本轮不改
  TF 链。为降低转向期残余 scan/TF 漂移对行人速度的影响，EMA measurement alpha 从 `0.30` 降到
  `0.25`；仅在 track 已通过原 4-frame 方向稳定门后，对超过 `0.70 rad`（约 40 度）的单帧 fitted
  velocity 方向跳变拒绝更新并保留上一完整速度。初始稳定判据、tracking fit 和 BRNE 输入格式不变。
- 为继续前移 sensor-input interaction 的可用反应时间，仅把 dynamic-agent 发布范围从 `3.5 m` 调到
  `4.0 m`、`interaction_entry_distance` 从 `2.80 m` 调到 `3.20 m`；event 分类、速度滤波、控制参数和
  BRNE core 均未改变。
- 为减少 agent 发布到 interaction 的等待，EMA 方向稳定窗口从 `4` 帧降为 `3` 帧；6 帧 dynamic
  confirmation、`0.08 m/s` 最小速度、`0.35 rad` 窗口方向一致性和稳定后的 `0.70 rad` 异常拒绝不变。
- Scene 3 的 head-on 合速度锥从相对 robot `-forward` 的 `0.52 rad` 扩展到 `1.05 rad`（约 60 度）；
  对当前 `0.25 m/s` 行人，60 度边界的 approach component 约 `0.125 m/s`，与既有 `0.12 m/s` 门限一致。
  因此相对 lateral axis 的约 30--90 度斜向接近使用持续定侧的 head-on event，近 lateral 的 0--30 度
  仍由 crossing event 处理。斜线 release 继续使用 orientation-independent frozen CV line 垂距和
  `0.58 m` clearance。另阻止 head-on release 后在尚未结束的同一 interaction 内重新锚定有限 proposal
  window；crossing/普通 interaction 的窗口保持不变。

## 2026-09-04 — RPP Scene 1 comparison overlay（人工 Demo 待验收）

- 新增 `rpp_scene1_comparison_demo.launch.py`，完整 Include 冻结的 Phase 10
  `phase10_bt_navigation_smoke.launch.py` 无 Recovery profile，故运行时仍是既有 Navfn、1 Hz official
  BT replan、Controller Server 和 RPP；没有修改任何 Nav2 YAML、没有新增 benchmark，也没有启动任何
  BRNE planner/shadow/gate。正式 `/cmd_vel` 仅应由 `controller_server` 发布。
- overlay 复用当前 world-constrained prismatic pedestrian SDF、JointController `Float64 -> gz.msgs.Double`
  bridge 和未修改的 crossing driver；robot/goal 与 BRNE Scene 1 均为 Phase 10 start/map `(0,0,0)` 到
  `(1,0,0)`，actor 仍从 world `(-3,-4.5,0.6,yaw=pi/2)` 以 `+0.25 m/s` 穿过到 Y `-2.5`。driver 改由
  `/rpp/ready`、`/rpp/pedestrian/odometry` 和 `/rpp/pedestrian/joint_velocity` 参数化，不使用任何
  `/brne/*` 控制 topic。
- 新的薄 coordinator 只读查询 Nav2 aggregate lifecycle active，随后发送一次 `NavigateToPose`；goal
  accepted 后以 transient-local `/rpp/ready=true` 解锁既有 driver。driver 仍独立要求真实 non-empty
  `/plan`、fresh odometry 和 `0.1 s` delay，故没有 launch sleep、伪造 plan、速度发布或第二套 pedestrian
  state machine。RPP 从 scan-derived Global/Local Costmap 获得行人占用；它不读取 pedestrian odometry、
  不预测速度，也不实现 BRNE passing-side logic。
- scoped build 以根 `.venv/bin/python -m colcon` 完成；实际 `resilient_nav_brne` package test 为
  `59 passed, 1 xfailed`，xfail 仍仅为 pinned scalar `traj_sim()` 缺少 `dt`。安装后的 coordinator 和
  shadow script shebang 均指向根 `.venv/bin/python`，`ros2 launch ... --show-args` 已成功解析 overlay。
  本轮没有运行 Gazebo/RViz 最终人工 RPP Demo；dynamic crossing behavior 留给用户验收。

## 2026-09-04 — BRNE console-script interpreter contract fixed

- 两次正式 Closed-loop launch 中，`brne_shadow_node` 都在启动不足一秒后 exit 1；`brne_control_gate`
  已 armed、`Float64 -> gz.msgs.Double` pedestrian bridge 已连通、planner 已可刷新 `/plan`，所以共同上游
  不是 actuator、ownership 或 path freshness。实际安装的 BRNE console scripts 的 shebang 是
  `/usr/bin/python3`，而系统 Python 没有 `numba`；pinned `brne.py` 在 import 时依赖它。shadow process
  因此无法发布 `/brne/ready=true` 或 raw command，driver/gate 的既有 fail-closed 合同正确地让两个实体
  保持零速。
- 根 `.venv` 具有 `numba 0.61.2`，但“激活 venv + 调用 `/usr/bin/colcon`”不会改变 colcon 的
  `sys.executable`，故 ament Python `develop --no-deps` 生成的 console scripts 仍固定为系统解释器。
  永久环境规则现写入 root `AGENTS.md`、`docs/ENVIRONMENT.md` 与 BRNE Demo 手册：任何
  `resilient_nav_brne` build/test 必须从 `ros2_ws/` 显式使用
  `../.venv/bin/python -m colcon`，禁止裸/系统 `colcon`；build 后必须检查 shadow executable 的 shebang。
- 已按该规则重建实际 `ros2_ws/install`。`brne_shadow_node`、control gate 和 pedestrian driver 的
  shebang 都指向根 `.venv/bin/python`，source 后 entry points 与当前 build source 一致且可导入 Numba。
  `resilient_nav_brne` package test 为 `55 passed, 1 xfailed`，汇总 `56 tests, 0 errors, 0 failures,
  1 skipped`；xfail 仍仅是 pinned scalar `traj_sim()` 的既有缺少 `dt` 问题。未运行 Gazebo 或最终
  Closed-loop Demo。

## 2026-09-04 — BRNE V1 / Scene 1 final closeout（pedestrian constraint verified）

- 保持人工 Closed-loop Demo 已验证的 `commitment_opposite_scale=0.25`，正式定义为 BRNE V1
  Scene 1 的项目级 temporal/proposal parameter，而非 BRNE 理论常数。没有更改
  `shadow_planner.py` 的 proposal-side commitment runtime 行为，也没有引入 support projection、动态
  gain、滤波、RNG、weight prior、safety mask 或其他 planner 机制。回归将 196 samples 的
  `28 linear × 7 angular` 网格冻结在最坏 committed raw nominal `±0.8 rad/s`：scale 后的
  `±0.2 rad/s` proposal 保留至少两个严格 committed-side angular bins 与一个 zero bin。
- crossing pedestrian 不再是会积分 roll/pitch 的自由刚体或 model-level `VelocityControl` actor。
  `brne_crossing_pedestrian.sdf` 现在以 `world -> crossing_joint (prismatic) -> body` 的结构只允许
  一条 DOF；joint zero 是 launch 的 spawn pose，limits 为相对零位 `0..2 m`。Gazebo Harmonic 在该
  dynamic model 的 local frame graph 中不能把 joint axis `expressed_in` 直接写成 `world`；最终采用
  `expressed_in="__model__"` 的 model `+X` axis，并以 spawn yaw `pi/2` 在 runtime 映射成 world `+Y`。
  这是 runtime 验证后的 frame/topology 选择，不是静态 SDF 名称推断。
- driver 只将原 model `Twist` actuator 替换为 `Float64` joint velocity，经
  `ros_gz_bridge` 转为 `gz.msgs.Double` 送入 `JointController`。既有 BRNE-ready、fresh odometry、fresh
  plan、timeout/fail-closed 与 crossing start/stop 状态机未重写；`OdometryPublisher` 保留，改为
  `dimensions=3` 以使 verifier 能观测真正的 Z/roll/pitch。既有 pedestrian odometry adapter 与
  `pedestrian_state.py` 未改。
- 实际 standalone Gazebo 验证只启动 world、pedestrian、bridge、probe（无 robot/Nav2/BRNE），以
  `+0.25 m/s` 完成 Y `-4.5 -> -2.50025 m`：start/end world pose 为
  `(-3.0,-4.5,0.6)` / `(-3.0,-2.50025,0.6)`，完成于 `8.05 s`，测得 `0.248416 m/s`。真实
  odometry 观察完整 `60.0 s`，X/Z drift、max |roll|、max |pitch|、max yaw drift 均为 `0`，运行中
  child-frame `twist.x` 峰值 `0.250000 m/s`、Y/Z 近零，最终 velocity 为零。`gazebo_world ->
  brne_pedestrian` 的 frame、child frame 与非零 timestamp 全程有效，collision geometry 未删除；因此
  满足结构性 upright 和原 adapter 输入合同。
- focused pytest 为 `30 passed`；隔离 scoped `resilient_nav_brne` build 成功。完整 package test 在
  同一隔离链（含其 `resilient_nav_interfaces` 依赖）为 `55 passed, 1 xfailed`，test-result 为
  `56 tests, 0 errors, 0 failures, 1 skipped`；strict xfail 继续仅对应 pinned upstream scalar
  `traj_sim()` 的已知缺少 `dt` 问题。本轮按授权未重新运行 robot + BRNE 最终 Closed-loop Demo；它保留给
  用户做最终人工验收。

## 2026-09-04 — BRNE passing-side proposal support restoration（Closed-loop Demo 未运行）

- 针对人工 crossing 中“先左避、local waypoint 回正后短暂右修、再左避”的已确认 proposal
  collapse 根因，shadow planner 仅为**单行人**保留短生命周期的 passing-side commitment。entry 从
  当前 BRNE mixed predicted trajectory 相对 pedestrian CV mean 的最小接近点空间侧向分离建立，
  不以 `angular.z` 建立；同时将 entry 时的 forward/left 单位轴固定在 odom frame，后续 local
  waypoint 改变不重新定义左右。多行人输入自动释放/禁用该单行人状态。
- commitment 不重写 BRNE command、weight、cost、safety mask 或行人随机采样；它只在 path nominal
  强烈朝 committed passing side 的反向转动、会令 pinned `get_ulist_essemble()` 丢失对侧 support 时，
  用项目参数 `commitment_opposite_scale=0.25` 缩小该 proposal nominal。因此最终 raw command
  仍是当前 weighted control sequence 的第一步，opposite-side candidates 仍被保留。
- release/reset 覆盖 fixed forward axis 后方通过、当前距离与 CV mean predicted minimum distance
  连续 clear、pedestrian stale/disappearance、goal、planner lifecycle 与 duration timeout；warm-up 的
  synthetic state 在返回前显式清空。每周期 debug diagnostics 提供 path/proposal nominal、angular
  support、safe candidate count、fixed axes、side 与 entry/release reason。
- 完整 `196×25` 数值 probe：left commitment 且 raw path nominal `-0.8 rad/s` 时 proposal 为
  `-0.2 rad/s`，pinned angular candidates 的 min/max 为 `-0.8/+0.4 rad/s`，同时存在正/负候选，
  直接证明 support collapse 已解除。BRNE package colcon test 为 `54 tests, 0 errors, 0 failures,
  1 skipped`（skip 仍为 pinned scalar `traj_sim()` strict xfail）；fresh process cold warm-up
  `5086.665 ms`，50 次 warm planner call mean/P50/P95/max 为 `12.507/11.780/16.865/19.697 ms`，
  P95 小于 5 Hz 的 `200 ms`。未运行 Gazebo 或最终人工 Closed-loop Demo，是否消除 left→right→left
  行为仍待人工验收。

## 2026-09-04 — BRNE mixed-strategy control extraction correction（Closed-loop Demo 未运行）

- 修复 wrapper 将 `(plan_steps, num_samples, 2)` control ensemble 的 time 与 sample 维一起求和
  的输出语义错误。现在只沿 robot sample 维对每个 time step 求 weighted mean，当前 raw command
  为该 sequence 的第一项，仍由既有 `0.30 m/s`、`0.80 rad/s` 有限边界约束。
- `/brne/optimal_path` 不再是最大 robot weight 的单条 candidate；它从当前 robot state 以完整
  weighted control sequence 调用 pinned `traj_sim_essemble()` 单 sample 重模拟，故红色预测线的
  第一 control 与 raw command 一致。没有调用或修改已知缺少 `dt` 的 pinned scalar `traj_sim()`。
- 新增回归覆盖 sample-axis-only mixing、plan length 不会放大第一 control、即时 command 与 sequence
  首项相等，以及预测 path 与混合 sequence 相等且不同于 argmax candidate。direct pytest 为 `10 passed`；
  package colcon test 为 `43 tests, 0 errors, 0 failures, 1 skipped`，skip 保持 pinned scalar
  `traj_sim()` strict xfail。
- 固定 `seed=1` 的 response probe 在修复后给出 left/right static steering
  `-0.721035918261/+0.721035918261 rad/s`；横穿/远离的 forward speed 分别为
  `0.131082938252/0.179345448824 m/s`，故合同改为横穿不得比同位置远离者更快，而非旧的饱和 steering
  比较。没有改动 BRNE 随机 pedestrian prediction 或输出滤波。
- 以完整 2-agent、`196×25` profile fresh process 测得 cold warm-up `5046.470 ms`；随后 50 次
  planner call mean / P50 / P95 / max 为 `12.655 / 11.981 / 16.657 / 18.386 ms`，P95 低于
  5 Hz 的 `200 ms` 周期。按授权没有运行 Gazebo 或完整人工 Closed-loop Demo。

## 2026-09-04 — BRNE pinned runtime algorithm profile restoration（Closed-loop Demo 未运行）

- `resilient_nav_brne` 的 wrapper runtime profile 已以 MurpheyLab/brne pinned revision
  `633a5cdcb39ab27f18b596cb8cb1968644f82391` 的
  `brne_nav/crowd_nav/config/brne.yaml` 为唯一算法基准：`maximum_agents=5`、
  `num_samples=196`、`dt=0.1`、`plan_steps=25`、`kernel_a1/kernel_a2=0.2/0.2`、
  `cost_a1/cost_a2/cost_a3=15/3/20`、`ped_sample_scale=0.1`。速度上限、nominal
  speed、corridor、geometry-derived close stop、goal tolerance 与 5 Hz replan 保持本项目值。
- ROS shadow node 现在显式声明并传入全部上述算法参数；其启动期 `warm_up()` 继续使用最终
  `ShadowPlannerConfig`，没有另建缩小 sample/horizon 的 warm-up profile。pinned Python core
  仍未使用 `ped_sample_scale` 入参，本轮只保持其官方配置和可观测性，未改数学核心。
- 以独立 fresh Python process、Demo 相同的一 robot + 一 crossing pedestrian、完整
  `196×25` profile 测得 cold warm-up `5060.636 ms`；随后 50 次完整 planner call 的 mean /
  P50 / P95 / max 为 `12.087 / 11.331 / 17.710 / 19.309 ms`。P95 与 max 均低于现有
  5 Hz（`200 ms`）控制周期；cold warm-up 仍只属于 `/brne/ready` 前的启动预算。
- BRNE package targeted pytest 和 package colcon test 为 `40 tests, 0 errors, 0 failures,
  1 skipped`；该 skip 仍是已记录的 pinned upstream scalar `traj_sim()` 缺少 `dt` 的 strict
  xfail。官方 cost profile 使 crossing/away steering 都可达到 `0.8 rad/s` 上限，故 causal
  probe 保留“crossing 不弱于 away”而不再要求未饱和时的固定差值；没有改 BRNE 输出。
- BRNE Demo RViz 只新增现有 `/global_costmap/published_footprint` 的绿色 `Robot Footprint`
  Polygon。没有运行最终完整人工 Closed-loop Demo，也没有添加 RNG 固定、滤波、hysteresis、
  跨周期平滑或其他算法补丁。

## 2026-09-03 — BRNE Closed-loop Demo 稳定性与启动顺序修复（人工 E2E 待执行）

- 首次人工运行显示原 `0.16 m × 1.70 m` 自由圆柱在 VelocityControl 水平接触下会倾倒；倒地后
  collision 几何不再匹配 BRNE point-agent 近距模型，且实际 pose 基本停止，不能作为有效 crossing
  demo。修复为半径 `0.18 m`、高度 `1.20 m`、`40 kg` inertia 的 kinematic、collision-visible
  actor，使其受接触时保持直立的规定平面运动；这不是 crowd dynamics 或真实人体模型。
- BRNE Shadow Node 在创建 subscriptions/timer 前同步调用一次固定 `warm_up()`，并以 transient-local
  `/brne/ready` 发布成功状态。crossing driver 现在同时等待 fresh Gazebo odometry、real `/plan` 和
  ready=true，才发 pedestrian command；warm-up 失败则 actor 不会起步，BRNE 也只发零 raw command。
- 既有单独 Transport 实测已经证明当前 Gazebo raw twist 为 child/body frame，故删除运行期
  `TwistFrameVerifier`、验证参数、延迟和相应测试。adapter 直接执行固定 child → world → odom 的
  SE(2) 转换，保留 finite/frame/stamp freshness fail-closed。
- 将 actor 初点从 robot odom `(0.5,-0.8)` 后移至 `(0.5,-1.0)`，目标为 `(0.5,1.0)`，减少起步时
  机器人 footprint 与 actor 物理半径的近距干扰。wrapper close-stop threshold 从 `0.35` 改为显式
  `0.58 m`（footprint 外接半径约 `0.391` + actor 半径 `0.18` + `0.01` padding）；mask 仍只检查
  每条 robot candidate 相对**当前** pedestrian center，未添加逐时刻行人预测或第二套避障逻辑，预测
  仍由上游 BRNE trajectory/weight 负责。
- 新增 BRNE package 自有 RViz 配置，复用 `/brne/optimal_path` 显示红色 `BRNE Selected Prediction`，
  不增加 topic，也不修改冻结的 Phase 10 RViz 配置。修复后 direct BRNE pytest 为 `37 passed, 1
  xfailed`，`resilient_nav_interfaces` + `resilient_nav_brne` symlink build 成功，colcon BRNE
  package test 同为 `37 passed, 1 xfailed`；xfail 仍是 pinned upstream scalar `traj_sim()` 的已知
  缺陷。独立进程的 runtime prewarm 为 cold `4958.195 ms`，后续 20 次 mean `1.257 ms`、P95
  `1.419 ms`。按用户约束不自动重启完整 Gazebo 图做最终 closed-loop 验收。

## 2026-09-03 — BRNE Closed-loop Demo 最小主链实现（人工 E2E 待执行）

- 在现有 `resilient_nav_brne` 内实现独立 closed-loop launch：只 Include 冻结的 Phase 10
  planner-only 链，以 1 Hz `ComputePathToPose(use_start=false, GridBased)` 刷新真实
  `/plan`；没有启动 `controller_server`、BT Navigator、selector 或 Recovery，也没有
  修改 Phase 10 navigation/localization/simulation/description 文件。
- `brne_control_gate` 只接受 `/brne/cmd_vel_raw`，默认 `armed=false` 且不创建正式速度
  publisher。显式 armed 后先检查 `/cmd_vel` endpoint，再作为唯一 owner 发布；raw stale、
  任意非有限值、倒车/非差速轴、速度越界均输出零，发现外部 publisher 时先零速再销毁自身
  publisher 并锁定 failure。
- 初版新增带 collision/inertial 的单圆柱 Gazebo pedestrian，只使用本机 Gazebo Harmonic 的
  `VelocityControl` 和 `OdometryPublisher`。其自由高圆柱实现及运行期 twist semantic verifier 已在
  同日“稳定性与启动顺序修复”条目中替换，不应作为当前能力描述。
- 一次独立 pedestrian-only Gazebo Transport 验证（没有启动 Phase 10、机器人或正式
  `/cmd_vel`）采集 20 条实际 odometry、19 对运动差分：pose-delta 速度约
  `(0.0, 0.238950) m/s`，raw twist 约 `(0.238606, 0.0) m/s`，按约 `pi/2` yaw 旋转后为
  `(0.000029, 0.238606) m/s`；world/child mean error 分别为 `0.340772/0.000340 m/s`，
  因而当前插件真实语义明确为 child frame。该数值来自实际消息，不是名称推断。
- 新增主链 focused tests 16 项全部通过；包含 Task 1 回归的 BRNE pytest 为
  `36 passed, 1 strict xfailed`，package build 成功，colcon package test 同为
  `36 passed, 1 xfailed`。xfail 仍是 pinned upstream scalar `traj_sim()` 已知缺陷。
- 完整 Phase 10 + BRNE Gazebo closed-loop 按用户要求没有自动执行。唯一人工启动命令、
  ownership/topic/TF/frame/速度/终点零速与清理证据清单见
  `docs/BRNE_CLOSED_LOOP_DEMO.md`；当前不得把组件级结果写成 Closed-loop Demo PASS。

## 2026-09-03 — BRNE integration side track Task 1 final closeout

- Task 1 保持 shadow-only：BRNE 只发布 `/brne/cmd_vel_raw` 和
  `/brne/optimal_path`，没有 selector、真实 pedestrian tracking、RPP 对比或正式
  `/cmd_vel` 控制接管。新增的 closeout observer 只读订阅真实输入和 `/brne/*` 输出，
  在一次 fresh Phase 10 `simple_reachable` 行动前启动，并将观测结果写为 JSON。
- 固定 `seed=1` 的纯数值 pedestrian probe 记录了可重复的相对运动因果合同：左/右侧
  静态行人分别产生 `-0.8/+0.8 rad/s` steering，左侧横穿为 `-0.8 rad/s`，强于同位置
  远离时的 `-0.648313 rad/s`；`0.35 m` 正前方行人得到精确零 command。它说明本 wrapper
  的 BRNE 数值结果会随输入行人位置/速度变化，不是性能或真实感知结论。
- 当前机器 2 agents × 16 samples × 12 steps core probe 的 cold JIT 为 `5372.318 ms`；
  随后 20 次 warm 为 mean `0.982 ms`、P50 `0.969 ms`、P95 `1.134 ms`。isolated ROS
  smoke 在停止三种输入后 `970.479 ms` 收到明确零 raw command，并确认停止后未再收到 Path；
  它覆盖长 JIT/输入 stale 时的 fail-closed 合同。
- timestamped-TF 修复后的 fresh Phase 10 run 中，导航 probe PASS（11 次 Path 更新，
  `NavigateToPose SUCCESS`、Controller 最大 `0.20 m/s` / `0.5251 rad/s`）。预启动
  observer 实际看到 `/odometry/filtered` 403、`/plan` 11、`/brne/odom` 91、
  `/brne/goal_pose` 91、pedestrian 202、BRNE optimal Path 11；frame 均为 `odom`
  （source Path 为 `map`）。一条 goal 以其 odom stamp 查询 `odom <- map` TF 后重算的
  waypoint 与发布 goal 的残差为 `0.0 m`，并且同 stamp `/brne/odom` 与源 odom 的
  `(x,y)` 完全一致。运行时 BRNE 计算为 `0.047--0.987 ms`（进程已预热）。
- 同一 fresh run 的 graph endpoint 显示 `/brne/cmd_vel_raw` publisher 为
  `brne_shadow_node`，正式 `/cmd_vel` publisher 为 `controller_server`，没有 BRNE
  endpoint。该 synthetic pedestrian 在真实路线下使安全 mask 全程选择零 raw command；
  这不是 BRNE/Phase 10 failure，而是保守输出。初版 observer 因错误地要求先有非零 raw
  才记录 stale-zero 而把该 JSON 标为 FAIL；已修正为 goal 静默后任何明确零 raw 都可作为
  stale 证据，但遵守“只做一次 fresh E2E”未为该观测器修正再跑导航。隔离 smoke 的 stale
  evidence 是本 Task 的最终动态 fail-closed 证据。
- Task 1 已验证 Python/Numba core、隔离 ROS wrapper、真实 Phase 10 odom/Path/TF
  输入映射、输出 bounds/frame 及 `/cmd_vel` 隔离；尚未验证真实行人状态估计、在真实
  人群中的安全/舒适性、RPP 对比、长期性能或任何机器人控制效果。pinned 上游 scalar
  `traj_sim()` 的 strict xfail 仍保留，未篡改上游数学。

## 2026-09-03 — BRNE integration side track Task 1.3 real-navigation shadow input

- `resilient_nav_brne` 新增独立 `brne_shadow_input_adapter`：只读订阅真实
  `/odometry/filtered` 和 `/plan`，要求 Phase 10 的 `odom` / `base_footprint` 与
  `map` frame 合同、有限数值、非零 stamp、ROS stamp age 和本地 receipt age 都有效。
  它以当前采用 odom 的 header stamp 查询同一时刻 `odom <- map` TF（不可用即拒绝，
  不回退 latest），把最近的 map-frame Path 按弧长选取 `0.8 m` 局部
  waypoint，才一起发布隔离的 `/brne/odom` 与 `/brne/goal_pose`。它不发布 Twist，
  不启动或修改 Phase 10 的任何 launch/参数。
- 空/非法/过期 Path、odom stale、Path 离当前 odom 超过 `0.75 m`、TF 缺失或异常时，
  adapter 停止转发；overlay 将 BRNE input timeout 固定为 `0.5 s`。因此旧 Path 不会
  继续驱动 shadow 输出。新增 pedestrian-only source 只发布 `/brne/pedestrians`，
  保留 Task 1 的确定性运动行人，不再伪造 odom 或 goal。
- Shadow Node 在计算前复制一致输入 generation，并在计算后再次检查 generation 与
  `time.monotonic()` freshness。新增慢计算回归确认超过 watchdog 的 JIT-like 计算只
  发布零 raw Twist 且不发布 Path，避免使用过期快照发出非零命令。
- focused pytest 为 `14 passed, 1 xfailed`；接口与 BRNE 的 package build/test 均
  通过（interfaces 4 个 pytest、BRNE 14 passed/1 upstream strict xfail）。xfail 仍是
  pinned 上游 scalar `traj_sim()` 漏传 `dt`，未修改上游算法核心。
- 在隔离 `ROS_DOMAIN_ID=72` / `GZ_PARTITION=resilient_nav_brne_task13_final` 中，未改动的
  Phase 10 `simple_reachable` probe PASS：`/plan` 9 次更新、导航成功、正式 Controller
  最大 `0.20 m/s` / `0.5069 rad/s`。同一运行的 BRNE Shadow 日志记录 20 次真实输入
  计算（首个**已预热进程内**观测 `1.620 ms`，其后 `0.064--0.196 ms`）；该轮不是
  cold-JIT 基准。运行时 `tf2_echo odom map` 得到有限 `odom <- map` 平移约
  `(-0.074,-0.064)`、yaw `0.007 rad`。`/brne/cmd_vel_raw` 唯一 publisher 是
  `brne_shadow_node`；正式 `/cmd_vel` 唯一 publisher 是既有 `controller_server`，BRNE
  不在其 endpoint 中。
- 动作完成后 adapter 按 stale Path 合同停止转发，观察到 `/brne/cmd_vel_raw` 显式零
  Twist。动作期间的 shadow 计算由日志直接证明；由于事后 observer 才完成 discovery，
  未保留一条同时间的 wire-level waypoint/非零 raw 消息。一次重复 probe 在已有运行图
  上因 `base_footprint -> map` past extrapolation 失败，未把它记为 BRNE 或导航失败；若
  需逐消息审计，应在下次 fresh process 的 action 前预启 observer。

## 2026-09-03 — BRNE integration side track Task 1.2 shadow node

- `resilient_nav_brne` 新增 `brne_shadow_node`，仅订阅 `/brne/odom`
  (`nav_msgs/Odometry`)、`/brne/goal_pose` (`geometry_msgs/PoseStamped`) 和
  `/brne/pedestrians` (`resilient_nav_interfaces/PedestrianArray`)；只发布
  `/brne/cmd_vel_raw` (`geometry_msgs/Twist`) 与 `/brne/optimal_path`
  (`nav_msgs/Path`)。未订阅 `/plan`，未发布或订阅正式 `/cmd_vel`，未启动
  Nav2/Gazebo，也没有 selector、TF adapter 或控制接管。
- 现有 `resilient_nav_interfaces` 最小扩展 `Pedestrian` (`Header`, `id`, `Pose`,
  `Twist`) 和 `PedestrianArray` (`Header`, `Pedestrian[]`)，字段固定匹配
  MurpheyLab/brne `633a5cd` 的接口合同；未引入 `crowd_nav_interfaces`。
- ROS wrapper 只借鉴上游采样/权重装配，数值 BRNE 核心保持原样；删除 Unitree
  `angular.z -= 0.040` 漂移补偿，所有 shadow Path 统一为 `odom` frame。三类输入
  缺失、空行人、非 `odom` frame、非有限数值或超时均 fail safe 发布零 raw Twist。
- synthetic runtime smoke 不启动 Gazebo/Nav2，先观察到缺输入零 raw Twist，再发布
  固定 odom、局部 goal 和一名运动行人。实际证据为三个输入均已收到、三次 BRNE
  compute、Path `odom`/12 poses、`linear.x=0.300000`、`angular.z=-0.621114`，均在
  `0.30 m/s` 与 `0.80 rad/s` 上限内。首次 JIT 为 `5001.182 ms`，第三次 warm
  compute 为 `1.872 ms`，只作为当前机器 shadow baseline。
- `ros2 topic info /brne/cmd_vel_raw -v` 显示唯一 publisher 为
  `brne_shadow_node`；同一隔离运行中 `/cmd_vel` 为 unknown topic，证明该 node
  不是正式控制 publisher。相关 package build/test 为 interfaces `6/0/0/0`、BRNE
  `10/0/0/1 skipped`；唯一 skipped 是 Task 1.1 已记录的 pinned upstream
  `traj_sim()` strict xfail。

## 2026-09-02 — BRNE integration side track Task 1.1 algorithm core

- 新增最小 `resilient_nav_brne` ament_python 包，仅保留 MurpheyLab/brne
  `633a5cd` 的 Python/Numba `brne.py` 数值核心；上游来源、路径、revision 和
  root GPL-3.0 许可证记录在包内 `UPSTREAM.md`。没有引入上游 ROS node、消息、
  launch、Unitree/ZED 或硬件逻辑。
- 根现有 `.venv` 安装 `numba 0.61.2` 与 `llvmlite 0.44.0`；复用系统站点
  `numpy 1.26.4`，没有使用 sudo、apt 或新虚拟环境。
- 确定性数值测试覆盖 covariance/Cholesky、ensemble trajectory、pairwise cost、
  BRNE weights 的 shape、finite、归一化和无 in-bounds robot candidate 的 `None`
  sentinel。直接 pytest 为 4 passed、1 strict xfailed；package colcon 为 5 tests、
  0 errors、0 failures、1 skipped（该 xfail）。
- 固定 2 agents × 16 samples × 12 steps probe 中，首次进程内 JIT 调用为
  `5170.887 ms`，随后 20 次平均为 `0.802 ms`；输出 shape 为 `[2, 16]`、finite，
  两个 agent 的 weight mean 均为 `1.0`。这是当前机器的开发基线，不代表 ROS
  runtime latency。
- 上游标量 `traj_sim()` 调用 `dyn_step()` 时漏传 `dt`，在 pinned source 中会抛出
  `TypeError`。本任务保留上游核心原样，并用 strict xfail 透明记录；可用的
  `traj_sim_essemble()` 已通过轨迹数值合同。没有 ROS 接口、Pedestrian.msg、Path
  adapter、Gazebo/Nav2、`/cmd_vel` 或 Phase 10 参数变更。

## 2026-08-30 — Phase 10 Task 5.3 engineering acceptance and Task 5.4 Goal Cancel implementation

- Task 5.3 r02 建立了核心 Recovery 因果链：官方 Recovery 实际触发（`recovery_count=1`）、临时 wall 由独立的 45 s sim-time 规则删除、`NavigateToPose SUCCESS/error=0`、Controller 恢复运动并自然停车。删墙后 Global Costmap 无法证明对原 11.32 m 长墙全段 clear；有限 LiDAR range、遮挡与长墙几何使它不能作为核心 Recovery 硬门槛。经用户批准，Task 5.3 以 engineering accepted 收口；不重跑、不改场景或 evaluator。
- Task 5.4 复用唯一 NavigateToPose ActionClient：initial Path 和 `0.20 m` filtered odom motion 后，对同一 goal handle 调官方 native cancel；记录 cancel request/ACK、CANCELED terminal、Controller 在 Runner teardown safety-zero 之前的停机命令与 GT/odom settle。它不创建第二套 Runner、不读 GT 控制、不启动 obstacle entity bridge，也不改 Task 1–3/Nav2 参数。静态单测、resource tests、包 build/test 通过；下一步仅是一条人工 host acceptance。

## 2026-08-30 — Phase 10 final closure

- Task 5.4 host acceptance PASS：`NavigateToPose=CANCELED`、`recovery_count=0`、Controller post-cancel zero、odom/GT settle、`failures=[]`、cleanup/evidence flush 完整。
- Task 5.2 r03 用当前 evaluator 离线重评为 PASS；旧 result 的 observation-window failure 保留为历史，不再作为能力否定。
- 全 workspace 在仓库 `.venv` + ROS Jazzy 环境完成 build/test，最终 `675 tests, 0 errors, 0 failures, 1 skipped`。普通 Codex sandbox 的 DDS socket 限制不代表项目失败。
- Phase 10 CLOSED — engineering accepted with known limitations；冻结能力、证据和 Phase 11 边界见 `PHASE10_SUMMARY.md` 与 `PHASE10_EVIDENCE_INDEX.md`。

## 2026-08-30 — Phase 10 Task 5.3 official Recovery implementation

- Task 5.2 r03 已作为 no-Recovery 对照冻结：真实 wall 经 `/scan → ObstacleLayer → Costmap` 后，最终记录 Planner-first Navfn `NO_VALID_PATH/208 → BT ABORT → FollowPath cancel → Controller stop`。原 `12 s` post-detection observation window 与这条因果/安全链无直接等价关系，现仅保留为 timing warning，不再要求重跑。
- Task 5.3 只 opt-in 使用本机 Nav2 Jazzy `1.3.12` 官方 `navigate_to_pose_w_replanning_and_recovery.xml` 和正式 `behavior_server`；baseline Task 1–5.2 仍使用无 Recovery XML，Planner、Controller、Costmap、AMCL、BT 参数均未改变。
- 行为服务器仅加载官方 Spin、BackUp、Wait；官方 ClearEntireCostmap 仍由 BT 调既有 Costmap service。生命周期 manager 在 recovery profile 下按 `planner_server → controller_server → behavior_server → bt_navigator` 管理，Runner 继续只观察 official manager aggregate active，不增加 per-node lifecycle polling。
- 现有 Gazebo injector 薄扩展为 `SpawnEntity → detection → frozen /clock deadline → DeleteEntity → global Costmap clear`；删除时刻从 Spawn ACK 加固定 `45.0 s` 推导，不订阅 GT、BT、recovery count 或 Planner/Controller，不直接改 Costmap。该 lifetime 是首次人工 discovery 前冻结的环境规则，若 evidence 表明删墙早于真实 Recovery，可在保存 evidence 后只重新冻结一次，不能 runtime 自适应。
- Task 5.3 evaluator 要求 delete 前 Planner/Controller failure、official clear、至少一个 Behavior Server action RUNNING 和 `recovery_count>0`，删除后 Global Costmap clear、安全 replan、Controller 重获 path 后运动、NavigateToPose SUCCESS、无 footprint collision/crossing、final stop 与 durable evidence。继承 localization/scan-TF 偏差仍只作 warning，除非直接破坏感知或安全合同。
- 本轮仅运行 Python/resource 单元测试、`colcon build/test --packages-select resilient_nav_navigation` 和 launch `--show-args`；没有启动 Gazebo、没有运行 Task 5.3 E2E，也没有 commit/push。下一步是一次人工 bounded Task 5.3 discovery。

## 2026-08-30 — Phase 10 Task 5.1 engineering acceptance、Task 5.2 discovery boundary

- `dynamic_obstacle_detour` 的 host fresh record 完整证明了动态环境链：initial Path 后 robot 先行进 `0.2020 m`，官方 SpawnEntity ACK，再由真实 `/scan` 进入 Local/Global ObstacleLayer；Global detection 后首条新 Path 延迟 `0.394 s`，共记录 40 条 safe post-obstacle Path，最后 NavigateToPose SUCCESS、0 Recovery、GT padded footprint 无新增 box 碰撞、Controller/odom/GT final-stop 和 process cleanup 都完整。
- 同一 record 的 `final_gt_position_error_m=0.309411786 m` 高于 inherited Task 4 healthy endpoint 阈值 `0.25 m`。这是 localization estimate 与 evaluator-only GT 的既有偏差观察，不是动态 obstacle 感知、replan、控制或安全失败。Task 5 evaluator 现在保留完整 inherited healthy metrics 和结构化 warning，但动态 Task 5.1 contract 不再把这个 endpoint warning 当成硬失败；没有为此修改 AMCL、Planner、Controller、Costmap、BT 或 Task 3 参数。
- Task 5.2 仍保持 no-Recovery baseline。首条 host run 的 evaluator mode 固定为 `discovery`：要求完整 Spawn/Costmap/GT/cmd_vel/BT evidence，并输出 `MEASURED` 的 Planner-first、Controller-first 或 unclassified 分支；它不接受 candidate code list，也不产生 PASS。只有人工审查 discovery 后，才把一个真实 `frozen_error_code` 写入后续独立 acceptance contract。

## 2026-08-28 — Phase 10 Task 3.3 BT Navigator + NavigateToPose 收口

- 官方 Jazzy `bt_navigator` 使用上游 `navigate_w_replanning_time.xml` 编排唯一的 Planner-owned Global Costmap 与 Controller-owned Local Costmap；BT 中没有 Recovery、Behavior Server 或第二张 Costmap。`RateController=1 Hz` 是周期重规划频率，`bt_loop_duration=10 ms` 和 Planner 的 `expected_planner_frequency=20 Hz` 不是同一合同。
- 初版 probe 在运行期用自身短历史 TF buffer 取 map pose；accelerated Gazebo 下这会触发 past extrapolation，probe 错误 cancel 了健康 `NavigateToPose`。修复后 path sweep 以已验证的 action feedback `current_pose` 为主证据，Plan 与 feedback 必须在冻结 0.50 s 关联窗口内对应（1.0 s 后仍缺失即 FAIL），不再静默漏验；TF 只保留 readiness 与 action 后最终交叉检查，并把历史扩至 60 s。
- fresh detour 与 fresh-restart simple 均 PASS：前者 17 次、后者 9 次 `ComputePathToPose` SUCCESS，均 0 recovery、每条 Path full-footprint sweep zero lethal/unknown、终点与 TF/feedback cross-check 合格并有 Controller 零速证据。独立 Planner 全场景与 Controller simple/detour 回归也通过。
- fresh-process 前必须检查无残留 `gz sim`/`clock_bridge`，并在 probe 发 goal 前证明确有且仅有一个单调 `/clock` publisher。一次 Gazebo server 虽存活但不发布 `/clock`，使 TF/生命周期永远不能 ready；这是环境启动失活而非 Nav2 行为失败，准确清理该实例后重跑通过。

## 2026-08-28 — Phase 10 Task 3.1 Planner Server + ComputePathToPose

- Task 3.1 复用 Task 2 的 Global Costmap 参数，但由官方 `planner_server` 内嵌拥有，不再启动第二个 standalone Global Costmap。这样 standalone Costmap smoke 仍是独立回归，而正式 planner 数据面只有一个 `/global_costmap/global_costmap` owner。
- 首版固定唯一 `GridBased=nav2_navfn_planner::NavfnPlanner`，并显式取 `tolerance=0.0`、`allow_unknown=false`：目的仅是让 exact occupied-goal contract 可判定，不构成算法或性能比较。
- 只读 probe 对成功 Path 同时检查 action、`/plan`、`/is_path_valid` 与 0.025 m 加密 Costmap 采样；绕障的直接线为 37 lethal samples、返回 Path 为零 lethal，避免只凭 RViz 判断。占用目标应选 frozen map 已观测到的障碍**表面** `(5.05, 2.15)`：collision object 的几何中心未必是 map 中 lethal cell。
- 两次 fresh-process run 均通过可达、绕障、`GOAL_OCCUPIED=206` 三场景，robot pose translation 均为 0。没有启动 Controller、BT Navigator、NavigateToPose、recovery 或 `/cmd_vel`，下一步若要让 Path 变成运动必须单独授权和验收。

## 2026-08-28 — Phase 10 Costmap rotation ghost 根因关闭与架构收口

- startup gate、提高 Costmap 频率和 scan/TF 诊断均未能解释“只在转动约四分之一圈、固定角度出现”的孤立短距点，故这些假设不作为正式修复保留。它们帮助缩小范围，但不是根因结论。
- 正反转 fresh-process endpoint 证据均将异常锁定为 `/scan` 的 `beam_index=0`：旧 `angle_min=-135°`、640 beams，短距约 `2.19–2.48 m`，相邻 beam 为正常远距返回。CW 663 个 scan 有 28 个候选、CCW 673 个 scan 有 27 个候选，均为该唯一边界 beam。
- 最小 source 修复将 GPU LiDAR horizontal samples `640→639`，并将 `min_angle` 向内移动旧 increment `pi/426`；`max_angle` 与其余 638 个间隔保持。它不创建 filter、不会把异常 hit 伪装成 clearing，也不修改 frozen map、EKF、AMCL 或 Nav2。
- 修复后 CW 669 与 CCW 645 个 scan 均为 639 beams，marking range 内的孤立短距候选均为 0；用户确认 Global/Local persistent ghost 消失。最终回归保留 Xacro FOV source contract；一次性 gate、rotation diagnostic/contract、专用 launch 和 tests 全部删除。
- 正式 Task 2 回到标准 Lifecycle Manager `autostart=true`，Global `1/1 Hz`、Local `5/2 Hz`。`sensor_frame=lidar_link`、`expected_update_rate=0.2` 与 Global probe 的 `scan.header.stamp` TF 查询保留；`observation_persistence=0.0` 是 Jazzy 默认行为，不再显式配置。
- 剩余边界：异常 ray 由 GPU renderer、Gazebo bridge 还是二者交互产生尚未唯一归因；standalone Costmap Ctrl-C teardown 偶发 `-11` 仍须独立最小复现，且本次结论不自动推广至其他 world/FOV 配置。

## 2026-08-27 — Phase 10 Task 2.3–2.4 Local Rolling Costmap、参数实验与联合验收

- Local Costmap 的职责是 `odom` 下、scan-only 的实时 rolling world model，故只采用官方 `ObstacleLayer` 与 `InflationLayer`，明确不加载 StaticLayer。冻结 `/map` 与 AMCL 的修正属于 `map` frame Global Costmap；混入 Local 会破坏二者分工并使局部窗口依赖定位修正。
- 固定 Local contract 为 `6x6 m`、`0.05 m`、`120x120`、`5/2 Hz`、`track_unknown_space=false`、`/scan` obstacle/raytrace range `2.5/3.0 m`，并复用 Task 2.1 的 8 点 collision-derived footprint 和 `0.01 m` padding。standalone `nav2_costmap_2d` 在本机接受 integer `width/height=6`；浮点 `6.0` 使该 executable 以 `-6` 退出，因此 YAML 以 integer 固定并由静态/运行时 probe 核验。
- rolling probe 不可按 standalone Costmap 的 OccupancyGrid header timestamp 查 TF：该时间戳可滞后窗口更新。probe 改在每个收到的 grid 时以最新 `odom -> base_footprint` 配对，并记录 header stamp 仅供审计。两次有界运动分别得到 `0.4171 m` / `0.2400 m` robot translation，origin-delta error `0.0171 m` / `0.0201 m`。`0.125 m` 固定 tolerance 由发布延迟、受限 smoke 速度及 cell quantization 的上界推导，不从结果反推。
- `ros_gz_sim create` 的命令行 pose 会覆盖 SDF model pose。受控 box 必须显式传 `-x -2.0 -y -3.5 -z 0.5`；若仅依赖文件内部 pose，模型会放在原点，导致错误的“未 marking”观察。显式 pose 下 Local watch cell `0 -> 99 -> 0`，直接证明 marking/clearing。
- parameter experiment 在三个 isolated fresh process 中以 launch-time overrides 完成，未使用 `ros2 param set`。冻结 evaluator 显示 wider `0.75 m` 相对 baseline `0.55 m` 的 extent 增加 `0.2021 m`、area 增加 `0.9500 m²`；同半径 steeper scaling `8` 相对 `3` 将 annulus median cost 从 `38` 降至 `8`，且 extent 不变。因而 range 与 cost falloff 都有机器可读证据，而不是只观察 RViz。
- joint launch 与 complete restart 均确认 Global/Local lifecycle active，Global 保持 frozen map metadata，Local 保持 odom rolling contract，TF ownership 仍唯一为 `AMCL map -> odom -> healthy EKF odom -> base_footprint`。没有启动 planner、controller、BT、recovery 或自主执行链。

## 2026-08-27 — Phase 10 Task 2.1–2.2 Footprint 与 Global Costmap 验收

- 当前 Xacro collision 几何在 `base_footprint` 地面投影的最小可审计 polygon 为 8 点：车体定义 `x=-0.35..0.15 m`、`y=±0.175 m`，驱动轮给出 `y=±0.215 m` 外缘；Costmap 使用该 polygon 和 `0.01 m` padding，而不以 `robot_radius` 近似或修改 URDF。
- `phase10_global_costmap_smoke.launch.py` 受作用域 Include 已有 localization smoke，并在独立 `/global_costmap` namespace 运行官方 `nav2_costmap_2d`、StaticLayer、ObstacleLayer、InflationLayer 和 Lifecycle Manager。Costmap global frame 是 `map`，robot base frame 是 `base_footprint`；不启动 local costmap、planner、controller、recovery 或 `/cmd_vel`。
- fresh-process 动态 probe 确认 Costmap lifecycle active，frozen Phase 9 map 的 metadata 与 costmap 一致，且 38 个 finite scan endpoint 的 static-map residual 为 median `0.014594 m`、P90 `0.026839 m`。这只是一项双墙/整体错位 sanity check，不是定位精度指标。
- 临时、可删除的 Gazebo box 在原先 free cell 使 Costmap lethal 值从 `0` 变为 `99`、相邻 inflation 值从 `0` 变为 `35`；删除模型后两值均回到 `0`。由此直接验证 scan marking、inflation 与 clearing，未写入 Phase 9 map 或 world 资产。
- 独立 `nav2_costmap_2d` executable 在本机不建立 Lifecycle Manager 的 managed-node bond，故 `bond_timeout=0.0`，并坚持以 get-state `active` 而非 manager process 存在作为 readiness 证据。Ctrl-C 时它曾 `-11` 退出；这是 upstream standalone teardown 边界，后续长期运行前应单独最小复现，不能把 active-state evidence 伪称为干净 teardown。

## 2026-08-27 — Phase 10 Task 1.3–1.5 Map Server、AMCL 与 fresh-process 定位验收

- Map-Server-only smoke 直接运行上游 `nav2_map_server` 与只管理 `map_server` 的上游 Lifecycle Manager；动态 probe 已确认 `active`、`/map` frame/size/resolution/origin 与安装态四个 Phase 9 资产 SHA-256 一致。因此 Phase 10 不复制 Map Server，也不让 AMCL 参与静态地图验收。
- 完整 smoke 复用 Phase 4 healthy sensor bridge、Phase 9 world 和既有 healthy EKF；Slam Toolbox 与旧 `odom_tf_broadcaster` 均不启动。TF ownership 的运行期直接 `/tf` evidence 为 AMCL `map -> odom` 271 次、healthy EKF `odom -> base_footprint` 358 次，组合为唯一 `map -> odom -> base_footprint` 链。
- `phase10_initial_pose_helper` 必须容忍 launch_ros 追加的 ROS arguments，不能重复声明 Jazzy 内建 `use_sim_time`，并应等 AMCL active、`/initialpose` subscriber 和仿真时钟开始后再发一条 map-frame pose。该 helper 实际完成显式 `(0,0,0)` 初始化并观察到 map-frame `/amcl_pose`。
- 动态 motion probe 在有界 3 秒弧线期间 PASS：两 lifecycle node active，scan frame `lidar_link`，particle cloud 4 条，AMCL covariance diagonal `0.02186149/0.03976580/0.01485930`。particle cloud 是 volatile 运动时证据，因此 probe 必须在运动前启动；不能在运动结束后订阅却把未收到旧消息误诊为 AMCL 失败。
- `resilient_nav_monitor/system_heartbeat` 仍在 Ctrl-C teardown 后重复调用 `rclpy.shutdown()` 并退出 1。这是既有 Phase 4 teardown 技术债，不改变 active lifecycle 或 localization dynamic evidence，且本轮没有修改无关 monitor 代码。

## 2026-08-27 — Phase 10 Task 1.1–1.2 Nav2 接口冻结与最小定位骨架

- 用户已安装官方 `ros-jazzy-navigation2` 与 `ros-jazzy-nav2-bringup` binary。本机实际核验 `nav2_bringup`、`nav2_map_server`、`nav2_amcl`、`nav2_lifecycle_manager` 均为 Jazzy `1.3.12`，位于 `/opt/ros/jazzy`；Map Server、AMCL 和 Lifecycle Manager 的 executable 均可发现。
- 已直接核验 installed `nav2_bringup/launch/localization_launch.py`：其参数为 `namespace`、`map`、`use_sim_time`、`params_file`、`autostart`、`use_composition`、`container_name`、`use_respawn`、`log_level`。非 composition 模式由上游统一启动 Map Server、AMCL 和 `lifecycle_manager_localization`，且 lifecycle 顺序为 `map_server`、`amcl`；项目 wrapper 必须 Include 该 launch，不复制内部实现。
- 新建 `resilient_nav_navigation`（ament_python）只安装参数、launch、RViz 和静态测试。`phase10_localization.launch.py` 默认消费已安装的 Phase 9 `phase9_map.yaml`，透传全部上游定位参数；默认 `use_sim_time=true`、`autostart=true`、`use_composition=False`（遵从上游 PythonExpression boolean contract）、`use_respawn=false`，不启动 Slam Toolbox/Gazebo/costmap/planner/controller/recovery，不提供导航目标或 `/cmd_vel`。
- Phase 9 资产 identity 已用 occupancy PGM/YAML 与 posegraph 四个 SHA-256 固定。Phase 9 最终总结是 automatic loop closure 的权威依据：最终无 manual-service run 有 `TryCloseLoop accepted → LinkChainToScan → CorrectPoses` 直接链，结论为 PASS。下方 2026-08-21 close-out 的 raw-evidence UNCONFIRMED 记述保留为当时观察记录，但与最终总结冲突；不得再把它当作当前结论，后续应单独整理该历史档案。
- 目标静态验证：package pytest 6 passed；`colcon build --symlink-install --packages-up-to resilient_nav_navigation` 完成 10 个相关包；package-level `colcon test` 为 6 tests、0 errors、0 failures、0 skipped；sourced install 的 wrapper `--show-args` 成功。受限环境默认 `~/.ros/log` 不可写，核验时设置 `ROS_LOG_DIR=/tmp/resilient_nav_phase10_ros_logs`；不是 Nav2 runtime 故障。
- 本轮未动态启动 Map Server/AMCL 或仿真，未验证 TF、scan QoS、lifecycle、`/initialpose` 或定位精度；更没有规划、控制或自主导航结论。

## 2026-08-21 Phase 9 close-out

- Borrow：固定尺度 2D Kabsch/Umeyama SVD alignment（时间关联后求一个 SE(2)）和 TUM ATE 的 timestamp association；Adapt：mapping 使用该 fixed-scale best-fit ATE，但 persisted map 的 `map` 已是固定 global frame，C1/C2/C3 使用实验前声明的固定 `T_map_odom`，把 fresh-odom GT/EKF 直接转到 `map`，不能由当前测试轨迹反拟合；Reject：完整 evo/SE(3)/RPE runtime suite。首帧/origin alignment、world/benchmark 中间变换和 shape diagnostic 已从正式 localization 路径删除。
- 上一份 C3 dynamic record 已作废：同次 launch 参数文件证明实际使用 `map_start_pose=(-3.5,-3.5,0)`，不是 M11 已验证的 different-start map-frame pose `(5.5,4.0,pi)`；其 raster 从约 `227x226` 扩至 `331x384` 是将 scan 放入旧图外的直接一致迹象，best-fit alignment 不得用于掩盖该错误。
- fixed-frame 重跑 corrected C3：Gazebo spawn `(2.0,0.5,pi)`、M8 graph 和 `map_start_pose=(5.5,4.0,pi)`；raw startup map pose 约 `(5.51,4.00,pi)`，确认初始区域正确。正式 transform 直接为 `T_map_odom=(5.5,4.0,pi)`，由 M8 spawn 与不同 start 的已验证相对位姿在采样前声明。SLAM absolute RMSE `0.3434 m/0.1788 rad`、endpoint `3.022 m/1.612 rad`，Healthy EKF `0.4594 m/0.1905 rad`、endpoint `3.879 m/1.623 rad`，故 SLAM 相对 Healthy EKF 为 IMPROVED，但不以未声明阈值宣称高精度 success。旧 C3 的约 pi yaw 在 evaluator 输入前已出现：位置对满足约 `(x,y)_slam=(x,y)_odom+(5.51,4.00)`，但 yaw 仍相差约 pi；因此 position Kabsch 求得约零旋转并把同一零旋转用于 yaw，留下约 pi residual。这是 paired pose frame-semantics 不一致，不是 Kabsch/数值或 double-alignment 错误。运行 `/map` 由 M8 约 `227x226` 扩至 `263x266`、y origin `-4.1332`，说明 localization process 的 runtime raster 输出扩张；它不改写固定 transform，也不能单独证明 initialization 失败。
- 本机 Jazzy Slam Toolbox `2.8.5` / Open Karto 源码表明：每个处理 scan 的 `AddEdges` 先添加 sequential、running-chain 和 nearby-chain edge；自动回环必须走 `FindPossibleLoopClosure -> coarse response/variance -> fine response -> LinkChainToScan -> CorrectPoses`。`graph_visualization` 将全部 mapper edges 画成同一 blue `slam_toolbox_edges`，不能区分 automatic constraint。Karto 的 `FireLoopClosure*` 只分发给 listener，当前 ROS runtime 未注册为 event/log topic。最终重叠走廊路线完整执行（5.58 m 历史段、离开、平行 return、再在历史中段重叠 5.58 m），未调用 manual service；临时开启已有 `debug_logging` 仍无 candidate/coarse/fine/link/correction log，并保留 4 条 TF timestamp/cache scan drop。因此 candidate chain、accepted match/constraint、post-loop optimization 均为 UNCONFIRMED，observable_pose_correction 为 NOT_DIRECTLY_OBSERVED；旧非相邻 edge CONFIRMED 结论已撤销，记录见 `run_c4_loop_closure_evidence.json`。
- `manual_fault_event` runtime failure 的实际根因是测试自身 rclpy probe 在创建前未给只读环境设置 `ROS_LOG_DIR`，而非 ACTIVE/ENDED QoS/DDS race；测试现给 probe 和 subprocess 复用 pytest-owned writable log path，未变更生产节点时序或 QoS。
- workspace 使用根 `.venv` 的 Python、其 site-packages 与既有 `/home/kylian/projects/agent-core` editable path，并以 sequential executor 运行。最新 aggregation 为 `590 tests / 0 errors / 1 failure / 1 skipped`：`test_camera_freeze_runtime` 在 source publisher 的 6 s DDS discovery 窗口内仍无 subscriber；单文件重跑复现同一 discovery failure，未修改、隐藏或跳过。普通系统 Python 的 Agent collection 缺少 Pydantic/agent-core path，不是 production 或本次 Phase 9 代码错误。

## 2026-08-21 Phase 9 healthy SLAM baseline

- Jazzy Slam Toolbox 被 Borrow 为健康 2D LiDAR mapping/localization runtime；TF ownership 保持 healthy EKF 的 `odom -> base_footprint` 与 Slam Toolbox 的 `map -> odom` 分离。
- M8 occupancy map 与 serialized pose graph 已保存、安装并在新进程重载；不同 spawn 的 map-frame 初值必须按建图时 odom 原点解释，不能把 Gazebo world 坐标直接当作 map 坐标。
- Phase 8 Ground Truth bridge/adapter 只复用于 evaluator-only overlay；它不得反馈到 SLAM、Health/Fusion 或 Agent。该初始公开 graph 结论已由同日 close-out 的实际 graph-edge capture 取代。

## 2026-08-15 — RA-1A real API final validation

### 当前事实

- 教师 Agent 的 Windows `.env` 只读映射到本仓库 Git 忽略的本地 `.env`，没有打印、记录或提交 API key。DashScope/Qwen `qwen3.7-flash` 最小 request 和 Robot benchmark 均确认真实网络调用。
- Qwen 模型对 `response_format=json_object` 返回 400。通用 agent-core 现在保持默认 structured-output 行为，但允许 Runtime 显式关闭该可选 capability；Analyzer 仍注入 Pydantic JSON schema 并严格解析。真实 runner 复用教师 Agent 的 `enable_thinking=false` provider option，未覆盖 Runtime-owned request 字段。
- Robot final route 现在注入 `DiagnosisResult` schema；这修复模型缺少字段合同导致的 invalid schema，不引入 CASE/Truth 内容。`BenchmarkCaseResult` 透传 per-case latency，便于审计真实 runs。
- Real smoke CASE-001/002/006/008 为 2/4 passed、component 3/3、fault/top-k 1/3、8 requests、14,677.1 ms、0 leakage/false diagnosis。完整八 Case coverage 为 3/8 passed、component 7/7、fault/top-k 2/7、16 requests、32,647.7 ms、0 leakage/false diagnosis；模型本轮没有选择只读 Tool。

### 问题与处理

- CASE-002/004/005/006/007 的 primary component 正确但 fault label 粒度不符合 fixture alias；记录为 model baseline，不修改 Prompt、Fixture、Truth 或 Scorer。
- 单次 `ra1a_real_benchmark --all` 尝试超过宿主约 30 秒 output window，未得到可审计 stdout；用同一 BatchBenchmarkRunner 分两组运行以取得全部八 Case 结果，并在 Audit 明确该执行限制。

## 2026-08-14 — RA-1A real-model offline validation closeout

### 当前事实

- 新增 `resilient_nav_agent.benchmark.real` 与 `ra1a_real_benchmark`。默认只跑 CASE-001、CASE-002、CASE-006、CASE-008；`--all` 才运行 CASE-001 至 CASE-008。两种报告明确标记 `REAL MODEL BENCHMARK`，不会与 `PIPELINE / FAKE BENCHMARK` 混淆。
- 真实 runner 复用 `OfflineRobotCase.agent_view()`、`BatchBenchmarkRunner`、`run_offline_diagnosis()`、`BenchmarkScorer` 和 agent-core `CompatibleModelClient`。client factory 的形参是 `OfflineAgentInput`，因此 `BenchmarkTruth` 仍只在单次 diagnosis 结束后由 Scorer 消费。
- runner 从 agent-core 既有 `AGENT_CORE_MODEL_*` 读取配置，不硬编码或输出 key；本条历史的配置缺失状态已由 2026-08-15 真实 API 验收取代。
- no-network regression 通过 injected `CompatibleModelClient` 运行 CASE-001 的 Analyzer → Tool → final 三阶段，验证三个 transport request 均为 `stream=False`，并扫描到 0 个 truth marker。Fake reference fixture 或标准答案未改变。

### 当前边界

- 真实 Provider baseline 已在 2026-08-15 记录；模型诊断失误仍只能记为 model baseline，不能对 CASE fixture、Prompt 或 Scorer 写特判。
- Live ROS、rosbag/live 输入、RAG/Memory、Planner、Recovery、Safety Gate action、Nav2 control 和 Multi-Agent 仍未授权。

## 2026-08-13 — RA-1A Offline Diagnosis 闭环收口

### 当前事实

- 新增三个 agent-core `ToolRegistry` Tool：Incident health snapshot、component health comparison 和 metric window inspection。Tool handler 只闭包读取 deep-copied `OfflineDiagnosisContext`，不接 ROS、文件、Shell、BenchmarkTruth 或状态变更接口。
- `run_offline_diagnosis()` 只接受 `OfflineAgentInput`，创建 truth-free context/extension/registry，调用外部 `AgentRuntime`，并保留 Tool records、Core Trace、model request count 和 latency。builder-only `OfflineRobotCase` 传入 Runtime 会直接拒绝。
- Core final answer 当前是 text；Robot service 使用严格 JSON parse、Pydantic `DiagnosisResult` validation、Incident contract check 和 Agent-output leakage scan。plain text、malformed JSON、Schema error、healthy false diagnosis 与受保护输出不会被包装成成功。
- 新增 `OfflineDiagnosisRun`、`BenchmarkCaseResult` 和 `BenchmarkReport`。Scorer 只接收 run 与独立 truth，deterministic 评估 Incident/Diagnosis 期望、primary component、fault alias、top-k、Evidence 引用/类型、可确定 unsupported claim、Tool/model 数、leakage 与 healthy false diagnosis，不再次调用模型。
- `ra1a_fake_benchmark` 对 8 个 reference fixture 运行完整链，明确输出 `PIPELINE / FAKE BENCHMARK`：8/8 passed，component/fault/top-k/evidence validity 均为 1.0，6 次 Tool call、0 Tool failure、22 次 model request、0 leakage、0 false diagnosis；7 个 completed、1 个 no_diagnosis，其余状态为 0。该结果只验证 pipeline，不代表真实模型智能。
- adversarial tests 覆盖 insufficient evidence、malformed final JSON、unknown Tool、invalid Tool arguments/repair failure、partial Tool failure、虚构 Evidence、hint 与 truth wording 不一致、受保护数据提及、healthy false diagnosis，以及 tool/no-tool path。
- Agent Input、Domain/Tool context、Tool output、发送给 completion 的 Agent messages、Tool records、Trace 和 `OfflineDiagnosisRun` 对禁止 token 的回归扫描均为 0；evaluator truth 仍只存在于 Scorer 一侧。
- package pytest、package colcon test 均为 88 项通过。九包构建成功；完整回归首次仅因受限 DDS socket 得到 3 个既有 runtime 环境失败，在允许本机 DDS 的环境只复跑受影响 health/camera 两包后，最终为 491 tests、0 errors、0 failures、1 skipped。

### 问题与处理

- healthy case 如果只用“没有 diagnosed result”判卷，malformed output 也可能误过。Scorer 改为 healthy 必须有严格 `no_diagnosis` result；新增 regression 后 malformed/blocked healthy output 均不能通过。
- 外部 agent-core 0.1.0 没有 final schema validation，这是当前 Core 能力缺口但不阻塞 Robot Domain；本阶段在 service boundary 最小补充 Robot-owned Schema 解析，没有复制 Tool/Trace/Runtime。
- 最小复现确认 `AgentRuntime` 会向 completion 传 `stream=False`，而 public `CompatibleModelClient.complete` 不接受 `stream`；直接组合时 underlying completion 0 次调用，analysis/final 都成为 request error。后续已在 sibling agent-core working tree 以通用 Core 修复、Runtime-owned kwargs 防覆盖和真实 client/runtime/tool integration regression 解决；Robot Domain 没有增加适配层。
- 修复后的无网络 CASE-001 probe 经 `CompatibleModelClient` 完成 Analyzer → Tool → final：3 次 transport call 均为 `stream=False`，strict diagnosis 与 benchmark 通过，Ground Truth leakage 为 0。agent-core 全量 99 项、Robot offline targeted 18 项通过；该 probe 不是 real-model benchmark，Core 变更尚未 commit 或发布。
- 当前环境的 `AGENT_CORE_MODEL_*`、Qwen/DashScope/OpenAI key 及模型配置均不存在，因此没有发起真实 Qwen E2E，也没有重试或请求新秘密。

### 当前边界

- 本阶段只完成 Offline、read-only Diagnosis。没有 Live ROS、rosbag ingestion、RAG、Skills、MCP、Planner、Recovery、Safety Gate action、参数写入或 `/cmd_vel`。
- Fake report 不是 real-model benchmark；真实 Qwen 的 structured-output、Tool usage 和诊断质量仍无结果。
- agent-core client/runtime 参数缺口已在独立 Core working tree 修复并通过 regression；应先 review/发布，再进行真实模型 E2E。

## 2026-08-13 — RA-1A Offline Case Checkpoint

### 当前事实

- 新增 `OfflineAgentInput`、独立 `BenchmarkTruth` 和 builder-only `OfflineRobotCase`；Runtime 只取得 `agent_view()`，不接收 truth envelope。
- `OfflineCaseBuilder` 先把 raw Health 经过既有 Sanitizer 构造成 Health/Evidence/Incident/Agent Input，再单独验证 truth mapping。8 个 `ra1a-reference-v1` case 覆盖 IMU、wheel、scan、camera 和 healthy control，均明确为 reference fixture。
- Prompt/Analyzer 升级为 Robot Diagnosis V1；Analyzer 与 DiagnosisResult 职责分离，当前只提供 `diagnose`、`needs_more_evidence`、`blocked` route。
- Agent Input、Domain context、只读 Tool context groundwork 和 Agent Trace 的禁止 token 扫描均为 0 命中。package pytest/colcon test 为 70 项通过；package build 通过。

### 问题与处理

- 直接运行 `/usr/bin/colcon test` 会用系统 Python 收集测试，而 Pydantic/agent-core 按 Step 1 约定安装在 `.venv`，因此出现 Pydantic import error。改由 `.venv/bin/python -m colcon test` 启动后 70 项全部通过；没有安装系统依赖。

### 当前边界

- 本次按紧急 STOP POINT 没有继续正式 Tools、OfflineDiagnosisRunner、structured final service、Benchmark Scorer/Report/batch 或 Langfuse-style observability，也没有运行 workspace regression。
- 真实 Qwen、Live ROS、ROS Adapter、rosbag parser、RAG、Skills、MCP、Planner 和 Recovery 继续 deferred。

## 2026-08-13 — RA-1A Step 1 Robot Agent Bootstrap

### 当前事实

- 新增第九个 ROS 2 包 `resilient_nav_agent`。它是 `ament_python` 包，但 RA-1A Step 1 的核心只使用纯 Python，不 import `rclpy`、`sensor_msgs`、`nav_msgs`、`geometry_msgs` 或 `diagnostic_msgs`。
- Pydantic v2 Schema 已覆盖 `HealthObservation`、`EvidenceItem`、`RobotIncident`、`DiagnosisHypothesis` 和 `DiagnosisResult`；所有主要模型 `extra="forbid"`，数值拒绝 NaN/Inf，Diagnosis 使用定性 support level 而非伪概率。
- `AgentInputSanitizer` 接受 plain Mapping，递归拒绝 FaultStatus/场景/benchmark truth 和危险字符串。现有 `/faulted/*` source topic 只用于 component normalization，最终 Agent-facing object 不保留 transport topic。
- 真实 `FaultStatus.msg` 字段确认包含 `scenario_id`、`scenario_seed`、`event_id`、`faulted_topic`、`model`、`severity`、`affected_fields` 和 `parameters_yaml` 等真值；全字段 fixture 无法作为 Health 输入通过。
- IMU、wheel、scan、camera 四类当前 `SensorHealth.msg` 形状均由测试 fixture 覆盖。UNKNOWN 的既有 `health_score=-1.0` 哨兵规范化为 `None`；`metric_names[]` / `metric_values[]` 经等长和有限数值检查转换为 dict。
- 最小 Incident builder 只允许 DEGRADED/FAULT，分别映射 warning/fault；HEALTHY/UNKNOWN 不创建故障诊断 Incident。Evidence 使用 canonical source，不保留 ROS topic。
- 仓库外 `Kyliangarnacho/agent-core` sibling 当前提交为 `0dcce13`，包版本 `0.1.0`；实际 editable import 指向 `/home/kylian/projects/agent-core/agent_core/__init__.py`，ResilientNavLab 内没有 `agent_core/` copy。
- 系统 Python 受 PEP 668 管理，因此没有使用 `--break-system-packages`；改用既有 `.gitignore` 覆盖的 `.venv --system-site-packages`。实际 Pydantic 为 `2.13.4`。
- `RobotDomainExtension` 通过外部 `AgentRuntime` 和纯内存 Fake completion 得到合法 `AgentResult`、`diagnose` route 和空 Tool records；没有创建真实模型 client 或调用 API。
- targeted tests 分别为 44、7、5 项通过；包级 pytest 和 colcon 均为 58 项通过。九包构建成功，最终工作空间汇总为 461 项、0 错误、0 失败、1 项跳过。

### 问题与处理

- 首轮 Ground Truth denylist 测试把 fixture secret 写成 `prohibited`，而固定安全错误文案也包含该普通单词，造成 13 个测试误判；改为唯一 fixture secret 后 37 项 targeted tests 全部通过，Sanitizer 逻辑未变。
- 首轮包级测试为 49 项通过、2 项 lint 失败；原因是 import 顺序/未使用 import 和两个 docstring 首词大小写。按 lint 精确建议做机械修复后 51 项全部通过。
- 首次 `colcon test-result` 错把结果路径作为 positional argument；当前 colcon 只接受 `--test-result-base`。改用实际 help 中的参数后得到包级 51/0/0/0 汇总，错误只影响结果查询命令。
- 首次受限沙箱全回归得到 5 个环境失败：DDS `getifaddrs/socket Operation not permitted` 和默认 `~/.ros/log` 只读。设置 `ROS_LOG_DIR=/tmp/resilient_nav_ra1a_ros_logs` 并在允许本机 DDS 的环境仅复跑受影响三包一次后全部通过；没有修改既有 ROS 代码或测试。

### 当前边界

- 本 Step 没有 Live ROS Adapter/subscriber、自动 Incident listener、rosbag Case Builder、正式 Robot Tools、RAG、Skills、MCP、Multi-Agent、Planner、Recovery、`/cmd_vel` 或 ROS 参数写入。
- 本 Step 没有调用真实 Qwen/LLM API，也没有修改 `SensorHealth.msg`、`FaultStatus.msg`、Health Monitor、EKF 或 Fault Injection 逻辑。
- `RobotAnalysis` 和 route 只用于外部 Core bootstrap smoke，不代表完整 Robot Diagnosis Prompt 或策略已完成。

## 2026-08-10 — 阶段 7.2 C920 baseline 与 camera health monitor v1

### 当前事实

- 实机只读执行 `v4l2-ctl --device /dev/video0 --list-ctrls-menus`，记录曝光、gain、白平衡、对焦、brightness、contrast、sharpness 等当前值；未执行控制写操作，完整结果见 `docs/PHASE7_2_CAMERA_CONTROLS_BASELINE.md`。
- `resilient_nav_health_assessment` 新增不依赖 ROS 的 `camera_health_features.py`，对常见 NumPy 灰度、RGB/BGR 和四通道图像输出灰度统计、分位数、暗亮比例、Laplacian 方差、边缘密度、熵、帧差和确定性帧指纹。
- 新增 `camera_health_feature_demo`，自动构造纯黑、纯白、均匀灰、灰度渐变、清晰棋盘、高斯模糊棋盘、重复帧和轻微变化帧，只调用既有特征 API 并打印紧凑对照表，不接入 ROS topic。
- `camera_health_calibrate` 现支持 `scenario_label` 和 `session_id`；每条 CSV 样本及 JSON 都保留场景/session 元数据。同一 baseline 根目录可容纳多个独立 session 子目录，自动 ID 使用 UTC 微秒时间，显式同名 ID 会拒绝启动而不覆盖旧数据。
- 新增纯分析 `camera_health_baseline_report`：扫描多个 session，输出全局和按 `scenario_label` 分组的 FPS、interarrival、max gap、亮度、Laplacian variance、edge density、entropy 和 frame difference 描述统计，同时生成 `baseline_report.json` 与终端表格。
- 读取 5-session `baseline_report.json`：共 3560 帧，observed FPS 为 `8.22--14.43 Hz`，全局 interarrival p95/p99 约 `0.158/0.249 s`，最大正常 gap 约 `0.382 s`。
- 新增 `camera_health_monitor`，默认订阅 `/camera/c920/image_raw`，复用现有特征和阶段 6 的通用 `HealthDecision`/`SensorHealth` 构造，以固定 `5 Hz` 发布 `/health/camera`。
- 初版正式故障为 stale 和 exact-fingerprint freeze；后续加入保守 underexposed/overexposed/blurred/low-information v1。stale 在停止收图时优先；所有候选、确认和恢复均按持续时间，不依赖固定帧数。
- underexposed 同时要求 `mean_gray<=6`、`p95<=8`、`dark_ratio>=0.90`。ACTIVE 开发汇总 `4.035/5.506/0.946` 满足规则；健康黑键盘最低 `p95=10.545` 不满足，健康近黑启动两帧仅持续约 `0.063 s`，再由 `0.6 s` confirmation 排除。
- overexposed 同时要求 `mean_gray>=170`、`p05>=150`、`p95>=180`。ACTIVE 开发汇总 `183.974/175.070/188.091` 满足；健康 baseline 的 mean 最大约 `163.255`、p05 最大约 `64.620`，局部高光产生的高 p95 不足以触发。bright ratio 不进入正式规则。
- blurred 要求近期 reference 同时满足 `Laplacian>=100`、`edge_density>=0.01`，当前同时降到 `<=10`、`<=0.001`，且 `gray_std>=20`、`entropy>=5.5`。blur pre/ACTIVE 为 `309.680/0.028` 与 `6.853/0.000`；低纹理 baseline 无法建立 reference。
- low-information v1 先要求最近 `3.0 s` 内有 `edge_density>=0.01 && entropy>=5.2` 的可用 reference，再要求当前 `edge_density<=0.0005 && entropy<=5.8 && dark_ratio>=0.20 && gray_std>=20`；遮挡 ACTIVE 汇总满足该组合，而健康低纹理场景不能建立 reference。bright ratio 在本次数据中接近零，不进入规则。
- development 参数为 stale `1.0 s`、freeze `2.0 s`、fault confirmation `0.6 s`、recovery `1.0 s`；low-information 复用同一 confirmation/recovery 时间结构。
- `health_score` 表示规则下当前数据健康程度；`confidence` 表示对当前状态判断的确定程度。二者由样本充分度、confirmation/recovery 进度和可解释 evidence 直接计算，不使用机器学习。
- 5 秒真实 C920 验证采集 54 帧、0 次转换错误，observed FPS 约 `11.0`；interarrival p50/p95/p99 约 `0.066/0.189/0.229 s`，max gap 约 `0.264 s`。该结果只验证短时链路，不作为健康阈值。
- 单元测试覆盖纯黑、纯白、纹理、模糊、低纹理、重复帧、不同帧、常见通道布局、浮点输入和错误输入。
- 仅构建 `resilient_nav_health_assessment` 成功；安装区可发现 demo、采集、report 和 monitor。包级测试最终汇总为 125 项、0 错误、0 失败、0 跳过；monitor 新增 15 项覆盖正常流、stale、freeze、stamp 不前进、静止微变化、confirmation、连续 recovery、8--15 Hz 波动、视觉 candidate、配置和 Launch。
- 无相机短时动态验证中，节点正常启动并持续发布 `/health/camera`；超过 development timeout/confirmation 后输出 `state=FAULT`、`detected_fault=stale`、`health_score=0.0`、`confidence=1.0`，Ctrl-C 后 cleanly 退出。
- stale 已由用户在真实 C920 链路完成实机触发与恢复验证；本次 freeze 专项工作没有调整 camera monitor 状态机或参数。
- 新增 `camera_freeze_source`：第一张有效真实 Image 到达后只缓存其完整像素与必要 metadata，默认以 `10 Hz` 向独立 `/test/camera/image_frozen` 发布像素完全相同、当前 ROS stamp 持续前进的副本；源输出同名或频率非法会拒绝启动，未获得有效源图像时不发布。
- 新增纯显示 `camera_health_watch`：在 `state`/`detected_fault` 变化时立即输出，状态不变时默认每 `5 s` 输出；单行包含 score、confidence、message age、rolling FPS、fingerprint identical duration、confirmation 和 recovery，且不产生任何判定。
- 新增 13 项 freeze source/watch/resource 测试；目标包构建成功，安装区可发现两个新入口，完整包级测试最终为 138 项、0 错误、0 失败、0 跳过。
- 新增真正的 freeze runtime 集成测试：测试代码发布有效 `rgb8` Image，经 `camera_freeze_source` 和独立 frozen topic 进入未修改的 camera monitor；验证持续消息、像素一致、stamp 前进、metadata、最终 `FAULT/freeze` 以及全程无 stale 抢占。
- `resilient_nav_fault_injection` 新增 `manual_fault_event`，复用既有 `FaultStatus`，只发布 SCHEDULED/ACTIVE/ENDED 人工真值时间窗，不修改相机或 health 数据；unit/runtime 测试均验证自动状态进展。
- `health_evaluator` 增加默认关闭的 `camera_health_topic`。camera 映射覆盖 stream_stop/freeze/underexposure/overexposure/blur/occlusion；事件结果显式区分 anomaly detected 与 exact classification match，同时保留阶段 6 所有旧字段和默认三传感器集合。
- 真值/evaluator 基础设施完成时包级测试为 `resilient_nav_fault_injection` 142 项和 `resilient_nav_health_assessment` 144 项，均通过。
- `resilient_nav_camera` 新增 `phase7_2_camera_health.launch.py`：原样 Include 阶段 7.1 C920 Launch，并启动 monitor；evaluator/watch 由条件控制，manual event 明确不自动启动。默认 source 为 raw，health 固定发布 `/health/camera`。
- evaluator 审计确认 detection delay、anomaly detected、exact classification 和 TP/FP/FN/TN 已存在；最小补充事件 ENDED 后第一条 HEALTHY 的 recovery time/delay，不改变旧计数语义。
- 联合 Launch runtime 测试关闭 camera 分支，确认 monitor 的 source/config、evaluator 的 camera topic/JSON、watch topic 和 use_sim_time 实际传递；关闭 evaluator/watch 时两个节点均不存在，未访问 `/dev/video0`。
- 本轮最终完整测试为 `resilient_nav_camera` 26 项、`resilient_nav_health_assessment` 146 项，均为 0 错误、0 失败、0 跳过；阶段 7.1 原 Launch/配置测试和阶段 6 测试全部通过。
- `camera_health_calibrate` 增加默认关闭的 truth 模式；开启时仅在特征计算后给样本附加 FaultStatus 标签，并把状态 transition 写入 summary。普通 baseline 新 truth 列为空，既有多 session report 继续通过。
- camera truth 状态变化触发 before/fault/after 三份额外 controls 文件；before 复用启动只读快照，ACTIVE/ENDED 重新执行只读 list controls，未增加 V4L2 写操作。
- 新增纯离线 `camera_fault_feature_report`，用默认 `2.0 s` margin 严格排除 ACTIVE/ENDED 边界两侧样本，输出三个阶段、三个视觉 family 的描述统计与 p05–p95 candidate interval，明确不是阈值。
- 最新 health assessment 构建成功，完整包级测试为 171 项、0 错误、0 失败、0 跳过；truth isolation、underexposure、overexposure、blurred、low-information、stale、freeze 和阶段 6 回归均通过。

### 问题与处理

- 受限设备命名空间内看不到 `/dev/video0`；改在获准的宿主环境执行同一条只读查询后成功取得真实控制状态。
- 初版低纹理测试图每 8 列存在一次灰度回绕，导致离散 Laplacian 方差高于过严的近零预期；确认算法正确后，把测试约束修正为低灰度跨度、低二阶变化且无显著边缘。
- 从工作空间根目录手动运行 flake8 时混入 build/install 和其他包既有告警；在目标包目录复核后定位并修正本次代码的两处 import 顺序和一个未使用导入。
- 首次真实 Ctrl+C 验证已成功落盘，但退出后 logger 尝试向已关闭的 rosout context 发布。修正为 context 有效时才记录落盘日志；重复验证后无该错误，三份文件仍齐全且 `stop_reason=keyboard_interrupt`。
- 当前 C920 画面整体偏暗，既有 `usb_cam` 启动日志显示 runtime brightness=50；两者仅记录为技术债，不在本轮推断因果或调整。运行期快照还会反映自动曝光、增益、白平衡和对焦的变化。新节点调用的 V4L2 命令只有 `--list-ctrls-menus`；文档明确区分驱动/相机自动控制与采集工具只读行为。
- 上一轮 baseline 扩展初次在受限沙箱复跑既有 runtime launch 测试时，DDS 因 `getifaddrs: Operation not permitted` 无法创建参与者，102 项基线表现为 1 项环境失败；当时在获准环境复跑的 110 项及本轮扩展后的 125 项均全部通过，没有为绕过环境限制修改阶段 6 代码或测试。
- monitor 首轮 flake8 定位到两处 import 顺序问题，按仓库规则调整后通过。首轮状态测试还把均匀暗灰图误当作“正常纹理”样本，代码正确生成三个视觉 candidate 并降低 score；测试改用确定性纹理图，并用真正均匀图单独验证视觉 candidate 边界。
- 安装后 `ros2 launch --show-args` 首次因默认 `~/.ros/log` 在受限环境只读而失败；把本次 `ROS_LOG_DIR` 定向到 `/tmp` 后参数解析通过。这不是 Launch 语法错误。
- freeze 专项完整测试首次在受限网络命名空间得到 137 项通过、1 项既有阶段 6 runtime 失败；日志为 `getifaddrs/socket: Operation not permitted`。在允许 DDS 本机通信的环境原样重跑后 138 项全部通过，没有修改阶段 6 测试或逻辑。
- 本次首次把两个包的 flake8 测试放在同一 pytest 进程收集，因同名 `test_flake8.py` 产生 import path mismatch；改为按包目录分别运行，不修改测试。freeze runtime 首轮话题含纯数字 PID token，违反 ROS 2 topic 命名规则；改为 `run_<pid>` 后通过。另移除 runtime 测试中的一个未使用 `Path` 导入。
- 联合 Launch 静态测试首轮只有 3 处双引号不符合包内 flake8 quote 规则；统一为单引号后通过，Launch 实现无需修改。
- 故障 report 测试首轮仅有一行 100 字符超过包内 99 字符限制；拆行后针对性 39 项和完整 160 项均通过。构建时因当前 shell 已包含同一 workspace install 前缀出现 colcon override 提示，但目标包仍成功构建。
- underexposure 首轮完整测试在受限网络命名空间得到 162 项通过、1 项既有阶段 6 runtime 失败；日志为 `getifaddrs/socket: Operation not permitted`。在允许 DDS 本机通信的环境重跑最终 164 项全部通过。另一次从仓库根目录单独调用包内 flake8 时误扫 build/install 和其他包；回到目标包目录及 colcon 测试后本包 lint 通过。
- low-information 首轮完整测试在受限环境得到 168 项通过、2 项 runtime 环境失败：一项因默认 `~/.ros/log` 只读，另一项因 DDS `getifaddrs/socket: Operation not permitted`。将 `ROS_LOG_DIR` 指向 `/tmp` 并在允许本机 DDS 的环境原样复跑后 170 项全部通过；没有修改 runtime 测试或阶段 6 逻辑。

### 当前边界

- 没有修改任何 C920 控制参数或 K/D，没有重复阶段 7.1 相机接入、标定、去畸变或 rosbag 流程。
- 没有实现新的图像修改型故障模型或通用生产阈值；manual event 只产生现有 FaultStatus 标签。C920 K/D 与阶段 6 旧语义未修改。

## 2026-08-09 — 阶段 7.1 C920 相机集成收尾

### 当前事实

- C920 已通过 WSL/USBIP + `usb_cam` 接入 ROS 2；正式链为 `/dev/video0` → `/camera/c920/image_raw`，采用 MJPG、1280×720、15 FPS request 和 `mmap`。
- 旧 K/D 已复用验证；正式 ROS `CameraInfo`、`image_proc` 去畸变至 `/camera/c920/image_rect` 和 `rectification_probe` 均已通过。
- 已录制 `/camera/c920/image_raw`、`/camera/c920/camera_info` 和 `/camera/c920/image_rect`，并完成无相机 rosbag 回放验证。

### 当前技术债

- WSL USB/IP 下仍偶发闪帧、帧率波动和图像偏暗；本阶段只记录，不修改驱动、采集参数或图像处理逻辑。

## 2026-08-09 — 阶段 7.1 C920 CameraInfo 运行时链与可选去畸变准备

### 当前事实

- `c920.launch.py` 新增 `enable_rectification` 参数，默认 `false`，因此现有 `/camera/c920/image_raw` 采集链不变。设为 `true` 时，`image_proc/rectify_node` 在 `/camera/c920` 命名空间订阅 `image_raw` 与同命名空间 CameraInfo，并输出 `/camera/c920/image_rect`。
- 新增 `rectification_probe`：同时订阅 raw、rect 与 `/camera/c920/camera_info`，检查 1280×720、`c920_camera_optical_frame`、非零时间戳、K/D/R/P 与正式 YAML 一致性、raw/rect 同 stamp 配对，以及两路实际接收帧率；它不读取图像 payload、不做视觉算法也不保存图像。
- 包清单声明 `image_proc` 运行依赖，`setup.py` 注册 `rectification_probe`。包级构建成功，19 项自动测试均通过；安装区和 `ros2 launch ... --show-args` 确认 CameraInfo URL 默认值和 `enable_rectification=false`。

### 当前边界

- 当前环境中没有 `/dev/video0`，且 `image_proc` 尚未安装；因此本轮没有启动相机或去畸变节点，不能把静态参数链写成 CameraInfo 已在运行时加载或 `image_rect` 已实际发布。
- 没有安装依赖、修改采集参数、生成新标定、覆盖 raw 图像、保存图像、使用 rosbag 或发布 TF。

## 2026-08-09 — C920 正式 ROS CameraInfo 接入

### 当前事实

- `resilient_nav_camera/config/c920_camera_info.yaml` 新增为 ROS camera_calibration 格式的正式资源：1280×720、`plumb_bob`、旧文件中未改动的 3×3 K 和 5 参数 D、单位 rectification `R` 与 `[K|0]` projection `P`。
- `c920.yaml` 声明 `camera_name=c920` 和包内 `camera_info_url`；`c920.launch.py` 以相同的 `package://resilient_nav_camera/config/c920_camera_info.yaml` 默认值显式传给 `usb_cam`，可通过 Launch 参数覆盖 URL。`/camera/c920/image_raw`、MJPG、1280×720、15 FPS、`mmap` 与 `c920_camera_optical_frame` 均未变。
- 包级构建成功；14 项自动测试通过。安装区 YAML 的 K/D、`R`、`P` 和 `camera_info_url` 已通过只读语义检查；本次没有重跑硬件采集或声称新的动态 CameraInfo 证据。

### 当前边界

- 没有加入 `image_proc`、新 TF、其他 ROS 包或新的标定求解；没有覆盖、生成或保存新的内参。
- C920 长期采集稳定性与 WSL USB/IP/MJPEG 问题仍按既有记录处理，未修改驱动或采集参数。

## 2026-08-09 — C920 旧内参临时复用验证器

### 当前事实

- `resilient_nav_camera` 新增 `calibration_reuse_validator`，可用 `ros2 run resilient_nav_camera calibration_reuse_validator` 启动；它只读 `/tmp/camera_params_old.yaml` 中的 OpenCV `camera_matrix` 与 `dist_coeffs`，不保存、覆盖或更新 K/D。
- 验证器订阅 `/camera/c920/image_raw`，固定使用 5×7 ChArUco、`DICT_5X5_100`、square `0.0288 m` 和 marker `0.0144 m`。它自动拒绝角点不足、连续帧运动过快以及 X/Y/Size/Skew 覆盖重复的候选帧。
- 每个接受 Pose 按排序后的 ChArUco ID 交替确定性划分 pose-fit 和 holdout；只用 pose-fit 与旧 K/D 执行 `solvePnP`，并仅以 holdout 角点计算重投影 RMSE。默认目标为 40 个差异 Pose，运行中输出接受数量、holdout RMSE 和覆盖值，结束时输出误差统计与覆盖范围。
- 所有采样门限都通过 ROS 参数暴露。包级 `colcon build --symlink-install --packages-select resilient_nav_camera` 成功，13 项自动测试均通过；没有运行新的硬件采样验证。

### 当前边界

- 本工具只用于评估旧内参是否可复用，不产生新标定、不覆盖 `CameraInfo`、不使用 `image_proc`、不发布 TF，也不代表已经完成相机标定或真实硬件定位。
- WSL USB/IP dropped buffers、MJPEG decode error 与实际帧率抖动仍只记录，未借本工具修改驱动或采集参数。

## 2026-08-08 — C920 硬件采集与元数据探针基线

### 当前事实

- 新增 `ament_python` 包 `resilient_nav_camera`，工作空间当前共 8 个包。它包含 C920 的 `usb_cam` 参数、`c920.launch.py` 与 `c920_probe`。
- `c920.yaml` 固定 `/dev/video0`、MJPG（`usb_cam` 参数为 `mjpeg2rgb`）、`1280x720`、`15 FPS`、`mmap` 和 `c920_camera_optical_frame` frame ID；没有写入标定 URL 或任何假标定参数。
- Launch 在 `/camera/c920` 命名空间启动 `usb_cam_node_exe`，发布 `/camera/c920/image_raw`，并启动订阅该绝对话题的探针。
- 探针不访问 Image 的 `data` 字段，以 `time.monotonic_ns()` 统计接收帧率及平均/最小/最大帧间隔；它逐帧检查 width、height、encoding、step、frame_id 和 header stamp 是否倒退，每 5 秒输出摘要。
- `colcon build --symlink-install --packages-select resilient_nav_camera` 成功；包级 8 项测试均通过。首次限时硬件启动摘要为 54 帧、`13.260 Hz`、平均/最小/最大帧间隔约 `75.416/32.756/168.427 ms`，图像元数据为 `1280x720`、`rgb8`、step `3840`、`camera_c920`，mismatch 和 stamp regression 均为 0。清理修复后的第二次限时启动输出 36 帧、`9.423 Hz`，探针正常结束且无 traceback。

### 问题与处理

- 首次包级测试的 `ament_flake8` 报告 4 项引号和 import 顺序问题。定位到摘要 f-string 与测试 import 排序后修改，重新构建并测试通过。
- 首次受限沙箱内启动被 ROS 2 Launch 日志目录的只读限制阻断。将本次日志定向到 `/tmp` 后在受限环境外完成限时验证；没有留下 `usb_cam_node_exe`、`c920_probe` 或 Launch 进程。
- `usb_cam` 在未配置标定时按其默认行为查询不存在的默认标定文件；本包没有伪造或提供标定参数。摘要后还出现少量 MJPEG 解码错误，因此当前证据仅证明短时链路、话题和元数据检查可运行，不能作为长期稳定性结论。

### 当前边界

- 本次没有实现标定、`image_proc`、TF、rosbag、相机故障模型或真实硬件定位。
- C920 是独立硬件采集接口；没有接入现有虚拟机器人、Gazebo、EKF、健康评估、自适应融合或容错导航链路。

## 2026-08-05 — 阶段 6 传感器健康评估收尾

### 当前事实

- 新增 `resilient_nav_health_assessment`，工作空间当前共 7 个包；它对 IMU、wheel 和 scan 发布 `SensorHealth`，覆盖 timing/stale/delay、wheel freeze、IMU bias 与 Lidar sector blindness。
- `health_evaluator` 将健康输出与 `FaultStatus` 真值对齐，输出混淆矩阵、检测延迟和分类结果；`phase6_health_evaluation.launch.py` 组合阶段 5/6 链路。
- Lidar 统一链评价为 `event_count=1`、检测延迟约 `0.6 s`、F1 约 `0.96`。2026-08-05 全工作空间构建完成 7 个包，测试为 249 项、0 错误、0 失败、1 项跳过。

### 当前边界

- 统一 Launch 的 `evaluator_output_json` 命令行覆盖仍不视为可靠；`health_evaluator.yaml` 固定 `/tmp/phase6_health_evaluation.json` 作为可运行回退。
- 自适应融合、容错导航、Nav2、SLAM、PointCloud2、RGB-D 故障和真实硬件实验仍未实现。

## 2026-08-03 — 阶段 5 可复现故障注入闭环收尾

### 当前事实

- 新增 `resilient_nav_interfaces` 和 `resilient_nav_fault_injection` 两个包，工作空间当前共 6 个包。
- `FaultStatus` 消息已用于发布故障真值标签，状态覆盖 `SCHEDULED`、`ACTIVE`、`ENDED` 和 `CANCELLED`。
- 阶段 5 首批故障模型包含 IMU bias、Gaussian noise、dropout、fixed delay，wheel odometry freeze，以及 LaserScan sector blindness。
- 新增统一入口 `phase5_fault_injection.launch.py`，支持 `scenario_file`、`use_rviz`、`record_bag` 和 `bag_output`，并 Include 阶段 4 健康链。
- 健康 EKF 继续输出 `/odometry/filtered` 并负责主 `odom -> base_footprint` TF；faulted EKF 订阅 `/faulted/wheel/odometry` 和 `/faulted/imu/data`，输出 `/odometry/faulted`，配置 `publish_tf=false`。
- `fault_probe` 已能输出单行 JSON，覆盖 IMU 差值、dropout、delay、wheel freeze、Lidar NaN 和健康/faulted EKF 差异。
- `phase5_record_bag` 使用场景 ID 与时间戳创建唯一目录；`phase5_replay_bag` 可不启动 Gazebo 回放。
- 收尾执行 `colcon build --symlink-install` 成功完成 6 个包；`colcon test && colcon test-result --verbose` 汇总为 193 项、0 错误、0 失败、1 项跳过。
- 动态验证结果：IMU bias active 平均差 `0.15 rad/s`；wheel freeze active raw 位移约 `0.50886 m`、faulted 位移 `0.0 m`；Lidar blindness NaN 比例约 `0.096997`。
- rosbag 记录目录 `/tmp/phase5_bags/wheel_freeze_ekf_comparison_20260803_013910` 含 12 个要求话题、47962 条消息；无 Gazebo 回放时关键话题可见。
- 最终进程检查未发现 Gazebo、RViz、bridge、EKF、注入器或 bag 进程残留。

### 学习要点

- 阶段 5 的核心边界是保留健康基线不被覆盖：raw topic 和 `/odometry/filtered` 持续存在，故障数据单独进入 `/faulted/*` 和 `/odometry/faulted`。
- faulted EKF 可以作为对照估计存在，但不能驱动主 TF；否则 RViz、RobotModel 和后续导航会混用健康与故障估计。
- 指标工具必须按 `FaultStatus` 活动窗口统计，否则故障前后透传样本会稀释 bias、noise 和 dropout 指标。
- fixed delay 的验证应比较消息 stamp 与接收 ROS 时间；probe 本身也必须使用仿真时间，否则会把墙钟 epoch 混入延迟统计。
- rosbag 记录目录必须包含场景 ID 和时间戳，且脚本要避免覆盖已有记录。

### 问题与处理

- 受限沙箱内启动 ROS 2/Gazebo 时出现 `getifaddrs: Operation not permitted` 和 DDS UDP transport 初始化错误。按权限规则在沙箱外重跑后，统一 Launch、bridge、注入器和 EKF 正常启动。
- `fault_probe` 初版把非活动窗口样本计入 IMU bias 平均值，首轮平均差约 `0.052`，低于配置 `0.15`。修复为默认使用仿真时间并按非 CANCELLED 的 FaultStatus 时间窗过滤，复验得到 active 平均差 `0.15`。
- 阶段 5 三个注入器初版 Ctrl-C 时重复 `rclpy.shutdown()`，退出码为 1。修复为捕获 `KeyboardInterrupt` 并仅在 `rclpy.ok()` 时 shutdown，后续停止 cleanly 退出。
- `phase5_replay_bag` 初版用 `subprocess.call` 包装 rosbag play，Ctrl-C 会打印 Python traceback。改为 `os.execvp()` 直接交给 rosbag 原生命令。
- 既有 `system_heartbeat` 在 Ctrl-C 时仍有重复 shutdown 异常；本次授权范围未修改 `resilient_nav_monitor`，已在阶段 5 总结中如实记录。

### 当前边界

- 阶段 5 已完成故障注入与实验复现闭环，不包含健康评估、自适应融合、容错导航、Nav2、SLAM、PointCloud2 或真实硬件实验。
- RGB-D 图像故障未进入阶段 5 首批模型。
- RViz 配置已提供并可选启动，本次动态验收主要使用 headless 运行，未保存截图证据。

## 2026-08-01 — 阶段 4 收尾状态同步

### 当前事实

- 阶段 4 已完成 IMU、二维 Lidar、RGB-D、传感器 TF、专用 RViz，以及 wheel odometry + IMU 的固定字段 EKF 基线。
- EKF 输出 `/odometry/filtered`，并在完整阶段 4 Launch 中独占 `odom -> base_footprint` TF；阶段 3 默认 `/odom` 与旧 TF broadcaster 保持可用。
- 收尾时完整执行四包 `colcon build --symlink-install` 和 `colcon test`；结果为 4 包构建成功，64 项测试、0 错误、0 失败、1 项跳过。
- 当前 EKF 源码和安装区均为 20 Hz；10 Hz 只作为短时负载诊断，不是最终配置。

### 学习要点

- 仓库只保留代码、技术基线文档和简短阶段状态；阶段结果交接和面向学习的总结材料应由外部流程单独交付。
- `--symlink-install` 的安装资源通常是符号链接，检查安装状态时不能只使用 `find -type f`，还应验证链接目标和路径存在性。
- 诊断性临时参数如果没有保留独立采样日志，只能记录目的和最终恢复状态，不能补造性能结论或把未来阶段内容写成已完成。

### 当前边界

- 截至 2026-08-01，阶段 4 到此完成，下一步计划进入阶段 5。
- 截至 2026-08-01，PointCloud2、真实标定、长期性能、SLAM、Nav2、故障注入、健康评估、自适应融合和容错导航仍未实现；当前状态已由 2026-08-03 阶段 5 记录更新。

## 2026-08-01 — 阶段 4 多传感器与 EKF 技术里程碑

### 当前事实

- 阶段 4 已完成 Gazebo IMU、单层二维 GPU Lidar 和 RGB-D camera，并通过定向 `ros_gz_bridge` 提供 `/imu/data`、`/scan` 及四个相机 Image/CameraInfo 接口；没有 PointCloud2 bridge。
- 新增 `phase4_imu_lidar_demo.launch.py`、`phase4_rgbd_demo.launch.py` 和 `phase4_sensors.rviz`。专用 RViz 以 `odom` 为 Fixed Frame，保留 RobotModel、TF、Best Effort LaserScan、彩色 Image，并新增 filtered odometry。
- 当前环境可发现 `robot_localization` 3.8.3 的 `ekf_node`。新增 `resilient_nav_localization` 包，真实完整入口为 `phase4_ekf_demo.launch.py`。
- 现有 Launch 链新增可透传的 `odom_ros_topic` 与 `start_odom_tf_broadcaster` 参数。阶段 4 完整链将原始 DiffDrive 里程计映射为 `/wheel/odometry` 并关闭旧 broadcaster；阶段 3 的默认 `/odom` 与旧 TF 节点保持可用。
- EKF 采用 `use_sim_time=true`、`two_d_mode=true`、20 Hz、`world_frame=odom`、`base_link_frame=base_footprint`，只融合 wheel `vx` 和 IMU yaw rate，输出 `/odometry/filtered` 并独占 `odom -> base_footprint` TF。
- 完整工作空间四包构建通过；测试汇总为 64 项、0 错误、0 失败、1 项跳过。Xacro、URDF、YAML、XML、Launch/Python 和资源安装均有自动检查。
- 动态直行后 wheel/filtered x 约为 `0.254200/0.253207 m`；旋转后 yaw 约为 `0.851/0.799 rad`。80 个 filtered 样本无 NaN 或明显跳变。
- EKF 20 Hz 配置的仿真时间戳实测为 `20.000 Hz`，墙钟约 `12.470 Hz`，同轮 Gazebo `real_time_factor≈0.6745`。暂停时 wheel、IMU、filtered 均停止，`/clock` 数值冻结；恢复后继续。

### 学习要点

- 原始轮式里程计使用 `/wheel/odometry`、融合结果使用 `/odometry/filtered`，可以明确区分测量与估计，为后续故障注入和对照实验保留稳定边界。
- DiffDrive pose 和 twist 来自同一轮编码来源；最小基线只融合 `vx`，避免无依据地重复融合同一信息。IMU orientation covariance 为零且加速度含重力，所以当前只使用具有非零 covariance 的 yaw rate。
- TF 发布权应随状态估计层切换：阶段 4 关闭旧 broadcaster，让 EKF 独占 odom TF；通过参数化复用而不是删除旧节点，才能保持阶段 3 启动行为不变。
- 仿真中的配置频率、消息仿真时间戳频率和墙钟到达率必须分别记录。低于 1 的 real-time factor 会降低墙钟吞吐，但不等于 sensor 或 EKF 的仿真时间配置失效。

### 问题与处理

- 30 Hz EKF 在同时运行 Gazebo GUI、RViz、RGB-D 和检查订阅时持续报告墙钟更新率偏低。对比诊断、仿真 stamp 与 world stats 后，把基线调整为 20 Hz；最终仿真 stamp 为精确 20 Hz，状态连续。
- 高并发验收时出现过一次 EKF update cycle 超时。输入和输出未中断，降低检查并发后双时钟采样、直行/旋转和连续性均通过，因此记录为主机负载下的瞬时警告，没有篡改 covariance 掩盖。
- RViz 启动首帧曾早于 TF cache，后续订阅与 TF 正常。Ctrl-C 时既有 heartbeat 仍有重复 shutdown 日志；按任务边界没有修改 `resilient_nav_monitor`，最终进程残留检查为空。
- 本次首次 GitHub 发布要求 GitHub CLI；初检时 `gh` 不存在。安装和浏览器认证状态须在发布前再次确认，不能伪造远端成功。

### 当前边界

- PointCloud2、真实编码器/IMU 标定、长期累计误差、map-frame 全局定位、SLAM 和 Nav2 尚未完成。
- 当前 EKF 是固定字段基线，不是故障感知或自适应融合。
- 阶段 4 RViz 未保存像素级截图的证据边界保持不变；外部交接材料不存放在仓库中。

## 2026-08-01 — 阶段 4 依赖安装准备与传感器坐标架构

### 当前事实

- 新增 `scripts/install_phase4_dependencies.sh`，只处理官方 Jazzy 包 `ros-jazzy-robot-localization`；脚本先加载 ROS 2 环境并用 `ros2 pkg prefix` 早退，缺失时要求交互式人工确认，唯一安装命令不带 `-y`。本次只执行 `bash -n`，没有运行脚本、`sudo` 或 `apt`。
- `resilient_nav_robot.urdf.xacro` 新增 IMU、二维 Lidar、相机安装链和未来机械臂安装基准，共六个 link 和六个 fixed joint；所有 xyz/rpy 由 Xacro property 集中管理。
- `camera_link` 的 +x 朝机器人前方，`camera_optical_frame` 使用 `[-pi/2, 0, -pi/2]` 固定旋转，形成 +x 右、+y 下、+z 前的 ROS optical frame。
- IMU、Lidar、相机支架和相机本体具有简单 visual；新增 link 没有 collision、inertial、Gazebo sensor 或新 plugin，`arm_mount_link` 只是空安装基准。
- 静态测试扩展为 12 个 pytest case，覆盖六个 link/joint、父子关系、外参、标准 optical rotation、visual-only 边界、禁止 sensor/新 plugin 及第三阶段底盘参数回归。
- 源码与安装后 Xacro 均可展开，`check_urdf` 成功。描述包构建成功，包级结果为 13 项、0 错误、0 失败、0 跳过；工作空间累计为 47 项、0 错误、0 失败、1 项既有跳过。

### 学习要点

- 把 `camera_mount_link` 与 `camera_link` 分层，可以让未来支架/云台变化整体作用于相机子树，同时保留相机本体与 optical frame 的标准轴约定。
- 安装坐标可以先于传感器插件建立，但必须把“存在 TF 基准”和“已经产生传感器数据”严格区分。
- 新增纯 visual 的 fixed link 不需要 collision 或 inertial；同时用回归测试锁定车体质量、惯性、轮径、轮距和插件，可避免阶段 4 准备工作改变阶段 3 动力学。
- 依赖安装脚本应先检查 ROS 包是否已发现，并在唯一目标包和唯一安装命令前设置显式人工确认，避免把依赖准备扩大成系统升级。

### 问题与处理

- 源码 Xacro 与安装后 Xacro 的展开文件直接执行 `cmp` 时在第 3 行不同。文本 diff 定位到 Xacro 自动生成注释记录了不同输入路径；删除自动生成注释后两份展开内容完全一致，且两份都通过 `check_urdf`。因此这是来源路径注释差异，不是安装产物陈旧或机器人语义不同。
- 在仓库根目录汇总测试时，首次向 `colcon test-result` 传入相对 `build/...`，工具因当前目录不是 `ros2_ws` 而报告路径不存在。改用 `/home/kylian/projects/resilient_nav_lab/ros2_ws/build/...` 绝对路径后，包级 13 项和工作空间累计 47 项结果均成功读取；该错误只影响结果查询命令，不影响先前已完成的构建或测试。

### 当前边界

- 没有安装 `robot_localization`，没有 Gazebo sensor、bridge、EKF 或机械臂模型。
- 没有启动 Gazebo 或 RViz；新增 visual 和运行时固定 TF 尚未动态观察。
- 尚未验证传感器消息、frame_id、频率、QoS、时间戳、噪声、同步或定位结果。

## 2026-07-29 — 阶段 3 收尾

### 当前事实

- 阶段 3 已按“虚拟差速机器人与基础运动”边界完成，正式总结见 `docs/PHASE3_SUMMARY.md`。
- 收尾执行 `colcon build --symlink-install`，`resilient_nav_description`、`resilient_nav_monitor` 和 `resilient_nav_simulation` 共 3 个包全部成功。
- 收尾执行 `colcon test` 和 `colcon test-result --verbose`，汇总为 43 项、0 错误、0 失败、1 项按既有配置跳过。
- 根据精简收尾要求，保留已经记录的机器人生成、直行、原地旋转、圆弧、停车、标准 ROS 2 话题、TF 和 Gazebo/RViz 同步结果，没有重复运行完整图形仿真或运动流程。
- 最终文档记录了完整 Demo、仅 Spawn、三种运动工具以及快速构建/测试命令。
- 收尾停止了本次 Gazebo/RViz Launch，进程检查没有发现相关残留；删除了本次隔离 ROS 日志、对应 Launch 参数文件和源码树 Python 缓存。
- 本次收尾没有修改功能代码。

### 学习要点

- 阶段收尾应区分“已完成的动态验收”和“收尾时重复执行的检查”；保留可追溯结果并只重跑快速自动化验证，可以避免把重复操作误写成新的实验。
- 完成阶段状态更新时，需要同步 README、范围、环境、学习日志、协作说明和正式总结，避免“进行中”“已完成”并存。
- 临时文件清理应限定到本次会话可确认的日志、参数和缓存，不能因为目录中存在旧文件就推断其都可删除。

### 当前边界

- 阶段 3 完成结论限于低速、短时、平地差速运动基线。
- 完整运动性能、传感器、`ros2_control`、Nav2、SLAM、定位和后续容错导航能力仍需单独授权。
- 既有 `system_heartbeat` 在整套 Launch Ctrl-C 时的重复 shutdown 警告未在本次收尾修改；它不影响已记录验收，且没有留下进程。

## 2026-07-29 — 阶段 3 运动测试工具与三模式验收

### 当前事实

- `resilient_nav_simulation` 新增 `motion_test` 可执行工具，支持直行、原地旋转和圆弧，线速度、角速度与持续时间可配置。
- 工具以 `20 Hz` 发布 `/cmd_vel`，启动前等待命令订阅者；正常结束、异常和 Ctrl-C 均通过同一清理路径重复发送 5 条零 Twist。
- 单元测试覆盖三种模式映射、参数校验、正常结束和模拟 Ctrl-C 自动停车；真实 Ctrl-C 后连续两次 `/odom` twist 也均为零。
- 初次旋转和圆弧测试发现 Gazebo 模型与 `/odom`/TF 存在约厘米级系统性偏差。对照几何后确认，DiffDrive 里程计以轮轴中点积分，而旧 `base_footprint` 位于其后方 `0.10 m`。
- `base_footprint` 已移到左右驱动轮轴中点，`base_link` 相对它为 `[-0.10, 0, 0.15] m`；物理几何、接触点和重心之间的相对关系不变。
- DiffDrive 新增显式 `frame_id=odom` 和 `child_frame_id=base_footprint`。实际 `/odom` frame 字段、ROS 侧 TF 与描述树现在使用同一语义。
- 修正后三组 `1.5 s` 最终 `/odom` `(x, y, yaw)` 分别为：直行 `(0.276800, 0.000000, 0.000)`，原地旋转 `(0.000000, 0.000000, 0.832)`，圆弧 `(0.261344, 0.073738, 0.550)`。
- 对应 Gazebo 位姿分别为 `(0.276159, 0.000000, 0.000)`、`(-0.005241, 0.002314, 0.780)` 和 `(0.254261, 0.069973, 0.525)`；最大位置差约 `8 mm`，最大航向差约 `0.052 rad`。
- 三组停止后的 `/odom` twist 均为零，连续 TF 样本保持不变；RViz 完成 OpenGL 4.5 初始化并通过内部 listener 订阅 `/tf`、`/tf_static`，RobotModel 订阅 `/robot_description`。
- 最终相关三包构建成功，测试汇总为 43 项、0 错误、0 失败、1 项按既有配置跳过；结束后没有相关后台进程残留。

### 学习要点

- 差速轮式里程计的参考点应与 `base_footprint` 一致。只让 frame 名称一致而物理原点不同，会在直行时隐藏问题，却在旋转和圆弧时形成确定性的轨迹偏差。
- `base_link` 可以继续表示车体几何中心，`base_footprint` 则表示地面上的轮轴参考点；二者通过固定 TF 表达偏移，比在 TF 广播节点中补偿运动位姿更清晰。
- 自动停车不能只依赖发布一次零速度。让清理路径在正常、异常和 Ctrl-C 下统一执行并短间隔重复发布，可以覆盖 DDS/bridge 的最终消息交付。
- RViz 同步应同时检查进程初始化、RobotModel 描述订阅、TF listener 订阅和数值 TF，而不只依据窗口是否打开。

### 当前边界

- 当前结果只验证给定低速、短时、平地命令，不代表速度精度、长距离里程计、轨迹跟踪、高速急停或控制鲁棒性已验收。
- Gazebo 物理位姿与轮式里程计仍有毫米级位置和小角度航向差；当前如实记录，没有引入传感器或额外定位来源去校正。
- 没有增加传感器、`ros2_control`、Nav2、SLAM 或新软件依赖。既有 `system_heartbeat` Ctrl-C 重复 shutdown 警告仍未在本任务中处理。

## 2026-07-29 — 阶段 3 Gazebo 与 RViz 联合演示

### 当前事实

- 新增 `phase3_demo.launch.py`，Include 现有 `phase3_spawn.launch.py` 并通过 `use_rviz` 条件启动一个 RViz；Gazebo、bridge、`robot_state_publisher` 和 `odom_tf_broadcaster` 没有重复定义。
- Demo 透传实体名和初始位姿参数；`use_rviz` 默认开启，关闭时仍运行相同 Spawn 链。
- 专用 `phase3_demo.rviz` 以 `odom` 为 Fixed Frame，启用 Grid、RobotModel 和 TF；Odometry 显示绑定 `/odom`，默认关闭供用户按需启用。
- `resilient_nav_simulation` 新增 `rviz2` 运行依赖和 RViz 资源安装规则。
- 相关 3 个包构建成功；测试汇总为 30 项、0 错误、0 失败、1 项跳过。
- 实际 Demo Launch 报告机器人生成成功，RViz 完成 OpenGL 4.5 初始化且未报告配置或 TF 错误；ROS 图只有一个 `robot_state_publisher`，没有独立关节状态发布器。
- 向 `/cmd_vel` 以 `0.2 m/s`、10 Hz 发送 15 条消息后立即发送零 Twist。Gazebo 模型 X 从 `0.000000 m` 移至 `0.466559 m`。
- 停止后 `/odom` X 为 `0.467200 m` 且 twist 全零，`odom -> base_footprint` TF X 为 `0.467 m`；RViz RobotModel 与 TF listener 正在消费现有描述和 TF 链，三侧位姿在毫米级一致。
- 验证结束后发送了明确停止命令，并停止整个 Launch；进程检查没有发现 Gazebo、RViz、bridge 或状态发布节点残留。

### 学习要点

- 联合演示应 Include 已验收的启动链，而不是复用会自行创建状态发布节点的独立 Display Launch；这样可以避免同名节点、重复 `/joint_states` 和重复 TF。
- 使用 `odom` 作为 RViz Fixed Frame 后，RobotModel 通过 `odom -> base_footprint -> ...` TF 链显示实际平移，而不是始终固定在机器人根坐标系原点。
- RViz 是否同步可以通过数据所有权和数值闭环验证：RobotModel 消费现有描述与 TF，同时 Gazebo 模型位姿、`/odom` 和 RViz 所用 TF 应在允许误差内一致。

### 当前边界

- `/odom` 显示只是已有轮式里程计的可选可视化，不是新增传感器。
- 没有加入传感器、`ros2_control`、额外 bridge、关节状态发布器或第二个 `robot_state_publisher`。
- Ctrl-C 时 RViz 和本次新增链路正常退出；既有 `system_heartbeat` 仍出现 `rcl_shutdown already called`，不影响联合演示验收。

## 2026-07-28 — 阶段 3 ROS odom TF 广播

### 当前事实

- `resilient_nav_monitor` 新增 `odom_tf_broadcaster`，订阅 `/odom` 并把消息位姿发布为 `odom -> base_footprint` 动态 TF；每条 TF 沿用对应里程计消息的 `header.stamp`。
- 包清单新增 `geometry_msgs`、`nav_msgs` 和 `tf2_ros` 依赖，`setup.py` 注册同名可执行入口。
- `phase3_spawn.launch.py` 在不改变启动命令的情况下启动新节点，并设置 `use_sim_time=true`。
- Gazebo 原生 TF/位姿输出没有加入 bridge；静态测试继续禁止 `gz.msgs.Pose_V`，避免与 ROS 侧广播节点形成重复 TF 来源。
- `colcon build --symlink-install --packages-up-to resilient_nav_simulation` 成功完成 3 个包；相关测试汇总为 28 项、0 错误、0 失败、1 项跳过。
- 实际 Launch 报告机器人实体创建成功；ROS 参数读取确认 `odom_tf_broadcaster` 的 `use_sim_time=True`。
- `/odom` 实测约为 45–47 Hz；`tf2_echo odom base_footprint` 连续输出时间为 `30.12`、`31.00`、`31.86`、`32.76`、`33.66 s` 的有效变换。
- `/tf` 只有 `robot_state_publisher` 和 `odom_tf_broadcaster` 两个发布者；`robot_bridge` 只处理 `/cmd_vel`、`/odom` 与 `/joint_states`，没有 TF 接口。

### 学习要点

- 由 `/odom` 消息生成 TF 时，应复制消息时间戳而不是读取回调时刻，才能让位姿和 TF 保持同一时间基准。
- Gazebo DiffDrive 的原生位姿输出与 ROS 侧 odom TF 是两种可选来源；当前只保留后者，可以明确所有权并避免同一变换重复发布。
- `use_sim_time` 是 ROS 节点参数；本节点虽然直接沿用消息时间戳，仍在阶段 3 Launch 中显式启用该参数，使节点时钟行为与整条仿真链一致。

### 当前边界

- 只发布 `odom -> base_footprint`，没有桥接 Gazebo 自带 TF，也没有增加传感器或 `ros2_control`。
- 当前验证覆盖静止机器人下的持续 TF 发布，没有扩展阶段 3 的运动性能范围。
- Ctrl-C 停止时新节点正常退出；既有 `system_heartbeat` 仍观察到 `rcl_shutdown already called`，不影响本次 TF 验收。

## 2026-07-28 — 阶段 3 纵向支撑与重心修正

### 当前事实

- 修正前 URDF 转换后的整机纵向重心约为 `x=-0.0058 m`，后球轮与驱动轮轴形成的支撑范围为 `[-0.18, 0] m`，重心距前支撑边仅约 `0.006 m`。
- 车体原点离地 `0.15 m`、碰撞盒高度 `0.15 m`，水平时底部离地 `0.075 m`；驱动轮和球形支撑轮的接地高度一致，因此没有修改车体高度、碰撞尺寸、轮径或轮距。
- 驱动轮轴从 `x=0` 前移到 `x=0.10 m`，球形支撑轮从 `x=-0.18 m` 后移到 `x=-0.20 m`，车体惯性原点从 `[0, 0, 0]` 调整为 `[-0.05, 0, -0.04] m`。
- 修正后展开 SDF 的车体/支撑轮合并惯性原点约为 `[-0.0558, 0, 0.1077] m`，整机纵向重心约为 `x=-0.031 m`，前后支撑余量均超过 `0.10 m`。
- Xacro、`check_urdf`、URDF 到 SDF 转换和 16 项聚焦静态测试通过；DiffDrive 轮径、轮距、Gazebo 话题与 ROS bridge 映射保持原值。
- `colcon build --symlink-install --packages-up-to resilient_nav_simulation` 成功完成 3 个包；描述包 8 项和仿真包 8 项测试通过，工作空间汇总为 23 项、0 错误、0 失败、1 项跳过。
- 实际 Launch 中机器人生成成功；落地静止时连续两次位姿的 roll/pitch/yaw 均约为零。
- 相同短时直行/停止序列后，模型 X 约为 `0.403759 m`，停止后连续两次 pitch 约为 `-0.000001 rad`，满足 `|pitch| < 0.05 rad`。
- `/odom` 停止速度为零，`/joint_states` 左右轮反馈正常；验证停止后没有后台进程残留。

### 学习要点

- 两轮差速机器人加单后支撑轮时，纵向重心不能只看车体几何中心；它必须在后支撑点与驱动轮轴构成的支撑区内部保留足够动态余量。
- 修正前落地静止虽然水平，但重心几乎位于前支撑边，驱动/制动后车体前缘会成为新的稳定接触点，形成约 `0.31 rad` 的残余 pitch。
- 同时前移驱动轮、后移支撑轮并降低/后移车体惯性原点，可以扩大有效支撑区并降低俯仰力矩，而无需改变轮径、轮距或通信接口。
- 对固定连接 link 应检查 URDF 转 SDF 后的合并惯性；仅查看各 URDF link 的局部 inertial origin 容易忽略转换后的整机重心。

### 当前边界

- 当前只验收水平落地和一次短时直行/停止后的 `|pitch| < 0.05 rad`，不代表更高速度、急停、倒车、转向或坡面姿态已经验证。
- 没有加入 odom TF、传感器、`ros2_control` 或新的 Gazebo/ROS 话题。
- Ctrl-C 停止时仍观察到既有 `system_heartbeat` 的 `rcl_shutdown already called`，不影响本次动力学验收。

## 2026-07-28 — 阶段 3 ROS 基础运动 bridge 与短时直行

### 当前事实

- `phase3_spawn.launch.py` 新增 `robot_bridge`，保持原启动命令不变，并继续通过阶段 2 Launch 保留 `/clock` bridge。
- ROS 2 `/cmd_vel` 单向桥接到 Gazebo `/model/resilient_nav_robot/cmd_vel`；Gazebo odometry 和 joint state 分别单向桥接并重映射到 ROS 2 `/odom` 与 `/joint_states`。
- bridge 话题随 `entity_name` 动态构造，非默认 Gazebo 实体名不会改变三个标准 ROS 2 话题名称。
- 包清单补充 `geometry_msgs`、`nav_msgs` 和 `sensor_msgs` 运行依赖。
- `colcon build --symlink-install --packages-up-to resilient_nav_simulation` 成功完成 3 个包；描述包 7 项和仿真包 8 项测试全部通过，工作空间汇总为 22 项、0 错误、0 失败、1 项跳过。
- 实际 Launch 报告三条 bridge 的方向和消息类型符合设计，机器人实体创建成功。
- 短时发送 `linear.x=0.2 m/s` 后发送全零 Twist，模型 X 从约 `0.000000 m` 移至 `0.549754 m`，Y 和 yaw 仍约为零。
- 停止后两次 Gazebo 位姿一致，`/odom` 报告 X 约 `0.5494 m` 且 twist 全零。
- `/joint_states` 同时包含左右轮关节，停止后的关节速度接近零；验证结束后没有后台进程残留。

### 学习要点

- `parameter_bridge` 的 `]` 可限定 ROS→Gazebo，`[` 可限定 Gazebo→ROS；对控制命令和反馈分别限定方向，避免不必要的双向回环。
- Launch 的 remapping 只改变 ROS 2 侧名称，因此可以保留 Gazebo 的模型/世界作用域话题，同时向 ROS 节点提供标准 `/cmd_vel`、`/odom` 和 `/joint_states`。
- `robot_state_publisher` 消费桥接后的 `/joint_states`，因此运行时 `/tf` 会更新轮关节变换，但这不等于发布了 odom TF。
- ROS CLI 自动发现 `/joint_states` 类型时曾受发现缓存影响；显式指定 `sensor_msgs/msg/JointState` 后成功读取实际消息。

### 当前边界

- 没有桥接 DiffDrive 的 Gazebo `/model/resilient_nav_robot/tf`，抽查 ROS `/tf` 仅见车体到左右轮的关节变换，没有 odom 到基座变换。
- 直行停止后的模型 pitch 约为 `0.309682 rad`；基础移动已确认，但支撑轮、姿态稳定性和动力学参数尚未调优。
- 当前只做一次短时直行和停止，不代表速度精度、转向、轨迹跟踪或控制鲁棒性已经验收。
- 没有加入传感器、`ros2_control`、Nav2、SLAM 或后续阶段能力。
- Ctrl-C 停止时仍观察到既有 `system_heartbeat` 的 `rcl_shutdown already called`，不影响本次 bridge 与运动验收。

## 2026-07-28 — 阶段 3 Gazebo 原生差速与关节状态插件

### 当前事实

- `resilient_nav_robot.urdf.xacro` 新增 Gazebo Harmonic `DiffDrive` 和 `JointStatePublisher` 系统插件，使用当前 `gz-*` 文件名和 `gz::sim::systems::*` 类名。
- DiffDrive 使用真实关节 `left_wheel_joint`、`right_wheel_joint`，轮距 `0.39 m`，轮半径 `0.10 m`。
- 默认 Gazebo Transport 话题明确为 `/model/resilient_nav_robot/cmd_vel`、`/model/resilient_nav_robot/odometry` 和 `/world/resilient_lab/model/resilient_nav_robot/joint_state`；自定义 `entity_name` 会同步替换话题中的模型名。
- Xacro 展开、`check_urdf` 和 URDF 到 SDF 转换通过；转换后的 SDF 保留两个插件及预期参数。
- `colcon build --symlink-install --packages-up-to resilient_nav_simulation` 成功完成 3 个包，描述包和仿真包共 14 项静态测试全部通过。
- 实际 Launch 报告实体创建成功，模型列表包含 `resilient_nav_robot`，落地 Z 位姿约为 `-0.000001 m`。
- `gz topic` 验证速度话题有 `gz.msgs.Twist` 订阅者，里程计有 `gz.msgs.Odometry` 发布者，关节状态有 `gz.msgs.Model` 发布者；实际关节状态消息包含左右轮关节。
- 现有 bridge 配置保持只桥接 `/clock`，本任务没有新增 ROS—Gazebo bridge、ROS 2 odom TF、传感器或 `ros2_control`。
- 验证停止后没有 Gazebo 或 ROS 2 后台进程残留。

### 学习要点

- 轮距应使用左右轮心之间的距离；当前 `wheel_y=0.195 m`，因此 DiffDrive 的 `wheel_separation=0.39 m`，不是车体宽度 `0.35 m`。
- JointStatePublisher 的显式 `<topic>` 能保留世界/模型作用域的 Gazebo 原生名称，重复 `<joint_name>` 可把发布内容限定为两个可动轮关节。
- 将 Launch 的 `entity_name` 传入 Xacro，可避免实体重命名后插件话题仍硬编码为默认模型名。
- Gazebo Transport 原生话题出现不代表 ROS 2 已获得对应数据；当前只有 `/clock` 经过 bridge。

### 当前边界

- 本次没有向 `cmd_vel` 发送命令，因此尚未验收行驶距离、转向方向、速度限制或轨迹精度。
- DiffDrive 自带的 Gazebo Transport 位姿输出没有桥接到 ROS 2，也没有接入 ROS TF 树。
- Display Launch 的 ROS 2 `/joint_states` 与 Gazebo 原生关节状态仍是独立链路。
- Ctrl-C 停止时仍观察到既有 `system_heartbeat` 的 `rcl_shutdown already called`，不影响本次插件和话题验收。

## 2026-07-28 — 阶段 3 Gazebo 无驱动物理落地

### 当前事实

- `resilient_nav_robot.urdf.xacro` 为车体、左右轮和球形支撑轮增加 Gazebo 命名材质及 `mu1`、`mu2` 接触摩擦。
- 左右轮摩擦设为 `1.0`，车体为 `0.5`，球形支撑轮为 `0.05`；阶段 2 地面碰撞增加显式 ODE 摩擦 `mu=1.0`、`mu2=1.0`。
- `resilient_nav_simulation` 新增 `phase3_spawn.launch.py`，通过 Include 复用 `phase2_world.launch.py`，并用 `robot_state_publisher` 和 `ros_gz_sim create` 从 `/robot_description` 创建实体。
- 默认实体名为 `resilient_nav_robot`，默认从 `z=0.25 m` 生成；初始 X、Y、Z 和 yaw 均可通过 Launch 参数配置，重复名称不允许自动重命名。
- Xacro、`check_urdf`、世界 SDF 和 URDF 到 SDF 的转换验证通过；转换后的四个实体碰撞均保留预期摩擦值和 Gazebo 材质。
- `colcon build --symlink-install --packages-up-to resilient_nav_simulation` 成功完成 3 个包。
- 描述包新增 4 项静态测试，仿真包扩展为 7 项静态测试，两包共 11 项全部通过。
- 实际 Launch 中 `ros_gz_sim create` 报告实体创建成功；Gazebo 模型列表包含阶段 2 三个静态模型和新机器人。
- 默认高度下落并稳定后，模型 XYZ 约为 `[-0.000000, 0.000000, -0.000001] m`，RPY 约为零；验证结束后没有后台进程残留。

### 学习要点

- 本机 Gazebo URDF 转换器可把 `<gazebo reference>` 中的 `mu1`、`mu2` 转换成 SDF ODE 接触摩擦。
- `<gazebo reference>` 中使用 `Gazebo/Blue` 等命名材质可生成带材质脚本的 SDF；嵌套 RGBA 材质会触发缺少字符串值的转换警告，因此 RViz 颜色继续由 URDF material 定义，Gazebo 覆盖使用命名材质。
- 将生成 Launch 放在 simulation 包并 Include 阶段 2 Launch，可以复用已验收世界、时钟桥和心跳链路，同时让机器人几何与物理参数继续归 description 包维护。
- `base_footprint` 在 URDF 到 SDF 转换时会吸收固定连接的车体和支撑轮；左右 continuous 轮关节保留为可动关节，不需要驱动插件也能完成重力和接触测试。

### 当前边界

- 当前只验证无驱动实体生成、自由落体和接触稳定，没有加入 Gazebo JointStatePublisher、DiffDrive、传感器、`ros2_control` 或任何控制命令。
- `robot_state_publisher` 当前只提供描述和固定 TF，Gazebo 中的轮关节状态尚未桥接回 ROS 2。
- 人工 Ctrl-C 停止包含阶段 2 心跳的 Launch 时，`system_heartbeat` 仍会报告一次 `rcl_shutdown already called`；这不影响本次生成与落地结论，且停止后无进程残留，后续可在监控节点维护任务中处理。

## 2026-07-28 — 阶段 3 RViz 显示与运行时 TF

### 当前事实

- `resilient_nav_description` 新增 `launch/display.launch.py` 和 `rviz/display.rviz`，CMake 安装规则同步覆盖 `launch/`、`rviz/` 和 `urdf/`。
- 包清单新增 `joint_state_publisher`、`joint_state_publisher_gui`、`launch`、`launch_ros`、`robot_state_publisher`、`rviz2` 和 `xacro` 运行依赖。
- Display Launch 从 Xacro 生成 `robot_description`，始终启动 `robot_state_publisher` 和 RViz。
- `use_gui:=false` 启动普通 `joint_state_publisher`；`use_gui:=true` 启动 `joint_state_publisher_gui`。
- `colcon build --symlink-install --packages-select resilient_nav_description` 成功完成 1 个包，安装空间包含 Launch、RViz 和 Xacro 资源。
- 两种 `use_gui` 分支均完成限时启动验证，RViz 成功初始化 OpenGL 4.5。
- 验证期间可见 `/joint_state_publisher`、`/robot_state_publisher` 和 `/rviz`，且 `/joint_states`、`/robot_description`、`/tf`、`/tf_static` 消息类型正确。
- `tf2_echo` 确认 `base_footprint` 到 `base_link` 的固定变换为 Z 轴 `0.150 m`。

### 学习要点

- 用 Launch 的 `IfCondition` 和 `UnlessCondition` 可以让普通与 GUI 关节状态发布器互斥，避免两个节点同时发布同一关节状态。
- `robot_description` 由 Launch 调用安装后的 Xacro 动态生成，可避免维护重复的展开 URDF。
- RViz 的 RobotModel 从 `/robot_description` 读取模型，TF 显示则用于检查 link 树是否随关节状态正确更新。

### 当前边界

- 当前关节状态来自独立 ROS 2 发布器，不是 Gazebo 仿真关节反馈。
- 没有加入 Gazebo 插件、传感器、`ros2_control`、差速命令或控制功能。
- 在该子任务验收时，模型尚未生成到 Gazebo；后续无驱动生成与落地结果见本日志更新的阶段 3 条目。

## 2026-07-28 — 阶段 3 基础差速机器人描述

### 当前事实

- 已创建 `ament_cmake` 包 `resilient_nav_description`，并安装包内 `urdf/` 描述资源。
- `resilient_nav_robot.urdf.xacro` 定义 `base_footprint`、`base_link`、左右驱动轮和一个球形支撑轮。
- `base_link`、驱动轮和支撑轮均包含基础几何 visual、collision、质量和惯性张量；左右驱动轮使用 continuous 关节，支撑轮当前使用 fixed 关节。
- 源码 Xacro 成功展开为 URDF，`check_urdf` 成功解析出以 `base_footprint` 为根的 5 个 link 和 4 个 joint。
- `colcon build --symlink-install --packages-select resilient_nav_description` 成功完成 1 个包。
- 加载工作空间后，`ros2 pkg prefix resilient_nav_description` 返回工作空间安装前缀；安装后的 Xacro 再次通过展开和 `check_urdf`。

### 学习要点

- `base_footprint` 适合作为机器人在地面的投影根坐标系，而带几何、碰撞和惯性的 `base_link` 可通过固定高度偏移与其连接。
- 圆柱驱动轮的几何轴需要旋转到车体 Y 轴，关节轴也应设置为 `0 1 0`，这样左右轮围绕轮轴旋转。
- Xacro 展开成功只验证宏和 XML 生成；继续使用 `check_urdf` 可以同时验证 link/joint 树和 URDF 语义。
- 对源码和安装后的描述资源分别复验，可以同时覆盖模型内容与 CMake 安装规则。

### 当前边界

- 当前只建立静态机器人描述，没有加入 Gazebo 插件、传感器、`ros2_control` 或控制功能。
- 尚未启动 `robot_state_publisher`、发布运行时 TF 或 JointState，也没有把机器人生成到 Gazebo。
- 差速运动链路和阶段 3 后续验收需要在单独任务中继续实施。

## 2026-07-27 — 阶段 2 Gazebo 基础仿真与时钟链路收尾

### 当前事实

- 已通过现有 ROS 2 Jazzy 软件源安装 `ros-jazzy-ros-gz`，Gazebo Sim 版本为 8.11.0；安装和验证日志分别保存在 `docs/gazebo_install_20260727.log` 与 `docs/gazebo_verify_20260727.log`。
- 官方 `shapes.sdf` 已通过脚本完成限时、无 GUI 的服务端启动检查，用户另行确认其图形世界正常打开。
- 已创建 `resilient_nav_simulation` 包，包含 `phase2_world.sdf`、`bridge.yaml`、`phase2_world.launch.py` 和静态资源测试。
- 用户确认自定义世界正常打开，并可见 `ground_plane`、`box_obstacle` 和 `cylinder_checkpoint`。
- 未启动桥接时，Gazebo Transport 可观察 `/clock`，ROS 2 不可观察 `/clock`；启动项目 Launch 后，ROS 2 可以观察 `/clock`。
- Launch 将 `system_heartbeat` 的 `use_sim_time` 设置为 `true`。暂停 Gazebo 时 ROS 2 `/clock` 和心跳停止，恢复 Gazebo 后二者继续，验证了仿真时钟驱动关系。
- 用户将 `box_obstacle` 的 pose 从 `2 0 0.5 0 0 0` 修改为 `3 1 0.5 0 0 0`，重新启动后确认 Gazebo 中的新坐标生效。

### 学习要点

- Gazebo Transport 和 ROS 2 Topic 是两套独立通信机制。即使两侧都使用 `/clock` 这一名称，也必须由 `ros_gz_bridge` 显式转换消息后才能互通。
- ROS 2 负责节点、参数和 ROS 图，Gazebo 负责世界与仿真时间，`ros_gz` 负责启动集成和选定数据的桥接；明确边界有助于定位“Gazebo 有数据但 ROS 2 看不到”的问题。
- SDF 定义 Gazebo 世界，`bridge.yaml` 定义跨中间件消息映射，Python Launch 文件负责把 Gazebo、桥和 ROS 2 节点组织成一次可复现启动。
- `src/` 是源码，`build/` 是中间产物，`install/` 是运行时可发现前缀，`log/` 保存构建和测试日志。`colcon build --symlink-install` 建立便于迭代的安装布局，而 `source install/setup.bash` 只是把该布局加载到当前 shell。
- 通过 ROS vendor 包安装的 `gz` 位于 `/opt/ros/jazzy/opt/gz_tools_vendor/bin`。安装前已经加载过 ROS 环境的 shell 不会自动获得后来新增的路径，需要重新加载 `/opt/ros/jazzy/setup.bash` 或打开新的已配置 shell；因此当时的 `gz` 命令不可见属于环境未刷新，而非安装失败。

### 当前边界

- 阶段 2 只完成静态世界、`/clock` 桥接、Launch 编排和已有心跳节点的仿真时间联动。
- 没有开始机器人、URDF、传感器、运动控制、Nav2、SLAM、故障注入、健康评估、自适应融合或容错导航。
- 本次收尾只更新文档并执行非图形验证，不安装软件、不启动 Gazebo 图形界面，也不修改 `resilient_nav_monitor` 或 `resilient_nav_simulation` 源码。

## 2026-07-24 — 实现 system_heartbeat 节点

### 当前事实

- `resilient_nav_monitor` 新增 `system_heartbeat` console 入口和同名节点。
- 节点每 1 秒在 `/system_heartbeat` 发布一次 `std_msgs/msg/String`，消息为 `alive count=N`，计数从 1 开始递增。
- 聚焦测试覆盖话题、周期和递增消息格式；完整测试结果为 4 项通过、1 项按生成器默认配置跳过、0 项失败。
- `colcon build --symlink-install` 成功完成 1 个包。
- 自动运行验证确认节点、话题类型和消息均符合约定，`ros2 topic hz` 测得频率为 `1.000 Hz`。
- 节点由 timeout 和清理逻辑自动停止，验证后没有后台进程残留。

### 当前边界

- `system_heartbeat` 只提供基础存活信号，不代表健康评估、故障检测或容错决策已经实现。
- 本次没有创建其他包或节点，没有安装软件，也没有安装 Gazebo、Nav2 或 SLAM。

## 2026-07-24 — 创建 resilient_nav_monitor 包骨架

### 当前事实

- 在 `ros2_ws/src/` 中创建了 `resilient_nav_monitor`，构建类型为 `ament_python`，许可证为 Apache-2.0。
- 包清单声明 `rclpy` 和 `std_msgs` 依赖，Python 打包配置、ament 资源索引和标准测试目录已建立。
- `colcon build --symlink-install` 成功完成 1 个包。
- 包标准测试结果为 2 项通过、1 项按生成器默认配置跳过、0 项失败。
- 加载工作空间环境后，`ros2 pkg prefix resilient_nav_monitor` 成功返回工作空间安装前缀。

### 当前边界

- 当前只创建规范包骨架，没有实现 `system_heartbeat`、其他节点或任何机器人功能。
- 本次没有安装软件，没有安装 Gazebo、Nav2 或 SLAM。
- 后续节点接口、行为和测试应在单独任务中明确设计和实现。

## 2026-07-24 — 阶段 1 ROS 2 工作空间基线

### 当前事实

- 已创建 `ros2_ws/src/`，并以 `.gitkeep` 保留空源码目录。
- 加载 ROS 2 Jazzy 环境后，在 `ros2_ws` 中执行空 `colcon build` 成功，结果为 `0 packages finished`。
- `ros2_ws/build/`、`ros2_ws/install/` 和 `ros2_ws/log/` 均由现有 `.gitignore` 规则排除，没有进入 Git。
- 当前尚未创建任何 ROS 2 包、项目节点、消息、启动文件或参数。

### 当前边界

- 本次没有安装软件，没有创建 ROS 2 包，也没有启动机器人功能开发。
- Gazebo、Nav2 和 SLAM 仍未安装。
- 后续首个功能包及其接口、构建和测试需要在单独任务中明确实施。

## 2026-07-24 — ROS 2 Jazzy 安装与基础通信验证

### 当前事实

- ROS 2 Jazzy 已安装，环境变量为 `ROS_DISTRO=jazzy`、`ROS_VERSION=2`。
- `ros2`、`colcon` 和 `rosdep` 命令均可用。
- 官方 `demo_nodes_cpp talker` 和 `demo_nodes_py listener` 通信验证通过。
- 运行期间 `/talker` 和 `/listener` 节点均存在，`/chatter` 话题类型为 `std_msgs/msg/String`。
- Gazebo、Nav2 和 SLAM 仍未安装。
- ROS 2 工作空间和项目 ROS 2 包仍未创建。

### 问题与修复

- 验证脚本启用了 `set -euo pipefail`。首次直接加载官方 `/opt/ros/jazzy/setup.bash` 时，官方脚本引用未定义的 `AMENT_TRACE_SETUP_FILES`，触发 nounset 错误。
- 修复仅作用于环境加载过程：加载前执行 `set +u`，加载后立即恢复 `set -u`；脚本仍保留 `set -e`、`set -u` 和 `pipefail`。
- 修复后验证脚本、详细 `ros2 doctor --report` 和官方 talker/listener 通信实验均完成。通信实验使用超时和清理机制，未留下后台演示节点。

### 当前边界

- 本次没有重新安装 ROS 2，也没有执行 apt、dpkg 或 sudo 安装。
- 本次没有安装 Gazebo、Nav2、SLAM 或其他 ROS 发行版。
- 本次没有创建工作空间、ROS 2 包或项目节点。
- 后续机器人仿真、导航和容错功能仍为未实施计划。

## 2026-07-24 — 第 0 阶段文档收尾

### 当前事实

- 项目名称确定为 **ResilientNavLab**。
- 项目目标是构建基于 ROS 2 的移动机器人多传感器故障注入、健康评估、自适应融合和容错导航平台。
- 项目仍处于初始化阶段。
- 当前环境为 Ubuntu 24.04.4 LTS，CPU 架构为 x86_64/amd64。
- Git 2.43.0、Python 3.12.3、Node.js v24.18.0、npm 11.16.0 和 Codex CLI 0.145.0 可用。
- 当前只有 `python3` 命令，没有 `python` 命令。
- ROS 2 与 Gazebo 均未安装或不可用。
- 尚未创建 ROS 2 工作空间和功能包，机器人功能开发尚未开始。

### 本阶段决策

- 先建立清晰的项目范围、环境基线和协作规则，再进入软件安装与代码开发。
- 不在第 0 阶段提前选择 ROS 2 发行版或 Gazebo 版本；选择必须基于 Ubuntu 24.04 的官方兼容关系另行确认。
- 后续设计采用模块化边界：故障注入、健康评估、自适应融合和容错导航应能分别开发、测试和对比。
- 实验设计必须重视可复现性，至少记录场景、参数、随机种子、故障真值和评价指标。

### 本次完成

- 补充项目入口和当前状态说明。
- 定义项目目标、计划内能力、当前边界与建议阶段路线。
- 记录系统、工具、ROS 2 和 Gazebo 的实际状态。
- 建立仓库协作规则和面向未来构建产物的忽略规则。

### 学习要点

- “命令不存在”与“环境未加载”可能表现相同，因此 ROS 2 状态同时通过 `ros2`、`ROS_DISTRO` 和 `/opt/ros` 交叉核验。
- 在创建工作空间前记录系统与工具版本，可以为后续依赖选择、问题复现和迁移提供基准。
- 项目文档必须区分“目标”“计划”和“已实现”，避免初始化仓库给出错误的功能完成预期。

### 后续待办（未执行）

- 调研并选择适配 Ubuntu 24.04 的 ROS 2 发行版。
- 确认 ROS 2 与 Gazebo 的官方兼容组合。
- 经明确授权后安装依赖并创建 ROS 2 工作空间。
- 设计首个最小可运行的仿真和数据链路里程碑。

### 操作声明

本次仅执行只读环境检查并修改项目文档；未安装任何软件，未执行 `sudo apt install`，未创建 ROS 2 工作空间或包，也未修改系统配置。
## 2026-08-28 — Phase 10 Task 3.2 Controller Server + FollowPath

- Navfn path 只保证 robot origin 走过 free cell，不能代表当前非对称 collision footprint 在转弯和终端朝向也安全。因此把 full-footprint sweep 放在 Planner 输出与 FollowPath 之间：用 raw `nav2_msgs/Costmap` 的 `254/255`，而不是 published `OccupancyGrid` 的 99/100 简化值；半 cell 平移和按外顶点半 cell 位移的旋转离散避免在 cell 间跳过碰撞。
- 这项 sweep 是项目侧可复用验收 gate，不替换 Navfn/RPP。fresh Planner-only simple/detour 分别完成 85/810 个 sweep pose，均为 zero lethal/unknown；合成单测验证 centerline false-safe、非对称终端转向、unknown 和越界拒绝。
- Controller 的正式 Local Costmap 必须由 `controller_server` 内嵌拥有，不能复用 standalone Task 2 executable 或再起第二张。RPP 复用 Task 2 scan-only odom rolling contract，且显式 `enable_stamped_cmd_vel=false` 匹配 Gazebo `Twist` bridge、`publish_zero_velocity=true`、`allow_reversing=false`、collision detection=true。
- two fresh-process FollowPath actions 均 PASS：simple/detour 最大 cross-track 为 `0.0154/0.0532 m`，最大线速度均 `0.20 m/s`，角速度 `0.376/0.600 rad/s`，全路径及运行期 footprint 均无 raw lethal/unknown，RPP collision arc 也无 lethal/unknown。Probe 只接受两条 YAML 固定场景，速度越界或 safety failure 即 cancel，并在 finally 只写 5 条零 Twist。
- 此结论只覆盖健康固定路径闭环，不外推到 BT、NavigateToPose、周期重规划、动态障碍、recovery、长距离性能、fault-aware navigation 或 Agent control。
## 2026-08-29 — Phase 10 Task 4 benchmark startup contract 收口（动态验证待执行）

- 审查确认 `planner_server/get_state async_send_request failed` 的发出者可以是官方 `lifecycle_manager_navigation`，不是 Runner 的只读 GetState observer；至少一类失败在 Planner configure 已完成且进程仍存活时发生，另一类则由 Gazebo create service/`/clock` 缺失、EKF 等待时钟和 `map -> base_footprint` 缺失引起，shutdown 后的 lifecycle 报错不能反推为初始根因。
- Task 4 移除固定 12 s navigation start delay；官方 Navigation Lifecycle Manager 继续独占 Planner/Controller/BT 的 configure/activate 顺序。Runner 的最小 readiness 顺序改为 `/clock`、定位 TF、只读 lifecycle、NavigateToPose action、raw Costmaps，并把阶段与最后错误写入 navigation evidence。
- batch 从 3 次 infrastructure retry 与 8 s unconditional settle 改为每个 logical run 一次尝试、首个失败 fail-fast。wall timeout 按 readiness + 冻结 action timeout + 明确 post-goal/result grace 推导；每个 run 保存独立 `GZ_PARTITION`、DDS domain、initial-pose result、trial manifest 和 launch log。
- multi-turn 继续冻结为 `(4.7, 5.3, pi/2)`、初始路径至少 `7.0 m`、两次有效转向；没有为名称或一次 startup failure 继续调场景。Task 4 尚未 PASS，下一步仅应运行一次 fresh multi-turn 验证；成功后才可运行正式 3×3。

### 动态执行结果

- 宿主 fresh `multi_turn_healthy` PASS：官方 navigation lifecycle manager 正常 active，initial path `7.5349 m`、2 turns、GT travel `7.4847 m`、final GT `0.0689 m/0.1162 rad`、navigation `42.094 s`、0 recovery、43 replan、final stop PASS。受限 sandbox 的先前尝试因 DDS UDP/getifaddrs 被系统拒绝而在 readiness 前退出，不作为 benchmark failure。
- 正式 3×3 已按新 fail-fast 合同运行：`simple_reachable-r01` PASS；`static_obstacle_detour-r01` 中 map-server configure 后出现 `failed to send response to /map_server/change_state`，Localization Lifecycle Manager 未能继续到 AMCL active，initial pose helper 因此失败，随后 Planner 等待缺失的 map TF。原始 result 保留 Runner 的下游 `infrastructure_localization_tf` 分类；随后补充的 batch classifier regression 会在未来 run 保留原始 evidence 的同时提升为 `infrastructure_nav2_lifecycle_service`。batch 没有重试、没有继续第三场景。
## 2026-08-30 — Phase 10 Task 5.1–5.2 dynamic-obstacle overlay

- 复用 Task 4 fresh-process trial、GT recorder、Runner 和 offline evaluator；没有新增 benchmark 框架、readiness gate、sleep/retry 或任何 Task 1–3/Nav2 参数修改。
- 运行时障碍只经官方 `ros_gz_bridge` 的 Gazebo `SpawnEntity` 服务创建；注入器等待 initial `/plan` 与普通 filtered odometry `0.20 m`，不读 GT、不直接写 Costmap、不改变 goal/BT action、也不发布速度。
- 5.1/5.2 都以 Task 4 detour 为基线；5.1 的绕行箱和 5.2 的横贯墙来自一次性离线 saved-map/footprint 连通性推导。5.2 明确测量无 Recovery baseline 的 ABORT，而不把外部 timeout/cancel 伪装成失败语义。
- 首次动态运行揭示 injector callback 内嵌套 executor 的实现错误，已改为一次异步 service future；后续 event 链已观测到 Path → motion gate → Spawn ACK → two Costmap detections，但前台执行环境在导航终态文件写出前中止 launch。故 Task 5 尚未 PASS，下一步只应完成一项完整 5.1 fresh trial + offline evaluator，再考虑单次 5.2。
## 2026-09-04 — BRNE Scene 2 sequential two-pedestrian implementation（人工 Closed-loop 待验收）

- 新增顺序双行人 Scene 2：robot goal 为 map `(2.8, 0)`；Ped1 保持 Scene 1 crossing，Ped2 位于 odom `x=1.8` 并反向 crossing。Ped2 只在 Ped1 的正常 separation release、robot map `x > 0.8` 和 `0.4 s` 场景延迟后启动；stale 或其他非正常 reset 不会解锁 Ped2。
- passing-side commitment 从 single-pedestrian global side 升级为 owner-ID state，仍只在 ROS-free wrapper proposal 层工作。回归定位到旧 protection 同时依赖 `0.05 m` lateral displacement，导致 commitment 已建立但前几轮 raw nominal 已反向时 proposal 尚未受保护；现在 opposite-turn nominal 超过 `0.2 rad/s` 即按冻结的 `commitment_opposite_scale=0.25` 缩放，不再复用 acquisition deadband 作为 protection 门。owner、side、frozen axes 和四个 5 Hz distance samples 保持；Scene 2 release 使用当前欧氏距离超过 `1.0 m` 且距离窗口持续增大，不使用 CV predicted minimum distance、clear-cycle 或 elapsed-plan timeout。acquire 与 BRNE weighted command 均未改。
- Gazebo adapter 由 singleton relay 扩展为一个 ID-stable aggregate `PedestrianArray` writer，Scene 1 默认 scalar 参数未变。Scene 2 two sources 映射为 IDs 1/2；Ped2 使用独立 model/topic/frame，保持 world-parent prismatic topology。
- 定向资源/纯逻辑测试为 `38 passed`；实际 venv `colcon` package 回归为 `64 passed, 1 xfailed`，xfail 仍仅是 pinned scalar `traj_sim()` 的既有缺 dt 问题。install console script shebang 已验证为根 `.venv/bin/python`，Scene 2 launch `--show-args` 成功。fresh process 的默认 `196×25` warm-up 为 `5098.239 ms`，随后双行人 50 次 planning mean/P95/max 为 `31.392/40.083/45.579 ms`，仍低于 `200 ms` 5 Hz 周期。按范围未自动运行最终 Gazebo Closed-loop Demo，人工验收待执行。
## 2026-09-04 — BRNE Scene 2 pedestrian aggregation startup repair

- 真实 Scene 2 runtime 定位到“行人会动、global Path 可见，但 BRNE red prediction 与 robot control 均缺失”的直接原因：双 source adapter 的空 Python list 参数被 Jazzy rclpy 推断为 `BYTE_ARRAY`，而 launch 覆盖为 string/integer arrays；adapter 因 `InvalidParameterTypeException` 在启动时退出，故 `/brne/pedestrians` 从未发布，shadow node 正确地 fail-closed。
- 最小修复将 Scene 2 source lists 改为显式 CSV scalar parameters，adapter 解析为 validated source tuple；Scene 1 的原 scalar input contract 不变。没有修改 BRNE core、proposal commitment、mask、control gate 或 5 Hz policy。
- venv package test 为 `65 passed, 1 xfailed`（xfail 仍为 pinned scalar `traj_sim()`）。在隔离 `arm_brne=false` 的 35 s Gazebo probe 中 adapter 保持存活，shadow node 连续发布至少 79 个双行人 plan，单次 `43–111 ms`，低于 `200 ms`；该 probe 没有创建正式 `/cmd_vel` publisher，未运行最终人工 Closed-loop Demo。

## 2026-09-04 — BRNE Scene 2 relative-separation release

- Scene 2 人工观察确认冻结的 `commitment_opposite_scale=0.25` 已能稳定保持 passing side，但以 `1.0 m` 或 `1.5 m` 固定绝对距离释放会让回归 global path 过晚。release 因而改为 owner-ID commitment 内记录 `min_distance_seen`，并只在当前欧氏距离相对该次交互最近点增加超过 `0.20 m`、且最近四次 5 Hz plan 的距离窗口呈分离趋势时正常释放。
- `0.20 m` 来自当前 `196×25`、5 Hz Scene 2 ROS-free rollout：候选 `0.15/0.20/0.25 m` 分别约在最近点后 `0.6/0.8/1.0 s` 触发；最终实现回放在最小距离约 `0.592 m`、当前距离约 `0.802 m` 时释放，高于 `0.58 m` close-stop 几何阈值，并比原 `1.0 m` 绝对阈值约提前 `0.8 s`。该离线结果不冒充 Gazebo 人工验收。
- 未加入 predicted-clear、timeout、cooldown、behind 或 counterfactual 条件；BRNE core、per-ID owner、0.25 proposal protection、weighted control、RNG 与 safety mask 均保持不变。
- 根 `.venv` 驱动的 scoped build 成功；包回归为 `68 passed, 1 xfailed`，xfail 仍仅是 pinned scalar `traj_sim()` 缺 dt 的既有合同。正式 install 的 `brne_shadow_node` shebang 仍指向根 `.venv/bin/python`，Scene 2 launch `--show-args` 可正常加载；未运行 Gazebo Closed-loop Demo。

## 2026-09-05 — BRNE Scene 1 sensor-input V1

- 新增独立 `brne_sensor_scene1_demo.launch.py`，将 crossing 移到 odom `x=1.0`，robot goal 设为 `(2.0, 0)`；BRNE 输入不再启动 Gazebo odometry adapter。物理行人的真值 odometry 只在 `/scenario/brne_pedestrian/odometry` 内供场景 driver 判断终点，唯一 `/brne/pedestrians` writer 是 LiDAR dynamic-agent node。
- 感知核心保持 ROS-free：对 `/scan` 做相邻点 clustering，以 scan timestamp 的 `lidar_link -> odom` TF 转换 centroid；跨帧最近邻匹配后，用多个背景 cluster 位移的鲁棒中值扣除公共 drift，再从 6 帧短历史拟合 residual velocity。达到净位移、速度和方向一致性门槛才提升为带稳定 track ID 的 dynamic agent；track 丢失立即删除。
- 运行探针发现，移动行人会遮挡/揭露较远静态轮廓，使其 centroid 在短时间内呈现与行人近似的速度。V1 保留 6 m 背景用于 drift，却只把 2.0 m 局部交互范围内的 dynamic track 送入 BRNE；该范围覆盖 Scene 1 行人的约 `1.0–1.4 m` 观测距离，并排除实测位于约 `2.85–5.4 m` 的 scan-shadow 假 track。
- 三次真实 Gazebo 检查均使用 `arm_brne=false`，因此 gate 没有创建 `/cmd_vel` publisher。最终 probe 中，行人启动前 dynamic agent 为 0；启动后约 6 帧建立唯一 track，传感器估计 `vy` 约 `0.21–0.24 m/s`；有效观测段只向 BRNE 发布该 agent，行人结束并丢失后恢复为空。BRNE JIT warm-up 后能持续规划。该证据只验证 sensor-input 链，不代替最终 armed 人工 Closed-loop 验收。
- 人工验收进一步暴露跨层职责冲突：原始 `/scan` 也进入 Navfn 的 global obstacle layer，导致同一个已确认行人既改变 global path、又作为 BRNE dynamic agent 改变局部 interaction。最小修复让 tracker 保留每个 cluster 的原 scan beam 区间，并额外发布 `/brne/static_scan`：只把当前已确认且实际送入 BRNE 的 dynamic cluster beam 改为有限 `range_max` clearing rays，其余量测不变。
- Phase 10 planner launch 新增默认仍为 `/scan` 的 planner-only remap 参数，所以既有 Phase 10 行为不变；仅 sensor Scene 1 令 `planner_server/global_costmap` 使用 `/brne/static_scan`，AMCL 与 tracker 仍使用原始 `/scan`。disarmed runtime 中该 topic 为 LiDAR node 单 publisher、global costmap 单 subscriber；行人确认后每帧实际 mask 约 `35–44` beams，BRNE agent/planning 持续有效，crossing 期间 global path 始终保持 80 poses。该结果验证数据流和 path 稳定性，不冒充最终 armed 行为验收。

## 2026-09-06 — BRNE dynamic-cluster global-costmap quarantine

- 审查确认 sensor Demo 的 planner remap 本身正确：只有 global planner 的 obstacle layer 使用
  `/brne/static_scan`，AMCL/tracker 仍使用原始 `/scan`。真正缺口是过滤门槛原先与 BRNE agent 发布门槛
  相同，导致新行人在 6 帧 dynamic confirmation 和后续 3 帧 EMA 稳定前仍可能被 global costmap 当作
  静态障碍；已确认 dynamic track 停止后也不会自动恢复 fixed 身份。
- 现在所有新出现、尺寸可跟踪的 cluster 从首帧开始进入 Navfn quarantine。连续 6 帧内最大位移不超过
  `0.02 m` 且 fitted speed 不超过 `0.04 m/s` 才确认为 fixed 并恢复其原始 scan beams；至少 3 帧出现
  `0.02 m` 净位移、`0.08 m/s` 速度和一致方向时继续隔离，但仍需原 6 帧 dynamic + 3 帧 EMA 门槛才向
  BRNE 发布。已发布 dynamic track 若连续 8 帧 filtered speed 不超过 `0.04 m/s`，则降级为 fixed。
  这只改变 global-costmap 输入身份管理，不改变 BRNE core、interaction、event 或 control。
- sensor Scene 1/Scene 2/Scene 3 共用的 RViz profile 新增 `/global_costmap/costmap` display，允许人工直接
  对照 global path 与动态行人占据。ROS-free targeted tests 为 `23 passed`；首次 package test 因 sandbox
  无法写 `~/.ros/log` 产生 3 个环境失败，设置 `ROS_LOG_DIR=/tmp/...` 后重跑为 `136 passed, 1 xfailed`，
  xfail 仍是 pinned scalar `traj_sim()` 缺 `dt`。`.venv` scoped build 成功，正式 shadow console script
  shebang 保持根 `.venv/bin/python`。本轮未运行 Gazebo，costmap/path 的动态效果仍待人工验收。
