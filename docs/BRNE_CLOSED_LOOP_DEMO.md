# BRNE V1 Closed-loop Demo

## 冻结状态

BRNE V1 已在 Scene 1 横穿、Scene 2 顺序双横穿和 Scene 3 迎面行人中形成统一闭环配置。
三套正式 Demo 使用同一个运行参数文件：
`ros2_ws/src/resilient_nav_brne/config/brne_v1_runtime.yaml`。各 Scene launch 只定义场景几何、
目标点、行人运动和启动编排，不再各自携带算法调参项。

正式 Demo 的动态行人状态来自机器人 LiDAR，不来自 Gazebo 行人真值：

```text
/scan + scan timestamp TF
  -> cluster / common-drift correction / track / velocity estimate
  -> /brne/pedestrians (odom, id, x, y, vx, vy)
  -> interaction lifecycle + mutually exclusive event policy
  -> pinned BRNE sampling/game weights
  -> time-aligned safety factors and event weights
  -> weighted control sequence
  -> /brne/cmd_vel_raw
  -> armed control gate
  -> /cmd_vel
```

`brne_closed_loop_demo.launch.py` 保留为 Gazebo odometry 输入的验证入口，用于隔离感知与控制问题；
它不是正式的 sensor-input 对照结果，也不能作为感知公平 benchmark。

## 冻结算法合同

### Pinned BRNE profile

BRNE 数学核心保持 pinned MurpheyLab/brne commit
`633a5cdcb39ab27f18b596cb8cb1968644f82391`，runtime profile 以该版本
`brne_nav/crowd_nav/config/brne.yaml` 为算法参数基准：

| 参数 | 冻结值 |
| --- | ---: |
| `maximum_agents` | 5 |
| `num_samples` | 196 |
| `dt` | 0.1 s |
| `plan_steps` | 25 |
| `kernel_a1 / kernel_a2` | 0.2 / 0.2 |
| `cost_a1 / cost_a2 / cost_a3` | 15 / 3 / 20 |
| `ped_sample_scale` | 0.1 |

项目机器人参数保持 `max_linear_velocity=0.30 m/s`、`max_angular_velocity=0.80 rad/s`、
`nominal_linear_velocity=0.20 m/s`、5 Hz replan 和 `goal_tolerance=0.20 m`。

### 无行人路径跟随

收到新鲜的空 `PedestrianArray` 时，wrapper 不运行无意义的多人博弈采样，而使用当前 Navfn local
waypoint 做确定性路径跟随。角速度由航向误差产生并受 `0.80 rad/s` 边界约束；线速度以
`0.20 m/s` 为上限，按 `cos²(heading_error)` 和目标距离降速。输入缺失、frame 错误或 stale 与
“新鲜地观测到无人”严格区分，前者仍 fail closed。

### Interaction lifecycle

每个 5 Hz 输出周期在 `odom` 中维护一个 interaction owner：

- 仅从 `3.20 m` 内的已确认 dynamic track 中选择最近者；
- owner 由 pedestrian ID 保持，不被其他 track 的排序覆盖；
- 记录该次 interaction 的 `min_distance_seen`；
- 当前距离相对最近点增加超过 `0.20 m`，且最近 4 个输出周期的距离趋势为正时释放；
- 已释放且仍可见的同一 track 不立即重入；track 消失后新 ID 可重新进入；
- stale、goal reached 和节点状态重置会清理 interaction/event temporal state。

一个 owner 同时最多属于一种事件。分类使用机器人当前坐标系中的完整速度方向：先检查迎面事件，
否则检查横穿事件，避免高速斜向行人同时触发两套策略。

### 横穿事件

横穿分类与近期交点条件为：

- `|v_lateral| >= 0.08 m/s`；
- lateral alignment `|v_lateral| / |v| >= 0.50`；
- `0 < t_cross <= 4.0 s`；
- forward-axis 交点位于 `[0.20, 2.00] m`。

触发后选择行人横向速度的反侧作为 passing side：

- 最初 5 个输出将同向 angular candidates 的附加权重置零；
- passing-behind candidates 的 BRNE 原始权重乘 `10.0`；
- 偏好侧、时间对齐距离过近但预测末端已分离至少 `0.20 m` 的 candidate 不硬删除，而乘
  `0.10` safety factor；
- 行人穿过 robot forward axis 后结束横穿 event，interaction lifecycle 可继续到正常释放。

### 迎面事件

迎面分类条件为 approach speed 至少 `0.12 m/s`，且速度方向位于机器人反向 forward axis 的
`1.05 rad`（约 60°）锥内。该范围故意比严格正迎面更宽，使 30°–60° 的强 approach 情形进入
更稳定的单侧绕行逻辑。

- 行人速度若相对正迎面中心线有至少 `0.17 rad` 的可靠 lateral direction，则固定选择其反侧；
- 近似完全正迎面时，由第一个超过 `0.05 rad/s` 的实际 BRNE angular output 选择一侧；
- event 内将另一侧 angular candidates 的附加权重置零；
- 机器人到 entry 时冻结的 pedestrian CV line 距离超过 `0.58 m` 后结束 event，避免无限侧移。

### Proposal support 与最终权重

event 选定 passing side 后仍不直接改最终 command。若 local waypoint 导致 raw nominal angular
以超过 `0.35 rad/s` 的幅度强烈指向反侧，则 proposal nominal 乘 `0.25`，恢复 pinned sampler
两侧的 angular support。横穿保护窗口为 5 个输出；迎面保护持续到迎面 event 结束。

最终 robot sample 权重为：

```text
BRNE core weight
  * crossing-side bias
  * time-aligned safety factor
  * crossing initial-direction mask
  * head-on direction mask
```

不需要单独预归一化：wrapper 在每个时间步沿 sample axis 除以最终权重和，得到完整
`(plan_steps, 2)` weighted control sequence。即时命令取第 0 步；`/brne/optimal_path` 由完整
weighted sequence 重新仿真。最终仍保留项目速度边界。全部 candidate 权重为零时发布 stop。

### LiDAR dynamic-agent 与 Global Costmap 边界

tracker 使用 scan 自身 timestamp 查询 TF，将 cluster centroid 转到 `odom`；背景 cluster 的鲁棒中值
用于扣除共模漂移，6 帧位置历史拟合速度，再做 `alpha=0.25` EMA。速度稳定需 3 帧，允许的相邻
方向变化为 `0.35 rad`；超过 `0.70 rad` 的方向突变拒绝并保留上一估计。最大 dynamic-agent 范围为
`4.0 m`。

所有新出现且可跟踪的 cluster 先从 Navfn 的 `/brne/static_scan` 隔离。连续 6 帧最大位移不超过
`0.02 m` 且 fitted speed 不超过 `0.04 m/s` 才确认为静态并恢复其 scan beams。dynamic track 连续
8 帧低于 `0.04 m/s` 后可降级为静态。AMCL 与 tracker 始终消费原始 `/scan`；只有 Navfn global
obstacle layer 消费 `/brne/static_scan`。RViz 显示 `/global_costmap/costmap` 供人工核对。

## 正式场景入口

激活仓库根 `.venv` 后，可直接运行：

```bash
# Scene 1：单行人横穿，sensor input
ros2 launch resilient_nav_brne brne_sensor_scene1_demo.launch.py \
  arm_brne:=true use_rviz:=true

# Scene 2：顺序双行人反向横穿，sensor input
ros2 launch resilient_nav_brne brne_scene2_demo.launch.py \
  arm_brne:=true use_rviz:=true

# Scene 3：单行人迎面，sensor input
ros2 launch resilient_nav_brne brne_scene3_head_on_demo.launch.py \
  arm_brne:=true use_rviz:=true
```

Scene 1 crossing 位于 odom `x=0.8`，速度 `0.25 m/s`，运动 3 m；robot goal 为 `(2.0, 0)`。
Scene 2 robot goal 为 `(2.8, 0)`，两名 prismatic pedestrian 顺序启动。Scene 3 行人从 robot 前方
约 3 m 处以 `0.20 m/s` 迎面运动，起点与路径几何保持已人工验证版本。不同速度属于场景输入，
planner/event 参数仍完全共用冻结 profile。

RPP 同场景对照入口保留为：

```bash
ros2 launch resilient_nav_brne rpp_scene1_comparison_demo.launch.py use_rviz:=true
```

RPP 复用 Phase 10 Navfn + BT Navigator + Regulated Pure Pursuit 参数，不启动 BRNE shadow/gate。

## 构建与检查

该包的 console scripts 必须由仓库 `.venv` 解释器生成：

```bash
cd /home/kylian/projects/resilient_nav_lab/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
../.venv/bin/python -m colcon build --symlink-install \
  --packages-select resilient_nav_brne
../.venv/bin/python -m colcon test --packages-select resilient_nav_brne
head -1 install/resilient_nav_brne/lib/resilient_nav_brne/brne_shadow_node
```

shebang 必须指向 `/home/kylian/projects/resilient_nav_lab/.venv/bin/python`。不得使用裸
`colcon` 重建本包。

## 人工验收重点

- RViz 中 global path、global costmap、sensor dynamic-agent marker 和 BRNE red prediction 均持续更新；
- 新 cluster 在身份确认前不污染 global costmap，动态行人不以“静态障碍 + agent”双重身份影响规划；
- Scene 1/2 横穿进入 crossing event，Scene 3 进入 head-on event，且一个 owner 不同时进入两类 event；
- robot 能形成明确 passing side，避让后正常回到 global path 并到达 goal；
- 输入 stale、全部 candidate 无效或 gate 未 armed 时保持 fail closed；
- prismatic pedestrian 的 collision/odometry 保持，运动结束后不倾倒。

## 明确保留的边界

- 不修改 pinned `brne.py` 数学核心、随机 pedestrian sampling 或 initial equilibrium weights；
- 不对 `/cmd_vel` 做低通滤波，不做 global-path/nominal freeze；
- `close_stop_threshold=0.20 m` 是当前 point-agent 数值门，不是完整机器人 footprint 与行人半径的
  几何安全证明；
- LiDAR V1 不做人类分类、遮挡续接或长期身份恢复；
- 目前只做人工场景验收，不声称形成统计 benchmark 或通用 social-navigation policy。
