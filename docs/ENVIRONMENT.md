# 开发环境基线

> 2026-09-06 更新：本机已核验 Jazzy Slam Toolbox 2.8.5、官方 Nav2 1.3.12 binary 与 Gazebo Harmonic；当前工作空间有 13 个 ROS package。Phase 10 与 BRNE V1 Scene 1/2/3 baseline 均已收口。BRNE 运行时使用仓库 `.venv` 中的 Numba，构建合同见下文。

## 核验信息

- 最近核验日期：2026-09-06
- 项目目录：`/home/kylian/projects/resilient_nav_lab`
- 当前阶段：阶段 0 至阶段 9、Phase 10、BRNE V1 Scene 1/2/3 baseline 与 RA-1A Offline Robot Diagnosis 已完成

本页记录核验时的实际环境，不代表未来项目最终采用的依赖组合。

## 系统与工具状态

| 项目 | 检测结果 | 状态 |
| --- | --- | --- |
| Ubuntu | Ubuntu 24.04.4 LTS（Noble Numbat） | 可用 |
| CPU 架构 | `x86_64`；Debian 架构名为 `amd64` | 可用 |
| Git | `git version 2.43.0`，路径 `/usr/bin/git` | 可用 |
| Python 3 | `Python 3.12.3`，路径 `/usr/bin/python3` | 可用 |
| `python` 命令 | 未找到 | 不可用；当前使用 `python3` |
| Node.js | `v24.18.0`，路径 `/usr/bin/node` | 可用 |
| npm | `11.16.0`，路径 `/usr/bin/npm` | 可用 |
| Codex CLI | `codex-cli 0.145.0`，路径 `/home/kylian/.npm-global/bin/codex` | 可用 |
| ROS 2 CLI | ROS 2 Jazzy，路径 `/opt/ros/jazzy/bin/ros2` | 可用 |
| ROS 2 环境 | `ROS_DISTRO=jazzy`，`ROS_VERSION=2` | 已安装并在交互式 Bash 中配置 |
| ROS 2 开发工具 | `colcon` 路径 `/usr/bin/colcon`；`rosdep` 路径 `/usr/bin/rosdep` | 可用 |
| 机器人描述工具 | `xacro`、`check_urdf` 路径均位于 `/opt/ros/jazzy/bin` | 可用；项目 Xacro 验证通过 |
| ROS 2 基础通信 | 官方 `demo_nodes_cpp talker` 与 `demo_nodes_py listener` | 通信验证通过 |
| ROS 2 工作空间 | `/home/kylian/projects/resilient_nav_lab/ros2_ws` | 已创建；空构建和 `colcon build --symlink-install` 均通过 |
| 项目 ROS 2 包 | 13 个包，含 `resilient_nav_slam`、`resilient_nav_navigation`、`resilient_nav_brne` | BRNE V1 使用独立 `.venv` build/runtime 合同；其最新测试与 build 结果记录在 `BRNE_CLOSED_LOOP_DEMO.md` 和 `LEARNING_LOG.md`。 |
| BRNE V1 | pinned MurpheyLab/brne + Numba wrapper | Scene 1/2/3 共用冻结 runtime profile；正式 pedestrian input 来自 LiDAR tracker，GT adapter 仅作隔离验证。 |
| 项目 ROS 2 节点 | `system_heartbeat`、`odom_tf_broadcaster` | 心跳发布及 `/odom` 到 `odom -> base_footprint` TF 的端到端验证通过 |
| 仿真资源包 | `resilient_nav_simulation`（`ament_cmake`） | 构建、运动工具测试、阶段 2 世界、阶段 3 生成和 Gazebo/RViz Demo Launch 验证通过 |
| 机器人描述包 | `resilient_nav_description`（`ament_cmake`） | Xacro、运行时 TF、RViz、Gazebo 材质、动力学支撑、DiffDrive、JointStatePublisher 和阶段 4 固定安装坐标验证通过 |
| 定位包 | `resilient_nav_localization`（`ament_cmake`） | EKF 配置、完整阶段 4 Launch、安装和自动测试验证通过 |
| 接口包 | `resilient_nav_interfaces`（`ament_cmake`） | `FaultStatus` 消息生成和依赖包构建验证通过 |
| 故障注入包 | `resilient_nav_fault_injection`（`ament_python`） | 阶段 5 IMU/wheel/Lidar 链保持；阶段 7.2 增加不修改数据的 `manual_fault_event` 真值窗 |
| 健康评估包 | `resilient_nav_health_assessment`（`ament_python`） | 阶段 6 默认三传感器语义保持；阶段 7.2 增加相机 baseline/monitor、freeze runtime 验证和可选 camera evaluator |
| 相机包 | `resilient_nav_camera`（`ament_python`） | 阶段 7.1 C920/CameraInfo/image_proc 保持；阶段 7.2 增加只组合现有节点的 health 联合 Launch |
| Robot Agent 包 | `resilient_nav_agent`（`ament_python`） | Offline Case、3 个只读 Tools、strict Runtime、Scorer/Batch 与 8-case Fake pipeline 通过；package 88 tests；无 Live ROS 节点 |
| Robot Agent Python 环境 | 仓库根 `.venv`（Git ignored，`--system-site-packages`） | Python 3.12.3；agent-core 0.1.0 editable import；Pydantic 2.13.4 |
| `robot_localization` | `3.8.3`，前缀 `/opt/ros/jazzy` | `ekf_node` 可发现；阶段 4 动态闭环验证通过 |
| Nav2 | official Jazzy binary，核心包 `1.3.12`，前缀 `/opt/ros/jazzy` | `nav2_bringup`、`nav2_map_server`、`nav2_amcl`、`nav2_lifecycle_manager`、`nav2_costmap_2d`、`nav2_planner`、`nav2_controller`、RPP、`nav2_bt_navigator` 与 `nav2_behaviors` 已核验。Map Server/AMCL、Costmap、Navfn/RPP、无 Recovery NavigateToPose 与 Task 5.1–5.4 都有 retained host evidence；Task 5.3 为 engineering acceptance，保留有限观测的 all-wall-clear limitation。 |
| Gazebo | Gazebo Harmonic；Gazebo Sim `8.11.0` | `gz` 可用，官方和项目世界均已验证 |
| ROS 2—Gazebo 集成 | `ros-jazzy-ros-gz` `1.0.22` | `/clock` 与阶段 3 基础运动话题的定向 bridge 已验证 |
| 仿真时钟链路 | Gazebo `/clock` → ROS 2 `/clock` | 单向桥接、暂停/恢复和 `use_sim_time` 联动验证通过 |

运行 `codex --version` 时，Codex 成功返回版本号，同时提示当前受限检查环境无法创建 PATH aliases。该提示不影响本次版本识别；如后续需要诊断 Codex PATH 行为，应在对应任务中单独复核。

## BRNE Python/Numba 构建运行时合同

`resilient_nav_brne` 的 pinned BRNE core 导入 `.venv` 中的 `numba 0.61.2`。该包是
ament Python package，console script 的 shebang 由**运行 colcon 的 Python 解释器**决定；激活
`.venv` 后再调用系统 `/usr/bin/colcon` 仍会生成 `#!/usr/bin/python3`，而系统 Python 没有
Numba。该状态会让 `brne_shadow_node` 在 import 阶段退出，并以 fail-closed 方式同时阻止 BRNE
raw command 和 pedestrian ready gate。

因此这是 BRNE 的永久环境约束：从 `ros2_ws/` 重建或测试 `resilient_nav_brne` 时，必须显式使用
`../.venv/bin/python -m colcon`；不得使用 `/usr/bin/colcon` 或裸 `colcon`。标准修复/构建命令为：

```bash
cd /home/kylian/projects/resilient_nav_lab/ros2_ws
set +u
source /opt/ros/jazzy/setup.bash
source ../.venv/bin/activate
source install/setup.bash
set -u
../.venv/bin/python -m colcon build --symlink-install \
  --packages-select resilient_nav_brne
../.venv/bin/python -m colcon test --packages-select resilient_nav_brne
```

构建后必须复核安装后的 shebang，而不只检查 source/install 文件哈希：

```bash
head -1 install/resilient_nav_brne/lib/resilient_nav_brne/brne_shadow_node
```

预期结果是仓库根 `.venv/bin/python`。若显示 `/usr/bin/python3`，不得启动 BRNE Demo；重新执行上述
显式 venv build。不要通过临时 `PYTHONPATH` 注入 Numba 来掩盖错误解释器。

项目根 `.venv/bin/activate` 会加载 Jazzy 与当前 `ros2_ws/install` overlay，并在加载期间临时处理
shell 的 `set -u`。因此完成上述 build 后，重新执行一次 `source .venv/bin/activate`，即可直接使用
`ros2 launch resilient_nav_brne ...`；不需要在每次 Demo 前手工 source 两个 ROS setup 文件。

## 软件职责边界

- **ROS 2 Jazzy** 负责项目节点、参数、ROS 图、ROS 2 Topic 和节点间通信。`system_heartbeat` 与 `odom_tf_broadcaster` 属于这一侧。
- **Gazebo Harmonic** 负责加载 SDF 世界、维护仿真状态、推进或暂停仿真时间，并通过 Gazebo Transport 发布仿真数据。
- **`ros_gz`** 是两套中间件之间的集成层。`ros_gz_sim` 负责从 ROS 2 Launch 启动 Gazebo，`ros_gz_bridge` 负责按配置转换消息；它不会把所有 Gazebo 话题自动变成 ROS 2 Topic。

Gazebo Transport 和 ROS 2 Topic 是彼此独立的通信域。`gz topic -l` 看到的 `/clock` 是 Gazebo Transport 话题，`ros2 topic list` 看到的是 ROS 2 Topic；名称相同并不代表数据天然互通。未启动桥接时，Gazebo 一侧可观察到 `/clock`，ROS 2 一侧看不到它。当前 `bridge.yaml` 显式把 `gz.msgs.Clock` 单向转换为 `rosgraph_msgs/msg/Clock`，桥接启动后 ROS 2 才能订阅 `/clock`。

## 阶段 2 资源与启动链路

- `worlds/phase2_world.sdf` 使用 SDF 描述世界、物理参数、光源以及 `ground_plane`、`box_obstacle`、`cylinder_checkpoint` 三个静态模型。SDF 是 Gazebo 的仿真输入，不是 ROS 2 节点配置。
- `config/bridge.yaml` 只声明 `/clock` 的 `GZ_TO_ROS` 单向桥接、两侧消息类型和时钟 QoS。
- `launch/phase2_world.launch.py` 解析已安装包的共享目录，启动自定义世界、`ros_gz_bridge` 和 `system_heartbeat`，并为心跳节点设置 `use_sim_time=true`。
- `resilient_nav_simulation` 只提供仿真资源和启动集成，不包含机器人、传感器、导航或故障注入实现。

## 阶段 3 机器人描述与显示基线

- `resilient_nav_description` 是 `ament_cmake` 包，安装 `urdf/` 目录中的描述资源。
- `urdf/resilient_nav_robot.urdf.xacro` 定义 `base_footprint`、`base_link`、左右驱动轮和一个球形支撑轮。
- `base_footprint` 通过 fixed 关节连接 `base_link`；左右轮通过 continuous 关节连接车体；支撑轮当前通过 fixed 关节连接车体。
- `base_link`、左右轮和支撑轮均包含基础 `visual`、`collision` 和 `inertial` 定义；`base_footprint` 仅作为无几何的根坐标系。
- 源码 Xacro 和安装后的 Xacro 均可展开，`check_urdf` 解析成功，根 link 为 `base_footprint`。
- `launch/display.launch.py` 从安装后的 Xacro 生成 `robot_description`，启动 `robot_state_publisher`、关节状态发布器和 RViz。
- `use_gui:=false` 启动 `joint_state_publisher`；`use_gui:=true` 改为启动 `joint_state_publisher_gui`。
- `rviz/display.rviz` 使用 `base_footprint` 作为 Fixed Frame，并显示 Grid、RobotModel 和 TF。
- 两种 `use_gui` 分支均完成限时启动验证；RViz 初始化成功，`/joint_states`、`/robot_description`、`/tf`、`/tf_static` 类型符合预期。
- `base_footprint` 位于左右驱动轮轴中点；`base_link` 保持车体几何中心，因此固定变换为 X `-0.100 m`、Z `0.150 m`。
- 物理 link 均通过 `<gazebo reference="...">` 设置 Gazebo 命名材质和摩擦：车体摩擦为 `0.5`，左右轮为 `1.0`，球形支撑轮为 `0.05`。
- 当前驱动轮轴位于车体坐标 `x=0.10 m`，球形支撑轮位于 `x=-0.20 m`；车体惯性原点相对 `base_link` 为 `[-0.05, 0, -0.04] m`。
- `base_link` 原点保持离地 `0.15 m`，车体碰撞盒尺寸保持 `0.50 × 0.35 × 0.15 m`，水平状态下车体底部离地 `0.075 m`；轮和球形支撑轮的最低点均为地面高度。
- 阶段 3 描述最初只包含 Gazebo Harmonic `DiffDrive` 和 `JointStatePublisher` 两个系统插件；当前阶段 4 已在相同 Xacro 中增加 IMU、二维 Lidar 和 RGB-D sensor，但仍不包含 transmission 或 `ros2_control`。

## 阶段 3 Gazebo 初始生成与落地基线

- `resilient_nav_simulation/launch/phase3_spawn.launch.py` 通过 Include 复用 `phase2_world.launch.py`，没有复制或新建世界。
- Launch 从安装后的 Xacro 生成 `/robot_description`，启动 `robot_state_publisher`，并由 `ros_gz_sim create` 向 `resilient_lab` 世界创建 `resilient_nav_robot`。
- 可通过 `entity_name`、`spawn_x`、`spawn_y`、`spawn_z` 和 `spawn_yaw` 配置实体名称及初始位姿；`allow_renaming=false`，重复实体名会被拒绝。
- 默认 `spawn_z=0.25 m`，用于观察机器人自由落体并与地面接触；地面碰撞新增显式 ODE 摩擦 `mu=1.0`、`mu2=1.0`。
- 实际 Launch 验证中，`create` 报告 `Entity creation successful`，`gz model --list` 同时列出阶段 2 的三个静态模型和 `resilient_nav_robot`。
- 落地稳定后，`gz model -m resilient_nav_robot -p` 返回 XYZ 约为 `[-0.000000, 0.000000, -0.000001] m`，RPY 约为零。
- 原有验证覆盖实体生成、重力和接触稳定；后续原生插件验证见下一节。

## 阶段 3 Gazebo 原生差速与关节状态插件

- Xacro 使用当前 Harmonic 插件标识 `gz-sim-diff-drive-system` / `gz::sim::systems::DiffDrive` 和 `gz-sim-joint-state-publisher-system` / `gz::sim::systems::JointStatePublisher`。
- DiffDrive 使用实际关节 `left_wheel_joint`、`right_wheel_joint`，轮距由左右轮中心位置得到 `0.39 m`，轮半径为 `0.10 m`。
- DiffDrive 显式使用 `frame_id=odom` 和 `child_frame_id=base_footprint`；桥接后的 `/odom` frame 字段与项目 TF 名称一致。
- 默认实体 `resilient_nav_robot` 的 Gazebo Transport 话题如下：

| Gazebo 原生话题 | 方向与消息类型 |
| --- | --- |
| `/model/resilient_nav_robot/cmd_vel` | DiffDrive 订阅 `gz.msgs.Twist` |
| `/model/resilient_nav_robot/odometry` | DiffDrive 发布 `gz.msgs.Odometry` |
| `/world/resilient_lab/model/resilient_nav_robot/joint_state` | JointStatePublisher 发布 `gz.msgs.Model` |

- `phase3_spawn.launch.py` 将 `entity_name` 同时传给 Xacro 的 `gazebo_model_name` 参数，因此使用非默认实体名时，三个显式话题中的模型名会同步变化。
- 实际启动再次报告 `Entity creation successful`，模型落地后 Z 位姿约为 `-0.000001 m`。
- `gz topic -l` 和 `gz topic -i` 确认三个话题均存在且角色、类型符合上表；里程计可读取一条消息，关节状态消息同时包含左右轮关节。
- DiffDrive 系统自身还会保留 Gazebo Transport 默认位姿输出；该输出没有桥接。当前 ROS odom TF 只由桥接后的 `/odom` 经项目节点生成。

## 阶段 3 ROS 基础运动 bridge 与直行验证

- 既有 `config/bridge.yaml` 继续只负责 Gazebo→ROS 2 `/clock`；`phase3_spawn.launch.py` 新增独立 `robot_bridge`，启动命令仍为 `ros2 launch resilient_nav_simulation phase3_spawn.launch.py`。
- `robot_bridge` 使用严格方向并重映射为标准 ROS 2 名称：

| 方向 | Gazebo Transport | ROS 2 |
| --- | --- | --- |
| ROS→Gazebo | `/model/resilient_nav_robot/cmd_vel` (`gz.msgs.Twist`) | `/cmd_vel` (`geometry_msgs/msg/Twist`) |
| Gazebo→ROS | `/model/resilient_nav_robot/odometry` (`gz.msgs.Odometry`) | `/odom` (`nav_msgs/msg/Odometry`) |
| Gazebo→ROS | `/world/resilient_lab/model/resilient_nav_robot/joint_state` (`gz.msgs.Model`) | `/joint_states` (`sensor_msgs/msg/JointState`) |

- 非默认 `entity_name` 会同步改变三条 Gazebo 源/目标话题，但 ROS 2 名称保持 `/cmd_vel`、`/odom` 和 `/joint_states`。
- `colcon build --symlink-install --packages-up-to resilient_nav_simulation` 成功完成 3 个包；描述包 7 项和仿真包 8 项测试全部通过，工作空间测试汇总为 22 项、0 错误、0 失败、1 项跳过。
- 默认位姿稳定后，向 `/cmd_vel` 发送 `linear.x=0.2 m/s` 的短时直行命令并随后发送全零 Twist。Gazebo 模型 X 从约 `0.000000 m` 移至 `0.549754 m`，Y 和 yaw 仍约为零。
- 停止后间隔 1 秒读取的两次模型位姿一致；`/odom` 给出 X 约 `0.5494 m` 且线速度、角速度均为零。
- `/joint_states` 消息包含 `left_wheel_joint` 和 `right_wheel_joint`，两轮位置均约为 `5.494 rad`，停止后速度接近零。
- 在该次 bridge 子任务验收时，`/tf` 抽查只包含 `base_link` 到左右轮 link 的关节变换；当时没有 odom 到机器人基座的变换，也没有桥接 Gazebo 的 `gz.msgs.Pose_V`。
- 初次直行基线停止后 pitch 约为 `0.309682 rad`、Z 约为 `0.004756 m`，该结果用于触发后续纵向支撑与重心修正，修正结果见下一节。
- 没有增加传感器、`ros2_control`、Nav2 或 SLAM；验证停止后没有 Gazebo、bridge 或 ROS 2 后台进程残留。

## 阶段 3 动力学姿态修正

- 修正前展开 SDF 中，整机纵向重心约为 `x=-0.0058 m`，支撑范围为后球轮 `x=-0.18 m` 到驱动轮轴 `x=0 m`；重心距前支撑边仅约 `0.006 m`，容易在驱动和制动后倾倒到车体前缘。
- 驱动轮轴前移到 `x=0.10 m`，球形支撑轮后移到 `x=-0.20 m`，车体惯性原点调整为 `x=-0.05 m`、`z=-0.04 m`。展开后的整机纵向重心约为 `x=-0.031 m`，到前后支撑边的余量均超过 `0.10 m`。
- `base_link` 高度、车体碰撞盒、轮半径 `0.10 m`、轮距 `0.39 m`、DiffDrive / JointStatePublisher 原生话题和全部 ROS bridge 映射均保持不变。
- `colcon build --symlink-install --packages-up-to resilient_nav_simulation` 成功完成 3 个包；描述包 8 项和仿真包 8 项测试全部通过，工作空间测试汇总为 23 项、0 错误、0 失败、1 项跳过。
- 机器人生成成功；落地稳定后间隔 2 秒读取的两次位姿均为 Z 约 `-0.000001 m`、roll/pitch/yaw 约为零。
- 使用与修正前相同的 ROS `/cmd_vel` 直行/停止序列后，模型 X 从约 `0.000000 m` 移至 `0.403759 m`，Y 和 yaw 约为零。
- 停止后间隔 2 秒读取的两次位姿一致，pitch 约为 `-0.000001 rad`，满足 `|pitch| < 0.05 rad`；`/odom` 给出 X 约 `0.4044 m` 且 twist 全零。
- `/joint_states` 仍包含左右轮关节且停止速度接近零；在该次动力学修正子任务验收时，ROS `/tf` 只包含 `base_link` 到左右轮 link 的关节变换，没有 odom 到基座变换。
- 该次动力学修正没有加入 odom TF、传感器或 `ros2_control`，停止后没有 Gazebo、bridge 或 ROS 2 后台进程残留；后续 odom TF 结果见下一节。

## 阶段 3 ROS odom TF

- `resilient_nav_monitor` 新增 `odom_tf_broadcaster`，运行依赖包括 `geometry_msgs`、`nav_msgs`、`rclpy` 和 `tf2_ros`，安装入口名称同为 `odom_tf_broadcaster`。
- 节点订阅标准 ROS 2 `/odom`，复制每条消息的完整位姿和 `header.stamp`，固定发布 `odom -> base_footprint` 动态 TF；它不使用 Gazebo 原始 frame 名称。
- `phase3_spawn.launch.py` 启动该节点并设置 `use_sim_time=true`，既有启动命令和 `/clock`、`/cmd_vel`、`/odom`、`/joint_states` bridge 保持不变。
- Gazebo 的原生 TF/位姿输出仍没有加入 `robot_bridge`。实际 ROS 图中 `/tf` 的发布者只有 `robot_state_publisher` 和 `odom_tf_broadcaster`；`robot_bridge` 只订阅 `/cmd_vel` 并发布 `/odom`、`/joint_states`。
- 构建成功完成 `resilient_nav_description`、`resilient_nav_monitor` 和 `resilient_nav_simulation`；相关测试汇总为 28 项、0 错误、0 失败、1 项跳过。
- 实际 Launch 中机器人实体创建成功，`odom_tf_broadcaster` 的 `use_sim_time` 为 `True`。`/odom` 抽测约为 45–47 Hz，`tf2_echo odom base_footprint` 连续取得时间为 `30.12`、`31.00`、`31.86`、`32.76`、`33.66 s` 的变换。
- 验证没有增加传感器、`ros2_control` 或新的 Gazebo bridge。Ctrl-C 停止时新节点正常退出；既有 `system_heartbeat` 仍出现一次 `rcl_shutdown already called`，停止后另行检查无仿真进程残留。

## 阶段 3 Gazebo 与 RViz 联合演示

- `resilient_nav_simulation/launch/phase3_demo.launch.py` Include 现有 `phase3_spawn.launch.py`，因此 Gazebo、两条 bridge、`robot_state_publisher`、`odom_tf_broadcaster` 和实体生成均沿用原链路；Demo 文件只条件启动一个 RViz。
- `use_rviz` 默认为 `true`，设为 `false` 可运行完全相同的 Spawn 链但不启动 RViz。`entity_name` 和四个初始位姿参数会透传给现有 Spawn Launch。
- `rviz/phase3_demo.rviz` 使用 `odom` 作为 Fixed Frame，默认启用 Grid、RobotModel 和 TF；Odometry 显示已绑定 `/odom` 但默认关闭，可在 Displays 面板中按需启用。
- RViz 节点使用 `use_sim_time=true`。运行时 ROS 图只有一个 `robot_state_publisher`，没有 `joint_state_publisher` 或 GUI 版本；`/joint_states` 唯一发布者为 `robot_bridge`。
- 相关 3 个包构建成功，测试汇总为 30 项、0 错误、0 失败、1 项跳过。Demo Launch 中机器人实体创建成功，RViz OpenGL 初始化成功，没有报告配置或 TF 错误。
- 运动前 Gazebo 模型 X 为 `0.000000 m`；向 `/cmd_vel` 以 `0.2 m/s` 发送 15 条 10 Hz 消息后立即发送零 Twist，停止后 Gazebo X 为 `0.466559 m`、ROS `/odom` X 为 `0.467200 m`、`odom -> base_footprint` TF X 为 `0.467 m`。
- 停止后的 `/odom` 线速度和角速度均为零。RViz RobotModel 订阅 `/robot_description`，其 TF listener 订阅现有 `/tf`；Gazebo、里程计和 RViz 使用的 TF 位姿在毫米级一致，验证联合显示随同一机器人状态同步移动。
- 本次没有增加传感器、`ros2_control` 或重复的状态发布节点。Ctrl-C 后 RViz、TF 广播和 bridge 正常退出，进程检查没有发现残留；既有 `system_heartbeat` 仍出现一次重复 shutdown 警告。

## 阶段 3 运动测试工具与三模式验收

- `resilient_nav_simulation/scripts/motion_test.py` 安装为 `ros2 run resilient_nav_simulation motion_test`，支持 `straight`、`spin` 和 `arc` 三种模式。
- `--linear-speed`、`--angular-speed` 和 `--duration` 分别配置线速度、角速度和墙钟持续时间；工具以 `20 Hz` 发布 `/cmd_vel`，启动运动前最多等待 `5 s` 发现订阅者。
- 正常结束、异常和 Ctrl-C 共用自动停车清理路径，连续发送 5 条零 Twist；真实 Ctrl-C 验证中工具打印 `Ctrl-C received; zero velocity sent.`，随后两次 `/odom` twist 均为零。
- 初次转向抽测发现 Gazebo DiffDrive 里程计积分的是驱动轮轴中点，而旧 `base_footprint` 位于车体几何中心，两者相差 `0.10 m`，导致旋转时 Gazebo 与 RViz/TF 出现系统性参考点偏差。
- 最小修正把 `base_footprint` 移到轮轴中点，车体、轮、支撑轮、重心和接触几何的相对关系保持不变；DiffDrive 同时显式声明 `odom -> base_footprint` frame。源码 Xacro、`check_urdf` 和 URDF→SDF 转换均通过。
- 三组最终测试均从独立初始位姿运行完整 Gazebo/RViz Demo，命令和停止后的结果如下：

| 模式 | 命令 | 最终 `/odom` `(x, y, yaw)` | 最终 Gazebo `(x, y, yaw)` |
| --- | --- | --- | --- |
| 直行 | `linear.x=0.2 m/s`，`1.5 s` | `(0.276800, 0.000000, 0.000)` | `(0.276159, 0.000000, 0.000)` |
| 原地旋转 | `angular.z=0.6 rad/s`，`1.5 s` | `(0.000000, 0.000000, 0.832)` | `(-0.005241, 0.002314, 0.780)` |
| 圆弧 | `linear.x=0.2 m/s`、`angular.z=0.4 rad/s`，`1.5 s` | `(0.261344, 0.073738, 0.550)` | `(0.254261, 0.069973, 0.525)` |

- 每组停止后的 `/odom` 线速度和角速度均为零；连续 TF 样本保持不变，且 `odom -> base_footprint` 与对应 `/odom` 位姿一致。
- RViz 三次均完成 OpenGL 4.5 初始化；最终节点图同时包含 `/rviz2`、其内部 transform listener、`/odom_tf_broadcaster`、`/robot_state_publisher` 和两条 bridge。RViz 订阅 `/robot_description`，其 listener 订阅 `/tf` 与 `/tf_static`。
- 参考点修正后，三组短时测试的 Gazebo—轮式里程计最大位置差约 `8 mm`，最大航向差约 `0.052 rad`。方向、停止状态和显示链同步；剩余差异属于物理接触结果与基于轮转角积分的里程计之间的模型误差，不代表已完成运动精度标定。
- 最终相关三包构建成功，测试汇总为 43 项、0 错误、0 失败、1 项按既有配置跳过。停止所有 Launch 后没有 Gazebo、RViz、bridge、TF 或运动工具进程残留。
- 本任务没有增加传感器、`ros2_control`、Nav2、SLAM 或新软件依赖；既有 `system_heartbeat` 在 Launch Ctrl-C 时仍报告一次重复 shutdown 警告。

## 阶段 3 收尾复核

- 2026-07-29 执行 `colcon build --symlink-install`，3 个包全部构建成功。
- 执行 `colcon test` 和 `colcon test-result --verbose`，结果为 43 项、0 错误、0 失败、1 项按既有配置跳过。
- 收尾保留本页已记录的机器人生成、三种短时运动、标准 ROS 2 话题、TF 和 Gazebo/RViz 同步结果，不重复运行完整图形仿真与运动流程。
- 收尾停止 Launch 后，未发现 Gazebo、RViz、bridge、TF、心跳或运动工具进程残留；本次 ROS 日志、Launch 参数文件和源码树 Python 缓存已清理。
- 正式验收结论、命令和交接边界见 `docs/PHASE3_SUMMARY.md`。

## 阶段 4 安装准备与固定坐标历史基线

- 初次本机只读调研时 `robot_localization` 尚未安装，APT 软件源候选为 `ros-jazzy-robot-localization` `3.8.3-1noble.20260615.152020`；这是当时快照，详见 `docs/PHASE4_LOCAL_REFERENCE.md`。当前环境已经可发现 3.8.3，见下一节。
- `scripts/install_phase4_dependencies.sh` 只处理官方 Jazzy 包 `ros-jazzy-robot-localization`：加载 `/opt/ros/jazzy/setup.bash`，通过 `ros2 pkg prefix robot_localization` 检查，缺失时显示唯一目标包并要求交互式人工确认，最终安装命令不带 `-y`。本次只执行 `bash -n` 静态检查，没有运行脚本、`sudo` 或 `apt`。
- 机器人 Xacro 新增 `imu_link`、`lidar_link`、`camera_mount_link`、`camera_link`、`camera_optical_frame` 和 `arm_mount_link`，全部通过 fixed joint 接入既有 `base_link` 树；安装外参由 Xacro property 集中维护。
- `camera_link` 保持 +x 前向，`camera_optical_frame` 使用 `rpy=[-pi/2, 0, -pi/2]`，对应 ROS 光学坐标 +x 右、+y 下、+z 前。
- 新增安装 link 只有 visual 或为空，不含 collision、inertial、Gazebo sensor 或新 plugin。第三阶段车体质量、质心、惯性、轮径、轮距、支撑结构、两个既有插件和话题配置保持不变。
- 源码及安装后的 Xacro 均可展开并通过 `check_urdf`。`colcon build --symlink-install --packages-select resilient_nav_description` 成功；包级测试为 13 项、0 错误、0 失败、0 跳过，当前工作空间累计结果为 47 项、0 错误、0 失败、1 项既有跳过。
- 该准备任务没有启动 Gazebo 或 RViz；其后传感器和 EKF 动态结果见对应阶段 4 基线文档。坐标与外参影响说明见 `docs/PHASE4_SENSOR_ARCHITECTURE.md`。

## 阶段 4 多传感器与 EKF 当前基线

- Xacro 已增加 100 Hz IMU、15 Hz 单层二维 GPU Lidar（270°、639 beams，旧 `-135°` 边界 ray 已排除）和 640×480、30 Hz、水平 FOV 1.047 rad 的 RGB-D camera；消息分别使用 `imu_link`、`lidar_link` 和 `camera_optical_frame`。
- ROS 2 稳定接口包含 `/imu/data`、`/scan`、`/camera/color/image_raw`、`/camera/color/camera_info`、`/camera/depth/image_raw` 和 `/camera/depth/camera_info`。没有 PointCloud2 bridge。
- `phase4_sensors.rviz` 以 `odom` 为 Fixed Frame，预配置 RobotModel、TF、Best Effort LaserScan、彩色图和 filtered odometry；浮点深度图与原始 wheel odometry 默认关闭。
- 当前可通过 `/opt/ros/jazzy` 发现 `robot_localization` 3.8.3 及 `ekf_node`。`resilient_nav_localization` 包提供 `config/ekf.yaml` 和完整入口 `phase4_ekf_demo.launch.py`。
- 阶段 4 完整链只把 Gazebo 原始里程计映射为 `/wheel/odometry`，EKF 输入 wheel `vx` 和 IMU yaw rate，输出 `/odometry/filtered`，并独占 `odom -> base_footprint` TF。阶段 3 默认 `/odom` 和旧 broadcaster 不变。
- EKF 频率配置为 20 Hz。80 样本测得仿真 stamp 频率 `20.000 Hz`、墙钟到达率约 `12.470 Hz`，同轮 Gazebo `real_time_factor≈0.6745`。
- 动态直行后 wheel/filtered x 分别约 `0.254200/0.253207 m`；旋转后 yaw 分别约 `0.851/0.799 rad`。无 NaN 或明显跳变；暂停后 wheel、IMU 和 EKF 停止，`/clock` stamp 冻结，恢复后继续。
- 完整技术证据分别见 `docs/PHASE4_IMU_LIDAR_BASELINE.md`、`docs/PHASE4_RGBD_BASELINE.md` 和 `docs/PHASE4_EKF_BASELINE.md`。
- 2026-08-01 收尾复验：完整 `colcon build --symlink-install` 成功完成 4 个包；`colcon test-result --verbose` 汇总为 64 项、0 错误、0 失败、1 项跳过；阶段 4 Launch、bridge/EKF YAML 和 RViz 均存在于 symlink 安装区。

## 工作空间目录与构建流程

`ros2_ws` 下四个目录的关系如下：

| 目录 | 作用 |
| --- | --- |
| `src/` | 受版本控制的包源码和资源，是开发时应修改的来源。 |
| `build/` | `colcon` 为每个包生成的中间构建目录。 |
| `install/` | 构建后的可发现安装前缀；ROS 2 运行时使用这里的包索引、可执行文件和共享资源。 |
| `log/` | `colcon build`、`colcon test` 和结果汇总产生的本地日志。 |

`build/`、`install/` 和 `log/` 都是可重新生成的本地产物，并由 `.gitignore` 排除；它们不是 `src/` 的替代品。

- `colcon build` 解析 `src/` 中的包并生成 `build/`、`install/` 和构建日志。
- `--symlink-install` 尽可能在 `install/` 中建立指向源码或构建产物的符号链接，便于 Python 和资源文件修改后的快速迭代；构建系统或安装规则变化后仍应重新构建。
- `source /opt/ros/jazzy/setup.bash` 加载 ROS 2 及其 vendor 环境；`source ros2_ws/install/setup.bash` 再把当前工作空间叠加到环境中。`source` 只改变当前 shell，不执行构建，也不会自动影响其他已打开的 shell。

当前工作空间包含九个包：

- `resilient_nav_monitor`：阶段 1 的心跳节点和阶段 3 的 odom TF 广播节点。
- `resilient_nav_simulation`：阶段 2 的 Gazebo 世界、桥接，以及阶段 3 的生成、Demo Launch、RViz 资源和运动测试工具。
- `resilient_nav_description`：阶段 3 的基础差速机器人描述及 Gazebo 原生差速/关节状态插件资源。
- `resilient_nav_localization`：阶段 4 的 EKF 参数、完整 Launch 入口和资源测试。
- `resilient_nav_interfaces`：阶段 5 的 `FaultStatus` 消息接口。
- `resilient_nav_fault_injection`：阶段 5 的故障模型、注入器、场景、统一 Launch、faulted EKF 配置、probe、RViz 和 bag 工具，以及阶段 7.2 manual truth event。
- `resilient_nav_health_assessment`：阶段 6 的健康监测、真值评价、配置、Launch 和测试，以及阶段 7.2 相机特征、baseline、stale/freeze monitor、freeze runtime 验证和可选 camera evaluator。
- `resilient_nav_camera`：C920 的 `usb_cam` 参数、正式 CameraInfo YAML、可选 `image_proc` 去畸变、启动入口、不解码 Image payload 的接收时序/元数据探针，以及临时旧 K/D ChArUco 复用验证器。
- `resilient_nav_agent`：RA-1A 的纯 Python Robot Domain；包含严格 Pydantic Schema、Agent Input Sanitizer、Incident/Evidence builder、Robot DomainExtension、只读 Tools、Offline Runtime、Benchmark/Batch 和外部 agent-core Fake 测试，不包含 ROS Adapter 或在线节点。

## C920 硬件采集基线

- `resilient_nav_camera/config/c920.yaml` 固定 `/dev/video0`、MJPG（`usb_cam` 的 `mjpeg2rgb`）、`1280x720`、`15 FPS` 与 `mmap`，并以 `package://resilient_nav_camera/config/c920_camera_info.yaml` 作为 `usb_cam` 的 `camera_info_url`。
- `c920.launch.py` 在 `/camera/c920` 命名空间启动 `usb_cam_node_exe`，因此其原始图像话题为 `/camera/c920/image_raw`，并同时启动 `c920_probe`。
- `c920_probe` 只读取 Image 的接收时刻、width、height、encoding、step、frame_id 和 header stamp；以 `time.monotonic_ns()` 累积帧率和平均/最小/最大帧间隔，每 5 秒输出一次摘要。
- `c920_camera_info.yaml` 是 1280×720 的 ROS camera_calibration 格式文件，使用旧文件中未改动的 5 参数 K/D、`plumb_bob`、单位 `R` 和 `[K|0]` 的 `P`；Launch 把同一默认 URL 显式传给 `usb_cam`，用户可用 `camera_info_url:=...` 覆盖。
- `c920.launch.py` 的 `enable_rectification` 默认 `false`；设为 `true` 时会在 `/camera/c920` 命名空间启动 `image_proc/rectify_node`，把相对 `image` 输入重映射为 `image_raw`，标准输出为 `/camera/c920/image_rect`，并启动只检查元数据、时间戳配对和收帧率的 `rectification_probe`。它不改变 `/camera/c920/image_raw`，也不发布 TF。
- 阶段 7.1 已在 C920 上验证正式 CameraInfo 与 `image_proc` 去畸变链；`rectification_probe` 检查通过。已录制 `/camera/c920/image_raw`、`/camera/c920/camera_info` 和 `/camera/c920/image_rect`，并在无相机条件下完成回放验证。
- `calibration_reuse_validator` 只读 `/tmp/camera_params_old.yaml` 的 OpenCV `camera_matrix` 与 `dist_coeffs`，在 `/camera/c920/image_raw` 上检测固定 5×7、`DICT_5X5_100`、square `0.0288 m`、marker `0.0144 m` 的 ChArUco；它以确定性 pose-fit/holdout 划分报告旧 K/D 的 holdout 重投影误差，不保存或发布标定数据。
- 当前已知技术债：WSL USB/IP 下仍偶发闪帧、帧率波动和图像偏暗；本阶段如实记录，未修改驱动、传输或图像处理参数。

## 阶段 7.2 相机控制与特征基线

- 2026-08-10 对 `/dev/video0` 实机执行只读 `v4l2-ctl --list-ctrls-menus`；完整状态见 `docs/PHASE7_2_CAMERA_CONTROLS_BASELINE.md`，没有执行任何相机控制写操作。
- `resilient_nav_health_assessment/camera_health_features.py` 只接受 NumPy 图像并输出灰度统计、分位数、暗亮比例、Laplacian 方差、边缘密度、熵、帧差和确定性帧指纹，不导入 ROS。
- `camera_health_feature_demo` 自动构造纯黑、纯白、均匀灰、灰度渐变、棋盘、同棋盘高斯模糊、重复帧和轻微变化帧，通过既有特征 API 输出紧凑对照表；运行命令为 `ros2 run resilient_nav_health_assessment camera_health_feature_demo`，不订阅 ROS topic。
- `camera_health_calibrate` 默认订阅 `/camera/c920/image_raw`，通过 `cv_bridge` 转为 NumPy 并复用同一特征 API；默认运行 `60 s`，以 `scenario_label=unspecified` 和自动 UTC `session_id` 写入 `/tmp/phase7_2_calibration/<session_id>/`。同一根目录可保存多个独立 session，显式同名 session 拒绝覆盖。
- `camera_health_baseline_report` 扫描根目录下的 session，按 session 汇总 FPS/max gap，按样本汇总 interarrival、亮度、Laplacian variance、edge density、entropy 和 frame difference；输出 `baseline_report.json` 与终端表格，只含全局/场景描述统计。
- 5-session report 共含 3560 帧，observed FPS 为 `8.22--14.43 Hz`、interarrival p95/p99 约 `0.158/0.249 s`、最大正常 gap 约 `0.382 s`。
- `camera_health_monitor` 默认订阅 `/camera/c920/image_raw` 并以 `5 Hz` 发布 `/health/camera`；复用现有特征与阶段 6 的 `HealthDecision`/`SensorHealth` 消息构造，正式故障包含 stale、exact-fingerprint freeze、underexposed、overexposed、带纹理 reference 的 blurred，以及带近期有信息 reference 的 low-information v1。
- development 配置使用 `stale_timeout_sec=1.0`、`freeze_duration_sec=2.0`、`fault_confirmation_sec=0.6`、`recovery_confirmation_sec=1.0`。underexposed 要求 `mean_gray<=6 && p95<=8 && dark_ratio>=0.90`；overexposed 要求 `mean_gray>=170 && p05>=150 && p95>=180`，不依赖 bright ratio。
- `camera_freeze_source` 默认只读 `/camera/c920/image_raw` 的第一张有效图像，并以 `10 Hz` 向独立 `/test/camera/image_frozen` 发布像素完全相同、ROS stamp 前进的副本；有效源图像到达前不发布。source/output/rate 均可配置，source 与 output 相同会拒绝启动。
- `camera_health_watch` 只订阅 `/health/camera`，状态或故障类型变化时立即打印，状态不变时默认每 `5 s` 打印；不发布消息也不参与 monitor 判断。
- freeze runtime 测试以测试代码构造 `rgb8` Image，贯通 source → frozen topic → monitor；确认消息持续、像素一致、stamp 严格前进、metadata 一致，最终进入 `FAULT/freeze`，且观察期没有 `stale`。
- `manual_fault_event` 启动后发布 SCHEDULED，并按 start delay/duration 自动发布 ACTIVE 和 ENDED；它只写 `/fault_injection/status`，不读写相机话题。
- `health_evaluator` 的 `camera_health_topic` 默认空值以保持阶段 6 旧行为；显式设置 `/health/camera` 后支持 camera model 映射，并分别输出异常检出与 exact classification match。
- evaluator 对已检测且 ENDED 的事件记录其后第一条 HEALTHY，输出 recovery time/delay；原 detection delay、anomaly detected、classification exact match 和 TP/FP/FN/TN 字段保留。
- `phase7_2_camera_health.launch.py` 复用原 `c920.launch.py`，默认启动 C920 + monitor，可选 evaluator/watch；runtime 测试以 `run_camera=false` 验证条件分支和实际参数服务，未打开真实设备。
- 本轮两个目标包重新构建成功；完整包级测试分别为 camera 26 项、health assessment 146 项，均为 0 错误、0 失败、0 跳过；上一轮 fault injection 142 项结果保持。
- calibrate 的 `record_fault_truth=false` 默认不订阅 FaultStatus；启用后 CSV 附加 event/model/state/severity，并在状态转换保存 before/fault/after controls。controls 命令仍只有 `--list-ctrls-menus`。
- `camera_fault_feature_report` 读取单个 truth session，以默认 `2.0 s` margin 排除 ACTIVE/ENDED 两侧过渡，按 pre/active/post 汇总 exposure、blur、low-information 特征和观测 p05–p95 区间，不生成阈值。
- `resilient_nav_health_assessment` 最新构建成功，完整包级测试为 171 项、0 错误、0 失败、0 跳过；欠曝、过曝、模糊、low-information、stale、freeze 和阶段 6 回归均通过。
- 无相机短时 Launch 验证确认 `/health/camera` 固定发布；超过 timeout/confirmation 后为 `state=3`、`detected_fault=stale`、`health_score=0.0`、`confidence=1.0`，进程随后 cleanly 退出。
- stale 已由用户在真实 C920 链路完成实机触发与恢复验证；本次没有重复或扩展 monitor 判定逻辑。
- 5 秒真实 C920 验证采集 54 帧、0 次转换错误，observed FPS 约 `11.0`；interarrival p50/p95/p99 约 `0.066/0.189/0.229 s`，max gap 约 `0.264 s`。该短时结果只验证数据链和输出，不作为最终 baseline 阈值。
- 运行期 controls 快照只执行 `v4l2-ctl --device /dev/video0 --list-ctrls-menus`。现有 `usb_cam` 启动日志会设置其自身控制默认值，自动曝光/白平衡也会使相关实时值变化；这些驱动/相机行为与新采集节点的只读快照必须区分。
- 当前没有修改相机数据的通用故障模型、通用生产阈值或 C920 K/D；阶段 6 默认三传感器语义保持不变。

## RA-1A Robot Agent 环境

- 系统没有 `python` 命令，使用 Python 3.12.3 的 `python3`。系统 Python 受 PEP 668 externally-managed 保护，直接 editable install 被安全拒绝，没有使用 `--break-system-packages`。
- 仓库根目录创建 `.venv` 并启用 `--system-site-packages`，因此同一解释器既能访问 ROS 2/colcon 系统包，又能隔离 Python Agent 依赖；`.venv/` 已由既有 `.gitignore` 排除。
- 独立仓库 `Kyliangarnacho/agent-core` 位于 ResilientNavLab 外的 `/home/kylian/projects/agent-core`，当前提交 `0dcce13`，editable import 路径为 `/home/kylian/projects/agent-core/agent_core/__init__.py`，包版本 `0.1.0`。
- agent-core 声明并安装 Pydantic `>=2.8`；本次实际为 Pydantic `2.13.4`。`resilient_nav_agent/setup.py` 同时声明 `agent-core>=0.1.0` 与 `pydantic>=2.8`，没有在源码中硬编码 sibling 绝对路径。
- Step 1 历史基线中，`resilient_nav_agent` package/colcon 为 58 tests，九包汇总为 461 tests、0 errors、0 failures、1 skipped；以下本轮结果已取代它作为当前验证状态。
- Robot Agent 当前从 plain Mapping 构建 sanitized OfflineAgentInput，并完成只读 Tool/strict Runtime/Benchmark 闭环；已完成 DashScope/Qwen `qwen3.7-flash` 真实 API baseline，但仍没有 Live ROS、RAG、Planner、Recovery 或控制权限。
- 本地、Git 忽略的 `.env` 可映射兼容 client 的 `AGENT_CORE_MODEL_*`；真实 four-case smoke 和完整八 Case coverage 已执行。Qwen 模型拒绝 `response_format=json_object`，real runner 仅关闭该可选 capability、传递 `enable_thinking=false`，并继续使用 schema prompt 与 strict parse。
- agent-core sibling checkout 的 `CompatibleModelClient.complete(stream=False)` 已与 Runtime 契约对齐，并拒绝 streaming 与 Runtime-owned kwargs 覆盖。无网络 Robot CASE-001 probe 经 `CompatibleModelClient` 完成 Analyzer → Tool → final，3 次 transport call 均收到 `stream=False`，诊断/scoring 通过且 leakage 为 0。
- 本轮 Agent package pytest/colcon test 为 88 项通过；九包 build 成功。完整 test 首轮 3 个失败均为已记录的 DDS socket 权限，获准环境只复跑受影响两包后最终为 491 tests、0 errors、0 failures、1 skipped。

## 阶段 5 故障注入闭环

- `resilient_nav_interfaces/msg/FaultStatus.msg` 提供 `SCHEDULED`、`ACTIVE`、`ENDED`、`CANCELLED` 状态枚举和故障真值字段。
- `resilient_nav_fault_injection` 提供 IMU bias、Gaussian noise、dropout、fixed delay，wheel odometry freeze，以及 LaserScan sector blindness。
- `launch/phase5_fault_injection.launch.py` 是阶段 5 统一入口，支持 `scenario_file`、`use_rviz`、`record_bag` 和 `bag_output`；它 Include 阶段 4 健康链，按场景启动注入器，并启动 `faulted_ekf_filter_node`。
- faulted EKF 订阅 `/faulted/wheel/odometry` 和 `/faulted/imu/data`，输出 `/odometry/faulted`，配置 `publish_tf=false`；健康 EKF 继续输出 `/odometry/filtered` 并负责主 `odom -> base_footprint` TF。
- `fault_probe` 使用仿真时间，在有限运行时长后输出单行 JSON，可量化 IMU 差值、dropout、delay、wheel freeze、Lidar NaN 和 EKF 差异。
- `phase5_record_bag` 以场景 ID 和时间戳创建唯一目录；`phase5_replay_bag` 可在不启动 Gazebo 的情况下回放。
- 2026-08-03 收尾构建和测试：`colcon build --symlink-install` 成功完成 6 个包；`colcon test && colcon test-result --verbose` 汇总为 193 项、0 错误、0 失败、1 项跳过。
- 动态验证在沙箱外运行，因为受限沙箱内 ROS 2 DDS/Gazebo 会因网络接口权限报 `getifaddrs: Operation not permitted`。验证记录见 `docs/PHASE5_SUMMARY.md`。

## 阶段 6 健康评估闭环

- `resilient_nav_health_assessment` 订阅原始 IMU、wheel odometry 和 LaserScan，发布 `/health/imu`、`/health/wheel` 和 `/health/scan`；监测范围为 timing/stale/delay、wheel freeze、IMU bias 和 Lidar sector blindness。
- `health_evaluator` 订阅健康输出与 `/fault_injection/status`，生成混淆矩阵、检测延迟和分类结果。`phase6_health_evaluation.launch.py` 组合阶段 5 注入链和阶段 6 节点。
- Lidar 统一链验收结果：`event_count=1`、检测延迟约 `0.6 s`、F1 约 `0.96`。2026-08-05 全工作空间构建成功完成 7 个包，测试汇总为 249 项、0 错误、0 失败、1 项跳过。
- `evaluator_output_json` 的命令行覆盖在统一 Launch 中仍不视为可靠；`config/health_evaluator.yaml` 固定 `/tmp/phase6_health_evaluation.json`，保证 evaluator 在该回退配置下可写出 JSON。

## 安装、桥接与人工验收

- `scripts/install_gazebo_harmonic.sh` 通过已有 ROS 2 软件源安装 `ros-jazzy-ros-gz`；安装日志为 `docs/gazebo_install_20260727.log`。
- `scripts/verify_gazebo_harmonic.sh` 验证 ROS 2 环境、`gz`、Gazebo Sim 版本、核心 `ros_gz` 包和官方 `shapes.sdf` 的无 GUI 服务端启动；日志为 `docs/gazebo_verify_20260727.log`。
- 安装与脚本日志确认 `ros-jazzy-ros-gz` 安装完成，Gazebo Sim 版本为 8.11.0，`ros_gz`、`ros_gz_sim`、`ros_gz_bridge`、`ros_gz_interfaces` 可发现。
- 用户人工确认官方 `shapes.sdf` 图形世界和项目自定义世界正常打开，自定义世界中的三个静态实体可见。
- 用户人工确认：不启动桥接时，Gazebo Transport 可见 `/clock` 而 ROS 2 不可见；启动项目 Launch 后，ROS 2 可见 `/clock`，且 `/system_heartbeat` 的 `use_sim_time` 为 `true`。
- 用户人工确认：暂停 Gazebo 后 ROS 2 `/clock` 和心跳停止，恢复后两者继续。
- 用户把 `box_obstacle` 的 pose 从 `2 0 0.5 0 0 0` 改为 `3 1 0.5 0 0 0`，重新启动后在 Gazebo 中确认新坐标生效。

完整人工验收清单见 `docs/PHASE2_SUMMARY.md`。

## 已知环境加载问题

本阶段安装完成后曾出现当前 shell 找不到 `gz` 的现象。通过 ROS 2 Jazzy 的 vendor 包安装时，`gz` 位于 `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`，不在系统默认的 `/usr/bin`。该目录由 `/opt/ros/jazzy/setup.bash` 的环境钩子加入 `PATH`；如果 shell 在安装前已经加载过 ROS 环境，或当前 shell 根本没有加载 ROS 环境，新路径不会自动出现。重新打开已配置的交互式 Bash，或在当前 shell 中重新执行：

```bash
source /opt/ros/jazzy/setup.bash
```

即可让 `gz` 可见。这是 shell 环境未刷新，不是 Gazebo 安装失败。脚本在 `set -u` 下加载官方 setup 文件时仍临时使用 `set +u`，以避免官方脚本引用未定义的 `AMENT_TRACE_SETUP_FILES`。

## 复核命令

以下均为只读检查命令：

```bash
sed -n '1,20p' /etc/os-release
lsb_release -a
uname -m
dpkg --print-architecture
git --version
python3 --version
node --version
npm --version
codex --version
command -v ros2
command -v colcon
command -v rosdep
printenv ROS_DISTRO
printenv ROS_VERSION
test -f /opt/ros/jazzy/setup.bash
ros2 pkg executables demo_nodes_cpp
ros2 pkg executables demo_nodes_py
command -v gz
gz sim --versions
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
command -v xacro
command -v check_urdf
```

项目工作空间复核：

```bash
cd /home/kylian/projects/resilient_nav_lab/ros2_ws
colcon list
colcon build --symlink-install
colcon test
colcon test-result --verbose
source install/setup.bash
ros2 pkg prefix resilient_nav_monitor
ros2 pkg prefix resilient_nav_simulation
ros2 pkg prefix resilient_nav_description
```

RA-1A Robot Agent 复核：

```bash
cd /home/kylian/projects/resilient_nav_lab
.venv/bin/python -c "import agent_core, pydantic; print(agent_core.__file__); print(pydantic.__version__)"
.venv/bin/python -m colcon list --base-paths ros2_ws/src

cd ros2_ws
../.venv/bin/python -m colcon build --symlink-install \
  --packages-select resilient_nav_agent
../.venv/bin/python -m colcon test --packages-select resilient_nav_agent
../.venv/bin/python -m colcon test-result \
  --test-result-base build/resilient_nav_agent --verbose

source install/setup.bash
ros2 run resilient_nav_agent ra1a_fake_benchmark
```

SDF 语义检查：

```bash
gz sdf -k src/resilient_nav_simulation/worlds/phase2_world.sdf
```

机器人描述复核：

```bash
xacro src/resilient_nav_description/urdf/resilient_nav_robot.urdf.xacro \
  -o /tmp/resilient_nav_robot.urdf
check_urdf /tmp/resilient_nav_robot.urdf
```

Display Launch：

```bash
ros2 launch resilient_nav_description display.launch.py
ros2 launch resilient_nav_description display.launch.py use_gui:=true
```

阶段 3 Gazebo 生成与落地：

```bash
ros2 launch resilient_nav_simulation phase3_spawn.launch.py
```

阶段 3 Gazebo 与 RViz 联合演示：

```bash
ros2 launch resilient_nav_simulation phase3_demo.launch.py
ros2 launch resilient_nav_simulation phase3_demo.launch.py use_rviz:=false
```

阶段 3 运动测试工具：

```bash
ros2 run resilient_nav_simulation motion_test straight \
  --linear-speed 0.2 --duration 1.5
ros2 run resilient_nav_simulation motion_test spin \
  --angular-speed 0.6 --duration 1.5
ros2 run resilient_nav_simulation motion_test arc \
  --linear-speed 0.2 --angular-speed 0.4 --duration 1.5
```

可选初始位姿示例：

```bash
ros2 launch resilient_nav_simulation phase3_spawn.launch.py \
  spawn_x:=0.5 spawn_y:=-0.5 spawn_z:=0.30 spawn_yaw:=0.0
```

命令未输出路径或返回非零状态时，应先确认 ROS 2 环境是否已在当前 shell 中加载，再判断软件是否未安装。

## 当前边界

阶段 2 已完成静态世界、Gazebo—ROS 2 `/clock` 桥和已有心跳节点的仿真时间联动。阶段 3 已完成基础机器人描述、独立关节状态、运行时 TF、独立和 Gazebo 联合 RViz 显示、Gazebo 水平落地、原生差速/关节状态插件、ROS 基础运动 bridge、ROS 侧 odom TF、运动测试工具，以及直行/旋转/圆弧/停车同步基线。阶段 4 已完成 IMU、二维 Lidar、RGB-D 和 wheel odometry + IMU EKF 基线。阶段 5 已完成可复现故障注入闭环；阶段 6 已完成健康评估与真值评价；阶段 7.1 已完成 C920 独立采集、CameraInfo、旧 K/D 复用、`image_proc` 去畸变和 rosbag 回放；阶段 8 自适应融合和阶段 9 健康二维 LiDAR SLAM 已完成。Phase 10 已动态验收 Map Server/AMCL、Costmap、Navfn/RPP、无 Recovery BT、Task 5.1 和 Task 5.2 baseline；Task 5.3 Recovery profile 已代码/静态验证，等待人工动态验收。RA-1A 已完成离线只读 Diagnosis/Benchmark 闭环。完整运动性能、PointCloud2、`ros2_control`、Live Robot Agent 和容错导航仍未实现。
