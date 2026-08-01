# 开发环境基线

## 核验信息

- 最近核验日期：2026-08-01
- 项目目录：`/home/kylian/projects/resilient_nav_lab`
- 当前阶段：阶段 0 至阶段 4 已完成；阶段 4 已建立 IMU、二维 Lidar、RGB-D、专用 RViz 和 wheel odometry + IMU EKF 基线

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
| 项目 ROS 2 包 | `resilient_nav_monitor`（`ament_python`） | 构建、自动测试和 `ros2 pkg prefix` 发现验证通过 |
| 项目 ROS 2 节点 | `system_heartbeat`、`odom_tf_broadcaster` | 心跳发布及 `/odom` 到 `odom -> base_footprint` TF 的端到端验证通过 |
| 仿真资源包 | `resilient_nav_simulation`（`ament_cmake`） | 构建、运动工具测试、阶段 2 世界、阶段 3 生成和 Gazebo/RViz Demo Launch 验证通过 |
| 机器人描述包 | `resilient_nav_description`（`ament_cmake`） | Xacro、运行时 TF、RViz、Gazebo 材质、动力学支撑、DiffDrive、JointStatePublisher 和阶段 4 固定安装坐标验证通过 |
| 定位包 | `resilient_nav_localization`（`ament_cmake`） | EKF 配置、完整阶段 4 Launch、安装和自动测试验证通过 |
| `robot_localization` | `3.8.3`，前缀 `/opt/ros/jazzy` | `ekf_node` 可发现；阶段 4 动态闭环验证通过 |
| Gazebo | Gazebo Harmonic；Gazebo Sim `8.11.0` | `gz` 可用，官方和项目世界均已验证 |
| ROS 2—Gazebo 集成 | `ros-jazzy-ros-gz` `1.0.22` | `/clock` 与阶段 3 基础运动话题的定向 bridge 已验证 |
| 仿真时钟链路 | Gazebo `/clock` → ROS 2 `/clock` | 单向桥接、暂停/恢复和 `use_sim_time` 联动验证通过 |

运行 `codex --version` 时，Codex 成功返回版本号，同时提示当前受限检查环境无法创建 PATH aliases。该提示不影响本次版本识别；如后续需要诊断 Codex PATH 行为，应在对应任务中单独复核。

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

- Xacro 已增加 100 Hz IMU、15 Hz 单层二维 GPU Lidar 和 640×480、30 Hz、水平 FOV 1.047 rad 的 RGB-D camera；消息分别使用 `imu_link`、`lidar_link` 和 `camera_optical_frame`。
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

当前工作空间包含四个包：

- `resilient_nav_monitor`：阶段 1 的心跳节点和阶段 3 的 odom TF 广播节点。
- `resilient_nav_simulation`：阶段 2 的 Gazebo 世界、桥接，以及阶段 3 的生成、Demo Launch、RViz 资源和运动测试工具。
- `resilient_nav_description`：阶段 3 的基础差速机器人描述及 Gazebo 原生差速/关节状态插件资源。
- `resilient_nav_localization`：阶段 4 的 EKF 参数、完整 Launch 入口和资源测试。

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

阶段 2 已完成静态世界、Gazebo—ROS 2 `/clock` 桥和已有心跳节点的仿真时间联动。阶段 3 已完成基础机器人描述、独立关节状态、运行时 TF、独立和 Gazebo 联合 RViz 显示、Gazebo 水平落地、原生差速/关节状态插件、ROS 基础运动 bridge、ROS 侧 odom TF、运动测试工具，以及直行/旋转/圆弧/停车同步基线。阶段 4 已完成 IMU、二维 Lidar、RGB-D 和 wheel odometry + IMU EKF 基线；阶段 5 尚未开始，完整运动性能、PointCloud2、`ros2_control`、Nav2、SLAM、故障注入、健康评估、自适应融合和容错导航均未实现。
