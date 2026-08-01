# ResilientNavLab

ResilientNavLab 是一个面向移动机器人的 ROS 2 实验与学习项目，目标是构建多传感器故障注入、健康评估、自适应融合和容错导航平台。

## 当前状态

项目已完成阶段 0 至阶段 4；当前最新完成阶段为**阶段 4：多传感器与局部定位基线**。

- ROS 2 Jazzy、`ros2_ws`、`resilient_nav_monitor` 和 `system_heartbeat` 的阶段 1 基线保持可用。
- 已通过 `ros-jazzy-ros-gz` 安装 Gazebo Harmonic；Gazebo Sim 版本为 8.11.0。
- 已创建 `resilient_nav_simulation` 包，包含自定义 SDF 世界、`/clock` 桥接配置、Python Launch 文件和静态资源测试。
- 自定义世界已在 Gazebo 中人工验证，可见 `ground_plane`、`box_obstacle` 和 `cylinder_checkpoint`。
- Launch 启动后，Gazebo 的仿真时钟经 `ros_gz_bridge` 出现在 ROS 2 `/clock`，`system_heartbeat` 使用 `use_sim_time=true`。
- 暂停和恢复 Gazebo 时，ROS 2 `/clock` 与心跳会同步停止和继续，说明节点确实由仿真时间驱动。
- 修改 `box_obstacle` 的 SDF 位姿并重新启动后，Gazebo 中的坐标变化已生效。
- 已创建 `resilient_nav_description` 包，包含一个经 `xacro` 和 `check_urdf` 验证的简单两轮差速机器人 Xacro。
- 当前机器人描述包含 `base_footprint`、`base_link`、左右驱动轮和一个球形支撑轮，并为实体 link 提供基础几何、碰撞和惯性定义。
- 已添加 `display.launch.py` 和 RViz 配置，可启动 `robot_state_publisher`、关节状态发布器和 RViz；`use_gui` 可在普通与 GUI 关节状态发布器之间切换。
- 已为车体、驱动轮和支撑轮补充 Gazebo 材质与接触摩擦，并为阶段 2 地面补充显式摩擦参数。
- `phase3_spawn.launch.py` 复用阶段 2 世界，从 `robot_description` 把机器人生成到 Gazebo；默认从 `z=0.25 m` 下落。
- 实际启动验证确认实体创建成功，机器人落地后的模型 Z 位姿约为 `-0.000001 m`。
- 机器人 Xacro 已加入 Gazebo Harmonic `DiffDrive` 和 `JointStatePublisher` 系统插件，使用实际左右轮关节、`0.39 m` 轮距和 `0.10 m` 轮半径。
- `phase3_spawn.launch.py` 已把 Gazebo 原生速度、里程计和关节状态分别桥接并重映射为 ROS 2 `/cmd_vel`、`/odom` 和 `/joint_states`，同时保留既有 `/clock` 桥。
- `resilient_nav_monitor` 已新增 `odom_tf_broadcaster`：订阅 `/odom`，沿用消息时间戳，把其中位姿持续发布为 `odom -> base_footprint` TF；阶段 3 Launch 以 `use_sim_time=true` 启动该节点。
- 已新增 `phase3_demo.launch.py`，完整复用现有 Spawn 链并通过 `use_rviz` 可选启动一个 RViz；专用配置以 `odom` 为 Fixed Frame，启用 Grid、RobotModel 和 TF，并预置可选的 `/odom` 显示。
- 已把驱动轮轴前移、球形支撑轮后移，并降低和后移车体惯性原点，使纵向重心从驱动轮轴边缘移到三点支撑区域内部。
- `resilient_nav_simulation` 已新增 `motion_test` 工具，可按设定的线速度、角速度和持续时间发布直行、原地旋转及圆弧命令；正常结束、异常和 Ctrl-C 都会重复发送零速度。
- `base_footprint` 已调整到左右驱动轮轴中点，DiffDrive 显式使用 `odom -> base_footprint` frame 语义，解决转向时 Gazebo 模型与 RViz/TF 因参考点不同产生的系统性偏差。
- 三组 `1.5 s` 短时测试已完成：直行最终 `/odom` 为 `(x, y, yaw) ≈ (0.2768, 0, 0)`，原地旋转约为 `(0, 0, 0.832)`，圆弧约为 `(0.2613, 0.0737, 0.550)`；三组停止后的 twist 均为零。
- Gazebo、`/odom`、TF 和 RViz 数据链已同步验证；短时测试中 Gazebo 与轮式里程计的最大位置差约 `8 mm`、最大航向差约 `0.052 rad`，后者保留为物理接触与轮式里程计之间的正常模型误差。
- 阶段 4 已建立 IMU、二维 Lidar 和 RGB-D Gazebo sensor，以及 `/imu/data`、`/scan` 和四个相机 ROS 2 基础接口；PointCloud2 未桥接。
- `robot_localization` 3.8.3 已可用；新增 `resilient_nav_localization` 包和真实完整入口 `phase4_ekf_demo.launch.py`，以 `/wheel/odometry` 与 `/imu/data` 生成 `/odometry/filtered` 和唯一的 `odom -> base_footprint` TF。
- `phase4_sensors.rviz` 保留 RobotModel、TF、Best Effort LaserScan 和彩色图，并显示 filtered odometry；阶段 3 独立启动仍保持 `/odom` 和原有 TF broadcaster。

阶段 3 已完成机器人描述、独立关节状态发布、运行时 TF、独立与 Gazebo 联合 RViz 显示、物理落地、Gazebo 原生差速插件、ROS 2 基础速度/里程计/仿真关节状态链路、ROS 侧 odom TF、运动测试工具，以及直行、原地旋转、圆弧和停车同步基线。阶段 4 已正式收尾，完成多传感器接口和 wheel odometry + IMU 的固定字段 EKF 基线；Gazebo 原生 TF 未桥接，PointCloud2、`ros2_control`、Nav2、SLAM、故障注入、健康评估、自适应融合和容错导航均未实现。

## 核心方向

项目后续计划围绕以下能力逐步展开：

1. 多传感器数据接入与统一管理。
2. 可控、可复现的传感器故障注入。
3. 在线健康状态评估与异常检测。
4. 基于健康状态的自适应多传感器融合。
5. 面向定位与导航任务的降级运行和容错决策。
6. 仿真与实验指标记录、对比和复现。

详细边界见 [docs/PROJECT_SCOPE.md](docs/PROJECT_SCOPE.md)。

## 当前环境摘要

| 项目 | 当前状态 |
| --- | --- |
| 操作系统 | Ubuntu 24.04.4 LTS（Noble Numbat） |
| CPU 架构 | x86_64（Debian 架构名：amd64） |
| Git | 2.43.0，可用 |
| Python | Python 3.12.3（`python3` 可用） |
| Node.js | v24.18.0，可用 |
| npm | 11.16.0，可用 |
| Codex CLI | 0.145.0，可用 |
| ROS 2 | Jazzy；`ROS_DISTRO=jazzy`，`ROS_VERSION=2` |
| ROS 2 工具 | `ros2`、`colcon`、`rosdep` 可用 |
| ROS 2 基础通信 | 官方 C++ talker 与 Python listener 通信验证通过 |
| ROS 2 工作空间 | `ros2_ws` 已创建；空构建和 `--symlink-install` 包构建均通过 |
| 项目 ROS 2 包 | `resilient_nav_monitor`、`resilient_nav_simulation`、`resilient_nav_description`、`resilient_nav_localization` 可构建并由 ROS 2 发现 |
| 项目 ROS 2 节点 | `system_heartbeat` 发布存活消息；`odom_tf_broadcaster` 从 `/odom` 发布 `odom -> base_footprint` |
| Gazebo | Gazebo Harmonic / Gazebo Sim 8.11.0，可用 |
| ROS 2—Gazebo 集成 | `/clock`、`/cmd_vel`、`/odom` 和 `/joint_states` 的阶段内定向桥接已验证 |
| 阶段 2 仿真资源 | 自定义静态世界、桥接配置和 Launch 集成已验证 |
| 阶段 3 描述与显示 | Xacro、运行时 TF、独立 Display Launch 和 Gazebo/RViz Demo Launch 已验证 |
| 阶段 3 Gazebo 生成 | 模型生成、Gazebo 材质与摩擦、自由落体和稳定落地已验证 |
| 阶段 3 Gazebo 基础运动 | 运动测试工具、直行/旋转/圆弧、自动停车及 Gazebo/RViz/TF 同步已验证 |
| 阶段 4 传感器与定位 | IMU、二维 Lidar、RGB-D、专用 RViz 和 `/wheel/odometry` + IMU 的 20 Hz 二维 EKF 基线已动态验证 |

完整核验结果和复核命令见 [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md)。

## 文档索引

- [项目范围](docs/PROJECT_SCOPE.md)
- [开发环境基线](docs/ENVIRONMENT.md)
- [学习与决策记录](docs/LEARNING_LOG.md)
- [阶段 2 收尾总结](docs/PHASE2_SUMMARY.md)
- [阶段 3 收尾总结](docs/PHASE3_SUMMARY.md)
- [阶段 3 本机参考](docs/PHASE3_LOCAL_REFERENCE.md)
- [阶段 4 本机参考](docs/PHASE4_LOCAL_REFERENCE.md)
- [阶段 4 传感器坐标架构](docs/PHASE4_SENSOR_ARCHITECTURE.md)
- [阶段 4 IMU 与 Lidar 基线](docs/PHASE4_IMU_LIDAR_BASELINE.md)
- [阶段 4 RGB-D 基线](docs/PHASE4_RGBD_BASELINE.md)
- [阶段 4 EKF 基线](docs/PHASE4_EKF_BASELINE.md)

## 近期里程碑

阶段 2 已完成 Gazebo Harmonic 安装、基础世界加载、`/clock` 桥接和 `use_sim_time` 联动验证。阶段 3 已完成机器人描述、运行时 TF、独立及 Gazebo 联合 RViz 显示、Gazebo 物理落地、原生插件、标准 ROS 基础运动话题桥接、ROS 侧 odom TF、运动测试工具，以及直行/旋转/圆弧/停车同步基线。阶段 4 已完成 IMU、二维 Lidar、RGB-D、专用 RViz 和 wheel odometry + IMU EKF 基线；阶段 5 尚未开始，PointCloud2、Nav2、SLAM、故障注入、健康评估、自适应融合和容错导航仍未实现。
