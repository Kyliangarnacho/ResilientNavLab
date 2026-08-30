# ResilientNavLab

ResilientNavLab 是一个面向移动机器人的 ROS 2 实验与学习项目，目标是构建多传感器故障注入、健康评估、自适应融合和容错导航平台。

## 当前状态

项目已完成阶段 0 至阶段 9，并完成 Robot Diagnostic Agent 的 RA-1A Offline Diagnosis 闭环。现有离线只读 Domain、严格 Schema、Ground Truth Sanitizer、双通道 Case 和 8 个 reference fixture 保持不变。

Phase 8 里程碑 1–4 已冻结四条实验链与真值隔离，并新增 `resilient_nav_fusion`：纯 Python `FusionPolicy` 可基于 wheel/IMU 健康状态给出测量接纳、显式协方差倍率、wheel yaw fallback 与恢复滞回；`measurement_adapter` 只发布匿名化 `/fusion/input/*` 和 `/fusion/status`；独立 adaptive EKF 只订阅这些输入，固定发布 `/odometry/adaptive` 且不发布 TF。没有控制或容错导航实现。

Phase 9 已完成健康二维 LiDAR SLAM baseline：`resilient_nav_slam` Borrow 本机 Jazzy Slam Toolbox，healthy EKF 独占 `odom -> base_footprint`，Slam Toolbox 独占 `map -> odom` 和 `/map`。Occupancy Map 与 Serialized Pose Graph 已保存并能在全新 process localization reload；Ground Truth 仅在独立 evaluation overlay 中用于事后评价。Mapping 的正式 SLAM RMSE 使用 timestamp-associated fixed-scale 2D best-fit SE(2)（SVD/Kabsch）；persisted-map localization 则只使用实验前声明的固定 `T_map_odom`，把该次 fresh-odom 的 GT/EKF 直接变换到保存的 M8 `map` frame，绝不从本次轨迹拟合。corrected dynamic different-start run 的 `T_map_odom=(5.5,4.0,pi)`、初始化区域及 fixed-frame SLAM/Healthy EKF RMSE 为 `0.3434 m/0.1788 rad` 与 `0.4594 m/0.1905 rad`；SLAM 相对 Healthy EKF 改善，但 endpoint error 仍分别为 `3.022 m/1.612 rad`，未设成功阈值，不美化为高精度定位。运行中 `/map` raster 会扩展，保留为 localization-runtime 观察。automatic loop closure 已 PASS：最终无 manual service 的 run 保留了上游直接证据 `TryCloseLoop accepted → LinkChainToScan → CorrectPoses`，证明候选接受、约束链接与位姿校正依次发生。Nav2、fault-aware SLAM 和 Adaptive EKF+SLAM 比较不在 Phase 9 范围。

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
- 阶段 5 已新增 `resilient_nav_interfaces` 和 `resilient_nav_fault_injection`，提供 `FaultStatus`、IMU bias/noise/dropout/fixed_delay、wheel freeze、Lidar sector blindness、统一 Launch、faulted EKF、probe、RViz 和 bag；阶段 7.2 复用同一接口新增不修改数据的 `manual_fault_event`。
- 阶段 5 统一链保持原始 `/imu/data`、`/wheel/odometry`、`/scan` 和健康 `/odometry/filtered` 不被覆盖；故障数据发布到 `/faulted/*`，faulted EKF 输出 `/odometry/faulted` 且 `publish_tf=false`。
- 阶段 6 已新增 `resilient_nav_health_assessment`，对 IMU、wheel 和 scan 输出 `SensorHealth`，覆盖 timing/stale/delay、wheel freeze、IMU bias 和 Lidar sector blindness；`health_evaluator` 以 `FaultStatus` 真值输出 JSON 评价，统一 Launch 为 `phase6_health_evaluation.launch.py`。
- Lidar 统一链评价得到 `event_count=1`、检测延迟约 `0.6 s`、F1 约 `0.96`。`evaluator_output_json` 命令行覆盖在统一 Launch 中仍不视为可靠路径，`health_evaluator.yaml` 固定 `/tmp/phase6_health_evaluation.json` 作为可运行回退。
- 阶段 7.1 已完成 `resilient_nav_camera` 的 C920 数据链：WSL/USBIP + `usb_cam` 从 `/dev/video0` 以 MJPG、`1280x720`、`15 FPS` request、`mmap` 发布 `/camera/c920/image_raw`；正式 CameraInfo、旧 K/D 复用验证、`image_proc` 去畸变至 `/camera/c920/image_rect`，以及 image_raw / camera_info / image_rect 的 rosbag 录制和无相机回放均已通过。
- 阶段 7.2 已只读记录 C920 控制状态并完成 5-session baseline；`camera_health_monitor` 正式判定 stale、exact-fingerprint freeze、underexposed、overexposed、需要近期纹理参考的 blurred，以及需要近期有信息 reference 的保守 low-information v1。freeze 自动 runtime 测试已贯通构造 Image、隔离 frozen topic 与 monitor；`manual_fault_event` 只标记人工故障真值窗，既有 `health_evaluator` 可显式订阅 camera。
- `phase7_2_camera_health.launch.py` 复用阶段 7.1 C920 Launch，并组合 monitor、可选 evaluator/watch；manual truth event 仍由第二终端单独启动。evaluator JSON 已覆盖 detection、classification、recovery 和混淆矩阵字段。
- `camera_health_calibrate` 可选记录 camera FaultStatus 到逐帧 CSV，并在 SCHEDULED/ACTIVE/ENDED 保存三阶段只读 V4L2 controls；`camera_fault_feature_report` 以 transition margin 比较 pre/active/post 分布，只输出描述统计和候选区间。
- RA-1A 已形成 `OfflineAgentInput → RobotDomainExtension → agent-core AgentRuntime → 0..N read-only Tools → strict DiagnosisResult → OfflineDiagnosisRun` 执行链；三个 Tool 分别提供 Incident health snapshot、组件 health 比较和 metric window 检查。
- evaluator 侧以独立 `BenchmarkTruth` 对 `OfflineDiagnosisRun` 判卷并生成 `BenchmarkCaseResult` / `BenchmarkReport`；会检测 component/fault/top-k、Evidence 引用、最小 Evidence 类型、可确定 unsupported claim、Tool/model 使用、leakage 与 healthy false diagnosis。
- Robot Agent 仍不订阅 Live ROS，不接收 evaluator truth，不提供 RAG、Planner、Recovery、参数写入或控制输出。`ra1a_real_benchmark` 已完成 DashScope/Qwen `qwen3.7-flash` 的真实四 Case smoke 和八 Case baseline；具体结果与模型失败边界见 Final Audit。

阶段 3 至 9 的既有基线保持完成。**Phase 10 CLOSED — engineering accepted with known limitations**：healthy saved-map 定位、Global/Local Costmap、Navfn、RPP、无 Recovery BT 1 Hz 重规划、GT-isolated benchmark、动态 detour、fully blocked safe failure、官方 Recovery 和 Goal Cancel 均已收口；Task 1–3/Nav2 参数未修改。严格 Task 4 9/9 未达成（8/8 valid PASS）、定位/scan-TF 偏差、长墙终止/clear 观测限制均透明保留。详见 `docs/PHASE10_SUMMARY.md`。fault-aware navigation 与 Agent navigation 尚未实现；RA-1A 仍为离线只读诊断。

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
| 项目 ROS 2 包 | 共 12 包；`resilient_nav_navigation` 是第 12 个 ROS package |
| 项目 ROS 2 节点 | `system_heartbeat` 发布存活消息；`odom_tf_broadcaster` 从 `/odom` 发布 `odom -> base_footprint` |
| Gazebo | Gazebo Harmonic / Gazebo Sim 8.11.0，可用 |
| ROS 2—Gazebo 集成 | `/clock`、`/cmd_vel`、`/odom` 和 `/joint_states` 的阶段内定向桥接已验证 |
| 阶段 2 仿真资源 | 自定义静态世界、桥接配置和 Launch 集成已验证 |
| 阶段 3 描述与显示 | Xacro、运行时 TF、独立 Display Launch 和 Gazebo/RViz Demo Launch 已验证 |
| 阶段 3 Gazebo 生成 | 模型生成、Gazebo 材质与摩擦、自由落体和稳定落地已验证 |
| 阶段 3 Gazebo 基础运动 | 运动测试工具、直行/旋转/圆弧、自动停车及 Gazebo/RViz/TF 同步已验证 |
| 阶段 4 传感器与定位 | IMU、二维 Lidar、RGB-D、专用 RViz 和 `/wheel/odometry` + IMU 的 20 Hz 二维 EKF 基线已动态验证 |
| 阶段 5 故障注入闭环 | IMU、wheel、Lidar 故障注入、`FaultStatus`、faulted EKF、`fault_probe`、RViz 和 rosbag 记录/回放已验证 |
| 阶段 6 健康评估 | IMU/wheel/scan 健康监测、`health_evaluator`、统一 Launch、JSON 评价与运行级参数服务测试已验证 |
| 阶段 7.1 C920 相机集成 | WSL/USBIP + `usb_cam`、正式 CameraInfo、旧 K/D 复用、`image_proc` 去畸变和相机 rosbag 无硬件回放已验证；仍有 WSL USB/IP 闪帧、帧率波动和偏暗技术债 |
| 阶段 7.2 相机健康 | 已完成真实 C920 健康监测、真值/评价、runtime 验证和 evaluation config 冻结；339 tests passed / 0 failures |
| Phase 10 Nav2 基线 | **CLOSED — engineering accepted with known limitations**。官方 Jazzy Nav2 1.3.12 binary；Task 1–3、Task 4（8/8 valid PASS）与 Task 5.1–5.4 已收口。5.2 冻结 Planner-first `NO_VALID_PATH/208` safe failure，5.3 engineering acceptance 保留有限观测 limitation，5.4 native Goal Cancel PASS；未修改 Task 1–3 参数。详见 `docs/PHASE10_SUMMARY.md`。 |
| Phase 10 Task 4 benchmark | **CLOSED — engineering accepted with a known infrastructure limitation**。GT-free `NavigateToPose` runner、evaluator-only GT recorder、offline evaluator、fresh-process isolation 与 teardown barrier 已实现。最终证据为 8/8 valid navigation PASS（simple 3/3、detour 3/3、multi-turn 2/2 valid）；multi-turn-r03 为 pre-goal infrastructure-invalid，严格 9/9 automated contract 未满足，不再追加动态验证。 |
| RA-1A Robot Agent | 3 个只读 Tools、strict Runtime、Scorer/Batch 和 8-case Fake pipeline 已完成；Agent package 88 tests，九包 491 tests / 0 failures / 1 skipped |

完整核验结果和复核命令见 [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md)。

## 文档索引

- [项目范围](docs/PROJECT_SCOPE.md)
- [开发环境基线](docs/ENVIRONMENT.md)
- [学习与决策记录](docs/LEARNING_LOG.md)
- [当前状态](docs/CURRENT_STATE.md)
- [Phase 10 Nav2 Jazzy 接口与定位骨架](docs/PHASE10_NAV2_JAZZY_INTERFACE.md)
- [Phase 10 开源项目基线](docs/PHASE10_OPEN_SOURCE_BASELINES.md)
- [Phase 10 Task 1 动态定位证据](docs/PHASE10_TASK1_LOCALIZATION_SMOKE.md)
- [Phase 10 Task 2 Footprint、Global/Local Costmap 与参数证据](docs/PHASE10_TASK2_COSTMAP_SMOKE.md)
- [Phase 10 Task 3.1 Planner Server smoke](docs/PHASE10_TASK3_PLANNER_SMOKE.md)
- [Phase 10 Task 3.2 Controller Server smoke](docs/PHASE10_TASK3_CONTROLLER_SMOKE.md)
- [Phase 10 Task 3.3 BT Navigator smoke](docs/PHASE10_TASK3_BT_NAVIGATION.md)
- [Phase 10 Task 4 Healthy Navigation Benchmark](docs/PHASE10_TASK4_HEALTHY_NAVIGATION_BENCHMARK.md)
- [Phase 10 Task 5 Dynamic Obstacles](docs/PHASE10_TASK5_DYNAMIC_OBSTACLES.md)
- [Phase 10 Final Summary](docs/PHASE10_SUMMARY.md)
- [Phase 10 Evidence Index](docs/PHASE10_EVIDENCE_INDEX.md)
- [Phase 10 Task 4 multi-turn fresh evidence](docs/PHASE10_TASK4_MULTI_TURN_20260829_HOST.md)
- [阶段 2 收尾总结](docs/PHASE2_SUMMARY.md)
- [阶段 3 收尾总结](docs/PHASE3_SUMMARY.md)
- [阶段 3 本机参考](docs/PHASE3_LOCAL_REFERENCE.md)
- [阶段 4 本机参考](docs/PHASE4_LOCAL_REFERENCE.md)
- [阶段 4 传感器坐标架构](docs/PHASE4_SENSOR_ARCHITECTURE.md)
- [阶段 4 IMU 与 Lidar 基线](docs/PHASE4_IMU_LIDAR_BASELINE.md)
- [阶段 4 RGB-D 基线](docs/PHASE4_RGBD_BASELINE.md)
- [阶段 4 EKF 基线](docs/PHASE4_EKF_BASELINE.md)
- [阶段 5 架构设计稿](docs/PHASE5_ARCHITECTURE.md)
- [阶段 5 收尾总结](docs/PHASE5_SUMMARY.md)
- [阶段 6 收尾总结](docs/PHASE6_SUMMARY.md)
- [阶段 7.2 C920 控制状态基线](docs/PHASE7_2_CAMERA_CONTROLS_BASELINE.md)
- [阶段 7.2 相机健康特征说明](docs/PHASE7_2_HEALTH_FEATURES.md)
- [Robot Agent 长期开发规则](docs/ROBOT_AGENT_DEV_RULES.md)
- [Robot Agent 开源项目基线](docs/OPEN_SOURCE_BASELINES.md)
- [RA-1A Step 1 Robot Agent Bootstrap](docs/RA1A_STEP1_ROBOT_AGENT_BOOTSTRAP.md)
- [RA-1A Offline Diagnosis Final](docs/RA1A_OFFLINE_DIAGNOSIS_FINAL.md)
- [RA-1A Final Audit](docs/RA1A_FINAL_AUDIT.md)

## 近期里程碑

阶段 2 至 9 的已授权基线保持完成。Phase 10 已 CLOSED — engineering accepted with known limitations；Task 1–5.4 与 evidence boundary 均冻结，详见 `docs/PHASE10_SUMMARY.md`。fault-aware/Agent navigation 与通用自主导航仍未实现。RA-1A 已完成离线、只读 Diagnosis pipeline 和 Fake 判卷基础设施；Fake 分数不代表真实 Robot Agent 智力，且当前没有 Live ROS Agent、真实模型结果、Agent Planner、Recovery、全部视觉退化、通用生产阈值、真实硬件定位或容错导航。WSL USB/IP 帧异常、帧率波动和偏暗画面仍作为已知验证边界保留。
