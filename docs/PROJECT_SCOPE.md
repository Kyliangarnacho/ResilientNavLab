# ResilientNavLab 项目范围

## 1. 项目愿景

ResilientNavLab 面向移动机器人在传感器异常、退化或失效条件下的持续定位与导航问题，计划构建一个基于 ROS 2、可配置、可复现、便于对比实验的平台。

平台将把“故障产生—状态感知—融合调整—导航降级—效果评价”组织成完整实验链路，而不是只实现单一算法演示。

## 2. 核心目标

- 接入移动机器人常用的多源传感器数据。
- 注入可控的传感器故障并记录真实故障标签。
- 在线评估传感器健康度和故障状态。
- 根据健康状态动态调整多传感器融合策略。
- 在部分观测不可靠时维持安全、可解释的降级导航能力。
- 建立可重复运行的仿真场景、实验配置和评价指标。

## 3. 计划内能力

### 3.1 多传感器输入

后续可逐步覆盖轮式里程计、IMU、激光雷达、GNSS、相机或其仿真等价数据。具体传感器组合应按阶段确定，不在初始化阶段锁定。

### 3.2 故障注入

计划支持可配置、可复现的故障模型，例如：

- 数据丢失、延迟、乱序或冻结。
- 偏置、漂移、噪声增强和离群值。
- 量程截断、间歇失效和完全失效。
- 与时间、空间区域或机器人状态相关的组合故障。

### 3.3 健康评估

计划利用数据质量、时序一致性、跨传感器残差和模型预测误差等信息，输出可供下游使用的健康状态、置信度或故障类别。

### 3.4 自适应融合

计划根据健康评估结果调整传感器权重、噪声模型、数据选择或融合模式，并保留固定策略作为实验基线。

### 3.5 容错导航

计划研究在传感器退化条件下的继续运行、受限运行、安全停车和恢复机制，并比较故障对定位、规划、控制和任务完成率的影响。

### 3.6 实验与评价

实验应记录场景、参数、随机种子、故障真值、健康评估输出和导航结果。候选指标包括故障检测延迟、误报率、定位误差、轨迹偏差、任务成功率、恢复时间和计算开销。

## 4. 当前阶段边界

阶段 0 至阶段 7.2 已经完成。阶段 4 已建立 IMU、二维 Lidar、RGB-D、专用 RViz，以及轮式里程计与 IMU 的固定字段二维 EKF 基线。阶段 5 已建立可复现故障注入闭环，包含 `FaultStatus` 真值标签、IMU/wheel/Lidar 首批故障模型、统一 Launch、faulted EKF 对照、`fault_probe` JSON 指标、RViz 和 rosbag 记录/回放。阶段 6 已建立 IMU/wheel/scan 的在线健康评估、`health_evaluator` 真值评价和统一 Launch；Lidar 统一链已得到 1 个事件、约 `0.6 s` 检测延迟和约 `0.96` F1。阶段 7.1 已完成 C920 独立采集、CameraInfo、旧 K/D 复用、去畸变和 rosbag 无相机回放。阶段 7.2 已完成 C920 baseline、stale/freeze、保守 underexposed/overexposed/blurred/low-information v1、freeze 自动 runtime 验证、人工 FaultStatus 时间窗和可选 camera evaluator；evaluation config 冻结用于本阶段可复现实验，并非通用生产标定。现有运动基线包含虚拟差速机器人的描述、独立关节状态、运行时 TF、独立和 Gazebo 联合 RViz 显示、Gazebo 物理落地、原生差速插件、ROS 2 基础运动话题、ROS 侧 odom TF、可安全停车的运动测试工具，以及直行、原地旋转和圆弧同步验收。

当前已完成：

- **阶段 0：初始化。** 明确项目目标和初始范围，记录环境基线，建立仓库协作约束、学习日志和忽略规则。
- **阶段 1：ROS 2 基础设施。** 安装 ROS 2 Jazzy，验证官方 talker/listener 通信，创建 `ros2_ws` 工作空间和 `resilient_nav_monitor` 包，实现并验证 `system_heartbeat` 节点。
- **阶段 2：Gazebo 基础仿真与时钟链路。** 安装并验证 Gazebo Harmonic 与 `ros_gz`，创建 `resilient_nav_simulation` 包，完成自定义 SDF 世界、Gazebo 到 ROS 2 的 `/clock` 单向桥接、Python Launch 集成和 `use_sim_time` 暂停/恢复联动验证。
- **阶段 3：虚拟差速机器人与基础运动（已完成）。** 创建 `resilient_nav_description` 包和简单两轮差速机器人 Xacro，完成静态模型、独立 Display Launch、Gazebo 材质与接触参数、复用阶段 2 世界的实体生成和物理落地、Gazebo Harmonic DiffDrive / JointStatePublisher，并桥接 ROS 2 `/cmd_vel`、`/odom` 和 `/joint_states`；新增从 `/odom` 发布 `odom -> base_footprint` 的 ROS 侧 TF 和只追加 RViz 的 Demo Launch；完成动力学姿态修正、轮轴参考点对齐、运动测试工具，以及短时直行、原地旋转、圆弧、正常结束和 Ctrl-C 停车验证。
- **阶段 4：传感器与定位基线（已完成）。** 已完成固定安装坐标、Gazebo IMU/二维 Lidar/RGB-D sensor、定向 `ros_gz_bridge`、专用 RViz，以及 `robot_localization` 的 `/wheel/odometry` + `/imu/data` 二维 EKF；输出 `/odometry/filtered`，并在完整阶段 4 Launch 中独占 `odom -> base_footprint` TF。
- **阶段 5：故障注入（已完成）。** 已完成 `resilient_nav_interfaces/FaultStatus`、`resilient_nav_fault_injection`、IMU bias/noise/dropout/fixed_delay、wheel freeze、Lidar sector blindness、统一 `phase5_fault_injection.launch.py`、faulted EKF `/odometry/faulted`、阶段 5 RViz、`fault_probe` 指标和 rosbag 记录/回放闭环。原始健康 topic 保持不覆盖，faulted EKF `publish_tf=false`。
- **阶段 6：健康评估（已完成）。** 已完成 `resilient_nav_health_assessment` 的 timing/stale/delay、wheel freeze、IMU bias 与 Lidar sector blindness 健康判定，`health_evaluator` 按 `FaultStatus` 输出 JSON 评价，`phase6_health_evaluation.launch.py` 统一阶段 5/6 链路。命令行 `evaluator_output_json` 覆盖仍不作为可靠入口；YAML 固定输出路径为可运行回退。
- **阶段 7.1：C920 相机集成（已完成）。** 已完成 WSL/USBIP + `usb_cam` 采集、正式 CameraInfo、旧 K/D 复用验证、`image_proc` 去畸变，以及 image_raw / camera_info / image_rect 的 rosbag 无相机回放；WSL USB/IP 下的偶发闪帧、帧率波动和图像偏暗仅记录为技术债。
- **阶段 7.2：相机健康（已完成）。** 已完成 baseline、monitor v1、自动验证、人工真值、camera evaluator、联合 Launch，以及 truth-labelled 故障特征采集/描述报告。正式故障包含 stale、exact-fingerprint freeze 和保守 underexposed/overexposed/blurred/low-information v1。

当前工作空间已有八个 ROS 2 软件包：

- `resilient_nav_monitor`：包含 `system_heartbeat` 和 `odom_tf_broadcaster` 节点。
- `resilient_nav_simulation`：包含阶段 2 的 Gazebo 世界、仅声明 `/clock` 的静态 bridge 配置，以及阶段 3 在 Launch 中动态建立的机器人基础运动 bridge、模型生成、Gazebo/RViz Demo、`motion_test` 工具和静态/单元测试。
- `resilient_nav_description`：包含阶段 3 的基础两轮差速机器人 Xacro、Gazebo 原生 DiffDrive / JointStatePublisher 插件、Display Launch 和 RViz 配置，以及阶段 4 的六个固定安装坐标。
- `resilient_nav_localization`：包含 `robot_localization` EKF 配置、完整阶段 4 启动入口和资源测试。
- `resilient_nav_interfaces`：包含阶段 5 `FaultStatus` 消息接口。
- `resilient_nav_fault_injection`：包含阶段 5 故障模型、注入器、场景 YAML、统一 Launch、faulted EKF 配置、probe、RViz、bag 工具和测试，以及阶段 7.2 不修改数据的 `manual_fault_event` 真值窗发布器。
- `resilient_nav_health_assessment`：包含阶段 6 链，以及阶段 7.2 相机特征、baseline/故障采集、描述报告、monitor、freeze 测试/观察、runtime 验证和可选 camera evaluator。
- `resilient_nav_camera`：包含 C920 的 `usb_cam` 基线配置、正式 CameraInfo YAML、阶段 7.1 C920 Launch、probe/去畸变/旧 K/D 验证，以及阶段 7.2 只组合现有节点的 camera health 联合 Launch。

当前明确未完成：

- ROS 2 `/cmd_vel`、`/odom` 和 `/joint_states` 已连接 Gazebo，短时直行、原地旋转、圆弧及停车已验证；尚未系统验收速度精度、长距离累计误差、轨迹跟踪或控制限制。
- Gazebo 原生 TF/位姿输出没有桥接；ROS TF 由 `odom_tf_broadcaster` 只根据桥接后的 `/odom` 单独发布，避免重复来源。
- IMU、二维 Lidar 和 RGB-D 基础接口已建立；IMU/wheel/Lidar 首批故障模型已完成；PointCloud2 bridge、RGB-D 故障模型和长期性能验收尚未完成。
- wheel odometry + IMU 的 odom-frame EKF 基线已建立；Nav2、SLAM、map-frame 全局定位和真实硬件定位尚未开始。
- 可复现故障注入与健康评估闭环已完成；自适应融合和容错导航尚未开发。
- 阶段 7.1 已完成 C920 的正式 CameraInfo、旧 K/D 复用、`image_proc` 去畸变和 image_raw / camera_info / image_rect 的 rosbag 无相机回放；真实机器人部署、相机再标定、TF、修改真实数据的相机故障模型和真实硬件定位尚未开始。阶段 7.2 的 freeze 专项源仅生成隔离测试输入，manual event 仅生成真值标签。
- 阶段 7.2 已完成保守的 stale/freeze/underexposed/overexposed/blurred/low-information v1、人工 `FaultStatus` 时间窗、camera evaluator 和恢复评价接口；冻结 config 只适用于本阶段评价，不构成通用生产阈值。

## 5. 当前不在范围内

- 阶段 3 已按当前基础运动边界收尾；完整运动性能、传感器和导航能力必须在后续任务中单独授权。
- 当前不安装或集成 Nav2、SLAM 及其他尚未授权的软件依赖。
- 阶段 7.2 已收尾；后续视觉故障规则、自适应融合或容错导航必须在单独任务中授权。
- 在仿真链路稳定并形成安全方案前，不开展真实机器人部署。
- 不把尚未验证的算法性能作为项目结论。

## 6. 建议阶段路线

1. **阶段 0：初始化（已完成）。** 完成范围、环境和协作基线。
2. **阶段 1：ROS 2 基础设施（已完成）。** 建立 ROS 2 Jazzy、工作空间、`resilient_nav_monitor` 和 `system_heartbeat` 基线。
3. **阶段 2：Gazebo 基础仿真与时钟链路（已完成）。** 建立 Gazebo Harmonic、`ros_gz`、`resilient_nav_simulation`、自定义世界、`/clock` 桥接和仿真时间联动。
4. **阶段 3：虚拟差速机器人与基础运动（已完成）。** 完成 URDF/Xacro、独立关节状态、运行时 TF、独立和 Gazebo 联合 RViz 显示、Gazebo 水平落地、原生插件、ROS 基础运动 bridge、ROS 侧 odom TF、运动测试工具及三种短时运动/停车同步基线；更完整的运动性能不属于本阶段完成结论。
5. **阶段 4：传感器与定位基线（已完成）。** 已建立 IMU、二维 Lidar、RGB-D、专用 RViz 和 wheel odometry + IMU EKF 基线；PointCloud2 和更高层定位导航仍需单独任务。
6. **阶段 5：故障注入（已完成）。** 已实现故障模型、场景配置、标签、faulted EKF 对照、probe、RViz 与 rosbag 回放闭环。
7. **阶段 6：健康评估（已完成）。** 已建立 IMU/wheel/scan 健康判定、真值评价和统一 Launch；自适应融合未包含在本阶段。
8. **阶段 7：自适应融合与容错导航（计划）。** 实现健康感知融合、降级决策、恢复机制与端到端对照实验。

每个尚未开始的阶段都必须在单独任务中明确授权后开展。
