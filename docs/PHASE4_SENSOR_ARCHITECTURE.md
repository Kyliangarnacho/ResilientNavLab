# 阶段 4 传感器与扩展安装坐标架构

## 1. 范围

- 记录日期：2026-08-01
- 机器人描述：`ros2_ws/src/resilient_nav_description/urdf/resilient_nav_robot.urdf.xacro`
- 坐标基准：所有安装链最终连接到第三阶段既有 `base_link`

本次只建立六个固定安装坐标及少量 RViz visual。没有加入 Gazebo sensor、ROS—Gazebo bridge、`robot_localization`/EKF、机械臂模型、collision 或 inertial；传感器数据链和定位仍未实现。

## 2. 坐标树

```text
base_link
├── imu_link
├── lidar_link
├── camera_mount_link
│   └── camera_link
│       └── camera_optical_frame
└── arm_mount_link
```

六个连接均为 fixed joint，因此 `robot_state_publisher` 会把它们作为静态 TF 发布。本文中的 `xyz` 和 `rpy` 都是相对直接父坐标的 joint origin，单位分别为米和弧度。

## 3. 外参表

| 坐标系 | fixed joint | 父坐标 | xyz | rpy | 当前用途 | 未来对应数据 | 修改外参的影响 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `imu_link` | `imu_joint` | `base_link` | `[-0.05, 0, 0.08]` | `[0, 0, 0]` | IMU 安装和数据 frame 基准；x/y 与车体质心对齐，z 位于车体顶部附近 | `sensor_msgs/msg/Imu` 的角速度、线加速度和姿态 | 改变 IMU 到机体的杠杆臂与轴向变换，会影响融合、重力方向处理和跨传感器残差；已有数据的外参假设也随之失效 |
| `lidar_link` | `lidar_joint` | `base_link` | `[0, 0, 0.20]` | `[0, 0, 0]` | 二维 Lidar 安装基准；高于车体顶部以减少自身遮挡 | `sensor_msgs/msg/LaserScan`，以及可选 `sensor_msgs/msg/PointCloud2` | 改变扫描平面在机器人坐标中的位置和朝向，会影响障碍物投影、地图/点云配准、代价地图和遮挡范围 |
| `camera_mount_link` | `camera_mount_joint` | `base_link` | `[0.18, 0, 0.11]` | `[0, 0, 0]` | 未来相机支架或云台的机械基准；把支架变化与相机本体外参分层 | 其自身不承载数据；作为全部相机数据 frame 的上游安装基准 | 修改它会整体移动或旋转 `camera_link`、`camera_optical_frame` 及所有未来 RGB、Depth 和点云数据 |
| `camera_link` | `camera_joint` | `camera_mount_link` | `[0.07, 0, 0.04]` | `[0, 0, 0]` | 相机本体基准，+x 朝机器人前方；相对 `base_link` 的合成位置为 `[0.25, 0, 0.15]` | 相机本体或驱动使用的非 optical frame、CameraInfo 的机械外参参考 | 修改它会改变相机相对支架的平移/姿态，并连带改变 optical frame、RGB、深度和点云注册 |
| `camera_optical_frame` | `camera_optical_joint` | `camera_link` | `[0, 0, 0]` | `[-π/2, 0, -π/2]` | ROS 标准光学坐标：+x 向图像右、+y 向图像下、+z 沿成像方向向前 | `sensor_msgs/msg/Image`、`sensor_msgs/msg/CameraInfo`、深度图和相机点云的典型 `frame_id` | 修改旋转会改变像素、深度和点云轴语义；错误的零旋转会把 +x 前向机械坐标误当成 +z 前向光学坐标 |
| `arm_mount_link` | `arm_mount_joint` | `base_link` | `[-0.10, 0, 0.075]` | `[0, 0, 0]` | 车体顶面上的未来机械臂安装基准；当前保持为空 link | 未来机械臂根 link、关节状态、末端 TF 和规划模型 | 修改它会整体改变未来机械臂相对底盘的工作空间、TF、碰撞模型和运动规划基准；当前没有机械臂数据 |

## 4. Xacro 参数组织

安装外参集中为下列 Xacro property，避免在 joint 中散落数值：

- `imu_xyz`、`imu_rpy`
- `lidar_xyz`、`lidar_rpy`
- `camera_mount_xyz`、`camera_mount_rpy`
- `camera_xyz`、`camera_rpy`
- `camera_optical_xyz`、`camera_optical_rpy`
- `arm_mount_xyz`、`arm_mount_rpy`

`imu_xyz` 复用第三阶段既有 `base_com_x`，使 IMU 水平位置与车体质心对齐；`arm_mount_xyz` 的 z 复用 `base_height / 2.0`，使安装基准位于车体顶面。修改这些 property 后必须重新展开 Xacro、检查 TF，并重新标定或确认受影响传感器的安装外参。

## 5. RViz visual 与动力学边界

- `imu_link`：`0.04 × 0.03 × 0.015 m` 橙色 box。
- `lidar_link`：半径 `0.04 m`、长度 `0.04 m` 的绿色 cylinder。
- `camera_mount_link`：`0.03 × 0.04 × 0.06 m` 灰色 box。
- `camera_link`：`0.06 × 0.08 × 0.04 m` 灰色 box。
- `camera_optical_frame` 和 `arm_mount_link`：空 link，只表示坐标基准。

这些标记没有 collision 或 inertial，因此不改变第三阶段质量、质心、惯性、接触几何或支撑范围。它们也没有 `<gazebo>` override、`<sensor>` 或新 `<plugin>`。

## 6. 第三阶段兼容性约束

本次保持以下基线不变：

- 车体尺寸 `0.50 × 0.35 × 0.15 m`。
- `base_link` 质量 `5.0 kg`，惯性原点 `[-0.05, 0, -0.04] m` 及惯性张量。
- 轮半径 `0.10 m`、轮宽 `0.04 m`、轮距 `0.39 m`。
- 左右轮 joint、球形支撑轮、接触参数和摩擦。
- Gazebo `DiffDrive` 与 `JointStatePublisher` 两个既有插件及其话题、关节和 frame 配置。
- `base_footprint`、阶段 3 Launch、bridge、`/cmd_vel`、`/odom`、`/joint_states` 和 `odom -> base_footprint` 行为。

新增固定 TF 会扩展 `base_link` 以下的描述树，但不会取代或改变第三阶段 TF 链的既有父子关系。

## 7. 当前未验证内容

- 尚未启动 RViz，因此 visual 的实际显示效果只经过 URDF 静态检查。
- 尚未启动 Gazebo，因此固定 visual 在 Gazebo 的合并/渲染效果未确认。
- 尚未创建或运行任何 IMU、Lidar、RGB、Depth 或 RGB-D sensor。
- 尚未确认传感器 topic、消息 `frame_id`、频率、QoS、噪声、时间戳或同步。
- 尚未安装或运行 `robot_localization`，没有 EKF 输入、输出或 TF 验证。
- `arm_mount_link` 不是机械臂模型，也没有碰撞、运动学或规划能力。
