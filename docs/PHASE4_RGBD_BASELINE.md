# 阶段 4 RGB-D 相机基础接口基线

## 1. 范围与结论

本任务在现有 `camera_mount_link -> camera_link -> camera_optical_frame`
固定坐标链上加入一个 Gazebo Harmonic `rgbd_camera`，并建立四个稳定的
ROS 2 接口：

- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/image_raw`
- `/camera/depth/camera_info`

真实启动文件是 `phase4_rgbd_demo.launch.py`，专用 RViz 配置是
`phase4_sensors.rviz`。Launch 复用 `phase4_imu_lidar_demo.launch.py`，因此阶段
3 的 `/cmd_vel`、`/odom`、`/joint_states`、运行时 TF，以及阶段 4 已有的
`/imu/data` 和 `/scan` 都保留不变。

本任务没有创建 PointCloud2 bridge、目标识别、点云处理、机械臂、
`/wheel/odometry` 或 EKF。

## 2. RGB、Depth 与 CameraInfo

- RGB Image 是逐像素颜色观测。本基线动态消息编码为 `rgb8`，每像素三个
  8 位通道；分辨率为 640×480，单帧 `step=1920`。
- Depth Image 是逐像素距离观测，不是彩色图。本基线动态消息编码为
  `32FC1`，即每像素一个 32 位浮点深度值；分辨率同为 640×480，单帧
  `step=2560`。有效范围由相机裁剪面约束为 0.1–10.0 m。
- CameraInfo 描述图像几何和标定参数，包括宽高、畸变模型以及 K、R、P
  矩阵；它不是图像像素。Gazebo Harmonic 的这个 RGB-D sensor 原生只发布
  一个 `/camera/camera_info`。由于本基线的 RGB 与深度来自同一传感器、使用
  同一光心、同一分辨率和同一 FOV，该原生标定信息分别桥接到 ROS 的彩色和
  深度 CameraInfo 稳定接口。

## 3. 完整数据链

数据链为：

`resilient_nav_robot.urdf.xacro`
→ Xacro 展开为 URDF
→ `ros_gz_sim create` 生成 Gazebo 实体
→ world 层 `gz::sim::systems::Sensors` 使用 `ogre2` 更新 RGB-D sensor
→ Gazebo Transport 发布 Image 和 CameraInfo
→ `ros_gz_bridge parameter_bridge` 按 `phase4_rgbd_bridge.yaml` 单向转换
→ ROS 2 `sensor_msgs/msg/Image` 与 `sensor_msgs/msg/CameraInfo`
→ RViz 的彩色 Image display 或命令行检查工具订阅。

相机 sensor 挂在 `camera_link`，该 link 的坐标约定为 x 向前；
`camera_joint` 的 RPY 为零，因此相机朝机器人前方。ROS 消息使用
`camera_optical_frame`，其固定旋转为 `(-pi/2, 0, -pi/2)`，符合光学帧
x 向右、y 向下、z 向前的约定。

## 4. Gazebo 与 ROS 2 topic 映射

动态 `gz topic -l` 确认了以下原生话题。映射全部为 `GZ_TO_ROS`，Image 和
CameraInfo 使用本机 `ros_gz_bridge` 已验证的类型转换与 `SENSOR_DATA` QoS。

| Gazebo Transport | Gazebo 类型 | ROS 2 topic | ROS 2 类型 |
| --- | --- | --- | --- |
| `/camera/image` | `gz.msgs.Image` | `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/camera_info` | `gz.msgs.CameraInfo` | `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera/depth_image` | `gz.msgs.Image` | `/camera/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/camera_info` | `gz.msgs.CameraInfo` | `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |

Gazebo 还原生发布 `/camera/points`，但本任务明确没有为它建立
`sensor_msgs/msg/PointCloud2` bridge。

## 5. 参数基线

| 参数 | 值 | 选择依据 |
| --- | ---: | --- |
| sensor type | `rgbd_camera` | 本机 Gazebo Harmonic 8.11.0 示例已验证 |
| 挂载 link | `camera_link` | 保持相机本体 x 轴向机器人前方 |
| message frame | `camera_optical_frame` | ROS 光学帧约定；动态消息头已确认 |
| RGB 分辨率 | 640×480 | 当前任务要求 |
| Depth 分辨率 | 640×480 | 单一 RGB-D camera 配置，与 RGB 对齐 |
| horizontal FOV | 1.047 rad（约 60°） | 沿用本机 RGB-D 示例，适合室内前视 |
| update rate | 30 Hz 仿真时间 | 沿用本机 RGB-D 示例；比 20 Hz 提供更连续运动观测 |
| near clip | 0.1 m | 沿用本机 Depth/RGB-D 示例的近裁剪基线 |
| far clip | 10.0 m | 本机 Depth 示例明确支持，且比 100 m 更符合室内范围 |

`gz sdf -p` 会警告 `gz_frame_id` 不是 SDFormat schema 的标准 sensor 子元素，
然后将其作为扩展字段保留。现有 IMU/Lidar 使用同一机制；本次动态消息头
进一步确认相机侧该字段实际生效。

## 6. RViz 配置

`phase4_sensors.rviz` 设置：

- Fixed Frame：`odom`；
- RobotModel：启用，描述来源 `/robot_description`；
- TF：启用；
- LaserScan：启用，topic 为 `/scan`，Reliability 为 Best Effort；
- Color Image：启用，topic 为 `/camera/color/image_raw`，Reliability 为
  Best Effort；
- Depth Image：已预配置 `/camera/depth/image_raw`，但默认关闭。

深度图为 `32FC1`。为避免不同 RViz/OpenGL 环境下浮点深度归一化造成误判，
它不作为默认显示。手动检查可在 Displays 中启用 `Depth Image (manual)`，
或运行：

```bash
ros2 topic echo /camera/depth/image_raw --once --no-arr
```

动态启动时 `rviz2` 成功打开，OpenGL 4.5 初始化成功；RViz 运行时订阅了
`/robot_description`、`/scan` 和 `/camera/color/image_raw`，并创建独立 TF
listener。`ros2 topic info /scan --verbose` 确认 RViz 订阅端实际为
`BEST_EFFORT`。由于当前自动化环境没有可用的桌面截图工具，未保存像素级
RViz 截图；运行时订阅、配置加载和错误日志用于验证自动显示链。

## 7. 动态证据

核验日期为 2026-08-01，启动命令为：

```bash
ros2 launch resilient_nav_simulation phase4_rgbd_demo.launch.py
```

### 7.1 话题与消息

`ros2 topic list -t` 同时看到四个相机接口，以及 `/cmd_vel`、`/odom`、
`/joint_states`、`/imu/data`、`/scan`、`/tf` 和 `/tf_static`。

| ROS topic | frame_id | 时间戳示例 | 宽×高 | 编码/模型 |
| --- | --- | --- | ---: | --- |
| `/camera/color/image_raw` | `camera_optical_frame` | `68.838000000 s` | 640×480 | `rgb8` |
| `/camera/depth/image_raw` | `camera_optical_frame` | `72.304000000 s` | 640×480 | `32FC1` |
| `/camera/color/camera_info` | `camera_optical_frame` | `71.676000000 s` | 640×480 | `plumb_bob` |
| `/camera/depth/camera_info` | `camera_optical_frame` | `71.908000000 s` | 640×480 | `plumb_bob` |

时间戳来自仿真时间。连续已接收消息的时间戳间隔出现 32–34 ms，与 30 Hz
仿真更新率一致；Best Effort 高带宽命令行订阅在机器负载较高时也报告过丢帧。

### 7.2 频率与实时因子

使用 `ros2 topic hz` 单路测得墙钟到达率：

- 彩色图最终窗口平均约 `11.238 Hz`；
- 深度图最终窗口平均约 `8.634 Hz`。

同一轮运行中 Gazebo `/stats` 的 `real_time_factor` 约为 `0.3171`。因此本机
同时运行 Gazebo GUI、RViz、640×480 RGB-D 渲染和 bridge 时没有维持实时
速度；不能把配置的 30 Hz 写成 30 Hz 墙钟吞吐成功。仿真时间戳间隔仍反映
30 Hz sensor 配置。

### 7.3 暂停与停止

调用 `/world/resilient_lab/control` 并请求 `pause: true` 后，Gazebo 返回
`data: true`；随后对 `/camera/color/image_raw` 执行 5 秒单消息等待，以
exit code 124 超时，期间没有新图像，确认图像随仿真暂停停止。

完成验证后向 Launch 发送 Ctrl-C。最终目标进程查询为空，`ros2 node list`
也为空，未发现 Gazebo、RViz、bridge、TF 或项目节点残留。

## 8. 错误现象、判断、定位、修改与验证

### 8.1 专用 RViz 首次未启动

- 现象：第一次启动有 Gazebo 和四条 RGB-D bridge 日志，但没有 `rviz2`
  process。
- 判断：不是图形或 OpenGL 失败，而是 Launch condition 为 false。
- 定位：外层与被包含的 Launch 都使用 `use_rviz`；给内层传入 `false` 时影响
  了外层同名 LaunchConfiguration。
- 修改：外层专用开关改为 `use_sensor_rviz`，默认仍为 `true`；内层继续显式
  使用 `use_rviz=false`，避免重复启动阶段 3 RViz。
- 验证：重建后日志出现 `rviz2-10 process started`，OpenGL 4.5 初始化成功，
  RViz 订阅 RobotModel、LaserScan 和彩色图所需话题。

### 8.2 图像墙钟频率低于 30 Hz并出现命令行丢帧

- 现象：`ros2 topic hz` 只有约 11.24/8.63 Hz，`ros2 topic echo` 偶尔提示
  `A message was lost`。
- 判断：30 Hz 是仿真时间更新配置，不保证低实时因子时的墙钟到达率；
  640×480 RGB 与 32FC1 Depth 也会增加复制和显示负载。
- 定位：Gazebo `/stats` 实测 `real_time_factor≈0.3171`；连续消息的仿真时间戳
  仍可见 32–34 ms 间隔。
- 修改：没有为了掩盖机器负载而降低任务要求的分辨率或虚报频率；保留
  30 Hz 配置并分别记录仿真时间和墙钟证据。
- 验证：四个接口持续存在，消息格式、frame、尺寸正确，暂停联动有效。

### 8.3 启动期显示与退出日志

- RViz 曾丢弃一条时间戳 `0.002 s`、早于 TF cache 的启动期 LaserScan；后续
  `/scan` 保持 publisher/subscription，未出现持续性 TF 错误。判断为初始
  TF 与传感器首帧的启动竞态，本任务没有修改第三阶段 TF。
- Ctrl-C 时既有 `system_heartbeat` 报 `rcl_shutdown already called` 并以 1
  退出。其他 bridge、robot_state_publisher、odom TF 和 RViz 正常退出，Gazebo
  接收 SIGINT 退出。由于本任务明确禁止修改 `resilient_nav_monitor`，只记录
  该既有停止路径问题；最终残留检查通过。

### 8.4 受限环境下的 Launch 只读解析

- 现象：首次运行 `ros2 launch ... --show-args` 时，ROS 2 尝试在
  `~/.ros/log` 建立日志目录，受限检查环境返回 `Read-only file system`。
- 判断与定位：失败发生在 Launch 日志目录初始化，不是 Python Launch 语法
  或参数声明错误。
- 修改：没有修改项目或系统路径；在获准使用正常 ROS 日志目录的环境中重跑。
- 验证：成功列出 `use_sensor_rviz=true`、实体名和全部 spawn 参数。

## 9. 静态、构建与测试证据

- Xacro 展开：成功；
- `check_urdf`：成功，完整固定 frame 树可解析；
- `gz sdf -p` URDF→SDF：成功，RGB-D 参数和 optical frame 扩展均保留；
- `colcon build --symlink-install --packages-select resilient_nav_description resilient_nav_simulation`：
  2 个包成功；
- `colcon test` 与 `colcon test-result --verbose`：58 tests，0 errors，
  0 failures，1 skipped；
- 对本任务涉及的 Launch 和资源测试执行 `ament_flake8`：3 files checked，
  no problems found；
- 新 Launch、bridge YAML 和 RViz 文件经 package 的 `config launch rviz worlds`
  目录安装规则安装，构建后的 share 目录可找到对应文件；
- `git diff --check`：通过，无输出。

## 10. 尚未完成或尚未独立验证

- 没有建立 `/camera/points` → `sensor_msgs/msg/PointCloud2` bridge，也没有点云
  处理、配准、滤波或显示；
- 没有目标识别或其他视觉算法；
- 没有 `/wheel/odometry`；现有阶段 3 `/odom` 保持不变；
- 没有安装、配置或启动 EKF，`robot_localization` 状态不因本任务改变；
- 没有机械臂功能；
- 没有保存 RViz 像素级截图，也没有人工逐像素判断彩色画面内容；已验证
  RViz 进程、配置加载、相关订阅和 QoS；
- 没有验收长时间 30 Hz 墙钟吞吐、RGB/Depth 硬同步误差、深度精度、相机
  畸变模型真实性或外参标定精度。
