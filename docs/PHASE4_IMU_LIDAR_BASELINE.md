# 阶段 4 IMU 与二维 Lidar 数据链基线

## 1. 范围与结论

- 记录日期：2026-08-01
- ROS 2：Jazzy
- Gazebo：Harmonic / Gazebo Sim 8.11.0
- 机器人描述：`ros2_ws/src/resilient_nav_description/urdf/resilient_nav_robot.urdf.xacro`
- 演示入口：`ros2 launch resilient_nav_simulation phase4_imu_lidar_demo.launch.py`

本基线首次打通 Gazebo IMU 与单层二维 GPU Lidar 到 ROS 2 的数据链。第四阶段 Launch 复用完整的第三阶段 Demo 链，只追加一个传感器 bridge；`/cmd_vel`、`/odom`、`/joint_states`、现有 TF 和 DiffDrive 保持原组织与命名。

本次没有启动 EKF，没有把 `/odom` 改为 `/wheel/odometry`，没有加入 RGB、Depth 或 RGB-D Camera，也没有桥接 PointCloud2。

## 2. 两条完整数据链

### 2.1 IMU

```text
phase2_world.sdf
  gz::sim::systems::Imu
    -> Xacro 的 imu_link / imu_sensor
    -> Gazebo Transport /imu/data (gz.msgs.IMU)
    -> sensor_bridge，GZ_TO_ROS
    -> ROS 2 /imu/data (sensor_msgs/msg/Imu)
```

IMU sensor 通过 `<gazebo reference="imu_link">` 附着到既有 `imu_link`。URDF 转 SDF 时 fixed link 会归并到 `base_footprint`，但转换结果保留 IMU 的合成 pose；`gz_frame_id=imu_link` 明确消息坐标系。

### 2.2 二维 Lidar

```text
phase2_world.sdf
  gz::sim::systems::Sensors，render_engine=ogre2
    -> Xacro 的 lidar_link / lidar_sensor
    -> Gazebo Transport /scan (gz.msgs.LaserScan)
    -> sensor_bridge，GZ_TO_ROS
    -> ROS 2 /scan (sensor_msgs/msg/LaserScan)
```

Lidar sensor 通过 `<gazebo reference="lidar_link">` 附着到既有 `lidar_link`，使用本机 Gazebo Harmonic 已验证的 `gpu_lidar` 和 `<lidar>` 配置。垂直方向只有一个样本，因此 ROS 输出是二维扫描。

GPU Lidar 在 Gazebo Transport 内部还会建立原生 `/scan/points` 流，这是该 Gazebo sensor 的伴随输出。本项目没有为它建立 bridge，实际 ROS 图中没有 `/scan/points` 或任何 `sensor_msgs/msg/PointCloud2` 话题。

## 3. Sensor 参数与选择理由

### 3.1 IMU

| 参数 | 值 | 理由 |
| --- | ---: | --- |
| `type` | `imu` | 使用 Gazebo Harmonic 的原生 IMU sensor |
| `update_rate` | `100 Hz` | 满足短周期角速度和加速度采样基线 |
| Gazebo / ROS 话题 | `/imu/data` | 直接提供标准 ROS IMU 接口，不需要 remap |
| frame | `imu_link` | 与既有固定安装坐标一致 |
| 角速度噪声 | Gaussian，mean `0`，stddev `0.0002 rad/s` | 小但非零；静止时可观察噪声，又不掩盖基础运动 |
| 线加速度噪声 | Gaussian，mean `0`，stddev `0.01 m/s²` | 小但非零；相对重力加速度足够小 |
| orientation | 启用 | Gazebo 8 的 IMU 能基于仿真姿态产生 orientation；不人为构造一个不受 sensor 支持的估计 |

Gazebo 根据上述标准差实际给出了角速度协方差对角项约 `4.0e-8` 和线加速度协方差对角项约 `1.0e-4`。Gazebo 的 orientation 是仿真姿态输出，本次实际消息的 orientation covariance 为全零；项目没有在 Xacro、bridge 或额外节点中伪造 orientation covariance。该零矩阵只记录当前模拟器输出能力，不是现实 IMU 标定结论，也没有被送入 EKF。

### 3.2 二维 Lidar

| 参数 | 值 | 理由 |
| --- | ---: | --- |
| `type` | `gpu_lidar` | 本机 Harmonic 8.11.0 已验证并由 Sensors System / Ogre 2 承载 |
| `update_rate` | `15 Hz` | 室内低速移动机器人可获得足够及时的障碍物更新，同时控制渲染负载 |
| 水平样本 | `640` | 与本机示例一致，提供约 `0.423°` 的角分辨率 |
| 水平视场 | `[-135°, +135°]`，共 `270°` | 覆盖前方和大部分侧后方，适合室内差速机器人 |
| 垂直样本 | `1`，角度 `0 rad` | 明确限制为单水平扫描层 |
| 最小 / 最大距离 | `0.08 / 12.0 m` | 避开传感器近场，同时覆盖当前 20 m 级室内测试世界 |
| 距离分辨率 | `0.01 m` | 与基础室内障碍检测精度相称 |
| 距离噪声 | Gaussian，mean `0`，stddev `0.005 m` | 小但非零，且小于声明的 1 cm 距离分辨率 |
| Gazebo / ROS 话题 | `/scan` | 直接提供标准 LaserScan 接口 |
| frame | `lidar_link` | 与既有固定安装坐标和扫描平面一致 |

## 4. Gazebo—ROS 映射与 Launch 结构

`config/phase4_sensor_bridge.yaml` 只包含以下两个 `GZ_TO_ROS` 映射，并使用 `SENSOR_DATA` QoS：

| Gazebo Transport | Gazebo 类型 | ROS 2 | ROS 类型 | 方向 |
| --- | --- | --- | --- | --- |
| `/imu/data` | `gz.msgs.IMU` | `/imu/data` | `sensor_msgs/msg/Imu` | Gazebo → ROS |
| `/scan` | `gz.msgs.LaserScan` | `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS |

`phase4_imu_lidar_demo.launch.py` Include `phase3_demo.launch.py`，透传 `use_rviz`、实体名和四个生成位姿参数，然后只启动一个 `sensor_bridge`。因此 Gazebo world、时钟 bridge、robot bridge、`robot_state_publisher`、`odom_tf_broadcaster`、实体生成和可选 RViz 都仍由第三阶段链管理。

world 层新增两个本机已核验 System：

- `gz-sim-sensors-system` / `gz::sim::systems::Sensors`，`render_engine=ogre2`，负责 GPU Lidar。
- `gz-sim-imu-system` / `gz::sim::systems::Imu`，负责 IMU。

## 5. frame_id、TF、频率、噪声与仿真时间

两个安装 joint 都是 `base_link` 的 fixed child：

| TF | translation (m) | rotation RPY (rad) | 实测消息 frame_id |
| --- | --- | --- | --- |
| `base_link -> imu_link` | `[-0.05, 0, 0.08]` | `[0, 0, 0]` | `imu_link` |
| `base_link -> lidar_link` | `[0, 0, 0.20]` | `[0, 0, 0]` | `lidar_link` |

动态验证中 `tf2_echo` 取得了上述两个变换。消息时间戳来自 Gazebo sensor 的仿真更新时间，而不是墙钟：并发抽样时 `/clock=144.322 s`，Lidar header 为 `144.409 s`；IMU 另一次 header 为 `141.140 s`。时间戳为百秒级仿真运行时间，而启动日志墙钟为 Unix epoch `1785515xxx s`。

配置频率与本次墙钟接收频率分别为：

| 话题 | 配置频率 | `ros2 topic hz` 短时结果 | 说明 |
| --- | ---: | ---: | --- |
| `/imu/data` | `100 Hz` | 约 `86.0 Hz` | 400 样本窗口；当时 Gazebo 实时因子低于 1 |
| `/scan` | `15 Hz` | 约 `13.1 Hz` | 69 样本窗口；与相同实时因子相符 |

这两个 `ros2 topic hz` 数值是按墙钟统计的到达率，不把它们误写为 sensor 配置值。暂停 Gazebo 后，两条话题分别连续测量 4 秒，仅报告“topic does not appear to be published yet”，没有收到新样本；恢复后 `/clock` 和两个 header 继续推进，验证传感器随仿真时间暂停与恢复。

## 6. 实际运行证据

本次在非 RViz 模式运行：

```bash
ros2 launch resilient_nav_simulation \
  phase4_imu_lidar_demo.launch.py use_rviz:=false
```

实际结果：

- `ros_gz_sim create` 报告 `Entity creation successful`。
- `sensor_bridge` 明确建立 `/imu/data` 的 `gz.msgs.IMU -> sensor_msgs/msg/Imu` 和 `/scan` 的 `gz.msgs.LaserScan -> sensor_msgs/msg/LaserScan` 两条 bridge。
- ROS 即时话题图包含 `/clock`、`/cmd_vel`、`/odom`、`/joint_states`、`/imu/data`、`/scan`、`/tf`；没有 PointCloud2 话题。
- 一条 IMU 消息为 `frame_id=imu_link`、stamp `55.850 s`；静止时角速度为约 `8.0e-5, 7.7e-5, -1.0e-4 rad/s`，线加速度约为 `0.0099, 0.0034, 9.8004 m/s²`，符合重力和非零噪声预期。
- 一条 LaserScan 为 `frame_id=lidar_link`、stamp `69.829 s`，角度 `[-2.35619, 2.35619] rad`、角增量 `0.00737463 rad`、量程 `[0.08, 12.0] m`，`ranges` 与 `intensities` 长度均为 640；实际 ranges 同时包含无返回的 `inf` 和约 `2.39–3.70 m` 的障碍物距离。
- 暂停服务返回 `data: true`；暂停后 IMU 与 scan 均无新消息；恢复服务同样返回 `data: true`。
- Ctrl-C 后 sensor/robot/clock bridge、状态发布器、odom TF 和 Gazebo 均退出；最终进程检查无残留。

## 7. 静态、构建与测试结果

- 源码 Xacro 成功展开，`check_urdf` 成功解析以 `base_footprint` 为根的完整树。
- `gz sdf -p` 成功把展开 URDF 转为 SDF，并保留两个 sensor、合成 pose、topic、噪声和 `gz_frame_id`。
- 聚焦源码 pytest：29 项通过。
- `colcon build --symlink-install --packages-select resilient_nav_description resilient_nav_simulation`：2 个包成功。
- 描述包：14 项通过；仿真包的 motion test 12 项和资源测试 15 项均通过。两个包的 `colcon test-result` 分别汇总为 15 项和 29 项，均为 0 错误、0 失败、0 跳过。
- `git diff --check`：通过。

静态测试覆盖 sensor 存在性、Gazebo reference parent、update rate、topic、frame、各轴噪声、单水平层、角度/量程、bridge 类型/方向、无 PointCloud2 bridge，以及第三阶段 `/cmd_vel`、`/odom`、`/joint_states` 和 odom TF 接口回归。

## 8. 错误、判断、定位、修改与验证

### 8.1 受限环境禁止 DDS / Gazebo 运行

- 错误现象：首次启动立即出现 `getifaddrs: Operation not permitted`、UDP socket 创建失败和 `~/.gz/auto_default.log` 不可写；Gazebo 在实体生成前退出。
- 判断：这是执行沙箱禁止本机网络接口和用户目录写入，不是 sensor、bridge 或 Launch 配置错误。
- 定位：Gazebo 与多个 ROS 进程同时报告相同权限错误，且静态构建、测试均已通过。
- 修改：没有修改项目代码；将 ROS/Gazebo 日志定向到 `/tmp`，并在允许本机 DDS/图形权限的环境重跑同一次针对性验证。
- 验证：机器人生成成功，两条 bridge 建立，实际 IMU/LaserScan 消息、TF、频率和暂停行为均可读取。

### 8.2 `gz_frame_id` 的 schema 警告

- 错误现象：URDF→SDF 和 Gazebo 加载时提示 `gz_frame_id` 未在标准 SDF sensor schema 定义，并说明会原样复制该字段。
- 判断：`gz_frame_id` 是 gz-sensors 读取的扩展字段，不是丢失字段或传感器加载失败。
- 定位：转换后的 SDF 保留 `<gz_frame_id>`；本机 gz-sensors 库包含该字段处理；Gazebo topic 实际出现。
- 修改：保留该扩展字段，以解决 fixed link 归并后的 ROS frame 命名；不增加额外消息重写节点。
- 验证：ROS 实际消息分别为 `frame_id=imu_link` 和 `frame_id=lidar_link`。

### 8.3 ROS daemon 缓存短暂遗漏新话题

- 错误现象：第一次普通 `ros2 topic list -t` 只显示第三阶段接口，未显示刚创建的两个 sensor topic。
- 判断：bridge 日志已经确认映射建立，Gazebo Transport 同时存在 `/imu/data` 与 `/scan`，更像 ROS CLI daemon 缓存。
- 定位：`/sensor_bridge` 节点存在；改用 `ros2 topic list -t --no-daemon` 后立即看到两个话题及正确类型。
- 修改：没有修改代码；动态验收命令改为 `--no-daemon` 读取即时图。
- 验证：后续 echo、hz 和暂停检查都成功订阅实际消息。

### 8.4 停止时既有 heartbeat 重复 shutdown

- 错误现象：Ctrl-C 时 `system_heartbeat` 仍报告既有 `rcl_shutdown already called`。
- 判断与定位：该现象已存在于第三阶段记录，属于 `resilient_nav_monitor` 的停止路径，不影响本次传感器数据，也不在当前授权修改范围内。
- 修改：未修改 `resilient_nav_monitor`。
- 验证：其余新增和既有进程均退出，最终进程检查无残留。

## 9. 尚未完成与待后续验证

- 未实现 RGB、Depth 或 RGB-D Camera，也没有图像、CameraInfo 或相机点云 bridge。
- 未把 `/odom` 改名为 `/wheel/odometry`；本次明确保持第三阶段 `/odom`。
- 未安装、配置或启动 `robot_localization` / EKF，没有滤波里程计或新的 TF 所有权。
- 未把 Gazebo 原生 `/scan/points` 桥接为 ROS PointCloud2；若未来确有点云需求，必须在单独任务中明确授权和验收。
- 当前频率是一次短时、当前主机负载下的墙钟到达率，不代表长期实时性能、丢包率或负载上限已经验收。
- orientation covariance 的零矩阵是当前 Gazebo 原生输出，不代表真实硬件标定；未来接入 EKF 前必须重新设计输入选择与 covariance 策略。
