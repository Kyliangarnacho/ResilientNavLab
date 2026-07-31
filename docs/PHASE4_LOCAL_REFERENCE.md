# 阶段 4 本机传感器与定位参考

> 本文是阶段 4 实施前的只读调研记录。它记录本机安装文件、命令输出和上游参数约定，不表示传感器模型、bridge、EKF 或定位基线已经实现。

## 1. 范围与证据等级

- 核验日期：2026-07-31
- ROS 2：Jazzy
- Gazebo：Harmonic / Gazebo Sim `8.11.0`
- `ros_gz_bridge`：`1.0.22-1noble.20260615.142443`
- `ros_gz_sim_demos`：`1.0.22`
- Gazebo vendor 包：`ros-jazzy-gz-sim-vendor` `0.0.10-1noble.20260604.111001`
- SDFormat vendor 包：`ros-jazzy-sdformat-vendor` `0.0.11-1noble.20260604.104102`

本文采用三种证据标记：

- **本机已验证**：已从本机文件内容、软件包数据库或只读命令输出直接确认。
- **上游约定对照，非本机验证**：本机缺少相应安装文件，内容来自上游 ROS 2 参数模板或通用 ROS 2 接口约定。
- **待动态确认**：必须在未来明确授权后启动 Gazebo、bridge 或节点，才能确认实际话题、频率、QoS、时间戳、TF 和数据内容。

本次没有修改 `ros2_ws/src`，没有安装软件，没有启动 Gazebo、RViz、bridge 或 `ekf_node`，也没有提交 Git。

## 2. 本机来源

### 2.1 Gazebo 与 SDFormat

本机已读取或查询以下来源：

- `/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/sensors.sdf`
- `/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/sensors_demo.sdf`
- `/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/gpu_lidar_sensor.sdf`
- `/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/depth_camera_sensor.sdf`
- `/opt/ros/jazzy/opt/sdformat_vendor/share/sdformat14/1.11/sensor.sdf`
- Sensors System 插件库：`/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim8-sensors-system.so.8.11.0`
- IMU System 插件库：`/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim8-imu-system.so.8.11.0`

`gz sim --version` 返回 `Gazebo Sim, version 8.11.0`。`gz plugin -i -p` 对两个插件库的只读检查分别识别出：

- `gz::sim::v8::systems::Sensors`，实现 `ISystemConfigure`、`ISystemUpdate`、`ISystemPostUpdate` 和 `ISystemReset` 等接口。
- `gz::sim::v8::systems::Imu`，实现 `ISystemPreUpdate` 和 `ISystemPostUpdate` 等接口。

SDFormat 1.11 本机 schema 明确列出 `camera`、`depth_camera`、`gpu_lidar`、`imu` 和 `rgbd_camera` 等 sensor type，并为所有传感器定义 `name`、`type`、`always_on`、`update_rate`、`visualize`、`topic`、`enable_metrics` 和可选 `pose` 等公共字段。

### 2.2 `ros_gz_bridge`

本机已读取：

- `/opt/ros/jazzy/include/ros_gz_bridge/ros_gz_bridge/convert/sensor_msgs.hpp`
- `/opt/ros/jazzy/share/ros_gz_sim_demos/launch/imu.launch.py`
- `/opt/ros/jazzy/share/ros_gz_sim_demos/launch/gpu_lidar_bridge.launch.py`
- `/opt/ros/jazzy/share/ros_gz_sim_demos/launch/depth_camera.launch.py`
- `/opt/ros/jazzy/share/ros_gz_sim_demos/launch/rgbd_camera.launch.py`
- `/opt/ros/jazzy/share/ros_gz_sim_demos/launch/rgbd_camera_bridge.launch.py`

`ros2 pkg prefix ros_gz_bridge` 和 `ros2 pkg prefix ros_gz_sim` 均返回 `/opt/ros/jazzy`。转换头文件同时声明了 ROS→Gazebo 和 Gazebo→ROS 两个方向的模板特化；示例 Launch 则给出了具体话题与类型组合。

### 2.3 ROS 2 时间与 `robot_localization`

本机已读取 `/opt/ros/jazzy/include/rclcpp/rclcpp/time_source.hpp` 和 `/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/time_source.py`。本机 `rclcpp` 版本为 `28.1.21`；C++ 头文件明确说明：节点的 `use_sim_time=true` 时，所附 ROS 时钟由收到的仿真时钟消息更新，否则使用系统时间，并通过 `/clock` 订阅接收仿真时间。本机 Python 实现还把未声明的 `use_sim_time` 声明为 `false`，并在未设置时使用 wall time；这是 ROS 2 时间机制的本机旁证，不是 `ekf_node` 的运行验证。

本机使用以下方式检查 `robot_localization`：

- `dpkg-query -W ros-jazzy-robot-localization`
- `ros2 pkg prefix robot_localization`
- `ros2 pkg executables robot_localization`
- `command -v ekf_node`
- `apt-cache policy ros-jazzy-robot-localization`
- 检查 `/opt/ros/jazzy/share/robot_localization` 和 `/opt/ros/jazzy/lib/robot_localization`

结果见第 6 节。

## 3. Gazebo Systems 的职责边界

### 3.1 渲染类 Sensors System

**本机已验证：** `sensors_demo.sdf`、`gpu_lidar_sensor.sdf` 和 `depth_camera_sensor.sdf` 都在 world 层使用：

```xml
<plugin
  filename="gz-sim-sensors-system"
  name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

本机示例由该 System 承载需要渲染的 camera、depth camera、RGB-D camera 和 GPU lidar。`render_engine` 的本机示例值是 `ogre2`。

### 3.2 IMU System

**本机已验证：** `sensors.sdf` 把 IMU 与渲染传感器分开，使用独立插件：

```xml
<plugin
  filename="gz-sim-imu-system"
  name="gz::sim::systems::Imu">
</plugin>
```

因此不能把“加入 Sensors System”误写成 IMU 的完整配置。按照本机示例，IMU 需要 `Imu` System；相机、Depth/RGB-D 和 GPU Lidar 使用 `Sensors` System。

**待动态确认：** 未来项目世界或机器人中是否还需要其他 world System、插件加载顺序、渲染后端可用性和无 GUI 服务端行为，本次没有启动 Gazebo，不能据此下结论。

## 4. 本机传感器示例字段

### 4.1 IMU

`sensors.sdf` 中的 IMU 为：

```xml
<sensor name="imu" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <visualize>true</visualize>
  <topic>imu</topic>
  <enable_metrics>true</enable_metrics>
</sensor>
```

**本机已验证：** 示例字段为 `type=imu`、`always_on=1`、`update_rate=100 Hz`、`topic=imu`、`visualize=true` 和 `enable_metrics=true`。示例注释建议用 `gz topic -e -t /imu` 读取输出，但本次没有运行该命令。

**待动态确认：** 实际 frame、方向/角速度/线加速度字段、协方差、噪声、时间戳和 100 Hz 实际到达率均未验证。该示例没有配置 `<imu>` 内部噪声参数，不能把“未配置”解释成已经验收的零噪声模型。

### 4.2 二维 GPU Lidar

`sensors_demo.sdf` 中的二维配置使用一个垂直样本：

| 字段 | 本机示例值 |
| --- | ---: |
| sensor `type` | `gpu_lidar` |
| `topic` | `lidar` |
| `update_rate` | `10 Hz` |
| horizontal `samples` | `640` |
| horizontal `resolution` | `1` |
| horizontal `min_angle` | `-1.396263 rad` |
| horizontal `max_angle` | `1.396263 rad` |
| vertical `samples` | `1` |
| vertical `resolution` | `0.01` |
| vertical `min_angle` / `max_angle` | `0` / `0 rad` |
| range `min` / `max` | `0.08` / `10.0 m` |
| range `resolution` | `0.01 m` |
| `visualize` | `true` |
| `enable_metrics` | `true` |

该文件使用 `<ray>` 容器；本机 `gpu_lidar_sensor.sdf` 的另一示例使用 `<lidar>` 容器，但配置了 `16` 个垂直样本，属于多层示例，不应把它当作本文的二维基线。SDFormat 本机 schema 还说明旧名称 `gpu_ray` 等价于 `gpu_lidar`，但推荐新名称 `gpu_lidar`。

**待动态确认：** `/lidar` 和 `/lidar/points` 是否实际出现、扫描方向、角度增量、无返回值表示、点云组织、真实发布频率和 QoS 均未运行验证。

### 4.3 Depth Camera

本机有两个静态例子：

| 字段 | `sensors_demo.sdf` | `depth_camera_sensor.sdf` |
| --- | ---: | ---: |
| sensor `type` | `depth_camera` | `depth_camera` |
| `topic` | `depth_camera` | `depth_camera` |
| `update_rate` | `10 Hz` | `10 Hz` |
| `horizontal_fov` | `1.05 rad` | `1.05 rad` |
| image `width × height` | `320 × 240` | `256 × 256` |
| image `format` | `R_FLOAT32` | `R_FLOAT32` |
| clip `near` / `far` | `0.1` / `10.0 m` | `0.1` / `10.0 m` |

**本机已验证：** 两个例子都由 Sensors System 和 `ogre2` 驱动，均把深度数据基话题设为 `depth_camera`。

**待动态确认：** 深度无效值编码、字节序、step、frame、camera info、点云派生方式、实际话题和到达频率未验证。`depth_camera.launch.py` 只声明 `/depth_camera` 的 Image bridge，不能据此声称 standalone Depth 示例还会桥接 CameraInfo 或 PointCloud2。

### 4.4 RGB-D Camera

`sensors_demo.sdf` 中 RGB-D 示例为：

| 字段 | 本机示例值 |
| --- | ---: |
| sensor `type` | `rgbd_camera` |
| `topic` | `rgbd_camera` |
| `always_on` | `1` |
| `update_rate` | `30 Hz` |
| `visualize` | `true` |
| `enable_metrics` | `true` |
| `horizontal_fov` | `1.047 rad` |
| image `width × height` | `320 × 240` |
| clip `near` / `far` | `0.1` / `100 m` |

本机 `rgbd_camera_bridge.launch.py` 以该世界为输入，并为 RGB 图像、CameraInfo、深度图像和点云分别声明 bridge，见下一节。

**待动态确认：** RGB 编码、深度编码、RGB 与深度时间同步、相机内参、点云坐标系、实际 30 Hz 到达率和 QoS 未验证。

## 5. `ros_gz_bridge` 消息映射

### 5.1 本机转换能力

`convert/sensor_msgs.hpp` 对以下类型同时声明 ROS→Gazebo 和 Gazebo→ROS 转换：

| Gazebo 消息 | ROS 2 消息 | 相关数据 |
| --- | --- | --- |
| `gz.msgs.IMU` | `sensor_msgs/msg/Imu` | IMU |
| `gz.msgs.LaserScan` | `sensor_msgs/msg/LaserScan` | 二维扫描 |
| `gz.msgs.PointCloudPacked` | `sensor_msgs/msg/PointCloud2` | Lidar/RGB-D 点云 |
| `gz.msgs.Image` | `sensor_msgs/msg/Image` | RGB 或深度图像 |
| `gz.msgs.CameraInfo` | `sensor_msgs/msg/CameraInfo` | 相机标定信息 |

这是**本机静态转换能力**，不代表任何 bridge 正在运行，也不代表这些 Gazebo 话题已经存在。

### 5.2 本机 demo 声明的具体话题

| 示例 | Gazebo/ROS 同名话题参数 | Gazebo 类型 | ROS 2 类型 |
| --- | --- | --- | --- |
| IMU | `/imu` | `gz.msgs.IMU` | `sensor_msgs/msg/Imu` |
| GPU Lidar scan | `/lidar` | `gz.msgs.LaserScan` | `sensor_msgs/msg/LaserScan` |
| GPU Lidar points | `/lidar/points` | `gz.msgs.PointCloudPacked` | `sensor_msgs/msg/PointCloud2` |
| standalone Depth | `/depth_camera` | `gz.msgs.Image` | `sensor_msgs/msg/Image` |
| RGB-D RGB image | `/rgbd_camera/image` | `gz.msgs.Image` | `sensor_msgs/msg/Image` |
| RGB-D camera info | `/rgbd_camera/camera_info` | `gz.msgs.CameraInfo` | `sensor_msgs/msg/CameraInfo` |
| RGB-D depth image | `/rgbd_camera/depth_image` | `gz.msgs.Image` | `sensor_msgs/msg/Image` |
| RGB-D points | `/rgbd_camera/points` | `gz.msgs.PointCloudPacked` | `sensor_msgs/msg/PointCloud2` |

同一个 RGB-D bridge demo 还声明普通 RGB camera 的 `/camera` Image 和 `/camera_info` CameraInfo，并把 ROS 侧分别 remap 到 `/camera/image` 和 `/camera/camera_info`。

本机 demo 参数使用 `topic@ros_type@gz_type` 形式，没有限制为单向。阶段 4 真正实施时仍需单独决定每条 bridge 的方向、ROS 侧命名、remap 和 QoS；本文不替未来实现作出该决定。

### 5.3 尚未验证的 bridge 行为

- 没有启动 `parameter_bridge`，所以没有观察 ROS 图或 Gazebo Transport 图。
- 没有确认消息中 `header.frame_id`、时间戳、协方差、编码、点字段或 endian/step。
- 没有确认 sensor data QoS、队列深度、可靠性或高带宽图像链的吞吐。
- 没有验证 demo 中的双向写法是否适合项目；传感器通常应考虑 Gazebo→ROS 的单向所有权，但这仍是未来设计项。

## 6. `robot_localization` 与 `ekf_node`

### 6.1 本机安装状态

**本机已验证：`robot_localization` 未安装。**

- `dpkg-query` 报告没有匹配的已安装包。
- `ros2 pkg prefix robot_localization` 返回 `Package not found`。
- `ros2 pkg executables robot_localization` 返回 `Package 'robot_localization' not found`。
- `command -v ekf_node` 无输出。
- `/opt/ros/jazzy/share/robot_localization` 和 `/opt/ros/jazzy/lib/robot_localization` 均不存在。

`apt-cache policy` 只表明软件源中存在候选包 `ros-jazzy-robot-localization` `3.8.3-1noble.20260615.152020`；“有候选版本”不等于“已经安装”。本次未下载或安装该包。

### 6.2 关键参数约定

由于本机没有该包，下表属于**上游约定对照，非本机 `ekf_node` 验证**。版本参考为 APT 候选 `3.8.3` 及上游 ROS 2 参数模板；安装后仍须用实际 Jazzy 包内的 `params/ekf.yaml`、可执行文件和运行参数重新确认。

上游参考：

- [`robot_localization` 3.8.3 源码标签](https://github.com/cra-ros-pkg/robot_localization/tree/3.8.3)
- [上游 ROS 2 `params/ekf.yaml`](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/params/ekf.yaml)

| 参数 | 上游约定 | 对本项目的未决事项 |
| --- | --- | --- |
| `odom_frame` | 默认 `odom`；短期连续但可漂移的世界固定坐标系 | 当前已有 `odom`，名称表面一致，仍需运行时核对 TF 所有权 |
| `base_link_frame` | 默认 `base_link`；机器人机体坐标系 | 当前运动参考点是 `base_footprint`，不能未经设计直接沿用默认值 |
| `world_frame` | 默认取 `odom_frame`；连续局部数据通常设为 `odom_frame`，融合会跳变的全局绝对位置时通常设为 `map_frame` | 阶段 4 的局部/全局融合边界尚未决定 |
| `two_d_mode` | 默认 `false`；设为 `true` 时忽略 3D 状态，适用于明确的平面运动场景 | 当前机器人是平地差速基线，但尚未授权选择参数值或验证 IMU 处理 |
| `publish_tf` | 默认 `true`；允许滤波节点广播输出 TF | 当前 `odom_tf_broadcaster` 已发布 `odom -> base_footprint`；未来若 EKF 也发布同一变换会形成重复 TF 来源 |
| `use_sim_time` | ROS 2 通用参数，通常默认 `false`；`true` 时使用 `/clock` 驱动 ROS 时间，否则使用系统时间 | 本机 rclcpp/rclpy 约定已验证，但 `ekf_node` 未安装、未启动，实际参数和时间戳行为待确认 |

TF 约定需要结合 `world_frame` 解读：

- `world_frame=odom_frame` 且 `publish_tf=true` 时，局部滤波通常负责 `odom_frame -> base_link_frame`。
- `world_frame=map_frame` 且 `publish_tf=true` 时，全局滤波通常输出 `map_frame -> odom_frame`，同时必须由其他来源提供 `odom_frame -> base_link_frame`。
- `publish_tf=false` 时，滤波器不应成为该输出 TF 的发布者；仍可发布滤波后的 odometry，但实际接口需安装后确认。

以上内容不是本项目配置决定。尤其是 `base_link` 与 `base_footprint`、现有 `odom_tf_broadcaster` 是否保留、局部 EKF 是否取得 `odom -> base_footprint` 所有权，必须在后续实施任务中明确设计并动态验收。

### 6.3 安装后需要动态确认

- `ros2 pkg prefix robot_localization`、Debian 版本、`ekf_node` 可执行文件和包内参数模板。
- 实际节点名与 YAML 顶层键是否匹配。
- `use_sim_time=true` 后节点时钟是否随 Gazebo `/clock` 暂停和恢复。
- 输入 `/odom`、IMU topic、frame 和协方差是否被接受，是否产生 diagnostics。
- `odometry/filtered` 的 frame、child frame、频率、时间戳和 covariance。
- `/tf` 的唯一发布者与变换方向，确认没有和 `odom_tf_broadcaster` 重复。
- `two_d_mode` 对 Z、roll、pitch 及其速度/加速度分量的实际处理。

## 7. 错误、定位与处理

### 7.1 `parameter_bridge --help` 日志目录错误

直接执行本机 `parameter_bridge --help` 时，程序在打印帮助前初始化 ROS 2 日志，并因受限环境不能写入 `/home/kylian/.ros/log/...` 而抛出：

```text
spdlog::spdlog_ex: Failed opening file ... Read-only file system
```

定位结果是日志目录不可写，不是 `ros_gz_bridge` 未安装或映射缺失。本次没有为了帮助输出而改写本机日志目录，也没有启动 bridge；所需映射已改由安装头文件和 demo Launch 交叉核验。**事实核验已解决，帮助命令本身未重跑。**

### 7.2 `bridge_types` 无输出

`ros2 run ros_gz_bridge bridge_types` 返回状态 0，但本机没有输出类型列表，因此不能用它作为本次映射证据。这不是非零错误；本次改读 `convert/sensor_msgs.hpp`，并用各传感器 demo Launch 核对具体类型字符串。**证据来源问题已解决。**

### 7.3 Jazzy 在线文档访问限制

ROS Jazzy 在线文档页面在本次检查环境中返回访问限制，无法用该页面核对 `robot_localization`。随后确认本机包缺失，并使用 APT 候选元数据与上游源码标签/ROS 2 参数模板作为非本机对照。**本机安装状态已确认；`ekf_node` 的精确本机行为仍未验证。**

## 8. 本次结论与边界

**本机已验证：**

- Gazebo Sim 为 `8.11.0`，Sensors 与 Imu 插件库存在且可由 `gz plugin` 识别。
- 渲染类传感器示例使用 `gz-sim-sensors-system` / `gz::sim::systems::Sensors` 和 `ogre2`；IMU 使用独立 Imu System。
- IMU、二维 GPU Lidar、Depth Camera 和 RGB-D Camera 的上述静态 SDF 字段存在。
- `ros_gz_bridge` `1.0.22` 的安装头文件包含 IMU、LaserScan、PointCloud2、Image 和 CameraInfo 的双向转换声明；demo Launch 包含上述具体话题映射。
- `robot_localization` 当前未安装，软件源候选版本为 `3.8.3`。
- ROS 2 Jazzy 本机 `rclcpp` 的 `use_sim_time` 与 `/clock` 约定存在。

**尚未验证或需要动态确认：**

- 所有传感器和 bridge 的运行时话题、频率、QoS、frame、时间戳、协方差、图像编码、点云布局与同步。
- 传感器在项目机器人上的 pose、frame 命名、噪声模型和最终参数。
- `robot_localization` 的本机包内容、`ekf_node` 可执行性、输入融合、输出 odometry、TF 和仿真时间行为。
- 阶段 4 最终传感器组合、bridge 方向、EKF frame 选择、`two_d_mode`、`publish_tf` 以及 TF 唯一所有权。

本文仅为下一步设计提供本机参考，不改变“阶段 4 尚未实现”的项目状态。
