# 阶段 3 本机 Gazebo 与 `ros_gz_sim create` 参考

> 本文是阶段 3 实施前的本机调研记录，保留当时的事实和边界。阶段 3 的最终实现与验收结论见 `docs/PHASE3_SUMMARY.md`。

## 1. 文档范围

- 核验日期：2026-07-28
- ROS 2：Jazzy
- Gazebo：Harmonic / Gazebo Sim 8.11.0
- `ros_gz_sim` Debian 包：`ros-jazzy-ros-gz-sim` `1.0.22-1noble.20260615.173223`
- Gazebo vendor 包：`ros-jazzy-gz-sim-vendor` `0.0.10-1noble.20260604.111001`

本文只整理本机已安装文件、命令帮助和短时无 GUI 运行的观察结果，用作阶段 3 后续设计参考。本次没有创建项目机器人、URDF/Xacro、桥接配置、Launch 文件或运动实现，也不表示阶段 3 已经实现。

检查前加载的环境为：

```bash
source /opt/ros/jazzy/setup.bash
source /home/kylian/projects/resilient_nav_lab/ros2_ws/install/setup.bash
```

主要本机参考文件：

- Gazebo DiffDrive 世界：`/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/diff_drive.sdf`
- Gazebo 多轮 DiffDrive 世界：`/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/diff_drive_skid.sdf`
- ROS—Gazebo DiffDrive 示例 Launch：`/opt/ros/jazzy/share/ros_gz_sim_demos/launch/diff_drive.launch.py`
- Joint States 示例 Launch：`/opt/ros/jazzy/share/ros_gz_sim_demos/launch/joint_states.launch.py`
- Joint States 示例 Xacro：`/opt/ros/jazzy/share/ros_gz_sim_demos/models/rrbot.xacro`
- 使用当前 Gazebo 名称的 JointStatePublisher 示例：`/opt/ros/jazzy/share/ros_gz_sim_demos/worlds/vehicle.sdf`
- `create` Launch 封装：`/opt/ros/jazzy/share/ros_gz_sim/launch/gz_spawn_model.launch.py`

## 2. DiffDrive 系统插件

### 2.1 插件标识

当前 Gazebo 示例使用：

```xml
<plugin
  filename="gz-sim-diff-drive-system"
  name="gz::sim::systems::DiffDrive">
</plugin>
```

本机插件库为：

```text
/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim8-diff-drive-system.so.8.11.0
```

`gz plugin -i -p <插件库>` 实际识别到的版本化实现类为 `gz::sim::v8::systems::DiffDrive`，并提供 `ISystemConfigure`、`ISystemPreUpdate` 和 `ISystemPostUpdate` 接口。

### 2.2 本机示例中的参数

`diff_drive.sdf` 的 `vehicle_blue` 显式配置如下：

| SDF 参数 | 示例值 | 含义 |
| --- | ---: | --- |
| `left_joint` | `left_wheel_joint` | 左轮关节名称 |
| `right_joint` | `right_wheel_joint` | 右轮关节名称 |
| `wheel_separation` | `1.25` | 左右轮间距 |
| `wheel_radius` | `0.3` | 车轮半径 |
| `odom_publish_frequency` | `1` | 里程计发布频率，Hz |
| `max_linear_acceleration` | `1` | 最大线加速度 |
| `min_linear_acceleration` | `-1` | 最小线加速度 |
| `max_angular_acceleration` | `2` | 最大角加速度 |
| `min_angular_acceleration` | `-2` | 最小角加速度 |
| `max_linear_velocity` | `0.5` | 最大线速度 |
| `min_linear_velocity` | `-0.5` | 最小线速度 |
| `max_angular_velocity` | `1` | 最大角速度 |
| `min_angular_velocity` | `-1` | 最小角速度 |

同一世界中的 `vehicle_green` 使用相同的关节、几何、加速度和速度参数，但没有显式设置 `odom_publish_frequency`。因此只能确认该参数可省略，不能从该示例推断其默认值。

`diff_drive_skid.sdf` 对同一个插件重复填写两个 `left_joint` 和两个 `right_joint`：

```xml
<left_joint>front_left_wheel_joint</left_joint>
<left_joint>rear_left_wheel_joint</left_joint>
<right_joint>front_right_wheel_joint</right_joint>
<right_joint>rear_right_wheel_joint</right_joint>
```

这表明左右关节参数可重复，用于同侧多轮的 skid-steer 配置。

本机另外两个 DiffDrive 世界 `triggered_publisher.sdf` 和 `tunnel.sdf` 显式使用：

```xml
<topic>cmd_vel</topic>
```

这给出了覆盖速度命令话题的本地文本示例；话题是否为绝对路径取决于参数值本身。

本机插件二进制还包含下列配置键，但上述两个核心示例没有全部实际配置：

- 通用限制：`min_velocity`、`max_velocity`、`min_acceleration`、`max_acceleration`、`min_jerk`、`max_jerk`
- 分方向 jerk 限制：`min_linear_jerk`、`max_linear_jerk`、`min_angular_jerk`、`max_angular_jerk`
- 话题和帧相关键：`topic`、`odom_topic`、`tf_topic`、`frame_id`、`child_frame_id`

这些键只记录为本机插件能力线索；本文不为未在本机文本示例中出现的键声明默认值。

### 2.3 实际话题和消息类型

短时无 GUI 启动本机 `diff_drive.sdf` 后，`vehicle_blue` 和 `vehicle_green` 都形成相同的模型命名话题。以下以 `vehicle_blue` 为例：

| Gazebo Transport 话题 | 插件角色 | Gazebo 消息类型 | 本机 DiffDrive Launch 中的 ROS 2 类型 |
| --- | --- | --- | --- |
| `/model/vehicle_blue/cmd_vel` | DiffDrive 订阅速度命令 | `gz.msgs.Twist` | `geometry_msgs/msg/Twist` |
| `/model/vehicle_blue/enable` | DiffDrive 订阅启停控制 | `gz.msgs.Boolean` | 未桥接 |
| `/model/vehicle_blue/odometry` | DiffDrive 发布里程计 | `gz.msgs.Odometry` | `nav_msgs/msg/Odometry` |
| `/model/vehicle_blue/tf` | DiffDrive 发布位姿变换 | `gz.msgs.Pose_V` | 未桥接 |

运行时 `gz topic -i` 观察结果：

- `cmd_vel` 和 `enable` 有插件订阅者；没有外部命令发布者时显示为无 publisher。
- `odometry` 和 `tf` 有插件发布者。
- `vehicle_green` 对应话题只需把路径中的 `vehicle_blue` 替换为 `vehicle_green`。

本机 `diff_drive.launch.py` 只桥接两个模型的 `cmd_vel` 和 `odometry`。其映射写法为：

```text
/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry
```

该 Launch 没有桥接 DiffDrive 的 `enable` 或 `tf`，因此不能仅凭此示例声称 ROS 2 侧已经存在对应话题。

## 3. JointStatePublisher 系统插件

### 3.1 当前名称与兼容名称

本机较新的 `vehicle.sdf` 使用当前 Gazebo 名称：

```xml
<plugin
  filename="gz-sim-joint-state-publisher-system"
  name="gz::sim::systems::JointStatePublisher">
</plugin>
```

本机 `rrbot.xacro` 和 `double_pendulum_model.sdf` 仍使用兼容的旧 Ignition 名称：

```xml
<plugin
  filename="ignition-gazebo-joint-state-publisher-system"
  name="ignition::gazebo::systems::JointStatePublisher">
</plugin>
```

Harmonic 本机原生插件库为：

```text
/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim8-joint-state-publisher-system.so.8.11.0
```

`gz plugin -i -p <插件库>` 实际识别到的版本化实现类为 `gz::sim::v8::systems::JointStatePublisher`，并提供 `ISystemConfigure` 和 `ISystemPostUpdate` 接口。

阶段 3 新内容如需使用该插件，应明确选择当前 `gz-*` 名称；不要因为本机旧 demo 仍可使用 `ignition-*` 兼容名，就把两套名称误认为两个不同插件。

### 3.2 本机示例参数行为

`vehicle.sdf`、`rrbot.xacro` 和 `double_pendulum_model.sdf` 的插件块都没有设置子参数。短时运行 `vehicle.sdf` 后，一条实际 `gz.msgs.Model` 消息包含：

- `left_wheel_joint`
- `right_wheel_joint`
- `caster_wheel`

因此本机无参数示例会发布该模型可用的关节状态。插件二进制中存在按 `joint_name` 查找关节以及忽略重复关节的诊断信息，但本机已安装的这些文本示例没有演示选择性 `joint_name` 配置；本文不把未实际示范的过滤写法作为已验收用法。

### 3.3 实际话题和消息类型

短时无 GUI 启动本机 `vehicle.sdf` 后，实际观察到：

| 项目 | 值 |
| --- | --- |
| 世界名称 | `demo` |
| 模型名称 | `vehicle` |
| Gazebo Transport 话题 | `/world/demo/model/vehicle/joint_state` |
| Gazebo 消息类型 | `gz.msgs.Model` |

本机 `joint_states.launch.py` 使用世界 `empty`、模型 `rrbot`，因此其桥接源话题为：

```text
/world/empty/model/rrbot/joint_state
```

桥接映射和 ROS 侧重映射为：

```text
Gazebo: /world/empty/model/rrbot/joint_state, gz.msgs.Model
ROS 2:  sensor_msgs/msg/JointState
重映射: joint_states
```

原始 `gz.msgs.Model` 不等同于 ROS 2 `sensor_msgs/msg/JointState`；后者由 `ros_gz_bridge` 转换产生。

## 4. `ros_gz_sim create` 实际用法

### 4.1 本机可执行文件与帮助

可执行文件：

```text
/opt/ros/jazzy/lib/ros_gz_sim/create
```

本机实际执行：

```bash
ROS_LOG_DIR=/tmp/resilient_nav_phase3_ros_logs \
  ros2 run ros_gz_sim create --help
```

得到的主用法为：

```text
create -world [arg] [-file FILE] [-param PARAM] [-topic TOPIC]
       [-string STRING] [-name NAME] [-allow_renaming RENAMING]
       [-x X] [-y Y] [-z Z] [-R ROLL] [-P PITCH] [-Y YAW]
```

帮助模式会初始化 ROS 2，所以在受限环境中需要把日志目录指向可写位置；`--help` 打印完成后返回状态 1 是该本机程序的实际行为，不代表帮助内容缺失。

### 4.2 `create` 参数表

| 参数 | 类型 | 默认值 | 本机帮助说明 |
| --- | --- | --- | --- |
| `world` | string | `""` | 目标 Gazebo world 名称 |
| `file` | string | `""` | 从文件加载 XML |
| `param` | string | `""` | 从 ROS 参数加载 XML |
| `topic` | string | `""` | 从 ROS string publisher 加载 XML |
| `string` | string | `""` | 直接从字符串加载 XML |
| `name` | string | `""` | 新实体名称 |
| `allow_renaming` | bool | `false` | 名称已占用时是否允许重命名 |
| `x` | double | `0` | 初始 X，单位 m |
| `y` | double | `0` | 初始 Y，单位 m |
| `z` | double | `0` | 初始 Z，单位 m |
| `R` | double | `0` | 初始 roll，单位 rad |
| `P` | double | `0` | 初始 pitch，单位 rad |
| `Y` | double | `0` | 初始 yaw，单位 rad |

其中 `file`、`param`、`topic` 和 `string` 是四种 XML 输入来源。本机帮助没有说明同时提供多种来源时的优先级，因此后续实现应一次只明确使用一种来源。

`topic` 输入使用 ROS 2 `std_msgs/msg/String` 承载 XML。`create` 最终向 Gazebo 的 `/world/<world>/create` 服务发送实体创建请求；本机二进制包含 `gz.msgs.EntityFactory` 请求和 `gz.msgs.Boolean` 响应类型。

### 4.3 命令行形式

按本机帮助，文件输入形式可写为：

```bash
ros2 run ros_gz_sim create \
  -world empty \
  -file /absolute/path/to/model.sdf \
  -name robot_name \
  -allow_renaming false \
  -x 0 -y 0 -z 0 \
  -R 0 -P 0 -Y 0
```

这是参数形态参考，本次只读取了帮助，没有实际调用创建服务或生成实体。

### 4.4 Python Launch 参数形式

本机 `joint_states.launch.py` 从 `robot_state_publisher` 发布的 `robot_description` 话题生成 `rrbot`：

```python
Node(
    package='ros_gz_sim',
    executable='create',
    parameters=[{
        'name': 'rrbot',
        'topic': 'robot_description',
    }],
)
```

另一个本机 demo 同时展示初始位姿：

```python
Node(
    package='ros_gz_sim',
    executable='create',
    parameters=[{
        'name': 'my_custom_model',
        'x': 1.2,
        'z': 2.3,
        'Y': 3.4,
        'topic': '/robot_description',
    }],
)
```

`gz_spawn_model.launch.py` 暴露的 Launch 参数名称与 `create` 节点参数有两处包装差异：

| Launch 参数 | 传给 `create` 的参数 |
| --- | --- |
| `model_string` | `string` |
| `entity_name` | `name` |

其余主要参数保持为 `world`、`file`、`topic`、`allow_renaming`、`x`、`y`、`z`、`R`、`P` 和 `Y`。

## 5. 本次核验边界

- 对 DiffDrive 和 JointStatePublisher 使用独立 `GZ_PARTITION` 做了短时、无 GUI 服务端运行，只查询话题、类型和一条关节状态消息，随后停止进程。
- `create` 只执行 `--help`，没有创建或删除任何实体。
- 没有验证未来项目模型的关节命名、轮距、轮径、坐标系、桥接方向、QoS 或控制性能。
- 本文是阶段 3 准备材料，不是阶段 3 功能验收记录。
