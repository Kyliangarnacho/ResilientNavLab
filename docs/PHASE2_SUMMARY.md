# 阶段 2 收尾总结

## 完成结论

阶段 2 已完成 Gazebo Harmonic 基础环境、自定义静态世界、Gazebo—ROS 2 `/clock` 单向桥接，以及 `system_heartbeat` 使用仿真时间的联动验证。

这一结论只覆盖基础仿真与时钟数据链路。当前没有机器人、URDF、传感器、运动与控制、Nav2、SLAM、故障注入、健康评估、自适应融合或容错导航能力。

## 系统职责边界

| 组成 | 当前职责 | 不负责的内容 |
| --- | --- | --- |
| ROS 2 Jazzy | 运行项目节点、管理参数和 ROS 图、承载 ROS 2 Topic | 不直接加载或推进 Gazebo 世界 |
| Gazebo Harmonic | 读取 SDF、运行物理世界、维护仿真状态和仿真时间、发布 Gazebo Transport 数据 | 不自动把数据发布为 ROS 2 Topic |
| `ros_gz` | 从 ROS 2 启动 Gazebo，并在明确配置的话题和消息类型之间桥接 | 不自动桥接所有话题，也不实现项目算法 |

Gazebo Transport 与 ROS 2 Topic 属于不同通信域。`gz topic -l` 和 `ros2 topic list` 各自查询一侧；话题同名并不代表互通。当前只有 `bridge.yaml` 中声明的 `/clock` 从 Gazebo 单向进入 ROS 2：

```text
Gazebo Sim
  └─ Gazebo Transport /clock (gz.msgs.Clock)
       └─ ros_gz_bridge
            └─ ROS 2 /clock (rosgraph_msgs/msg/Clock)
                 └─ system_heartbeat (use_sim_time=true)
```

## 阶段 2 文件作用

- `worlds/phase2_world.sdf`：描述 `resilient_lab` 世界、物理参数、光源和三个静态实体。它是 Gazebo 的场景输入。
- `config/bridge.yaml`：声明 `/clock` 的 Gazebo 话题、ROS 2 Topic、两侧消息类型、`GZ_TO_ROS` 方向和时钟 QoS。
- `launch/phase2_world.launch.py`：定位安装后的包资源，启动 Gazebo、自定义世界、时钟桥和 `system_heartbeat`，并设置 `use_sim_time=true`。
- `test/test_simulation_resources.py`：静态检查 SDF XML、关键实体、桥接配置和 Launch 组成，不替代图形界面或运行时人工验收。

## 工作空间与命令

`ros2_ws/src/` 保存包源码；`build/` 保存包级中间构建文件；`install/` 保存 ROS 2 可发现的安装布局；`log/` 保存构建、测试和结果日志。后三者可由工具重新生成并已被 Git 忽略。

`colcon build` 从 `src/` 发现和构建包。`--symlink-install` 尽可能让 `install/` 链接到源码或构建产物，缩短 Python 和资源文件的迭代周期，但构建规则或安装清单变化后仍须重新构建。`source /opt/ros/jazzy/setup.bash` 加载 ROS 2 和 Gazebo vendor 环境；`source install/setup.bash` 再叠加本工作空间，使 ROS 2 能发现这里构建的包。`source` 不会构建项目，且只影响当前 shell。

## 人工验收记录

以下结果由用户人工确认，本总结不补充未测量的数据：

1. `ros-jazzy-ros-gz` 安装成功。
2. Gazebo Sim 版本为 8.11.0。
3. 官方 `shapes.sdf` 图形世界正常打开。
4. Gazebo Transport 中可以观察 `/clock`。
5. 未启动桥接时，ROS 2 看不到 `/clock`。
6. `resilient_nav_simulation` 自定义世界正常打开。
7. 世界中显示 `ground_plane`、`box_obstacle` 和 `cylinder_checkpoint`。
8. 启动 Launch 后，ROS 2 可以看到 `/clock`。
9. `/system_heartbeat` 的 `use_sim_time` 为 `true`。
10. 暂停 Gazebo 后，ROS 2 `/clock` 和心跳停止。
11. 恢复 Gazebo 后，ROS 2 `/clock` 和心跳继续。
12. 用户把 `box_obstacle` 的 pose 从 `2 0 0.5 0 0 0` 修改为 `3 1 0.5 0 0 0`，重新启动后在 Gazebo 中验证新坐标生效。

## `gz` 命令环境问题

`ros-jazzy-ros-gz` 使用 ROS vendor 前缀提供 Gazebo 工具，`gz` 实际位于 `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`。安装完成不会反向修改已经打开 shell 的环境；未加载 ROS 2 setup，或在安装前已加载过 setup 且没有重新加载时，`PATH` 中没有这个新目录，因此会表现为找不到 `gz`。

在当前 shell 中重新执行：

```bash
source /opt/ros/jazzy/setup.bash
```

即可加载 vendor 环境。这一问题的原因是 shell 环境未刷新，不是安装失败。

## 阶段边界与后续

阶段 2 到此结束，完成项仅包括：

- Gazebo Harmonic 与 ROS 2 Jazzy 集成安装验证；
- 官方示例世界和项目静态世界加载；
- `/clock` 从 Gazebo 到 ROS 2 的单向桥接；
- `system_heartbeat` 的 `use_sim_time` 暂停/恢复联动；
- SDF 资源修改经重新启动生效；
- 仿真资源包的构建和静态测试。

下一阶段尚未开始。机器人、URDF、传感器、运动插件、Nav2、SLAM，以及故障注入、健康评估、融合与容错导航的设计和实现，都需要后续任务单独授权和验收。
