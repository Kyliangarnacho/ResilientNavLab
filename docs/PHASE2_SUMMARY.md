# 阶段 2 总结：Gazebo 基础仿真

## 完成内容

- 安装并验证 Gazebo Harmonic / Gazebo Sim 8.11.0 与 `ros_gz`。
- 建立 `resilient_nav_simulation`，提供自定义 SDF 世界、静态障碍物和 Python launch。
- 将 Gazebo Transport `/clock` 单向桥接为 ROS 2 `/clock`。
- 验证 `system_heartbeat` 使用 `use_sim_time=true`，随 Gazebo 暂停和恢复。

## 关键边界

Gazebo Transport 与 ROS 2 Topic 是两个通信域，只有 bridge 明确声明的消息才会互通。本阶段只建立
世界与仿真时钟，不包含机器人、传感器、控制或导航。

主要入口：

```bash
ros2 launch resilient_nav_simulation phase2_world.launch.py
```
