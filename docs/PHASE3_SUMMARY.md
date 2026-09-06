# 阶段 3 总结：差速机器人与基础运动

## 完成内容

- 建立 `resilient_nav_description` 的两轮差速机器人 Xacro，包含 visual、collision、inertial 和接触参数。
- Gazebo Harmonic `DiffDrive` / `JointStatePublisher` 使用 `0.39 m` 轮距和 `0.10 m` 轮半径。
- 桥接 ROS 2 `/cmd_vel`、`/odom`、`/joint_states`；ROS 侧根据 `/odom` 唯一发布
  `odom -> base_footprint`。
- 完成 Gazebo/RViz 联合显示，以及直行、原地旋转、圆弧和结束停车验证。

## 保留结果与边界

短时测试中 Gazebo 与轮式里程计最大位置差约 `8 mm`、最大航向差约 `0.052 rad`。本阶段只证明
低速平地基础运动链可用，不代表长距离精度、高速控制或真实机器人性能。

```bash
ros2 launch resilient_nav_simulation phase3_demo.launch.py
```
