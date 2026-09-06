# 阶段 4 总结：传感器与 EKF 定位基线

## 完成内容

- 在机器人模型中加入 IMU、二维 LiDAR 和 RGB-D sensor，并固定各自安装坐标。
- 建立 `/imu/data`、`/scan`、RGB/Depth image 与 CameraInfo 接口；PointCloud2 未桥接。
- 使用 `robot_localization` 3.8.3 融合 `/wheel/odometry` 与 `/imu/data`，发布
  `/odometry/filtered`。
- 完整阶段 4 launch 中，healthy EKF 是唯一 `odom -> base_footprint` 动态 TF owner。
- RViz 可同时观察 RobotModel、TF、LaserScan、相机与 filtered odometry。

## 边界

这是仿真传感器与二维 odom-frame EKF 基线，不包含全局定位、SLAM、故障数据、自适应融合或
真实硬件标定。

