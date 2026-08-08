# 当前状态

阶段 0 至 6，以及阶段 7.1 已完成。

- C920 已经 WSL/USBIP + `usb_cam` 接入 ROS 2：`/dev/video0` 以 MJPG、1280×720、15 FPS request、`mmap` 发布 `/camera/c920/image_raw`。
- 旧 K/D 已复用验证；正式 `CameraInfo`、`image_proc` 去畸变至 `/camera/c920/image_rect` 与 `rectification_probe` 已通过。
- `/camera/c920/image_raw`、`/camera/c920/camera_info`、`/camera/c920/image_rect` 已完成 rosbag 录制和无相机回放验证。

已知技术债：WSL USB/IP 下偶发闪帧、帧率波动和图像偏暗，暂不处理。

阶段 7.2、真实硬件定位、相机故障模型、Nav2、SLAM、自适应融合和容错导航不在本次范围内。
