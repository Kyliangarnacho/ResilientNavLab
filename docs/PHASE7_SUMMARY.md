# 阶段 7 总结：C920 相机与健康监测

## 完成内容

- 在 WSL/USBIP 下用 `usb_cam` 接入 C920，配置 MJPG、1280×720、15 FPS request。
- 提供正式 CameraInfo，复用并验证既有 K/D，使用 `image_proc` 发布去畸变图像。
- 完成 image、CameraInfo、rectified image 的 rosbag 记录与无相机回放。
- 建立 camera health v1：stale、exact-fingerprint freeze，以及保守的
  underexposed/overexposed/blurred/low-information 判定。
- `manual_fault_event` 只发布人工真值时间窗，不修改图像。

## 已知限制

WSL USB/IP 下仍可能闪帧和帧率波动，画面偏暗；当前阈值只属于本阶段 baseline，不是通用相机质量标准。

