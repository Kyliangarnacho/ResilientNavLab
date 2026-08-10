# 阶段 7.2 C920 控制状态基线

核验日期：2026-08-10

设备：WSL 中已 attach 的 Logitech C920，`/dev/video0`
只读命令：`v4l2-ctl --device /dev/video0 --list-ctrls-menus`

本次只读取控制状态，没有执行 `--set-ctrl`，也没有修改相机参数。

## 当前控制状态

| 控制项 | 当前值 | 范围 / 默认值 | 状态 |
| --- | ---: | --- | --- |
| brightness | 128 | 0–255 / 128 | active |
| contrast | 128 | 0–255 / 128 | active |
| saturation | 128 | 0–255 / 128 | active |
| white_balance_automatic | 1 | bool / 1 | 自动白平衡开启 |
| gain | 0 | 0–255 / 0 | active |
| power_line_frequency | 2 | 0 Disabled；1 50 Hz；2 60 Hz / 2 | 60 Hz |
| white_balance_temperature | 4000 | 2000–6500 / 4000 | inactive（自动白平衡开启） |
| sharpness | 128 | 0–255 / 128 | active |
| backlight_compensation | 0 | 0–1 / 0 | active |
| auto_exposure | 3 | 1 Manual；3 Aperture Priority / 3 | Aperture Priority Mode |
| exposure_time_absolute | 250 | 3–2047 / 250 | inactive（自动曝光开启） |
| exposure_dynamic_framerate | 1 | bool / 0 | active |
| pan_absolute | 0 | -36000–36000，步长 3600 / 0 | active |
| tilt_absolute | 0 | -36000–36000，步长 3600 / 0 | active |
| focus_absolute | 0 | 0–250，步长 5 / 0 | inactive（连续自动对焦开启） |
| focus_automatic_continuous | 1 | bool / 1 | 连续自动对焦开启 |
| zoom_absolute | 100 | 100–500 / 100 | active |

该表是读取时刻的真实设备状态，不是阶段 7.2 推荐值或后续健康判定阈值。

后续短时真实采集验证启动了既有 `usb_cam` 链。该驱动启动日志显示其设置 brightness，运行期自动曝光、自动白平衡和自动对焦也会改变部分只读实时值；因此 `camera_health_calibrate` 会在每次运行中另存当时的只读控制快照。这里保留的是驱动启动前的原始读取结果，两者不应混写。
