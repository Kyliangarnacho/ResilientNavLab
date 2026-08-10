# ResilientNavLab 学习与决策日志

本日志按日期记录项目中的事实、判断、经验和后续问题。尚未实施或验证的内容应标记为计划或待办。

## 2026-08-10 — 阶段 7.2 C920 baseline 与 camera health monitor v1

### 当前事实

- 实机只读执行 `v4l2-ctl --device /dev/video0 --list-ctrls-menus`，记录曝光、gain、白平衡、对焦、brightness、contrast、sharpness 等当前值；未执行控制写操作，完整结果见 `docs/PHASE7_2_CAMERA_CONTROLS_BASELINE.md`。
- `resilient_nav_health_assessment` 新增不依赖 ROS 的 `camera_health_features.py`，对常见 NumPy 灰度、RGB/BGR 和四通道图像输出灰度统计、分位数、暗亮比例、Laplacian 方差、边缘密度、熵、帧差和确定性帧指纹。
- 新增 `camera_health_feature_demo`，自动构造纯黑、纯白、均匀灰、灰度渐变、清晰棋盘、高斯模糊棋盘、重复帧和轻微变化帧，只调用既有特征 API 并打印紧凑对照表，不接入 ROS topic。
- `camera_health_calibrate` 现支持 `scenario_label` 和 `session_id`；每条 CSV 样本及 JSON 都保留场景/session 元数据。同一 baseline 根目录可容纳多个独立 session 子目录，自动 ID 使用 UTC 微秒时间，显式同名 ID 会拒绝启动而不覆盖旧数据。
- 新增纯分析 `camera_health_baseline_report`：扫描多个 session，输出全局和按 `scenario_label` 分组的 FPS、interarrival、max gap、亮度、Laplacian variance、edge density、entropy 和 frame difference 描述统计，同时生成 `baseline_report.json` 与终端表格。
- 读取 5-session `baseline_report.json`：共 3560 帧，observed FPS 为 `8.22--14.43 Hz`，全局 interarrival p95/p99 约 `0.158/0.249 s`，最大正常 gap 约 `0.382 s`。
- 新增 `camera_health_monitor`，默认订阅 `/camera/c920/image_raw`，复用现有特征和阶段 6 的通用 `HealthDecision`/`SensorHealth` 构造，以固定 `5 Hz` 发布 `/health/camera`。
- 初版正式故障为 stale 和 exact-fingerprint freeze；后续加入保守 underexposed/overexposed/blurred/low-information v1。stale 在停止收图时优先；所有候选、确认和恢复均按持续时间，不依赖固定帧数。
- underexposed 同时要求 `mean_gray<=6`、`p95<=8`、`dark_ratio>=0.90`。ACTIVE 开发汇总 `4.035/5.506/0.946` 满足规则；健康黑键盘最低 `p95=10.545` 不满足，健康近黑启动两帧仅持续约 `0.063 s`，再由 `0.6 s` confirmation 排除。
- overexposed 同时要求 `mean_gray>=170`、`p05>=150`、`p95>=180`。ACTIVE 开发汇总 `183.974/175.070/188.091` 满足；健康 baseline 的 mean 最大约 `163.255`、p05 最大约 `64.620`，局部高光产生的高 p95 不足以触发。bright ratio 不进入正式规则。
- blurred 要求近期 reference 同时满足 `Laplacian>=100`、`edge_density>=0.01`，当前同时降到 `<=10`、`<=0.001`，且 `gray_std>=20`、`entropy>=5.5`。blur pre/ACTIVE 为 `309.680/0.028` 与 `6.853/0.000`；低纹理 baseline 无法建立 reference。
- low-information v1 先要求最近 `3.0 s` 内有 `edge_density>=0.01 && entropy>=5.2` 的可用 reference，再要求当前 `edge_density<=0.0005 && entropy<=5.8 && dark_ratio>=0.20 && gray_std>=20`；遮挡 ACTIVE 汇总满足该组合，而健康低纹理场景不能建立 reference。bright ratio 在本次数据中接近零，不进入规则。
- development 参数为 stale `1.0 s`、freeze `2.0 s`、fault confirmation `0.6 s`、recovery `1.0 s`；low-information 复用同一 confirmation/recovery 时间结构。
- `health_score` 表示规则下当前数据健康程度；`confidence` 表示对当前状态判断的确定程度。二者由样本充分度、confirmation/recovery 进度和可解释 evidence 直接计算，不使用机器学习。
- 5 秒真实 C920 验证采集 54 帧、0 次转换错误，observed FPS 约 `11.0`；interarrival p50/p95/p99 约 `0.066/0.189/0.229 s`，max gap 约 `0.264 s`。该结果只验证短时链路，不作为健康阈值。
- 单元测试覆盖纯黑、纯白、纹理、模糊、低纹理、重复帧、不同帧、常见通道布局、浮点输入和错误输入。
- 仅构建 `resilient_nav_health_assessment` 成功；安装区可发现 demo、采集、report 和 monitor。包级测试最终汇总为 125 项、0 错误、0 失败、0 跳过；monitor 新增 15 项覆盖正常流、stale、freeze、stamp 不前进、静止微变化、confirmation、连续 recovery、8--15 Hz 波动、视觉 candidate、配置和 Launch。
- 无相机短时动态验证中，节点正常启动并持续发布 `/health/camera`；超过 development timeout/confirmation 后输出 `state=FAULT`、`detected_fault=stale`、`health_score=0.0`、`confidence=1.0`，Ctrl-C 后 cleanly 退出。
- stale 已由用户在真实 C920 链路完成实机触发与恢复验证；本次 freeze 专项工作没有调整 camera monitor 状态机或参数。
- 新增 `camera_freeze_source`：第一张有效真实 Image 到达后只缓存其完整像素与必要 metadata，默认以 `10 Hz` 向独立 `/test/camera/image_frozen` 发布像素完全相同、当前 ROS stamp 持续前进的副本；源输出同名或频率非法会拒绝启动，未获得有效源图像时不发布。
- 新增纯显示 `camera_health_watch`：在 `state`/`detected_fault` 变化时立即输出，状态不变时默认每 `5 s` 输出；单行包含 score、confidence、message age、rolling FPS、fingerprint identical duration、confirmation 和 recovery，且不产生任何判定。
- 新增 13 项 freeze source/watch/resource 测试；目标包构建成功，安装区可发现两个新入口，完整包级测试最终为 138 项、0 错误、0 失败、0 跳过。
- 新增真正的 freeze runtime 集成测试：测试代码发布有效 `rgb8` Image，经 `camera_freeze_source` 和独立 frozen topic 进入未修改的 camera monitor；验证持续消息、像素一致、stamp 前进、metadata、最终 `FAULT/freeze` 以及全程无 stale 抢占。
- `resilient_nav_fault_injection` 新增 `manual_fault_event`，复用既有 `FaultStatus`，只发布 SCHEDULED/ACTIVE/ENDED 人工真值时间窗，不修改相机或 health 数据；unit/runtime 测试均验证自动状态进展。
- `health_evaluator` 增加默认关闭的 `camera_health_topic`。camera 映射覆盖 stream_stop/freeze/underexposure/overexposure/blur/occlusion；事件结果显式区分 anomaly detected 与 exact classification match，同时保留阶段 6 所有旧字段和默认三传感器集合。
- 真值/evaluator 基础设施完成时包级测试为 `resilient_nav_fault_injection` 142 项和 `resilient_nav_health_assessment` 144 项，均通过。
- `resilient_nav_camera` 新增 `phase7_2_camera_health.launch.py`：原样 Include 阶段 7.1 C920 Launch，并启动 monitor；evaluator/watch 由条件控制，manual event 明确不自动启动。默认 source 为 raw，health 固定发布 `/health/camera`。
- evaluator 审计确认 detection delay、anomaly detected、exact classification 和 TP/FP/FN/TN 已存在；最小补充事件 ENDED 后第一条 HEALTHY 的 recovery time/delay，不改变旧计数语义。
- 联合 Launch runtime 测试关闭 camera 分支，确认 monitor 的 source/config、evaluator 的 camera topic/JSON、watch topic 和 use_sim_time 实际传递；关闭 evaluator/watch 时两个节点均不存在，未访问 `/dev/video0`。
- 本轮最终完整测试为 `resilient_nav_camera` 26 项、`resilient_nav_health_assessment` 146 项，均为 0 错误、0 失败、0 跳过；阶段 7.1 原 Launch/配置测试和阶段 6 测试全部通过。
- `camera_health_calibrate` 增加默认关闭的 truth 模式；开启时仅在特征计算后给样本附加 FaultStatus 标签，并把状态 transition 写入 summary。普通 baseline 新 truth 列为空，既有多 session report 继续通过。
- camera truth 状态变化触发 before/fault/after 三份额外 controls 文件；before 复用启动只读快照，ACTIVE/ENDED 重新执行只读 list controls，未增加 V4L2 写操作。
- 新增纯离线 `camera_fault_feature_report`，用默认 `2.0 s` margin 严格排除 ACTIVE/ENDED 边界两侧样本，输出三个阶段、三个视觉 family 的描述统计与 p05–p95 candidate interval，明确不是阈值。
- 最新 health assessment 构建成功，完整包级测试为 171 项、0 错误、0 失败、0 跳过；truth isolation、underexposure、overexposure、blurred、low-information、stale、freeze 和阶段 6 回归均通过。

### 问题与处理

- 受限设备命名空间内看不到 `/dev/video0`；改在获准的宿主环境执行同一条只读查询后成功取得真实控制状态。
- 初版低纹理测试图每 8 列存在一次灰度回绕，导致离散 Laplacian 方差高于过严的近零预期；确认算法正确后，把测试约束修正为低灰度跨度、低二阶变化且无显著边缘。
- 从工作空间根目录手动运行 flake8 时混入 build/install 和其他包既有告警；在目标包目录复核后定位并修正本次代码的两处 import 顺序和一个未使用导入。
- 首次真实 Ctrl+C 验证已成功落盘，但退出后 logger 尝试向已关闭的 rosout context 发布。修正为 context 有效时才记录落盘日志；重复验证后无该错误，三份文件仍齐全且 `stop_reason=keyboard_interrupt`。
- 当前 C920 画面整体偏暗，既有 `usb_cam` 启动日志显示 runtime brightness=50；两者仅记录为技术债，不在本轮推断因果或调整。运行期快照还会反映自动曝光、增益、白平衡和对焦的变化。新节点调用的 V4L2 命令只有 `--list-ctrls-menus`；文档明确区分驱动/相机自动控制与采集工具只读行为。
- 上一轮 baseline 扩展初次在受限沙箱复跑既有 runtime launch 测试时，DDS 因 `getifaddrs: Operation not permitted` 无法创建参与者，102 项基线表现为 1 项环境失败；当时在获准环境复跑的 110 项及本轮扩展后的 125 项均全部通过，没有为绕过环境限制修改阶段 6 代码或测试。
- monitor 首轮 flake8 定位到两处 import 顺序问题，按仓库规则调整后通过。首轮状态测试还把均匀暗灰图误当作“正常纹理”样本，代码正确生成三个视觉 candidate 并降低 score；测试改用确定性纹理图，并用真正均匀图单独验证视觉 candidate 边界。
- 安装后 `ros2 launch --show-args` 首次因默认 `~/.ros/log` 在受限环境只读而失败；把本次 `ROS_LOG_DIR` 定向到 `/tmp` 后参数解析通过。这不是 Launch 语法错误。
- freeze 专项完整测试首次在受限网络命名空间得到 137 项通过、1 项既有阶段 6 runtime 失败；日志为 `getifaddrs/socket: Operation not permitted`。在允许 DDS 本机通信的环境原样重跑后 138 项全部通过，没有修改阶段 6 测试或逻辑。
- 本次首次把两个包的 flake8 测试放在同一 pytest 进程收集，因同名 `test_flake8.py` 产生 import path mismatch；改为按包目录分别运行，不修改测试。freeze runtime 首轮话题含纯数字 PID token，违反 ROS 2 topic 命名规则；改为 `run_<pid>` 后通过。另移除 runtime 测试中的一个未使用 `Path` 导入。
- 联合 Launch 静态测试首轮只有 3 处双引号不符合包内 flake8 quote 规则；统一为单引号后通过，Launch 实现无需修改。
- 故障 report 测试首轮仅有一行 100 字符超过包内 99 字符限制；拆行后针对性 39 项和完整 160 项均通过。构建时因当前 shell 已包含同一 workspace install 前缀出现 colcon override 提示，但目标包仍成功构建。
- underexposure 首轮完整测试在受限网络命名空间得到 162 项通过、1 项既有阶段 6 runtime 失败；日志为 `getifaddrs/socket: Operation not permitted`。在允许 DDS 本机通信的环境重跑最终 164 项全部通过。另一次从仓库根目录单独调用包内 flake8 时误扫 build/install 和其他包；回到目标包目录及 colcon 测试后本包 lint 通过。
- low-information 首轮完整测试在受限环境得到 168 项通过、2 项 runtime 环境失败：一项因默认 `~/.ros/log` 只读，另一项因 DDS `getifaddrs/socket: Operation not permitted`。将 `ROS_LOG_DIR` 指向 `/tmp` 并在允许本机 DDS 的环境原样复跑后 170 项全部通过；没有修改 runtime 测试或阶段 6 逻辑。

### 当前边界

- 没有修改任何 C920 控制参数或 K/D，没有重复阶段 7.1 相机接入、标定、去畸变或 rosbag 流程。
- 没有实现新的图像修改型故障模型或通用生产阈值；manual event 只产生现有 FaultStatus 标签。C920 K/D 与阶段 6 旧语义未修改。

## 2026-08-09 — 阶段 7.1 C920 相机集成收尾

### 当前事实

- C920 已通过 WSL/USBIP + `usb_cam` 接入 ROS 2；正式链为 `/dev/video0` → `/camera/c920/image_raw`，采用 MJPG、1280×720、15 FPS request 和 `mmap`。
- 旧 K/D 已复用验证；正式 ROS `CameraInfo`、`image_proc` 去畸变至 `/camera/c920/image_rect` 和 `rectification_probe` 均已通过。
- 已录制 `/camera/c920/image_raw`、`/camera/c920/camera_info` 和 `/camera/c920/image_rect`，并完成无相机 rosbag 回放验证。

### 当前技术债

- WSL USB/IP 下仍偶发闪帧、帧率波动和图像偏暗；本阶段只记录，不修改驱动、采集参数或图像处理逻辑。

## 2026-08-09 — 阶段 7.1 C920 CameraInfo 运行时链与可选去畸变准备

### 当前事实

- `c920.launch.py` 新增 `enable_rectification` 参数，默认 `false`，因此现有 `/camera/c920/image_raw` 采集链不变。设为 `true` 时，`image_proc/rectify_node` 在 `/camera/c920` 命名空间订阅 `image_raw` 与同命名空间 CameraInfo，并输出 `/camera/c920/image_rect`。
- 新增 `rectification_probe`：同时订阅 raw、rect 与 `/camera/c920/camera_info`，检查 1280×720、`c920_camera_optical_frame`、非零时间戳、K/D/R/P 与正式 YAML 一致性、raw/rect 同 stamp 配对，以及两路实际接收帧率；它不读取图像 payload、不做视觉算法也不保存图像。
- 包清单声明 `image_proc` 运行依赖，`setup.py` 注册 `rectification_probe`。包级构建成功，19 项自动测试均通过；安装区和 `ros2 launch ... --show-args` 确认 CameraInfo URL 默认值和 `enable_rectification=false`。

### 当前边界

- 当前环境中没有 `/dev/video0`，且 `image_proc` 尚未安装；因此本轮没有启动相机或去畸变节点，不能把静态参数链写成 CameraInfo 已在运行时加载或 `image_rect` 已实际发布。
- 没有安装依赖、修改采集参数、生成新标定、覆盖 raw 图像、保存图像、使用 rosbag 或发布 TF。

## 2026-08-09 — C920 正式 ROS CameraInfo 接入

### 当前事实

- `resilient_nav_camera/config/c920_camera_info.yaml` 新增为 ROS camera_calibration 格式的正式资源：1280×720、`plumb_bob`、旧文件中未改动的 3×3 K 和 5 参数 D、单位 rectification `R` 与 `[K|0]` projection `P`。
- `c920.yaml` 声明 `camera_name=c920` 和包内 `camera_info_url`；`c920.launch.py` 以相同的 `package://resilient_nav_camera/config/c920_camera_info.yaml` 默认值显式传给 `usb_cam`，可通过 Launch 参数覆盖 URL。`/camera/c920/image_raw`、MJPG、1280×720、15 FPS、`mmap` 与 `c920_camera_optical_frame` 均未变。
- 包级构建成功；14 项自动测试通过。安装区 YAML 的 K/D、`R`、`P` 和 `camera_info_url` 已通过只读语义检查；本次没有重跑硬件采集或声称新的动态 CameraInfo 证据。

### 当前边界

- 没有加入 `image_proc`、新 TF、其他 ROS 包或新的标定求解；没有覆盖、生成或保存新的内参。
- C920 长期采集稳定性与 WSL USB/IP/MJPEG 问题仍按既有记录处理，未修改驱动或采集参数。

## 2026-08-09 — C920 旧内参临时复用验证器

### 当前事实

- `resilient_nav_camera` 新增 `calibration_reuse_validator`，可用 `ros2 run resilient_nav_camera calibration_reuse_validator` 启动；它只读 `/tmp/camera_params_old.yaml` 中的 OpenCV `camera_matrix` 与 `dist_coeffs`，不保存、覆盖或更新 K/D。
- 验证器订阅 `/camera/c920/image_raw`，固定使用 5×7 ChArUco、`DICT_5X5_100`、square `0.0288 m` 和 marker `0.0144 m`。它自动拒绝角点不足、连续帧运动过快以及 X/Y/Size/Skew 覆盖重复的候选帧。
- 每个接受 Pose 按排序后的 ChArUco ID 交替确定性划分 pose-fit 和 holdout；只用 pose-fit 与旧 K/D 执行 `solvePnP`，并仅以 holdout 角点计算重投影 RMSE。默认目标为 40 个差异 Pose，运行中输出接受数量、holdout RMSE 和覆盖值，结束时输出误差统计与覆盖范围。
- 所有采样门限都通过 ROS 参数暴露。包级 `colcon build --symlink-install --packages-select resilient_nav_camera` 成功，13 项自动测试均通过；没有运行新的硬件采样验证。

### 当前边界

- 本工具只用于评估旧内参是否可复用，不产生新标定、不覆盖 `CameraInfo`、不使用 `image_proc`、不发布 TF，也不代表已经完成相机标定或真实硬件定位。
- WSL USB/IP dropped buffers、MJPEG decode error 与实际帧率抖动仍只记录，未借本工具修改驱动或采集参数。

## 2026-08-08 — C920 硬件采集与元数据探针基线

### 当前事实

- 新增 `ament_python` 包 `resilient_nav_camera`，工作空间当前共 8 个包。它包含 C920 的 `usb_cam` 参数、`c920.launch.py` 与 `c920_probe`。
- `c920.yaml` 固定 `/dev/video0`、MJPG（`usb_cam` 参数为 `mjpeg2rgb`）、`1280x720`、`15 FPS`、`mmap` 和 `c920_camera_optical_frame` frame ID；没有写入标定 URL 或任何假标定参数。
- Launch 在 `/camera/c920` 命名空间启动 `usb_cam_node_exe`，发布 `/camera/c920/image_raw`，并启动订阅该绝对话题的探针。
- 探针不访问 Image 的 `data` 字段，以 `time.monotonic_ns()` 统计接收帧率及平均/最小/最大帧间隔；它逐帧检查 width、height、encoding、step、frame_id 和 header stamp 是否倒退，每 5 秒输出摘要。
- `colcon build --symlink-install --packages-select resilient_nav_camera` 成功；包级 8 项测试均通过。首次限时硬件启动摘要为 54 帧、`13.260 Hz`、平均/最小/最大帧间隔约 `75.416/32.756/168.427 ms`，图像元数据为 `1280x720`、`rgb8`、step `3840`、`camera_c920`，mismatch 和 stamp regression 均为 0。清理修复后的第二次限时启动输出 36 帧、`9.423 Hz`，探针正常结束且无 traceback。

### 问题与处理

- 首次包级测试的 `ament_flake8` 报告 4 项引号和 import 顺序问题。定位到摘要 f-string 与测试 import 排序后修改，重新构建并测试通过。
- 首次受限沙箱内启动被 ROS 2 Launch 日志目录的只读限制阻断。将本次日志定向到 `/tmp` 后在受限环境外完成限时验证；没有留下 `usb_cam_node_exe`、`c920_probe` 或 Launch 进程。
- `usb_cam` 在未配置标定时按其默认行为查询不存在的默认标定文件；本包没有伪造或提供标定参数。摘要后还出现少量 MJPEG 解码错误，因此当前证据仅证明短时链路、话题和元数据检查可运行，不能作为长期稳定性结论。

### 当前边界

- 本次没有实现标定、`image_proc`、TF、rosbag、相机故障模型或真实硬件定位。
- C920 是独立硬件采集接口；没有接入现有虚拟机器人、Gazebo、EKF、健康评估、自适应融合或容错导航链路。

## 2026-08-05 — 阶段 6 传感器健康评估收尾

### 当前事实

- 新增 `resilient_nav_health_assessment`，工作空间当前共 7 个包；它对 IMU、wheel 和 scan 发布 `SensorHealth`，覆盖 timing/stale/delay、wheel freeze、IMU bias 与 Lidar sector blindness。
- `health_evaluator` 将健康输出与 `FaultStatus` 真值对齐，输出混淆矩阵、检测延迟和分类结果；`phase6_health_evaluation.launch.py` 组合阶段 5/6 链路。
- Lidar 统一链评价为 `event_count=1`、检测延迟约 `0.6 s`、F1 约 `0.96`。2026-08-05 全工作空间构建完成 7 个包，测试为 249 项、0 错误、0 失败、1 项跳过。

### 当前边界

- 统一 Launch 的 `evaluator_output_json` 命令行覆盖仍不视为可靠；`health_evaluator.yaml` 固定 `/tmp/phase6_health_evaluation.json` 作为可运行回退。
- 自适应融合、容错导航、Nav2、SLAM、PointCloud2、RGB-D 故障和真实硬件实验仍未实现。

## 2026-08-03 — 阶段 5 可复现故障注入闭环收尾

### 当前事实

- 新增 `resilient_nav_interfaces` 和 `resilient_nav_fault_injection` 两个包，工作空间当前共 6 个包。
- `FaultStatus` 消息已用于发布故障真值标签，状态覆盖 `SCHEDULED`、`ACTIVE`、`ENDED` 和 `CANCELLED`。
- 阶段 5 首批故障模型包含 IMU bias、Gaussian noise、dropout、fixed delay，wheel odometry freeze，以及 LaserScan sector blindness。
- 新增统一入口 `phase5_fault_injection.launch.py`，支持 `scenario_file`、`use_rviz`、`record_bag` 和 `bag_output`，并 Include 阶段 4 健康链。
- 健康 EKF 继续输出 `/odometry/filtered` 并负责主 `odom -> base_footprint` TF；faulted EKF 订阅 `/faulted/wheel/odometry` 和 `/faulted/imu/data`，输出 `/odometry/faulted`，配置 `publish_tf=false`。
- `fault_probe` 已能输出单行 JSON，覆盖 IMU 差值、dropout、delay、wheel freeze、Lidar NaN 和健康/faulted EKF 差异。
- `phase5_record_bag` 使用场景 ID 与时间戳创建唯一目录；`phase5_replay_bag` 可不启动 Gazebo 回放。
- 收尾执行 `colcon build --symlink-install` 成功完成 6 个包；`colcon test && colcon test-result --verbose` 汇总为 193 项、0 错误、0 失败、1 项跳过。
- 动态验证结果：IMU bias active 平均差 `0.15 rad/s`；wheel freeze active raw 位移约 `0.50886 m`、faulted 位移 `0.0 m`；Lidar blindness NaN 比例约 `0.096997`。
- rosbag 记录目录 `/tmp/phase5_bags/wheel_freeze_ekf_comparison_20260803_013910` 含 12 个要求话题、47962 条消息；无 Gazebo 回放时关键话题可见。
- 最终进程检查未发现 Gazebo、RViz、bridge、EKF、注入器或 bag 进程残留。

### 学习要点

- 阶段 5 的核心边界是保留健康基线不被覆盖：raw topic 和 `/odometry/filtered` 持续存在，故障数据单独进入 `/faulted/*` 和 `/odometry/faulted`。
- faulted EKF 可以作为对照估计存在，但不能驱动主 TF；否则 RViz、RobotModel 和后续导航会混用健康与故障估计。
- 指标工具必须按 `FaultStatus` 活动窗口统计，否则故障前后透传样本会稀释 bias、noise 和 dropout 指标。
- fixed delay 的验证应比较消息 stamp 与接收 ROS 时间；probe 本身也必须使用仿真时间，否则会把墙钟 epoch 混入延迟统计。
- rosbag 记录目录必须包含场景 ID 和时间戳，且脚本要避免覆盖已有记录。

### 问题与处理

- 受限沙箱内启动 ROS 2/Gazebo 时出现 `getifaddrs: Operation not permitted` 和 DDS UDP transport 初始化错误。按权限规则在沙箱外重跑后，统一 Launch、bridge、注入器和 EKF 正常启动。
- `fault_probe` 初版把非活动窗口样本计入 IMU bias 平均值，首轮平均差约 `0.052`，低于配置 `0.15`。修复为默认使用仿真时间并按非 CANCELLED 的 FaultStatus 时间窗过滤，复验得到 active 平均差 `0.15`。
- 阶段 5 三个注入器初版 Ctrl-C 时重复 `rclpy.shutdown()`，退出码为 1。修复为捕获 `KeyboardInterrupt` 并仅在 `rclpy.ok()` 时 shutdown，后续停止 cleanly 退出。
- `phase5_replay_bag` 初版用 `subprocess.call` 包装 rosbag play，Ctrl-C 会打印 Python traceback。改为 `os.execvp()` 直接交给 rosbag 原生命令。
- 既有 `system_heartbeat` 在 Ctrl-C 时仍有重复 shutdown 异常；本次授权范围未修改 `resilient_nav_monitor`，已在阶段 5 总结中如实记录。

### 当前边界

- 阶段 5 已完成故障注入与实验复现闭环，不包含健康评估、自适应融合、容错导航、Nav2、SLAM、PointCloud2 或真实硬件实验。
- RGB-D 图像故障未进入阶段 5 首批模型。
- RViz 配置已提供并可选启动，本次动态验收主要使用 headless 运行，未保存截图证据。

## 2026-08-01 — 阶段 4 收尾状态同步

### 当前事实

- 阶段 4 已完成 IMU、二维 Lidar、RGB-D、传感器 TF、专用 RViz，以及 wheel odometry + IMU 的固定字段 EKF 基线。
- EKF 输出 `/odometry/filtered`，并在完整阶段 4 Launch 中独占 `odom -> base_footprint` TF；阶段 3 默认 `/odom` 与旧 TF broadcaster 保持可用。
- 收尾时完整执行四包 `colcon build --symlink-install` 和 `colcon test`；结果为 4 包构建成功，64 项测试、0 错误、0 失败、1 项跳过。
- 当前 EKF 源码和安装区均为 20 Hz；10 Hz 只作为短时负载诊断，不是最终配置。

### 学习要点

- 仓库只保留代码、技术基线文档和简短阶段状态；阶段结果交接和面向学习的总结材料应由外部流程单独交付。
- `--symlink-install` 的安装资源通常是符号链接，检查安装状态时不能只使用 `find -type f`，还应验证链接目标和路径存在性。
- 诊断性临时参数如果没有保留独立采样日志，只能记录目的和最终恢复状态，不能补造性能结论或把未来阶段内容写成已完成。

### 当前边界

- 截至 2026-08-01，阶段 4 到此完成，下一步计划进入阶段 5。
- 截至 2026-08-01，PointCloud2、真实标定、长期性能、SLAM、Nav2、故障注入、健康评估、自适应融合和容错导航仍未实现；当前状态已由 2026-08-03 阶段 5 记录更新。

## 2026-08-01 — 阶段 4 多传感器与 EKF 技术里程碑

### 当前事实

- 阶段 4 已完成 Gazebo IMU、单层二维 GPU Lidar 和 RGB-D camera，并通过定向 `ros_gz_bridge` 提供 `/imu/data`、`/scan` 及四个相机 Image/CameraInfo 接口；没有 PointCloud2 bridge。
- 新增 `phase4_imu_lidar_demo.launch.py`、`phase4_rgbd_demo.launch.py` 和 `phase4_sensors.rviz`。专用 RViz 以 `odom` 为 Fixed Frame，保留 RobotModel、TF、Best Effort LaserScan、彩色 Image，并新增 filtered odometry。
- 当前环境可发现 `robot_localization` 3.8.3 的 `ekf_node`。新增 `resilient_nav_localization` 包，真实完整入口为 `phase4_ekf_demo.launch.py`。
- 现有 Launch 链新增可透传的 `odom_ros_topic` 与 `start_odom_tf_broadcaster` 参数。阶段 4 完整链将原始 DiffDrive 里程计映射为 `/wheel/odometry` 并关闭旧 broadcaster；阶段 3 的默认 `/odom` 与旧 TF 节点保持可用。
- EKF 采用 `use_sim_time=true`、`two_d_mode=true`、20 Hz、`world_frame=odom`、`base_link_frame=base_footprint`，只融合 wheel `vx` 和 IMU yaw rate，输出 `/odometry/filtered` 并独占 `odom -> base_footprint` TF。
- 完整工作空间四包构建通过；测试汇总为 64 项、0 错误、0 失败、1 项跳过。Xacro、URDF、YAML、XML、Launch/Python 和资源安装均有自动检查。
- 动态直行后 wheel/filtered x 约为 `0.254200/0.253207 m`；旋转后 yaw 约为 `0.851/0.799 rad`。80 个 filtered 样本无 NaN 或明显跳变。
- EKF 20 Hz 配置的仿真时间戳实测为 `20.000 Hz`，墙钟约 `12.470 Hz`，同轮 Gazebo `real_time_factor≈0.6745`。暂停时 wheel、IMU、filtered 均停止，`/clock` 数值冻结；恢复后继续。

### 学习要点

- 原始轮式里程计使用 `/wheel/odometry`、融合结果使用 `/odometry/filtered`，可以明确区分测量与估计，为后续故障注入和对照实验保留稳定边界。
- DiffDrive pose 和 twist 来自同一轮编码来源；最小基线只融合 `vx`，避免无依据地重复融合同一信息。IMU orientation covariance 为零且加速度含重力，所以当前只使用具有非零 covariance 的 yaw rate。
- TF 发布权应随状态估计层切换：阶段 4 关闭旧 broadcaster，让 EKF 独占 odom TF；通过参数化复用而不是删除旧节点，才能保持阶段 3 启动行为不变。
- 仿真中的配置频率、消息仿真时间戳频率和墙钟到达率必须分别记录。低于 1 的 real-time factor 会降低墙钟吞吐，但不等于 sensor 或 EKF 的仿真时间配置失效。

### 问题与处理

- 30 Hz EKF 在同时运行 Gazebo GUI、RViz、RGB-D 和检查订阅时持续报告墙钟更新率偏低。对比诊断、仿真 stamp 与 world stats 后，把基线调整为 20 Hz；最终仿真 stamp 为精确 20 Hz，状态连续。
- 高并发验收时出现过一次 EKF update cycle 超时。输入和输出未中断，降低检查并发后双时钟采样、直行/旋转和连续性均通过，因此记录为主机负载下的瞬时警告，没有篡改 covariance 掩盖。
- RViz 启动首帧曾早于 TF cache，后续订阅与 TF 正常。Ctrl-C 时既有 heartbeat 仍有重复 shutdown 日志；按任务边界没有修改 `resilient_nav_monitor`，最终进程残留检查为空。
- 本次首次 GitHub 发布要求 GitHub CLI；初检时 `gh` 不存在。安装和浏览器认证状态须在发布前再次确认，不能伪造远端成功。

### 当前边界

- PointCloud2、真实编码器/IMU 标定、长期累计误差、map-frame 全局定位、SLAM 和 Nav2 尚未完成。
- 当前 EKF 是固定字段基线，不是故障感知或自适应融合。
- 阶段 4 RViz 未保存像素级截图的证据边界保持不变；外部交接材料不存放在仓库中。

## 2026-08-01 — 阶段 4 依赖安装准备与传感器坐标架构

### 当前事实

- 新增 `scripts/install_phase4_dependencies.sh`，只处理官方 Jazzy 包 `ros-jazzy-robot-localization`；脚本先加载 ROS 2 环境并用 `ros2 pkg prefix` 早退，缺失时要求交互式人工确认，唯一安装命令不带 `-y`。本次只执行 `bash -n`，没有运行脚本、`sudo` 或 `apt`。
- `resilient_nav_robot.urdf.xacro` 新增 IMU、二维 Lidar、相机安装链和未来机械臂安装基准，共六个 link 和六个 fixed joint；所有 xyz/rpy 由 Xacro property 集中管理。
- `camera_link` 的 +x 朝机器人前方，`camera_optical_frame` 使用 `[-pi/2, 0, -pi/2]` 固定旋转，形成 +x 右、+y 下、+z 前的 ROS optical frame。
- IMU、Lidar、相机支架和相机本体具有简单 visual；新增 link 没有 collision、inertial、Gazebo sensor 或新 plugin，`arm_mount_link` 只是空安装基准。
- 静态测试扩展为 12 个 pytest case，覆盖六个 link/joint、父子关系、外参、标准 optical rotation、visual-only 边界、禁止 sensor/新 plugin 及第三阶段底盘参数回归。
- 源码与安装后 Xacro 均可展开，`check_urdf` 成功。描述包构建成功，包级结果为 13 项、0 错误、0 失败、0 跳过；工作空间累计为 47 项、0 错误、0 失败、1 项既有跳过。

### 学习要点

- 把 `camera_mount_link` 与 `camera_link` 分层，可以让未来支架/云台变化整体作用于相机子树，同时保留相机本体与 optical frame 的标准轴约定。
- 安装坐标可以先于传感器插件建立，但必须把“存在 TF 基准”和“已经产生传感器数据”严格区分。
- 新增纯 visual 的 fixed link 不需要 collision 或 inertial；同时用回归测试锁定车体质量、惯性、轮径、轮距和插件，可避免阶段 4 准备工作改变阶段 3 动力学。
- 依赖安装脚本应先检查 ROS 包是否已发现，并在唯一目标包和唯一安装命令前设置显式人工确认，避免把依赖准备扩大成系统升级。

### 问题与处理

- 源码 Xacro 与安装后 Xacro 的展开文件直接执行 `cmp` 时在第 3 行不同。文本 diff 定位到 Xacro 自动生成注释记录了不同输入路径；删除自动生成注释后两份展开内容完全一致，且两份都通过 `check_urdf`。因此这是来源路径注释差异，不是安装产物陈旧或机器人语义不同。
- 在仓库根目录汇总测试时，首次向 `colcon test-result` 传入相对 `build/...`，工具因当前目录不是 `ros2_ws` 而报告路径不存在。改用 `/home/kylian/projects/resilient_nav_lab/ros2_ws/build/...` 绝对路径后，包级 13 项和工作空间累计 47 项结果均成功读取；该错误只影响结果查询命令，不影响先前已完成的构建或测试。

### 当前边界

- 没有安装 `robot_localization`，没有 Gazebo sensor、bridge、EKF 或机械臂模型。
- 没有启动 Gazebo 或 RViz；新增 visual 和运行时固定 TF 尚未动态观察。
- 尚未验证传感器消息、frame_id、频率、QoS、时间戳、噪声、同步或定位结果。

## 2026-07-29 — 阶段 3 收尾

### 当前事实

- 阶段 3 已按“虚拟差速机器人与基础运动”边界完成，正式总结见 `docs/PHASE3_SUMMARY.md`。
- 收尾执行 `colcon build --symlink-install`，`resilient_nav_description`、`resilient_nav_monitor` 和 `resilient_nav_simulation` 共 3 个包全部成功。
- 收尾执行 `colcon test` 和 `colcon test-result --verbose`，汇总为 43 项、0 错误、0 失败、1 项按既有配置跳过。
- 根据精简收尾要求，保留已经记录的机器人生成、直行、原地旋转、圆弧、停车、标准 ROS 2 话题、TF 和 Gazebo/RViz 同步结果，没有重复运行完整图形仿真或运动流程。
- 最终文档记录了完整 Demo、仅 Spawn、三种运动工具以及快速构建/测试命令。
- 收尾停止了本次 Gazebo/RViz Launch，进程检查没有发现相关残留；删除了本次隔离 ROS 日志、对应 Launch 参数文件和源码树 Python 缓存。
- 本次收尾没有修改功能代码。

### 学习要点

- 阶段收尾应区分“已完成的动态验收”和“收尾时重复执行的检查”；保留可追溯结果并只重跑快速自动化验证，可以避免把重复操作误写成新的实验。
- 完成阶段状态更新时，需要同步 README、范围、环境、学习日志、协作说明和正式总结，避免“进行中”“已完成”并存。
- 临时文件清理应限定到本次会话可确认的日志、参数和缓存，不能因为目录中存在旧文件就推断其都可删除。

### 当前边界

- 阶段 3 完成结论限于低速、短时、平地差速运动基线。
- 完整运动性能、传感器、`ros2_control`、Nav2、SLAM、定位和后续容错导航能力仍需单独授权。
- 既有 `system_heartbeat` 在整套 Launch Ctrl-C 时的重复 shutdown 警告未在本次收尾修改；它不影响已记录验收，且没有留下进程。

## 2026-07-29 — 阶段 3 运动测试工具与三模式验收

### 当前事实

- `resilient_nav_simulation` 新增 `motion_test` 可执行工具，支持直行、原地旋转和圆弧，线速度、角速度与持续时间可配置。
- 工具以 `20 Hz` 发布 `/cmd_vel`，启动前等待命令订阅者；正常结束、异常和 Ctrl-C 均通过同一清理路径重复发送 5 条零 Twist。
- 单元测试覆盖三种模式映射、参数校验、正常结束和模拟 Ctrl-C 自动停车；真实 Ctrl-C 后连续两次 `/odom` twist 也均为零。
- 初次旋转和圆弧测试发现 Gazebo 模型与 `/odom`/TF 存在约厘米级系统性偏差。对照几何后确认，DiffDrive 里程计以轮轴中点积分，而旧 `base_footprint` 位于其后方 `0.10 m`。
- `base_footprint` 已移到左右驱动轮轴中点，`base_link` 相对它为 `[-0.10, 0, 0.15] m`；物理几何、接触点和重心之间的相对关系不变。
- DiffDrive 新增显式 `frame_id=odom` 和 `child_frame_id=base_footprint`。实际 `/odom` frame 字段、ROS 侧 TF 与描述树现在使用同一语义。
- 修正后三组 `1.5 s` 最终 `/odom` `(x, y, yaw)` 分别为：直行 `(0.276800, 0.000000, 0.000)`，原地旋转 `(0.000000, 0.000000, 0.832)`，圆弧 `(0.261344, 0.073738, 0.550)`。
- 对应 Gazebo 位姿分别为 `(0.276159, 0.000000, 0.000)`、`(-0.005241, 0.002314, 0.780)` 和 `(0.254261, 0.069973, 0.525)`；最大位置差约 `8 mm`，最大航向差约 `0.052 rad`。
- 三组停止后的 `/odom` twist 均为零，连续 TF 样本保持不变；RViz 完成 OpenGL 4.5 初始化并通过内部 listener 订阅 `/tf`、`/tf_static`，RobotModel 订阅 `/robot_description`。
- 最终相关三包构建成功，测试汇总为 43 项、0 错误、0 失败、1 项按既有配置跳过；结束后没有相关后台进程残留。

### 学习要点

- 差速轮式里程计的参考点应与 `base_footprint` 一致。只让 frame 名称一致而物理原点不同，会在直行时隐藏问题，却在旋转和圆弧时形成确定性的轨迹偏差。
- `base_link` 可以继续表示车体几何中心，`base_footprint` 则表示地面上的轮轴参考点；二者通过固定 TF 表达偏移，比在 TF 广播节点中补偿运动位姿更清晰。
- 自动停车不能只依赖发布一次零速度。让清理路径在正常、异常和 Ctrl-C 下统一执行并短间隔重复发布，可以覆盖 DDS/bridge 的最终消息交付。
- RViz 同步应同时检查进程初始化、RobotModel 描述订阅、TF listener 订阅和数值 TF，而不只依据窗口是否打开。

### 当前边界

- 当前结果只验证给定低速、短时、平地命令，不代表速度精度、长距离里程计、轨迹跟踪、高速急停或控制鲁棒性已验收。
- Gazebo 物理位姿与轮式里程计仍有毫米级位置和小角度航向差；当前如实记录，没有引入传感器或额外定位来源去校正。
- 没有增加传感器、`ros2_control`、Nav2、SLAM 或新软件依赖。既有 `system_heartbeat` Ctrl-C 重复 shutdown 警告仍未在本任务中处理。

## 2026-07-29 — 阶段 3 Gazebo 与 RViz 联合演示

### 当前事实

- 新增 `phase3_demo.launch.py`，Include 现有 `phase3_spawn.launch.py` 并通过 `use_rviz` 条件启动一个 RViz；Gazebo、bridge、`robot_state_publisher` 和 `odom_tf_broadcaster` 没有重复定义。
- Demo 透传实体名和初始位姿参数；`use_rviz` 默认开启，关闭时仍运行相同 Spawn 链。
- 专用 `phase3_demo.rviz` 以 `odom` 为 Fixed Frame，启用 Grid、RobotModel 和 TF；Odometry 显示绑定 `/odom`，默认关闭供用户按需启用。
- `resilient_nav_simulation` 新增 `rviz2` 运行依赖和 RViz 资源安装规则。
- 相关 3 个包构建成功；测试汇总为 30 项、0 错误、0 失败、1 项跳过。
- 实际 Demo Launch 报告机器人生成成功，RViz 完成 OpenGL 4.5 初始化且未报告配置或 TF 错误；ROS 图只有一个 `robot_state_publisher`，没有独立关节状态发布器。
- 向 `/cmd_vel` 以 `0.2 m/s`、10 Hz 发送 15 条消息后立即发送零 Twist。Gazebo 模型 X 从 `0.000000 m` 移至 `0.466559 m`。
- 停止后 `/odom` X 为 `0.467200 m` 且 twist 全零，`odom -> base_footprint` TF X 为 `0.467 m`；RViz RobotModel 与 TF listener 正在消费现有描述和 TF 链，三侧位姿在毫米级一致。
- 验证结束后发送了明确停止命令，并停止整个 Launch；进程检查没有发现 Gazebo、RViz、bridge 或状态发布节点残留。

### 学习要点

- 联合演示应 Include 已验收的启动链，而不是复用会自行创建状态发布节点的独立 Display Launch；这样可以避免同名节点、重复 `/joint_states` 和重复 TF。
- 使用 `odom` 作为 RViz Fixed Frame 后，RobotModel 通过 `odom -> base_footprint -> ...` TF 链显示实际平移，而不是始终固定在机器人根坐标系原点。
- RViz 是否同步可以通过数据所有权和数值闭环验证：RobotModel 消费现有描述与 TF，同时 Gazebo 模型位姿、`/odom` 和 RViz 所用 TF 应在允许误差内一致。

### 当前边界

- `/odom` 显示只是已有轮式里程计的可选可视化，不是新增传感器。
- 没有加入传感器、`ros2_control`、额外 bridge、关节状态发布器或第二个 `robot_state_publisher`。
- Ctrl-C 时 RViz 和本次新增链路正常退出；既有 `system_heartbeat` 仍出现 `rcl_shutdown already called`，不影响联合演示验收。

## 2026-07-28 — 阶段 3 ROS odom TF 广播

### 当前事实

- `resilient_nav_monitor` 新增 `odom_tf_broadcaster`，订阅 `/odom` 并把消息位姿发布为 `odom -> base_footprint` 动态 TF；每条 TF 沿用对应里程计消息的 `header.stamp`。
- 包清单新增 `geometry_msgs`、`nav_msgs` 和 `tf2_ros` 依赖，`setup.py` 注册同名可执行入口。
- `phase3_spawn.launch.py` 在不改变启动命令的情况下启动新节点，并设置 `use_sim_time=true`。
- Gazebo 原生 TF/位姿输出没有加入 bridge；静态测试继续禁止 `gz.msgs.Pose_V`，避免与 ROS 侧广播节点形成重复 TF 来源。
- `colcon build --symlink-install --packages-up-to resilient_nav_simulation` 成功完成 3 个包；相关测试汇总为 28 项、0 错误、0 失败、1 项跳过。
- 实际 Launch 报告机器人实体创建成功；ROS 参数读取确认 `odom_tf_broadcaster` 的 `use_sim_time=True`。
- `/odom` 实测约为 45–47 Hz；`tf2_echo odom base_footprint` 连续输出时间为 `30.12`、`31.00`、`31.86`、`32.76`、`33.66 s` 的有效变换。
- `/tf` 只有 `robot_state_publisher` 和 `odom_tf_broadcaster` 两个发布者；`robot_bridge` 只处理 `/cmd_vel`、`/odom` 与 `/joint_states`，没有 TF 接口。

### 学习要点

- 由 `/odom` 消息生成 TF 时，应复制消息时间戳而不是读取回调时刻，才能让位姿和 TF 保持同一时间基准。
- Gazebo DiffDrive 的原生位姿输出与 ROS 侧 odom TF 是两种可选来源；当前只保留后者，可以明确所有权并避免同一变换重复发布。
- `use_sim_time` 是 ROS 节点参数；本节点虽然直接沿用消息时间戳，仍在阶段 3 Launch 中显式启用该参数，使节点时钟行为与整条仿真链一致。

### 当前边界

- 只发布 `odom -> base_footprint`，没有桥接 Gazebo 自带 TF，也没有增加传感器或 `ros2_control`。
- 当前验证覆盖静止机器人下的持续 TF 发布，没有扩展阶段 3 的运动性能范围。
- Ctrl-C 停止时新节点正常退出；既有 `system_heartbeat` 仍观察到 `rcl_shutdown already called`，不影响本次 TF 验收。

## 2026-07-28 — 阶段 3 纵向支撑与重心修正

### 当前事实

- 修正前 URDF 转换后的整机纵向重心约为 `x=-0.0058 m`，后球轮与驱动轮轴形成的支撑范围为 `[-0.18, 0] m`，重心距前支撑边仅约 `0.006 m`。
- 车体原点离地 `0.15 m`、碰撞盒高度 `0.15 m`，水平时底部离地 `0.075 m`；驱动轮和球形支撑轮的接地高度一致，因此没有修改车体高度、碰撞尺寸、轮径或轮距。
- 驱动轮轴从 `x=0` 前移到 `x=0.10 m`，球形支撑轮从 `x=-0.18 m` 后移到 `x=-0.20 m`，车体惯性原点从 `[0, 0, 0]` 调整为 `[-0.05, 0, -0.04] m`。
- 修正后展开 SDF 的车体/支撑轮合并惯性原点约为 `[-0.0558, 0, 0.1077] m`，整机纵向重心约为 `x=-0.031 m`，前后支撑余量均超过 `0.10 m`。
- Xacro、`check_urdf`、URDF 到 SDF 转换和 16 项聚焦静态测试通过；DiffDrive 轮径、轮距、Gazebo 话题与 ROS bridge 映射保持原值。
- `colcon build --symlink-install --packages-up-to resilient_nav_simulation` 成功完成 3 个包；描述包 8 项和仿真包 8 项测试通过，工作空间汇总为 23 项、0 错误、0 失败、1 项跳过。
- 实际 Launch 中机器人生成成功；落地静止时连续两次位姿的 roll/pitch/yaw 均约为零。
- 相同短时直行/停止序列后，模型 X 约为 `0.403759 m`，停止后连续两次 pitch 约为 `-0.000001 rad`，满足 `|pitch| < 0.05 rad`。
- `/odom` 停止速度为零，`/joint_states` 左右轮反馈正常；验证停止后没有后台进程残留。

### 学习要点

- 两轮差速机器人加单后支撑轮时，纵向重心不能只看车体几何中心；它必须在后支撑点与驱动轮轴构成的支撑区内部保留足够动态余量。
- 修正前落地静止虽然水平，但重心几乎位于前支撑边，驱动/制动后车体前缘会成为新的稳定接触点，形成约 `0.31 rad` 的残余 pitch。
- 同时前移驱动轮、后移支撑轮并降低/后移车体惯性原点，可以扩大有效支撑区并降低俯仰力矩，而无需改变轮径、轮距或通信接口。
- 对固定连接 link 应检查 URDF 转 SDF 后的合并惯性；仅查看各 URDF link 的局部 inertial origin 容易忽略转换后的整机重心。

### 当前边界

- 当前只验收水平落地和一次短时直行/停止后的 `|pitch| < 0.05 rad`，不代表更高速度、急停、倒车、转向或坡面姿态已经验证。
- 没有加入 odom TF、传感器、`ros2_control` 或新的 Gazebo/ROS 话题。
- Ctrl-C 停止时仍观察到既有 `system_heartbeat` 的 `rcl_shutdown already called`，不影响本次动力学验收。

## 2026-07-28 — 阶段 3 ROS 基础运动 bridge 与短时直行

### 当前事实

- `phase3_spawn.launch.py` 新增 `robot_bridge`，保持原启动命令不变，并继续通过阶段 2 Launch 保留 `/clock` bridge。
- ROS 2 `/cmd_vel` 单向桥接到 Gazebo `/model/resilient_nav_robot/cmd_vel`；Gazebo odometry 和 joint state 分别单向桥接并重映射到 ROS 2 `/odom` 与 `/joint_states`。
- bridge 话题随 `entity_name` 动态构造，非默认 Gazebo 实体名不会改变三个标准 ROS 2 话题名称。
- 包清单补充 `geometry_msgs`、`nav_msgs` 和 `sensor_msgs` 运行依赖。
- `colcon build --symlink-install --packages-up-to resilient_nav_simulation` 成功完成 3 个包；描述包 7 项和仿真包 8 项测试全部通过，工作空间汇总为 22 项、0 错误、0 失败、1 项跳过。
- 实际 Launch 报告三条 bridge 的方向和消息类型符合设计，机器人实体创建成功。
- 短时发送 `linear.x=0.2 m/s` 后发送全零 Twist，模型 X 从约 `0.000000 m` 移至 `0.549754 m`，Y 和 yaw 仍约为零。
- 停止后两次 Gazebo 位姿一致，`/odom` 报告 X 约 `0.5494 m` 且 twist 全零。
- `/joint_states` 同时包含左右轮关节，停止后的关节速度接近零；验证结束后没有后台进程残留。

### 学习要点

- `parameter_bridge` 的 `]` 可限定 ROS→Gazebo，`[` 可限定 Gazebo→ROS；对控制命令和反馈分别限定方向，避免不必要的双向回环。
- Launch 的 remapping 只改变 ROS 2 侧名称，因此可以保留 Gazebo 的模型/世界作用域话题，同时向 ROS 节点提供标准 `/cmd_vel`、`/odom` 和 `/joint_states`。
- `robot_state_publisher` 消费桥接后的 `/joint_states`，因此运行时 `/tf` 会更新轮关节变换，但这不等于发布了 odom TF。
- ROS CLI 自动发现 `/joint_states` 类型时曾受发现缓存影响；显式指定 `sensor_msgs/msg/JointState` 后成功读取实际消息。

### 当前边界

- 没有桥接 DiffDrive 的 Gazebo `/model/resilient_nav_robot/tf`，抽查 ROS `/tf` 仅见车体到左右轮的关节变换，没有 odom 到基座变换。
- 直行停止后的模型 pitch 约为 `0.309682 rad`；基础移动已确认，但支撑轮、姿态稳定性和动力学参数尚未调优。
- 当前只做一次短时直行和停止，不代表速度精度、转向、轨迹跟踪或控制鲁棒性已经验收。
- 没有加入传感器、`ros2_control`、Nav2、SLAM 或后续阶段能力。
- Ctrl-C 停止时仍观察到既有 `system_heartbeat` 的 `rcl_shutdown already called`，不影响本次 bridge 与运动验收。

## 2026-07-28 — 阶段 3 Gazebo 原生差速与关节状态插件

### 当前事实

- `resilient_nav_robot.urdf.xacro` 新增 Gazebo Harmonic `DiffDrive` 和 `JointStatePublisher` 系统插件，使用当前 `gz-*` 文件名和 `gz::sim::systems::*` 类名。
- DiffDrive 使用真实关节 `left_wheel_joint`、`right_wheel_joint`，轮距 `0.39 m`，轮半径 `0.10 m`。
- 默认 Gazebo Transport 话题明确为 `/model/resilient_nav_robot/cmd_vel`、`/model/resilient_nav_robot/odometry` 和 `/world/resilient_lab/model/resilient_nav_robot/joint_state`；自定义 `entity_name` 会同步替换话题中的模型名。
- Xacro 展开、`check_urdf` 和 URDF 到 SDF 转换通过；转换后的 SDF 保留两个插件及预期参数。
- `colcon build --symlink-install --packages-up-to resilient_nav_simulation` 成功完成 3 个包，描述包和仿真包共 14 项静态测试全部通过。
- 实际 Launch 报告实体创建成功，模型列表包含 `resilient_nav_robot`，落地 Z 位姿约为 `-0.000001 m`。
- `gz topic` 验证速度话题有 `gz.msgs.Twist` 订阅者，里程计有 `gz.msgs.Odometry` 发布者，关节状态有 `gz.msgs.Model` 发布者；实际关节状态消息包含左右轮关节。
- 现有 bridge 配置保持只桥接 `/clock`，本任务没有新增 ROS—Gazebo bridge、ROS 2 odom TF、传感器或 `ros2_control`。
- 验证停止后没有 Gazebo 或 ROS 2 后台进程残留。

### 学习要点

- 轮距应使用左右轮心之间的距离；当前 `wheel_y=0.195 m`，因此 DiffDrive 的 `wheel_separation=0.39 m`，不是车体宽度 `0.35 m`。
- JointStatePublisher 的显式 `<topic>` 能保留世界/模型作用域的 Gazebo 原生名称，重复 `<joint_name>` 可把发布内容限定为两个可动轮关节。
- 将 Launch 的 `entity_name` 传入 Xacro，可避免实体重命名后插件话题仍硬编码为默认模型名。
- Gazebo Transport 原生话题出现不代表 ROS 2 已获得对应数据；当前只有 `/clock` 经过 bridge。

### 当前边界

- 本次没有向 `cmd_vel` 发送命令，因此尚未验收行驶距离、转向方向、速度限制或轨迹精度。
- DiffDrive 自带的 Gazebo Transport 位姿输出没有桥接到 ROS 2，也没有接入 ROS TF 树。
- Display Launch 的 ROS 2 `/joint_states` 与 Gazebo 原生关节状态仍是独立链路。
- Ctrl-C 停止时仍观察到既有 `system_heartbeat` 的 `rcl_shutdown already called`，不影响本次插件和话题验收。

## 2026-07-28 — 阶段 3 Gazebo 无驱动物理落地

### 当前事实

- `resilient_nav_robot.urdf.xacro` 为车体、左右轮和球形支撑轮增加 Gazebo 命名材质及 `mu1`、`mu2` 接触摩擦。
- 左右轮摩擦设为 `1.0`，车体为 `0.5`，球形支撑轮为 `0.05`；阶段 2 地面碰撞增加显式 ODE 摩擦 `mu=1.0`、`mu2=1.0`。
- `resilient_nav_simulation` 新增 `phase3_spawn.launch.py`，通过 Include 复用 `phase2_world.launch.py`，并用 `robot_state_publisher` 和 `ros_gz_sim create` 从 `/robot_description` 创建实体。
- 默认实体名为 `resilient_nav_robot`，默认从 `z=0.25 m` 生成；初始 X、Y、Z 和 yaw 均可通过 Launch 参数配置，重复名称不允许自动重命名。
- Xacro、`check_urdf`、世界 SDF 和 URDF 到 SDF 的转换验证通过；转换后的四个实体碰撞均保留预期摩擦值和 Gazebo 材质。
- `colcon build --symlink-install --packages-up-to resilient_nav_simulation` 成功完成 3 个包。
- 描述包新增 4 项静态测试，仿真包扩展为 7 项静态测试，两包共 11 项全部通过。
- 实际 Launch 中 `ros_gz_sim create` 报告实体创建成功；Gazebo 模型列表包含阶段 2 三个静态模型和新机器人。
- 默认高度下落并稳定后，模型 XYZ 约为 `[-0.000000, 0.000000, -0.000001] m`，RPY 约为零；验证结束后没有后台进程残留。

### 学习要点

- 本机 Gazebo URDF 转换器可把 `<gazebo reference>` 中的 `mu1`、`mu2` 转换成 SDF ODE 接触摩擦。
- `<gazebo reference>` 中使用 `Gazebo/Blue` 等命名材质可生成带材质脚本的 SDF；嵌套 RGBA 材质会触发缺少字符串值的转换警告，因此 RViz 颜色继续由 URDF material 定义，Gazebo 覆盖使用命名材质。
- 将生成 Launch 放在 simulation 包并 Include 阶段 2 Launch，可以复用已验收世界、时钟桥和心跳链路，同时让机器人几何与物理参数继续归 description 包维护。
- `base_footprint` 在 URDF 到 SDF 转换时会吸收固定连接的车体和支撑轮；左右 continuous 轮关节保留为可动关节，不需要驱动插件也能完成重力和接触测试。

### 当前边界

- 当前只验证无驱动实体生成、自由落体和接触稳定，没有加入 Gazebo JointStatePublisher、DiffDrive、传感器、`ros2_control` 或任何控制命令。
- `robot_state_publisher` 当前只提供描述和固定 TF，Gazebo 中的轮关节状态尚未桥接回 ROS 2。
- 人工 Ctrl-C 停止包含阶段 2 心跳的 Launch 时，`system_heartbeat` 仍会报告一次 `rcl_shutdown already called`；这不影响本次生成与落地结论，且停止后无进程残留，后续可在监控节点维护任务中处理。

## 2026-07-28 — 阶段 3 RViz 显示与运行时 TF

### 当前事实

- `resilient_nav_description` 新增 `launch/display.launch.py` 和 `rviz/display.rviz`，CMake 安装规则同步覆盖 `launch/`、`rviz/` 和 `urdf/`。
- 包清单新增 `joint_state_publisher`、`joint_state_publisher_gui`、`launch`、`launch_ros`、`robot_state_publisher`、`rviz2` 和 `xacro` 运行依赖。
- Display Launch 从 Xacro 生成 `robot_description`，始终启动 `robot_state_publisher` 和 RViz。
- `use_gui:=false` 启动普通 `joint_state_publisher`；`use_gui:=true` 启动 `joint_state_publisher_gui`。
- `colcon build --symlink-install --packages-select resilient_nav_description` 成功完成 1 个包，安装空间包含 Launch、RViz 和 Xacro 资源。
- 两种 `use_gui` 分支均完成限时启动验证，RViz 成功初始化 OpenGL 4.5。
- 验证期间可见 `/joint_state_publisher`、`/robot_state_publisher` 和 `/rviz`，且 `/joint_states`、`/robot_description`、`/tf`、`/tf_static` 消息类型正确。
- `tf2_echo` 确认 `base_footprint` 到 `base_link` 的固定变换为 Z 轴 `0.150 m`。

### 学习要点

- 用 Launch 的 `IfCondition` 和 `UnlessCondition` 可以让普通与 GUI 关节状态发布器互斥，避免两个节点同时发布同一关节状态。
- `robot_description` 由 Launch 调用安装后的 Xacro 动态生成，可避免维护重复的展开 URDF。
- RViz 的 RobotModel 从 `/robot_description` 读取模型，TF 显示则用于检查 link 树是否随关节状态正确更新。

### 当前边界

- 当前关节状态来自独立 ROS 2 发布器，不是 Gazebo 仿真关节反馈。
- 没有加入 Gazebo 插件、传感器、`ros2_control`、差速命令或控制功能。
- 在该子任务验收时，模型尚未生成到 Gazebo；后续无驱动生成与落地结果见本日志更新的阶段 3 条目。

## 2026-07-28 — 阶段 3 基础差速机器人描述

### 当前事实

- 已创建 `ament_cmake` 包 `resilient_nav_description`，并安装包内 `urdf/` 描述资源。
- `resilient_nav_robot.urdf.xacro` 定义 `base_footprint`、`base_link`、左右驱动轮和一个球形支撑轮。
- `base_link`、驱动轮和支撑轮均包含基础几何 visual、collision、质量和惯性张量；左右驱动轮使用 continuous 关节，支撑轮当前使用 fixed 关节。
- 源码 Xacro 成功展开为 URDF，`check_urdf` 成功解析出以 `base_footprint` 为根的 5 个 link 和 4 个 joint。
- `colcon build --symlink-install --packages-select resilient_nav_description` 成功完成 1 个包。
- 加载工作空间后，`ros2 pkg prefix resilient_nav_description` 返回工作空间安装前缀；安装后的 Xacro 再次通过展开和 `check_urdf`。

### 学习要点

- `base_footprint` 适合作为机器人在地面的投影根坐标系，而带几何、碰撞和惯性的 `base_link` 可通过固定高度偏移与其连接。
- 圆柱驱动轮的几何轴需要旋转到车体 Y 轴，关节轴也应设置为 `0 1 0`，这样左右轮围绕轮轴旋转。
- Xacro 展开成功只验证宏和 XML 生成；继续使用 `check_urdf` 可以同时验证 link/joint 树和 URDF 语义。
- 对源码和安装后的描述资源分别复验，可以同时覆盖模型内容与 CMake 安装规则。

### 当前边界

- 当前只建立静态机器人描述，没有加入 Gazebo 插件、传感器、`ros2_control` 或控制功能。
- 尚未启动 `robot_state_publisher`、发布运行时 TF 或 JointState，也没有把机器人生成到 Gazebo。
- 差速运动链路和阶段 3 后续验收需要在单独任务中继续实施。

## 2026-07-27 — 阶段 2 Gazebo 基础仿真与时钟链路收尾

### 当前事实

- 已通过现有 ROS 2 Jazzy 软件源安装 `ros-jazzy-ros-gz`，Gazebo Sim 版本为 8.11.0；安装和验证日志分别保存在 `docs/gazebo_install_20260727.log` 与 `docs/gazebo_verify_20260727.log`。
- 官方 `shapes.sdf` 已通过脚本完成限时、无 GUI 的服务端启动检查，用户另行确认其图形世界正常打开。
- 已创建 `resilient_nav_simulation` 包，包含 `phase2_world.sdf`、`bridge.yaml`、`phase2_world.launch.py` 和静态资源测试。
- 用户确认自定义世界正常打开，并可见 `ground_plane`、`box_obstacle` 和 `cylinder_checkpoint`。
- 未启动桥接时，Gazebo Transport 可观察 `/clock`，ROS 2 不可观察 `/clock`；启动项目 Launch 后，ROS 2 可以观察 `/clock`。
- Launch 将 `system_heartbeat` 的 `use_sim_time` 设置为 `true`。暂停 Gazebo 时 ROS 2 `/clock` 和心跳停止，恢复 Gazebo 后二者继续，验证了仿真时钟驱动关系。
- 用户将 `box_obstacle` 的 pose 从 `2 0 0.5 0 0 0` 修改为 `3 1 0.5 0 0 0`，重新启动后确认 Gazebo 中的新坐标生效。

### 学习要点

- Gazebo Transport 和 ROS 2 Topic 是两套独立通信机制。即使两侧都使用 `/clock` 这一名称，也必须由 `ros_gz_bridge` 显式转换消息后才能互通。
- ROS 2 负责节点、参数和 ROS 图，Gazebo 负责世界与仿真时间，`ros_gz` 负责启动集成和选定数据的桥接；明确边界有助于定位“Gazebo 有数据但 ROS 2 看不到”的问题。
- SDF 定义 Gazebo 世界，`bridge.yaml` 定义跨中间件消息映射，Python Launch 文件负责把 Gazebo、桥和 ROS 2 节点组织成一次可复现启动。
- `src/` 是源码，`build/` 是中间产物，`install/` 是运行时可发现前缀，`log/` 保存构建和测试日志。`colcon build --symlink-install` 建立便于迭代的安装布局，而 `source install/setup.bash` 只是把该布局加载到当前 shell。
- 通过 ROS vendor 包安装的 `gz` 位于 `/opt/ros/jazzy/opt/gz_tools_vendor/bin`。安装前已经加载过 ROS 环境的 shell 不会自动获得后来新增的路径，需要重新加载 `/opt/ros/jazzy/setup.bash` 或打开新的已配置 shell；因此当时的 `gz` 命令不可见属于环境未刷新，而非安装失败。

### 当前边界

- 阶段 2 只完成静态世界、`/clock` 桥接、Launch 编排和已有心跳节点的仿真时间联动。
- 没有开始机器人、URDF、传感器、运动控制、Nav2、SLAM、故障注入、健康评估、自适应融合或容错导航。
- 本次收尾只更新文档并执行非图形验证，不安装软件、不启动 Gazebo 图形界面，也不修改 `resilient_nav_monitor` 或 `resilient_nav_simulation` 源码。

## 2026-07-24 — 实现 system_heartbeat 节点

### 当前事实

- `resilient_nav_monitor` 新增 `system_heartbeat` console 入口和同名节点。
- 节点每 1 秒在 `/system_heartbeat` 发布一次 `std_msgs/msg/String`，消息为 `alive count=N`，计数从 1 开始递增。
- 聚焦测试覆盖话题、周期和递增消息格式；完整测试结果为 4 项通过、1 项按生成器默认配置跳过、0 项失败。
- `colcon build --symlink-install` 成功完成 1 个包。
- 自动运行验证确认节点、话题类型和消息均符合约定，`ros2 topic hz` 测得频率为 `1.000 Hz`。
- 节点由 timeout 和清理逻辑自动停止，验证后没有后台进程残留。

### 当前边界

- `system_heartbeat` 只提供基础存活信号，不代表健康评估、故障检测或容错决策已经实现。
- 本次没有创建其他包或节点，没有安装软件，也没有安装 Gazebo、Nav2 或 SLAM。

## 2026-07-24 — 创建 resilient_nav_monitor 包骨架

### 当前事实

- 在 `ros2_ws/src/` 中创建了 `resilient_nav_monitor`，构建类型为 `ament_python`，许可证为 Apache-2.0。
- 包清单声明 `rclpy` 和 `std_msgs` 依赖，Python 打包配置、ament 资源索引和标准测试目录已建立。
- `colcon build --symlink-install` 成功完成 1 个包。
- 包标准测试结果为 2 项通过、1 项按生成器默认配置跳过、0 项失败。
- 加载工作空间环境后，`ros2 pkg prefix resilient_nav_monitor` 成功返回工作空间安装前缀。

### 当前边界

- 当前只创建规范包骨架，没有实现 `system_heartbeat`、其他节点或任何机器人功能。
- 本次没有安装软件，没有安装 Gazebo、Nav2 或 SLAM。
- 后续节点接口、行为和测试应在单独任务中明确设计和实现。

## 2026-07-24 — 阶段 1 ROS 2 工作空间基线

### 当前事实

- 已创建 `ros2_ws/src/`，并以 `.gitkeep` 保留空源码目录。
- 加载 ROS 2 Jazzy 环境后，在 `ros2_ws` 中执行空 `colcon build` 成功，结果为 `0 packages finished`。
- `ros2_ws/build/`、`ros2_ws/install/` 和 `ros2_ws/log/` 均由现有 `.gitignore` 规则排除，没有进入 Git。
- 当前尚未创建任何 ROS 2 包、项目节点、消息、启动文件或参数。

### 当前边界

- 本次没有安装软件，没有创建 ROS 2 包，也没有启动机器人功能开发。
- Gazebo、Nav2 和 SLAM 仍未安装。
- 后续首个功能包及其接口、构建和测试需要在单独任务中明确实施。

## 2026-07-24 — ROS 2 Jazzy 安装与基础通信验证

### 当前事实

- ROS 2 Jazzy 已安装，环境变量为 `ROS_DISTRO=jazzy`、`ROS_VERSION=2`。
- `ros2`、`colcon` 和 `rosdep` 命令均可用。
- 官方 `demo_nodes_cpp talker` 和 `demo_nodes_py listener` 通信验证通过。
- 运行期间 `/talker` 和 `/listener` 节点均存在，`/chatter` 话题类型为 `std_msgs/msg/String`。
- Gazebo、Nav2 和 SLAM 仍未安装。
- ROS 2 工作空间和项目 ROS 2 包仍未创建。

### 问题与修复

- 验证脚本启用了 `set -euo pipefail`。首次直接加载官方 `/opt/ros/jazzy/setup.bash` 时，官方脚本引用未定义的 `AMENT_TRACE_SETUP_FILES`，触发 nounset 错误。
- 修复仅作用于环境加载过程：加载前执行 `set +u`，加载后立即恢复 `set -u`；脚本仍保留 `set -e`、`set -u` 和 `pipefail`。
- 修复后验证脚本、详细 `ros2 doctor --report` 和官方 talker/listener 通信实验均完成。通信实验使用超时和清理机制，未留下后台演示节点。

### 当前边界

- 本次没有重新安装 ROS 2，也没有执行 apt、dpkg 或 sudo 安装。
- 本次没有安装 Gazebo、Nav2、SLAM 或其他 ROS 发行版。
- 本次没有创建工作空间、ROS 2 包或项目节点。
- 后续机器人仿真、导航和容错功能仍为未实施计划。

## 2026-07-24 — 第 0 阶段文档收尾

### 当前事实

- 项目名称确定为 **ResilientNavLab**。
- 项目目标是构建基于 ROS 2 的移动机器人多传感器故障注入、健康评估、自适应融合和容错导航平台。
- 项目仍处于初始化阶段。
- 当前环境为 Ubuntu 24.04.4 LTS，CPU 架构为 x86_64/amd64。
- Git 2.43.0、Python 3.12.3、Node.js v24.18.0、npm 11.16.0 和 Codex CLI 0.145.0 可用。
- 当前只有 `python3` 命令，没有 `python` 命令。
- ROS 2 与 Gazebo 均未安装或不可用。
- 尚未创建 ROS 2 工作空间和功能包，机器人功能开发尚未开始。

### 本阶段决策

- 先建立清晰的项目范围、环境基线和协作规则，再进入软件安装与代码开发。
- 不在第 0 阶段提前选择 ROS 2 发行版或 Gazebo 版本；选择必须基于 Ubuntu 24.04 的官方兼容关系另行确认。
- 后续设计采用模块化边界：故障注入、健康评估、自适应融合和容错导航应能分别开发、测试和对比。
- 实验设计必须重视可复现性，至少记录场景、参数、随机种子、故障真值和评价指标。

### 本次完成

- 补充项目入口和当前状态说明。
- 定义项目目标、计划内能力、当前边界与建议阶段路线。
- 记录系统、工具、ROS 2 和 Gazebo 的实际状态。
- 建立仓库协作规则和面向未来构建产物的忽略规则。

### 学习要点

- “命令不存在”与“环境未加载”可能表现相同，因此 ROS 2 状态同时通过 `ros2`、`ROS_DISTRO` 和 `/opt/ros` 交叉核验。
- 在创建工作空间前记录系统与工具版本，可以为后续依赖选择、问题复现和迁移提供基准。
- 项目文档必须区分“目标”“计划”和“已实现”，避免初始化仓库给出错误的功能完成预期。

### 后续待办（未执行）

- 调研并选择适配 Ubuntu 24.04 的 ROS 2 发行版。
- 确认 ROS 2 与 Gazebo 的官方兼容组合。
- 经明确授权后安装依赖并创建 ROS 2 工作空间。
- 设计首个最小可运行的仿真和数据链路里程碑。

### 操作声明

本次仅执行只读环境检查并修改项目文档；未安装任何软件，未执行 `sudo apt install`，未创建 ROS 2 工作空间或包，也未修改系统配置。
