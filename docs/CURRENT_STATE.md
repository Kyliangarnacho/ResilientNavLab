# 当前状态

阶段 0 至 7.2 已完成。**RA-1A OFFLINE CASE CHECKPOINT COMPLETE**：Step 1 的 Robot Schema、Ground Truth Sanitizer、Incident/Evidence builder 和独立 agent-core Fake integration 全部保留；本 checkpoint 进一步完成 OfflineAgentInput/BenchmarkTruth 双通道、OfflineCaseBuilder、8 个 reference case，以及第一版正式 Robot Diagnostic Prompt/Analyzer/route。Agent package 当前 70 tests、0 errors、0 failures、0 skipped；最近完整 workspace 基线仍为 Step 1 的 461 tests、0 errors、0 failures、1 skipped，本轮按 STOP POINT 未重跑 workspace regression。

- 独立 `Kyliangarnacho/agent-core` 以仓库外 sibling editable install 接入，实际版本 `0.1.0`；Pydantic 版本为 `2.13.4`。ResilientNavLab 内没有复制 `agent_core/` 源码。
- `AgentInputSanitizer` 接受 plain Mapping，递归拒绝 FaultStatus/场景/benchmark truth；现有 `/faulted/*` source topic 只用于 component normalization，最终 Agent-facing object 不保留 transport topic。
- IMU、wheel、scan、camera 的真实 `SensorHealth.msg` 字段形状已由 fixture 覆盖，并统一转换为 `HealthObservation`；未修改原消息或监测算法。
- `RobotDomainExtension` 已经通过外部 `AgentRuntime` 和 Fake completion 得到合法 `AgentResult`；没有真实模型 API、Tool 或网络推理。
- `OfflineRobotCase` 只在 builder 层组合 `OfflineAgentInput` 与 `BenchmarkTruth`；`agent_view()`、Domain context、`OfflineDiagnosisContext` 和 Agent Trace 的 Ground Truth leakage 测试均为 0。
- `ra1a-reference-v1` 提供 IMU bias、wheel freeze、Lidar sector blindness、camera stale/freeze/underexposed/blurred 和 healthy control；全部明确为 reference fixture，不是 recorded run。
- Robot Diagnostic Prompt V1 要求引用 evidence ID、区分 detector hint 与 diagnosis 并允许 insufficient evidence；Analyzer 只做 route，当前 route 为 `diagnose`、`needs_more_evidence`、`blocked`。

- C920 已经 WSL/USBIP + `usb_cam` 接入 ROS 2：`/dev/video0` 以 MJPG、1280×720、15 FPS request、`mmap` 发布 `/camera/c920/image_raw`。
- 旧 K/D 已复用验证；正式 `CameraInfo`、`image_proc` 去畸变至 `/camera/c920/image_rect` 与 `rectification_probe` 已通过。
- `/camera/c920/image_raw`、`/camera/c920/camera_info`、`/camera/c920/image_rect` 已完成 rosbag 录制和无相机回放验证。
- C920 曝光、gain、白平衡、对焦和图像控制已通过 `v4l2-ctl` 只读记录，没有修改参数。
- `resilient_nav_health_assessment` 已增加纯 NumPy 相机特征、baseline 采集/report 和发布 `/health/camera` 的 `camera_health_monitor`；正式故障为 stale、stamp 持续前进时的 exact-fingerprint freeze、保守 underexposed/overexposed、需要近期纹理参考的 blurred，以及需要近期有信息 reference 的 low-information v1。`camera_freeze_source` 只向独立 `/test` 话题重复首张有效真实图像，`camera_health_watch` 只显示既有健康消息。
- freeze runtime 测试已自动验证持续 frozen Image、像素一致、stamp 前进、最终 `FAULT/freeze` 且无 stale 抢占；`health_evaluator` 的 camera 订阅默认关闭，显式启用后可区分异常检出与类别精确匹配。
- `manual_fault_event` 只向既有 FaultStatus topic 发布 SCHEDULED/ACTIVE/ENDED 人工真值窗，不修改传感器数据。
- `phase7_2_camera_health.launch.py` 复用原 C920 Launch，默认启动 C920 + monitor，并可选启用 evaluator/watch；manual event 不自动启动。evaluator 已记录 detection、classification、TP/FP/FN/TN 和事件结束后的首次 HEALTHY recovery。
- calibrate 的 truth 模式只给 CSV 附加 event/model/state/severity，并保存 before/fault/after 只读 controls；离线 report 使用可配置 margin 分段并输出描述统计及非阈值候选区间。

## Phase 7.2 验证边界与已知限制

- WSL2 USB/IP 下的 C920 偶发帧异常和 FPS 波动仍存在。
- 当前 C920 画面整体偏暗，运行期曾观察到 `brightness=50`；为保持 baseline 一致性，本阶段没有中途调整控制值。
- exposure v1 仅覆盖严重、明显的全局欠曝和过曝，不宣称覆盖所有轻度曝光异常。
- `low_information` 在人工 occlusion 实验中受输入操作不稳定影响，并与 `blurred`、`underexposed` 存在分类竞争。
- 部分最终实验的 FaultStatus 与实际物理操作没有严格硬同步，因此 detection delay、FP 和 F1 并非所有场景的精确物理性能指标。
- 10 分钟 mixed run 含未标注白纸/低信息刺激；其 alarm fraction 不是正式 false-positive benchmark。

本阶段的 evaluation config 已冻结用于可复现实验，不等同于通用生产标定。Robot Agent 当前没有正式 Robot Tools、OfflineDiagnosisRunner、strict final DiagnosisResult service、Benchmark Scorer/Report、真实模型、Live ROS Adapter、自动 Incident listener、rosbag parser、RAG、Planner、Recovery 或控制权限。修改图像的数据故障模型、真实硬件定位、Nav2、SLAM、自适应融合和容错导航仍未实现。
