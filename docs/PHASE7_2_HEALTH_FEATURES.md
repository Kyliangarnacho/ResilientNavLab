# 阶段 7.2 相机健康特征说明

本页说明 `camera_health_features.py`、`camera_health_calibrate` 和 `camera_health_baseline_report` 记录与汇总的描述性特征。它们用于观察 C920 baseline 数据的分布，不直接产生 HEALTHY/FAULT，不是最终故障阈值。

## 视觉特征

| 特征 | 主要测量内容 | 不能单独证明什么 |
| --- | --- | --- |
| `mean_gray` | 整帧平均灰度，反映画面整体亮度 | 不能单独区分曝光异常与本来就暗或亮的场景 |
| `gray_std` | 灰度离散程度，反映画面对比和纹理起伏 | 低值不等于失焦；均匀墙面本来就可能低纹理 |
| `p05` / `p95` | 灰度分布低端和高端的稳健位置 | 不能单独判断黑位、白位或动态范围是否异常 |
| `dark_ratio` | 位于特征定义暗区间的像素比例 | 高值不必然是欠曝，也可能是夜间或黑色物体 |
| `bright_ratio` | 位于特征定义亮区间的像素比例 | 高值不必然是过曝，也可能是白墙、窗口或灯具 |
| `laplacian_variance` | 二阶空间变化，通常随细节和清晰边缘增多而升高 | 低值不能单独证明失焦；场景低纹理、运动模糊和缩放都会影响它 |
| `edge_density` | 具有明显局部灰度梯度的像素比例 | 不能单独证明画面清晰；噪声、重复纹理和场景内容都会改变它 |
| `entropy` | 灰度直方图的信息多样性 | 高熵不一定健康，随机噪声也可能提高熵；低熵可能只是单色场景 |
| `frame_diff_mean` | 当前帧与前一帧的平均绝对灰度差 | 接近零不能单独证明冻结，静止相机和静止场景也会产生重复帧 |
| `frame_fingerprint` | 规范化灰度帧的确定性指纹，用于精确识别重复内容 | 指纹不同不表示故障，也不能量化变化的视觉重要性 |

## 时序统计

- `interarrival_sec` 使用进程单调时钟计算相邻接收间隔，不受系统墙钟校时回拨影响。
- `observed_fps` 使用首末样本间隔和其中的帧间隔数量计算；少于两帧时为 `null`。
- `p50`、`p95`、`p99` 和 `max_gap` 描述接收抖动与长间隔，但不能单独区分 USB/IP、解码、DDS、主机负载或相机侧原因。
- ROS `header stamp` 和墙钟接收时间逐帧保留，供后续离线检查；当前工具不据此分类故障。

## Baseline 输出边界

`camera_health_calibrate` 默认订阅 `/camera/c920/image_raw`，通过 `cv_bridge` 转换后严格调用现有特征函数，并写出逐帧 CSV、描述性 JSON 和只读 V4L2 控制快照。`output_directory` 表示 baseline 根目录，每次采集都写入独立的 `<output_directory>/<session_id>/`，不会覆盖已有 session。`scenario_label` 同时写入每条 CSV 样本和 session JSON；`session_id` 为空时自动生成 UTC 微秒时间标识，显式 ID 已存在时采集拒绝启动。

统计结果依赖场景、光照、相机姿态、USB/IP 和主机负载；一次 baseline 不能直接推广为最终视觉退化阈值。下文 monitor 的视觉规则均保持保守的 development v1 状态。

默认运行方式：

```bash
ros2 run resilient_nav_health_assessment camera_health_calibrate
```

指定场景、session、运行时长和 baseline 根目录：

```bash
ros2 run resilient_nav_health_assessment camera_health_calibrate --ros-args \
  -p scenario_label:=stationary_indoor_daylight \
  -p session_id:=c920_20260810_run01 \
  -p duration_sec:=60.0 \
  -p output_directory:=/tmp/phase7_2_baselines/
```

该命令输出 `/tmp/phase7_2_baselines/c920_20260810_run01/feature_samples.csv`、`baseline_summary.json` 和 `controls_snapshot.txt`。到达 `duration_sec` 或收到 Ctrl+C 时均走同一幂等落盘路径；无数据时仍生成 CSV 表头和带 `null` 描述统计的 JSON。

多次采集可复用同一个根目录，只需使用不同 `session_id` 或保留空值让工具自动生成：

```bash
ros2 run resilient_nav_health_assessment camera_health_calibrate --ros-args \
  -p scenario_label:=stationary_indoor_daylight \
  -p output_directory:=/tmp/phase7_2_baselines/
```

## 多 session 描述统计

`camera_health_baseline_report` 是不启动 ROS 节点的纯分析工具。它扫描 baseline 根目录中的直接 session 子目录，读取 `baseline_summary.json` 和 `feature_samples.csv`，输出全局统计、按 `scenario_label` 分组统计、session 清单和无效 session 的跳过原因：

```bash
ros2 run resilient_nav_health_assessment camera_health_baseline_report \
  /tmp/phase7_2_baselines/
```

默认写出 `/tmp/phase7_2_baselines/baseline_report.json`，并在终端打印简洁对照表。`observed_fps` 和 `max_gap_sec` 按独立 session 汇总；`interarrival_sec`、`mean_gray`、`laplacian_variance`、`edge_density`、`entropy` 和 `frame_diff_mean` 按样本汇总。这样不会把不同采集之间的空档误当作帧间隔。工具只给出 count、mean、std、min/max 和分位数，不生成阈值、健康状态、评分或置信度。

## Camera health monitor v1

`camera_health_monitor` 默认订阅 `/camera/c920/image_raw`，复用同一 `camera_health_features`，以 `5 Hz` 向 `/health/camera` 发布既有 `resilient_nav_interfaces/SensorHealth`。它不接入 `FaultStatus` 或 evaluator。

```text
/camera/c920/image_raw
  -> cv_bridge
  -> camera_health_features
  -> rolling timing / stamp / fingerprint state
  -> duration confirmation and recovery
  -> /health/camera (SensorHealth)
```

独立启动：

```bash
ros2 launch resilient_nav_health_assessment camera_health_monitor.launch.py
```

当前最终 `FAULT` 有六类：

- `stale`：超过 `stale_timeout_sec` 持续没有新 Image，并继续满足 confirmation 时间。它依据接收时间，不依据请求的 15 FPS。
- `freeze`：Image 持续到达、header stamp 每帧持续前进，但规范化灰度 fingerprint 完全相同达到 `freeze_duration_sec`，并继续满足 confirmation 时间。停止收图时 stale 优先，因此不会被误写成 freeze。
- `underexposed`：当前帧同时满足 `mean_gray <= 6`、`p95 <= 8` 和 `dark_ratio >= 0.90`，并持续满足 confirmation 时间。三个条件是 AND，不允许由平均亮度或暗像素比例单独触发。
- `overexposed`：当前帧同时满足 `mean_gray >= 170`、`p05 >= 150` 和 `p95 >= 180`，并持续满足 confirmation 时间。`bright_ratio` 不参与正式判定。
- `blurred`：近期先观察到 `Laplacian >= 100` 且 `edge_density >= 0.01` 的纹理参考，随后当前帧同时降到 `Laplacian <= 10`、`edge_density <= 0.001`，并仍满足 `gray_std >= 20`、`entropy >= 5.5`。这表示原本有纹理的场景丢失了细节，不把天然低纹理直接当作失焦。
- `low_information`：最近 `3 s` 内先观察到 `edge_density >= 0.01` 且 `entropy >= 5.2` 的有信息 reference，随后当前帧同时满足 `edge_density <= 0.0005`、`entropy <= 5.8`、`dark_ratio >= 0.20` 和 `gray_std >= 20`。它表示可供后续视觉算法使用的空间/纹理信息持续塌缩，不断言物理原因一定是遮挡。

判定优先级为 stale → underexposed → overexposed → low-information → blurred → freeze。low-information 仅在比 blur 更具体的 edge/entropy/dark/std 联合证据成立时位于 blur 之前；blur 开发段的 `dark_ratio≈0.176` 不满足 low-information，而遮挡开发段的稳定汇总满足。视觉故障开发数据可能形成长时间相同 fingerprint，因此视觉分类仍优先于 freeze；若停止收图，stale 覆盖全部视觉证据。

状态转换使用持续时间而不是固定帧数：候选先进入 `DEGRADED`，持续 `fault_confirmation_sec` 才锁存 `FAULT`；故障证据消失后仍保持 `FAULT`，只有 stamp 和有效图像内容连续稳定达到 `recovery_confirmation_sec` 才恢复 `HEALTHY`。启动样本不足时为 `UNKNOWN`。

### Development 参数依据

5-session baseline 共 3560 帧，observed FPS 为 `8.22--14.43 Hz`，全局 interarrival p95/p99 约 `0.158/0.249 s`，最大正常 gap 约 `0.382 s`。因此开发配置采用：

| 参数 | v1 值 | 依据与边界 |
| --- | ---: | --- |
| `stale_timeout_sec` | `1.0 s` | 约为实测最大 gap 的 2.6 倍，不从请求 15 FPS 推导 |
| `freeze_duration_sec` | `2.0 s` | 只针对连续完全相同 fingerprint；仍需真实冻结实验复核 |
| `fault_confirmation_sec` | `0.6 s` | 防止单次 timer 检查直接进入 FAULT |
| `recovery_confirmation_sec` | `1.0 s` | 防止单帧变化立即恢复 |
| `window_duration_sec` | `3.0 s` | 覆盖当前 8--15 Hz 波动下的滚动 FPS 和 gap |
| `dark_mean_gray_candidate` | `6` | 强欠曝时全图平均亮度上界；必须与另外两项同时满足 |
| `dark_p95_candidate` | `8` | 约 95% 像素都必须处于很低亮度；健康黑键盘 baseline 最低约 `10.545` |
| `dark_ratio_candidate` | `0.90` | 至少 90% 像素位于既有暗像素定义内；不能单独触发 |
| `bright_mean_gray_candidate` | `170` | 明显全画面变亮的平均值下界；必须与两个分位数同时满足 |
| `bright_p05_candidate` | `150` | 连最暗端 5% 像素也必须很亮，用于排除正常高光和亮物体 |
| `bright_p95_candidate` | `180` | 高端亮度必须同步升高；不能单独触发 |
| `blur_reference_max_age_sec` | `3.0 s` | 只接受近期纹理参考；进入 blur candidate/FAULT 后锁存至恢复 |
| `blur_reference_laplacian_variance_min` | `100` | 参考画面必须具有明显二阶细节 |
| `blur_reference_edge_density_min` | `0.01` | 参考画面必须同时有足够边缘，低纹理场景不能建立 reference |
| `blur_laplacian_variance_candidate` | `10` | 当前 Laplacian 相对参考至少出现数量级下降 |
| `blur_edge_density_candidate` | `0.001` | 当前边缘密度相对参考至少出现数量级下降 |
| `blur_gray_std_min` / `blur_entropy_min` | `20` / `5.5` | 当前画面仍需保留强度变化和信息量，排除空白/无信息画面 |
| `low_information_reference_max_age_sec` | `3.0 s` | 只接受近期有信息 reference；进入候选或故障后锁存至恢复 |
| `low_information_reference_edge_density_min` / `entropy_min` | `0.01` / `5.2` | reference 必须同时有足够边缘和灰度信息，天然低纹理不能建立 reference |
| `low_information_edge_density_candidate` / `entropy_candidate` | `0.0005` / `5.8` | 当前边缘近乎归零且信息量同步处于遮挡开发区间 |
| `low_information_dark_ratio_min` / `gray_std_min` | `0.20` / `20` | 暗区占比和仍然较高的灰度起伏只作辅助证据；不假设 gray std 越低越异常 |

阈值同时读取 5-session baseline 和 `underexposure_dev_001` 报告确定。开发实验 ACTIVE 汇总为 `mean_gray=4.035`、`p95=5.506`、`dark_ratio=0.946`，三项均越过候选界限；稳定段约 93.5% 样本满足联合规则，最长连续约 `17.2 s`。健康黑键盘/低亮场景虽然出现 `mean_gray=1.660`、`dark_ratio=0.981`，但其最低 `p95=10.545`，不满足联合规则。另一健康 session 的采集起始处有两帧近黑过渡，持续约 `0.063 s`，低于 `0.6 s` confirmation，因此不会进入最终 FAULT。

欠曝规则有意只覆盖持续、接近全画面变黑的情况。过曝阈值同时读取 5-session baseline 和 `overexposure_dev_001`：ACTIVE 汇总为 `mean_gray=183.974`、`p05=175.070`、`p95=188.091`，三项均满足；稳定段约 83.1% 样本满足，最长连续约 `10.5 s`。健康 baseline 的 `mean_gray` 最大约 `163.255`、`p05` 最大约 `64.620`；健康纹理场景即使 `p95` 可达 `247.956`，低端分位仍远低于阈值，因此不会仅因局部高光误报。`bright_ratio` 在故障段仍约为零，只保留为观测指标。

blur 绝对特征无法单独分离健康数据：`stationary_indoor_daylight`、`moving_textured_bright` 和 `stationary_bright_low_texture` 都有持续低 Laplacian/低 edge 帧。因此 v1 必须先建立纹理 reference。blur 开发实验 pre 为 `309.680/0.028`，ACTIVE 为 `6.853/0.000`，且 ACTIVE 的 `gray_std`/`entropy` 约 `24.377/5.776`；稳定段约 95.7% 满足当前退化条件。`stationary_bright_low_texture` 的 edge 最大约 `0.00753`，达不到 `0.01` reference；全部健康 baseline 只有 moving-textured-indoor 出现两个孤立候选帧，连续时长为零，无法通过 confirmation。

low-information 的真正主区分力是 edge 从近期有信息 reference 塌缩到近零；entropy 的降低提供第二项信息量证据，dark ratio 和升高而非降低的 gray std 只作辅助。遮挡开发实验 pre/ACTIVE 分别约为 `edge=0.028/0.000`、`entropy=5.533/4.697`、`dark_ratio=0.190/0.444`、`gray_std=27.915/58.869`；bright ratio 均接近零，没有进入规则。`stationary_bright_low_texture` 健康场景的 edge 最大约 `0.00753`，无法建立 `0.01` reference，因此即使当前画面边缘少也不会直接候选。5-session 健康序列在该 reference 与四项联合条件下没有持续候选。

underexposed、overexposed、blurred 和 low-information 的 confirmation elapsed 均使用连续候选 Image 的首末接收时间差，并至少要求 `min_samples`，不会让单张异常帧靠 timer 等待进入 FAULT。这些分界仍来自有限开发数据，配置保持 development 状态。

`health_score` 是规则的当前数据健康程度：稳定流接近 `1.0`，confirmation pending 为中间值，underexposed/overexposed/blurred/low-information、freeze、stale 确认后分别为 `0.2/0.1/0.0`。`confidence` 是对当前规则判断的确定程度：启动期随有效样本增加，候选期随 confirmation 时间增加，确认故障时为 `1.0`，恢复锁存期间随健康证据累积下降，恢复后重新回到稳定流置信度。二者都不是机器学习结果。

## Freeze 专项测试链与状态观察

`camera_freeze_source` 是隔离的测试输入源，不修改也不覆盖真实相机话题。它等待 `/camera/c920/image_raw` 的第一张有效图像，缓存完整像素和必要 metadata；随后只在 `/test/camera/image_frozen` 按配置频率重复发布相同像素，同时使用当前 ROS 时间持续更新 `header.stamp`。获得有效源图像前不会发布。

```text
/camera/c920/image_raw
  -> camera_freeze_source (cache first valid image)
  -> /test/camera/image_frozen
  -> camera_health_monitor (image_topic override)
  -> /health/camera
  -> camera_health_watch (display only)
```

先启动真实 C920，再分别启动冻结测试源、指向测试话题的 monitor 和观察工具：

```bash
ros2 run resilient_nav_health_assessment camera_freeze_source --ros-args \
  -p source_topic:=/camera/c920/image_raw \
  -p output_topic:=/test/camera/image_frozen \
  -p publish_rate_hz:=10.0

ros2 launch resilient_nav_health_assessment camera_health_monitor.launch.py \
  image_topic:=/test/camera/image_frozen

ros2 run resilient_nav_health_assessment camera_health_watch --ros-args \
  -p health_topic:=/health/camera \
  -p unchanged_print_period_sec:=5.0
```

测试时同一 `/health/camera` 只运行一个 camera monitor，避免多个发布者混淆观察结果。`camera_health_watch` 在 `state` 或 `detected_fault` 变化时立即打印，状态不变时按低频周期打印；它只显示 score、confidence 和既有 metrics，不参与判断。该链不产生 `FaultStatus`，也不接入 evaluator；`camera_health_monitor` 的 stale/freeze 状态机和 development 参数保持不变。

## Freeze 自动 runtime 集成验证

包级 runtime 测试使用测试代码构造有效 `rgb8` Image，经过真实 ROS 2 topic 依次进入 `camera_freeze_source`、独立 frozen topic 和未修改的 `camera_health_monitor`。测试确认 frozen topic 持续有消息、所有像素完全一致、stamp 严格前进且 metadata 保持一致；使用仅限测试的缩短 duration 参数后，最终状态为 `FAULT/freeze`，整个观察序列没有出现 `stale`。测试探针只存在于 pytest 进程，不安装新的永久测试节点。

## 人工故障真值与 camera evaluator

`manual_fault_event` 只发布既有 `/fault_injection/status` `FaultStatus`，不订阅、修改或转发任何相机数据。它启动时发布 `SCHEDULED`，经过 `start_delay_sec` 自动发布 `ACTIVE`，再经过 `duration_sec` 发布 `ENDED`：

```bash
ros2 run resilient_nav_fault_injection manual_fault_event --ros-args \
  -p sensor:=camera \
  -p model:=freeze \
  -p event_id:=manual_camera_freeze_001 \
  -p severity:=1.0 \
  -p start_delay_sec:=3.0 \
  -p duration_sec:=5.0
```

它只标记操作者计划同步执行的真实故障时间窗，不负责施加故障。`sensor`、`model` 和 `event_id` 必须非空，severity 限于 `[0, 1]`，duration 必须为正。

`health_evaluator` 新增默认空值的 `camera_health_topic`。空值继续保持阶段 6 的 IMU/wheel/scan 三传感器行为；显式启用方式为：

```bash
ros2 launch resilient_nav_health_assessment \
  health_evaluator_minimal.launch.py \
  camera_health_topic:=/health/camera \
  evaluator_output_json:=/tmp/phase7_2_camera_evaluation.json
```

评价时应先启动 evaluator 和 camera monitor，再启动 `manual_fault_event`，确保 evaluator 收到 ACTIVE 时间窗而不是只看到最后的 ENDED 状态。

```text
人工故障时间窗 -> manual_fault_event -> /fault_injection/status --+
                                                               |
/camera/c920/image_raw -> camera_health_monitor -> /health/camera +
                                                               |
                                  health_evaluator <------------+
```

camera truth model 到现有 `detected_fault` 的期望映射为：

| truth model | expected detected_fault |
| --- | --- |
| `stream_stop` | `stale` |
| `freeze` | `freeze` |
| `underexposure` | `underexposed` |
| `overexposure` | `overexposed` |
| `blur` | `blurred` |
| `occlusion` | `low_information` |

异常检出与分类匹配是两个独立结果：`SensorHealth` 为 `DEGRADED`/`FAULT` 即记为检测到异常并进入 TP/FP/FN/TN；只有 `detected_fault` 与上表期望字符串相等时才是 exact classification match。因此“检测到异常但类别错误”仍是 detection TP，同时 classification mismatch。该映射不创建新的曝光、模糊或低信息判定规则；若 monitor 没有产生相应异常，evaluator 只会如实记录未检出。事件 recovery 的最小补充见下文联合入口后的 JSON 完整性说明。

## 真实故障实验联合入口

`resilient_nav_camera/phase7_2_camera_health.launch.py` 复用原有 `c920.launch.py`，并组合 camera monitor、可选 evaluator 和可选 watch。默认只启动原 C920 链与 monitor，不自动启动 `manual_fault_event`：

```bash
ros2 launch resilient_nav_camera phase7_2_camera_health.launch.py \
  run_evaluator:=true \
  evaluator_output_json:=/tmp/phase7_2_camera_health_evaluation.json \
  run_watch:=true
```

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `enable_rectification` | `false` | 原样转发给阶段 7.1 `c920.launch.py` |
| `health_config` | `camera_health.yaml` | camera monitor 参数文件 |
| `source_topic` | `/camera/c920/image_raw` | monitor 订阅图像 |
| `run_camera` | `true` | 包含原 C920 Launch；仅回放/runtime 测试时关闭 |
| `run_evaluator` | `false` | 启动 evaluator 并订阅 `/health/camera` |
| `evaluator_output_json` | `/tmp/phase7_2_camera_health_evaluation.json` | evaluator JSON 路径 |
| `run_watch` | `false` | 启动只读终端观察工具 |
| `use_sim_time` | `false` | 传给 health 节点的时间源参数 |

```text
c920.launch.py -> source_topic -> camera_health_monitor -> /health/camera
                                                   |             |
                                                   |             +-> camera_health_watch (optional)
/fault_injection/status ----------------------------+-> health_evaluator (optional)
```

真实人工故障仍在第二终端单独启动 `manual_fault_event`。联合 Launch 不覆盖阶段 7.1 camera info URL、C920 参数、K/D 或 image_proc 行为。

### Evaluator JSON 完整性

现有 evaluator 已能记录 camera detection delay、anomaly detected、classification exact match 和 TP/FP/FN/TN。本轮最小补充 recovery：对已检测且已收到 `ENDED` 的事件，记录其后第一条 `HEALTHY` 的 `recovery_time_sec`，并以 truth `end_time` 计算 `recovery_delay_sec`。若事件已经结束但尚无健康样本，`recovery.observed=false`；未检测或事件未结束时为 `null`。旧字段和阶段 6 混淆矩阵计算保持不变。

## 真实视觉故障特征采集与报告

`camera_health_calibrate` 新增默认关闭的 `record_fault_truth`。关闭时不创建 FaultStatus 订阅，继续执行原健康 baseline 流程；开启时只把 `/fault_injection/status` 的 camera 事件附加到 CSV，不进入特征函数或 monitor：

```bash
ros2 run resilient_nav_health_assessment camera_health_calibrate --ros-args \
  -p record_fault_truth:=true \
  -p scenario_label:=manual_underexposure \
  -p session_id:=underexposure_run_01 \
  -p output_directory:=/tmp/phase7_2_fault_features/ \
  -p duration_sec:=60.0
```

```text
/camera/c920/image_raw -> existing camera features -> feature_samples.csv
                                                      + event_id
/fault_injection/status -> optional truth recorder -> + truth_model/state/severity
                           |
                           +-> before_controls.txt
                           +-> fault_controls.txt  (ACTIVE)
                           +-> after_controls.txt  (ENDED)
```

CSV 始终包含 `event_id`、`truth_model`、`truth_state`、`severity`；普通 baseline 中为空。truth session 的 `baseline_summary.json` 额外保存状态 transition 及其接收时间。before 使用采集启动时的只读快照，并在 SCHEDULED/ACTIVE 首次到达时落盘；ACTIVE 和 ENDED 后分别重新执行一次只读 `v4l2-ctl --list-ctrls-menus`。任何路径都不执行 `--set-ctrl` 或其他写操作。

离线报告命令：

```bash
ros2 run resilient_nav_health_assessment camera_fault_feature_report \
  /tmp/phase7_2_fault_features/underexposure_run_01 \
  --transition-margin-sec 2.0
```

默认输出 session 内的 `camera_fault_feature_report.json` 和紧凑终端表格。分段严格使用 FaultStatus 状态实际接收时刻：

- `healthy_pre`：早于 `ACTIVE - margin`，truth 为空或 SCHEDULED；
- `active_fault`：晚于 `ACTIVE + margin` 且早于 `ENDED - margin`，truth 必须为 ACTIVE；
- `recovered_post`：晚于 `ENDED + margin`，truth 必须为 ENDED；
- 等于边界、落在两侧 margin 内或 truth state 不一致的样本均计入 `excluded_transition_sample_count`。

JSON 按 exposure、blur、low-information 三个 family 保存三个阶段的 count/mean/std/min/p05/p50/p95/max，并给出各阶段观测 p05–p95 candidate interval。exposure 包含 mean gray、p05/p95、dark/bright ratio；blur 包含 Laplacian variance、edge density、entropy；low-information 包含 gray std、edge density、entropy、dark/bright ratio。`thresholds_generated=false` 且 `candidate_intervals_are_thresholds=false`；报告不修改 `camera_health.yaml`。
