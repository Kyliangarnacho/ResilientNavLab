# resilient_nav_agent

`resilient_nav_agent` 是 ResilientNavLab 的离线 Robot Diagnostic Agent Domain 包。RA-1A 提供纯 Python 数据合同、Ground Truth Sanitizer、Incident/Evidence builder、三个只读 Robot Tools、strict Offline Runtime，以及 deterministic Benchmark/Batch。

## 边界

- Detection 仍由既有确定性 Health Monitor 负责。
- 本包不订阅 ROS topic，不 import ROS message 类型，不发布控制命令。
- `FaultStatus` 和 fault injection truth 不得进入本包的 Agent Input。
- `agent_core` 必须来自独立 `Kyliangarnacho/agent-core` 安装，不得复制到本仓库。
- Tool 只读取 sanitized `OfflineDiagnosisContext`，不接 ROS、文件、Shell 或机器人状态。
- `ra1a_real_benchmark` 是明确标记为 `REAL MODEL BENCHMARK` 的可选离线入口；
  没有有效 `AGENT_CORE_MODEL_*` 配置时它会安全报告 blocked，不发 API 请求。
- 当前没有真实 LLM 结果、Planner、Recovery 或 Live ROS Adapter。

## 开发依赖

系统 Python 受 PEP 668 管理时，可使用仓库根目录中被 Git 忽略的虚拟环境：

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -e ../agent-core
```

上例中的 `../agent-core` 只是 sibling checkout 示例；源码和 package metadata 不硬编码开发机绝对路径。`setup.py` 声明 `agent-core>=0.1.0` 和 `pydantic>=2.8`。

## 测试

```bash
cd ros2_ws/src/resilient_nav_agent
../../../.venv/bin/python -m pytest -q

cd ../../..
cd ros2_ws
../.venv/bin/python -m colcon build --symlink-install \
  --packages-select resilient_nav_agent
../.venv/bin/python -m colcon test --packages-select resilient_nav_agent
../.venv/bin/python -m colcon test-result \
  --test-result-base build/resilient_nav_agent --verbose

source install/setup.bash
ros2 run resilient_nav_agent ra1a_fake_benchmark
ros2 run resilient_nav_agent ra1a_real_benchmark       # CASE-001/002/006/008
ros2 run resilient_nav_agent ra1a_real_benchmark --all # CASE-001..008
```

真实入口使用 agent-core 既有 `AGENT_CORE_MODEL_API_KEY`、
`AGENT_CORE_MODEL_NAME`（可选 `AGENT_CORE_MODEL_BASE_URL`、
`AGENT_CORE_MODEL_TIMEOUT_SECONDS`）配置；不要把 secret 写入文件、fixture 或报告。

完整架构、验证证据和限制见 [RA-1A final audit](../../../docs/RA1A_FINAL_AUDIT.md)。
