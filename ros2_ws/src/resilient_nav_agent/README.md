# resilient_nav_agent

`resilient_nav_agent` 是 ResilientNavLab 的离线 Robot Diagnostic Agent Domain 包。RA-1A Step 1 只提供纯 Python 数据合同、Ground Truth Sanitizer、Incident/Evidence builder 和独立 agent-core 的 Fake Runtime bootstrap。

## 边界

- Detection 仍由既有确定性 Health Monitor 负责。
- 本包不订阅 ROS topic，不 import ROS message 类型，不发布控制命令。
- `FaultStatus` 和 fault injection truth 不得进入本包的 Agent Input。
- `agent_core` 必须来自独立 `Kyliangarnacho/agent-core` 安装，不得复制到本仓库。
- 当前没有真实 LLM、Robot Tool、Planner、Recovery 或 Live ROS Adapter。

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
```

完整架构、验证证据和限制见 [RA-1A Step 1 报告](../../../docs/RA1A_STEP1_ROBOT_AGENT_BOOTSTRAP.md)。
