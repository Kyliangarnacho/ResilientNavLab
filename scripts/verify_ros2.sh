#!/usr/bin/env bash

set -euo pipefail

stage() {
  printf '\n==> [%s] %s\n' "$1" "$2"
}

pass() {
  printf '通过：%s\n' "$1"
}

fail() {
  printf '失败：%s\n' "$1" >&2
  exit 1
}

require_command() {
  local command_name="$1"

  command -v "${command_name}" >/dev/null 2>&1 ||
    fail "未找到命令：${command_name}"
  pass "命令可用：${command_name}"
}

stage "1/5" "检查 ROS 2 Jazzy 环境文件"
readonly JAZZY_SETUP="/opt/ros/jazzy/setup.bash"
[[ -f "${JAZZY_SETUP}" ]] ||
  fail "${JAZZY_SETUP} 不存在。"
pass "${JAZZY_SETUP} 存在。"

stage "2/5" "加载 ROS 2 Jazzy 环境并检查环境变量"
set +u
# shellcheck disable=SC1091
source "${JAZZY_SETUP}"
set -u

[[ "${ROS_DISTRO:-}" == "jazzy" ]] ||
  fail "ROS_DISTRO 应为 jazzy，实际为 ${ROS_DISTRO:-未设置}。"
pass "ROS_DISTRO=jazzy"

[[ "${ROS_VERSION:-}" == "2" ]] ||
  fail "ROS_VERSION 应为 2，实际为 ${ROS_VERSION:-未设置}。"
pass "ROS_VERSION=2"

stage "3/5" "检查 ROS 2 CLI 和演示节点"
require_command ros2

if ros2 pkg executables demo_nodes_cpp 2>/dev/null |
  awk '$1 == "demo_nodes_cpp" && $2 == "talker" { found = 1 } END { exit(found ? 0 : 1) }'; then
  pass "demo_nodes_cpp 中存在 talker。"
else
  fail "demo_nodes_cpp 中未找到 talker。"
fi

if ros2 pkg executables demo_nodes_py 2>/dev/null |
  awk '$1 == "demo_nodes_py" && $2 == "listener" { found = 1 } END { exit(found ? 0 : 1) }'; then
  pass "demo_nodes_py 中存在 listener。"
else
  fail "demo_nodes_py 中未找到 listener。"
fi

stage "4/5" "检查开发工具和 ros2 doctor 子命令"
require_command colcon
require_command rosdep

if ros2 doctor --help >/dev/null 2>&1; then
  pass "ros2 doctor 子命令可用。"
else
  fail "ros2 doctor 子命令不可用。"
fi

stage "5/5" "执行基础 ros2 doctor 检查"
printf '提示：ros2 doctor 输出 warning 不一定代表验证失败，请结合具体内容判断。\n'
if ros2 doctor; then
  pass "ros2 doctor 基础检查已完成。"
else
  fail "ros2 doctor 返回非零状态，请检查上方诊断信息。"
fi

printf '\nROS 2 Jazzy 验证完成；未启动 talker 或 listener。\n'
