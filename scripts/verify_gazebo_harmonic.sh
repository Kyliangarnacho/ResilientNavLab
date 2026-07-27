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

stage "1/5" "加载 ROS 2 Jazzy 环境"
readonly JAZZY_SETUP="/opt/ros/jazzy/setup.bash"
[[ -f "${JAZZY_SETUP}" ]] ||
  fail "${JAZZY_SETUP} 不存在。"

set +u
# shellcheck disable=SC1091
source "${JAZZY_SETUP}"
set -u

[[ "${ROS_DISTRO:-}" == "jazzy" ]] ||
  fail "ROS_DISTRO 应为 jazzy，实际为 ${ROS_DISTRO:-未设置}。"
pass "ROS 2 Jazzy 环境已加载。"

stage "2/5" "检查 Gazebo 命令"
command -v gz >/dev/null 2>&1 ||
  fail "未找到命令：gz"
pass "命令可用：gz"
printf 'Gazebo Sim 版本信息：\n'
gz sim --versions
pass "gz sim --versions 执行成功。"

stage "3/5" "检查 Gazebo Sim 帮助"
gz sim --help >/dev/null
pass "gz sim --help 执行成功。"

stage "4/5" "检查 ros_gz 软件包"
require_command ros2
for package_name in \
  ros_gz \
  ros_gz_sim \
  ros_gz_bridge \
  ros_gz_interfaces
do
  if ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
    pass "ROS 2 软件包可用：${package_name}"
  else
    fail "ROS 2 软件包不可用：${package_name}"
  fi
done

stage "5/5" "检查 shapes.sdf 可被找到并启动"
require_command timeout
shapes_log="$(mktemp "${TMPDIR:-/tmp}/resilient-nav-gz-shapes.XXXXXX.log")"
readonly shapes_log
trap 'rm -f "${shapes_log}"' EXIT

set +e
timeout --signal=INT --kill-after=5s 15s \
  gz sim -s -r shapes.sdf >"${shapes_log}" 2>&1
shapes_status=$?
set -e

case "${shapes_status}" in
  0)
    pass "shapes.sdf 已被 Gazebo 找到并完成启动。"
    ;;
  124)
    pass "shapes.sdf 已被 Gazebo 找到并启动；达到 15 秒验证时限后已停止。"
    ;;
  *)
    printf 'Gazebo 启动输出：\n' >&2
    sed -n '1,160p' "${shapes_log}" >&2
    fail "shapes.sdf 启动失败，gz sim 返回状态 ${shapes_status}。"
    ;;
esac

printf '\nGazebo Harmonic 与 ROS 2 Jazzy 集成验证完成。\n'
