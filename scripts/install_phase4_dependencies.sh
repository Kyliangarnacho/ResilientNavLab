#!/usr/bin/env bash

set -euo pipefail

readonly ROS_SETUP="/opt/ros/jazzy/setup.bash"
readonly ROS_PACKAGE="robot_localization"
readonly APT_PACKAGE="ros-jazzy-robot-localization"

fail() {
  printf '错误：%s\n' "$1" >&2
  exit 1
}

if (( $# != 0 )); then
  fail "本脚本不接受参数；它只处理 ${APT_PACKAGE}。"
fi

[[ -r "${ROS_SETUP}" ]] ||
  fail "无法读取 ${ROS_SETUP}，不能加载 ROS 2 Jazzy 环境。"

# ROS 2 的环境脚本可能读取尚未定义的变量，因此只在 source 期间关闭 nounset。
set +u
# shellcheck disable=SC1091
source "${ROS_SETUP}"
set -u

command -v ros2 >/dev/null 2>&1 ||
  fail "加载 ${ROS_SETUP} 后仍未找到 ros2 命令。"

if ros2 pkg prefix "${ROS_PACKAGE}" >/dev/null 2>&1; then
  installed_prefix="$(ros2 pkg prefix "${ROS_PACKAGE}")"
  printf '%s 已安装，ROS 2 前缀为：%s\n' \
    "${APT_PACKAGE}" "${installed_prefix}"
  printf '无需执行任何安装操作。\n'
  exit 0
fi

printf '未发现 ROS 2 包 %s。\n' "${ROS_PACKAGE}"
printf '准备通过当前系统已配置的软件源安装且仅安装：\n'
printf '  %s\n' "${APT_PACKAGE}"
printf '脚本不会添加软件源，也不会执行 upgrade 或 autoremove。\n'

[[ -t 0 ]] ||
  fail "需要交互式终端进行人工确认；未执行安装。"

read -r -p "确认执行 sudo apt install ${APT_PACKAGE} 吗？[y/N] " answer
case "${answer}" in
  y | Y | yes | YES | Yes)
    ;;
  *)
    printf '用户取消；未执行安装。\n'
    exit 0
    ;;
esac

printf '即将执行：sudo apt install %s\n' "${APT_PACKAGE}"
sudo apt install "${APT_PACKAGE}"

printf '%s 安装命令已完成。\n' "${APT_PACKAGE}"
