#!/usr/bin/env bash

set -euo pipefail

readonly ROS_APT_SOURCE_DEB="/tmp/ros2-apt-source.deb"

stage() {
  printf '\n==> [%s] %s\n' "$1" "$2"
}

fail() {
  printf '错误：%s\n' "$1" >&2
  exit 1
}

stage "1/6" "检查操作系统、架构、locale、Ubuntu 软件源和 curl"

[[ -r /etc/os-release ]] || fail "无法读取 /etc/os-release，不能确认操作系统版本。"

# shellcheck disable=SC1091
source /etc/os-release

[[ "${ID:-}" == "ubuntu" ]] ||
  fail "仅支持 Ubuntu；当前系统 ID 为 ${ID:-未知}。"
[[ "${VERSION_ID:-}" == "24.04" ]] ||
  fail "仅支持 Ubuntu 24.04；当前版本为 ${VERSION_ID:-未知}。"

ubuntu_codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
[[ "${ubuntu_codename}" == "noble" ]] ||
  fail "仅支持 Ubuntu noble；当前代号为 ${ubuntu_codename:-未知}。"

kernel_arch="$(uname -m)"
[[ "${kernel_arch}" == "x86_64" || "${kernel_arch}" == "amd64" ]] ||
  fail "仅支持 x86_64/amd64；当前内核架构为 ${kernel_arch}。"

debian_arch="$(dpkg --print-architecture 2>/dev/null || true)"
[[ "${debian_arch}" == "amd64" ]] ||
  fail "仅支持 Debian amd64 架构；当前架构为 ${debian_arch:-未知}。"

locale_charmap="$(locale charmap 2>/dev/null || true)"
case "${locale_charmap^^}" in
  UTF-8 | UTF8) ;;
  *)
    fail "当前 locale 不支持 UTF-8（字符集：${locale_charmap:-未知}）；请先手动配置 UTF-8 locale。"
    ;;
esac

readonly UBUNTU_SOURCES="/etc/apt/sources.list.d/ubuntu.sources"
[[ -r "${UBUNTU_SOURCES}" ]] ||
  fail "无法读取 ${UBUNTU_SOURCES}，不能确认 noble-updates。"

awk '
  $1 == "Suites:" {
    for (i = 2; i <= NF; i++) {
      if ($i == "noble-updates") {
        found = 1
      }
    }
  }
  END { exit(found ? 0 : 1) }
' "${UBUNTU_SOURCES}" ||
  fail "${UBUNTU_SOURCES} 中未找到 noble-updates；脚本不会自动重写软件源。"

command -v curl >/dev/null 2>&1 ||
  fail "未找到 curl；请先手动提供 curl 后再运行。"

printf '预检通过：Ubuntu 24.04 noble、amd64、UTF-8、noble-updates 和 curl 均符合要求。\n'

stage "2/6" "更新 apt 索引并安装 ROS 2 仓库配置所需工具"
sudo apt update
sudo apt install -y locales software-properties-common curl

stage "3/6" "启用 Ubuntu universe 仓库"
sudo add-apt-repository -y universe

stage "4/6" "获取并安装 noble 对应的最新 ros2-apt-source"
ROS_APT_SOURCE_VERSION="$(
  curl --fail --silent --show-error --location \
    https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
    awk -F '"' '
      $2 == "tag_name" {
        version = $4
      }
      END {
        if (version != "") {
          print version
        }
      }
    '
)"
[[ -n "${ROS_APT_SOURCE_VERSION}" ]] ||
  fail "无法从 GitHub latest release 解析 ROS_APT_SOURCE_VERSION。"
[[ "${ROS_APT_SOURCE_VERSION}" =~ ^[0-9]+([.][0-9]+)*$ ]] ||
  fail "ROS_APT_SOURCE_VERSION 格式异常：${ROS_APT_SOURCE_VERSION}。"

readonly ROS_APT_SOURCE_VERSION
readonly ROS_APT_SOURCE_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${ubuntu_codename}_all.deb"

rm -f "${ROS_APT_SOURCE_DEB}"
trap 'rm -f "${ROS_APT_SOURCE_DEB}"' EXIT

printf '下载 ros2-apt-source %s（Ubuntu %s）到 %s。\n' \
  "${ROS_APT_SOURCE_VERSION}" "${ubuntu_codename}" "${ROS_APT_SOURCE_DEB}"
curl --fail --show-error --location \
  --output "${ROS_APT_SOURCE_DEB}" \
  "${ROS_APT_SOURCE_URL}"
sudo dpkg -i "${ROS_APT_SOURCE_DEB}"

stage "5/6" "更新并升级系统已有软件包"
sudo apt update
sudo apt upgrade -y

stage "6/6" "仅安装 ROS 2 Jazzy Desktop 和 ROS 开发工具"
sudo apt install -y ros-jazzy-desktop ros-dev-tools

printf '\nROS 2 Jazzy 安装命令已完成。\n'
printf '脚本未修改 ~/.bashrc；请在需要使用 ROS 2 的终端中手动 source /opt/ros/jazzy/setup.bash。\n'
