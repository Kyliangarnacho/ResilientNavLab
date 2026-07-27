#!/usr/bin/env bash

set -euo pipefail

printf '将通过现有 ROS 2 软件源安装 Gazebo Harmonic 的 ROS 2 Jazzy 集成包。\n'
printf '不会添加第三方软件源，也不会安装 Gazebo Classic 或其他 ROS 2 发行版。\n'

sudo apt update
sudo apt install -y ros-jazzy-ros-gz

printf 'ros-jazzy-ros-gz 安装命令已完成。\n'
