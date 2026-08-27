#!/usr/bin/env bash
# 在容器裡建置並啟動 bb_joy。掛載進來的原始碼會被 symlink 進 /ros2_ws。
set -e

source /opt/ros/humble/setup.bash

if [ "${RMW_IMPLEMENTATION}" = "rmw_zenoh_cpp" ]; then
  ros2 pkg prefix rmw_zenoh_cpp >/dev/null 2>&1 || {
    apt-get update && apt-get install -y ros-humble-rmw-zenoh-cpp
  }
fi

# joy_linux 不在 ros-base 映像裡，第一次跑會自己裝
ros2 pkg prefix joy_linux >/dev/null 2>&1 || {
  apt-get update && apt-get install -y ros-humble-joy-linux
}

WS=/ros2_ws
mkdir -p "${WS}/src"
ln -sfn /home/BB-joy "${WS}/src/bb_joy"

cd "${WS}"
colcon build --symlink-install --packages-select bb_joy
source "${WS}/install/setup.bash"

echo "RMW=${RMW_IMPLEMENTATION}  PAD=${PAD}  DEV=${JOY_DEV}  MODE=${OUTPUT_MODE}"

exec ros2 launch bb_joy bb_joy.launch.py \
  pad:="${PAD:-xbox}" \
  device:="${JOY_DEV:-/dev/input/js0}" \
  output_mode:="${OUTPUT_MODE:-twist}"
