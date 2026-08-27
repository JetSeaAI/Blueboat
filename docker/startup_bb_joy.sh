#!/usr/bin/env bash
# 在容器裡建置並啟動 bb_joy。掛載進來的原始碼會被 symlink 進 /ros2_ws。
#
# 這支是給 docker compose 當 command 用的（結尾會 exec 掉自己）。
# 不要 source 它 —— exec 會把你的互動 shell 直接換掉，terminal 會消失。
# 要手動跑就直接執行：  ./docker/startup_bb_joy.sh

# 被 source 的話擋下來，不然 set -e 和 exec 會把使用者的 shell 幹掉
if [ "${BASH_SOURCE[0]}" != "${0}" ]; then
  echo "⚠️  不要 source 這支腳本，直接執行它：" >&2
  echo "    ${BASH_SOURCE[0]}" >&2
  return 1
fi

set -e

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "❌ 找不到 /opt/ros/humble/setup.bash" >&2
  echo "   這支腳本只能在 ROS2 容器裡跑，不是 host。先進容器：" >&2
  echo "   cd ~/Blueboat/docker && docker compose run --rm ros2-base-js-blueboat bash" >&2
  exit 1
fi

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
ln -sfn /home/Blueboat "${WS}/src/bb_joy"

cd "${WS}"
colcon build --symlink-install --packages-select bb_joy
source "${WS}/install/setup.bash"

echo "RMW=${RMW_IMPLEMENTATION}  PAD=${PAD}  DEV=${JOY_DEV}  MODE=${OUTPUT_MODE}"

exec ros2 launch bb_joy bb_joy.launch.py \
  pad:="${PAD:-xbox}" \
  device:="${JOY_DEV:-/dev/input/js0}" \
  output_mode:="${OUTPUT_MODE:-twist}"
