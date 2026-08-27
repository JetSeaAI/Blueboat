#!/usr/bin/env bash
# Blueboat 手把遙控 一鍵啟動
#
#   ./run.sh              起 zenoh router（需要的話）+ 手把遙控
#   ./run.sh shell        進容器 bash，手動 debug 用
#   ./run.sh joy          只看 /joy，確認手把訊號
#   ./run.sh vel          看送出去的速度指令
#   ./run.sh rcout        看送出去的 RC override
#   ./run.sh rcin         看飛控實際收到的 RC（判斷 override 有沒有被接受）
#   ./run.sh fc           看飛控的 STATUSTEXT（切模式被拒的理由在這）
#   ./run.sh gps          看 GPS fix / 衛星數 / EKF（GUIDED 進不去先看這）
#   ./run.sh rctest       不經過 bb_joy，直接送一個固定 RC override 測飛控
#                         （CH=1 PWM=1600 可覆蓋）
#   ./run.sh sysid        查 SYSID_MYGCS 和 mavros 的 system_id
#   ./run.sh logs         看 log
#   ./run.sh down         全部停掉
#
# 環境變數：
#   PAD=xbox|ps5          手把型號        （預設 xbox）
#   JOY_DEV=/dev/input/jsN 手把裝置       （預設 /dev/input/js0）
#   OUTPUT_MODE=twist|rc_override         （預設 twist）
#   NO_ROUTER=1           不要自己起 router

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE=(docker compose -p blueboat -f "${HERE}/docker/docker-compose.yaml")
BB=ros2-base-js-blueboat

export PAD="${PAD:-xbox}"
export JOY_DEV="${JOY_DEV:-/dev/input/js0}"
export OUTPUT_MODE="${OUTPUT_MODE:-twist}"

info() { echo -e "\033[36m▸\033[0m $*"; }

# 在跑著的容器裡執行一段 ROS2 指令
in_container() {
  docker inspect -f '{{.State.Running}}' "${BB}" 2>/dev/null | grep -q true \
    || die "容器 ${BB} 沒在跑。先開另一個 terminal 執行 ./run.sh"
  docker exec -it "${BB}" bash -ic \
    "source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash 2>/dev/null; $1"
}
warn() { echo -e "\033[33m⚠\033[0m  $*" >&2; }
die()  { echo -e "\033[31m✘\033[0m $*" >&2; exit 1; }

# zenoh router 在不在？用 bash 內建的 /dev/tcp 探，不依賴 ss/netstat/nc
router_is_up() {
  (exec 3<>/dev/tcp/127.0.0.1/7447) >/dev/null 2>&1
}

check_pad() {
  if [ ! -e "${JOY_DEV}" ]; then
    warn "找不到 ${JOY_DEV}"
    local found
    found=$(ls /dev/input/js* 2>/dev/null || true)
    if [ -n "${found}" ]; then
      warn "但有偵測到：${found}"
      warn "用 JOY_DEV=<裝置> $0 指定，例如 JOY_DEV=/dev/input/js1 $0"
    else
      warn "系統上沒有任何 /dev/input/js*，手把可能沒插好或驅動沒載入。"
    fi
    die "手把沒接上，先處理再跑。"
  fi
  info "手把：${JOY_DEV}"
}

start_router() {
  if [ "${NO_ROUTER:-0}" = "1" ]; then
    info "NO_ROUTER=1，跳過 router"
    return
  fi
  if router_is_up; then
    info "zenoh router 已經在跑（127.0.0.1:7447），沿用現有的"
    return
  fi
  info "7447 沒人聽，起一個 zenoh router"
  "${COMPOSE[@]}" --profile router up -d zenoh-router
  for _ in $(seq 1 30); do
    router_is_up && { info "router 起來了"; return; }
    sleep 1
  done
  warn "等了 30 秒 router 還沒 listen，繼續往下跑（peer multicast 在同一台機器上仍可能通）"
}

case "${1:-up}" in
  up)
    check_pad
    start_router
    info "PAD=${PAD}  JOY_DEV=${JOY_DEV}  OUTPUT_MODE=${OUTPUT_MODE}"
    info "第一次跑會 apt 裝套件加 colcon build，比較久是正常的"
    "${COMPOSE[@]}" up "${BB}"
    ;;

  shell)
    start_router
    info "進容器。裡面要先 source /opt/ros/humble/setup.bash"
    "${COMPOSE[@]}" run --rm --name bb-shell "${BB}" bash
    ;;

  joy)
    in_container 'ros2 topic echo /joy'
    ;;

  vel)
    in_container 'ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped'
    ;;

  rcin)
    in_container 'ros2 topic echo /mavros/rc/in'
    ;;

  rcout)
    in_container 'ros2 topic echo /mavros/rc/override'
    ;;

  fc)
    in_container 'ros2 topic echo /mavros/statustext/recv'
    ;;

  gps)
    in_container '
      for t in /mavros/global_position/raw/fix \
               /mavros/global_position/raw/satellites \
               /mavros/estimator_status; do
        echo "--- ${t} ---"
        # 沒有訊息是常見的（例如 ArduPilot 不發 estimator_status），
        # 那只是「這台沒這個資料」，不是錯誤
        timeout 3 ros2 topic echo --once "${t}" || echo "  (3 秒內沒有訊息)"
      done'
    ;;

  rctest)
    # 不經過 bb_joy，直接對 /mavros/rc/override 發一個固定值。
    # 用來分辨「飛控不吃 override」還是「我的節點有問題」。
    CH="${CH:-1}"
    PWM="${PWM:-1600}"
    warn "會持續對 ch${CH} 送 ${PWM}，其餘通道 NOCHANGE。"
    warn "確認螺旋槳淨空或船已架起再繼續。Ctrl-C 停止。"
    read -r -p "繼續？(y/N) " ans
    [ "${ans:-N}" = "y" ] || die "取消"
    CHANNELS=$(python3 -c "
ch, pwm = ${CH}, ${PWM}
v = [65535] * 18
v[ch - 1] = pwm
print(v)")
    in_container "ros2 topic pub -r 10 /mavros/rc/override \
      mavros_msgs/msg/OverrideRCIn '{channels: ${CHANNELS}}'"
    ;;

  sysid)
    info "ArduPilot 只接受來自 SYSID_MYGCS 那個 sysid 的 RC override"
    in_container '
      echo "--- 飛控的 SYSID_MYGCS ---"
      ros2 run mavros mavparam get SYSID_MYGCS 2>/dev/null \
        || echo "  (mavparam 讀不到，改用 Mission Planner / QGC 查)"
      echo "--- mavros 自己的 system_id ---"
      ros2 param get /mavros system_id 2>/dev/null \
        || echo "  (讀不到，看 apm.launch 裡的設定，預設是 1)"'
    ;;

  logs)
    "${COMPOSE[@]}" logs -f --tail=100 "${BB}"
    ;;

  down)
    info "停掉所有 container"
    "${COMPOSE[@]}" --profile router down
    ;;

  *)
    # 印出檔案開頭的註解區塊當作說明
    awk 'NR>1 && /^#/ {sub(/^# ?/,""); print; next} NR>1 {exit}' "${BASH_SOURCE[0]}"
    exit 1
    ;;
esac
