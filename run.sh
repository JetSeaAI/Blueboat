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
#   ./run.sh servo        看飛控送給 ESC 的 PWM（override 生效與否看這個）
#   ./run.sh rate         調高 MAVLink stream rate（rc/in 空的時候先試）
#   ./run.sh state        看飛控狀態（armed / mode）
#   ./run.sh mode MANUAL  切模式
#   ./run.sh arm          解鎖（會先問你）
#   ./run.sh disarm       上鎖
#   ./run.sh fixsysid     把飛控的 SYSID_MYGCS 設成 mavros 的 system_id
#                         （SID=1 可覆蓋）
#   ./run.sh diag         一次收集 rc_override 卡關需要的所有資訊
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

  arm|disarm)
    val=$([ "$1" = "arm" ] && echo true || echo false)
    if [ "$1" = "arm" ]; then
      warn "解鎖後推進器可能立刻轉動。確認螺旋槳淨空或船已架起。"
      read -r -p "繼續？(y/N) " ans
      [ "${ans:-N}" = "y" ] || die "取消"
    fi
    in_container "ros2 service call /mavros/cmd/arming \
      mavros_msgs/srv/CommandBool '{value: ${val}}'"
    ;;

  mode)
    [ -n "${2:-}" ] || die "用法：./run.sh mode MANUAL|HOLD|GUIDED|AUTO"
    in_container "ros2 service call /mavros/set_mode \
      mavros_msgs/srv/SetMode '{base_mode: 0, custom_mode: \"$2\"}'"
    ;;

  state)
    in_container 'ros2 topic echo /mavros/state'
    ;;

  fixsysid)
    # 讓 mavros 的 sysid 和飛控的 MAV_GCS_SYSID 對上，
    # 否則 ArduPilot 會靜靜丟掉所有 RC override。
    #
    # 飛控參數要走 ParamSetV2 service —— ros2 param set 對 mavros 沒用。
    SID="${SID:-1}"
    warn "會把飛控的 MAV_GCS_SYSID 改成 ${SID}（寫進飛控的永久設定）。"
    warn "如果你們也用 Mission Planner 操控，它通常是 255，改完那邊的"
    warn "搖桿/override 功能會失效。不確定的話改 mavros 端比較安全："
    warn "  在 apm.launch 把 system_id 設成 255"
    read -r -p "繼續改飛控？(y/N) " ans
    [ "${ans:-N}" = "y" ] || die "取消"
    in_container '
      set_one() {
        ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
          "{force_set: true, param_id: \"$1\", value: {type: 2, integer_value: '"${SID}"'}}" \
          2>/dev/null | grep -q "success=True"
      }
      if set_one MAV_GCS_SYSID; then
        echo "已設定 MAV_GCS_SYSID = '"${SID}"'"
      elif set_one SYSID_MYGCS; then
        echo "已設定 SYSID_MYGCS = '"${SID}"'"
      else
        echo "設不了。改用 Mission Planner / QGC，或把 mavros 的 system_id 改成 255"
      fi'
    ;;

  servo)
    # 飛控實際送給 ESC 的 PWM。override 有沒有生效看這個，不是看 rc/in。
    in_container 'ros2 topic echo /mavros/rc/out'
    ;;

  rate)
    info "把 MAVLink stream rate 調高（rc/in 空的時候先試這個）"
    in_container '
      ros2 run mavros mavsys rate --all 10 2>/dev/null \
        || echo "  (mavsys 不可用，改用 Mission Planner 設 SR0_* 參數)"'
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
      echo "--- mavros 自己的 system_id ---"
      ros2 param get /mavros system_id 2>/dev/null || echo "  (讀不到)"
      echo
      echo "--- 飛控的 SYSID_MYGCS ---"
      # ROS2 版 mavros 把飛控參數掛在 param plugin 的 node 底下，
      # 用 ros2 param 讀，不是 mavparam CLI。先 pull 一次確保有快取。
      ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull \
        "{force_pull: false}" >/dev/null 2>&1 || true
      found=
      for n in /mavros/param /mavros; do
        for k in SYSID_MYGCS MAV_GCS_SYSID; do
          out=$(ros2 param get "$n" "$k" 2>/dev/null) && {
            echo "  $n $k -> $out"
            found=1
          }
        done
      done
      if [ -z "$found" ]; then
        echo "  (ros2 param 讀不到。現有的 param 相關 node："
        ros2 node list 2>/dev/null | grep -i param | sed "s/^/    /"
        echo "   或直接用 Mission Planner / QGC 查 SYSID_MYGCS)"
      fi'
    ;;

  diag)
    # 一次收集判斷 rc_override 卡在哪需要的所有資訊
    in_container '
      echo "════ 1. 飛控狀態（armed / mode 要對）════"
      timeout 3 ros2 topic echo --once /mavros/state || echo "  (沒訊息)"

      echo
      echo "════ 2. GCS sysid（兩邊必須一致）════"
      echo "-- mavros system_id --"
      ros2 param get /mavros system_id 2>/dev/null || echo "  (讀不到)"
      # 第一次 pull 要把飛控上千個參數抓回來，可能要 1-2 分鐘。
      # 沒抓完的話 ros2 param get 會回 "Parameter not set" —— 那是還沒同步，
      # 不是飛控上沒有這個參數。
      echo "  (正在同步飛控參數，第一次可能要 1-2 分鐘，請不要中斷…)"
      timeout 180 ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull \
        "{force_pull: false}" >/dev/null 2>&1 \
        || echo "  (參數同步逾時，下面的值可能不準)"
      for k in MAV_GCS_SYSID SYSID_MYGCS; do
        for n in /mavros/param /mavros; do
          out=$(ros2 param get "$n" "$k" 2>/dev/null) && echo "-- $k -- $out"
        done
      done

      echo
      echo "════ 3. 相關飛控參數 ════"
      for k in FS_THR_ENABLE RC_OPTIONS RCMAP_THROTTLE RCMAP_ROLL \
               SERVO1_FUNCTION SERVO3_FUNCTION RC3_MIN RC3_MAX RC3_TRIM; do
        for n in /mavros/param /mavros; do
          out=$(ros2 param get "$n" "$k" 2>/dev/null) && { echo "  $k = $out"; break; }
        done
      done

      echo
      echo "════ 4. 飛控的話（failsafe / 拒絕理由）════"
      timeout 5 ros2 topic echo /mavros/statustext/recv || echo "  (5 秒內沒訊息)"'
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
