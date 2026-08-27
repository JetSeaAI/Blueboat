# BB-joy

用 Xbox / PS5 手把透過 **MAVROS** 遙控 USV（ArduPilot / ArduRover 韌體）。

輸出等同於已經確認可以讓船動的那條指令，只是把固定值換成手把讀值：

```bash
ros2 topic pub -r 10 /mavros/setpoint_velocity/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

## 這個套件做什麼

```
手把 --(/dev/input/js0)--> joy_linux_node --(/joy)--> bb_joy_teleop --> MAVROS --> 飛控
```

預設 `output_mode: twist`，以 10 Hz 發 `/mavros/setpoint_velocity/cmd_vel_unstamped`。
另外保留 `output_mode: rc_override`（發 `/mavros/rc/override` 直接覆寫 PWM，
飛控要在 `MANUAL`），沒用到的話可以忽略。

也會呼叫 `/mavros/cmd/arming` 解鎖上鎖、`/mavros/set_mode` 切模式。

## 操控方式

| 動作 | Xbox | PS5 |
| --- | --- | --- |
| **Deadman（按住才會出力）** | RB | R1 |
| 前進 | RT（類比，踩多深走多快） | R2 |
| 後退 | LT | L2 |
| 轉向 | 左搖桿 左右 | 左搖桿 左右 |
| 慢速 / 全速切換 | LB | L1 |
| 解鎖 ARM | Start | Options |
| 上鎖 DISARM | Back | Create |
| 切 MANUAL | A | ✕ |
| 切 HOLD | B | ○ |
| 切 GUIDED | X | △ |
| 切 AUTO | Y | □ |

左搖桿只用左右軸，上下不接任何功能。RT 和 LT 同時踩會互相抵銷（`前進 - 後退`），
不會因為誤觸而突然衝出去。

## 速度設定

船目前限速 0.5 m/s，所以 config 是這樣配的：

| 參數 | 值 | 意思 |
| --- | --- | --- |
| `max_linear_speed` | `0.5` | RT 踩到底的前進速度 |
| `max_reverse_speed` | `0.3` | LT 踩到底的後退速度（倒車保守一點） |
| `max_angular_speed` | `0.5` | 搖桿打滿的角速度 rad/s |
| `scale_low` | `0.5` | 慢速檔倍率（開機預設） |
| `scale_high` | `1.0` | 全速檔倍率，按 LB / L1 切換 |

實際輸出：

| 操作 | 慢速檔（預設） | 全速檔 |
| --- | --- | --- |
| RT 踩到底 | `linear.x = +0.25` | `linear.x = +0.50` |
| LT 踩到底 | `linear.x = -0.15` | `linear.x = -0.30` |

要改上限直接調 `config/xbox.yaml` 的 `max_linear_speed`，不用動程式。

## 安全機制

- **Deadman**：RB / R1 沒按住，一律送 0，手一放船就停。
- **看門狗**：`/joy` 超過 `joy_timeout`（0.5 秒）沒更新就送 0。手把斷線、USB 拔掉都會觸發。
- **慢速檔開機預設**：要全速得手動按一下 LB / L1。
- **扳機初始值處理**：joy_linux 在扳機「第一次被踩下去」之前會一直回 `0.0`，
  那個 `0.0` 落在區間正中間，直接換算會變成「油門半開」。
  程式在讀到第一個非零值之前一律當成沒踩，所以啟動當下不會自己往前跑。

`joy_linux_node` 的 `autorepeat_rate` 設 20 Hz，搖桿不動也會持續有訊息，
不會被看門狗誤判成斷線。

## 在 Docker 裡跑（ROS2 只有容器裡有）

```bash
git clone https://github.com/JetSeaAI/Blueboat.git ~/Blueboat
cd ~/Blueboat
./run.sh
```

就這樣。`run.sh` 會依序：檢查手把 → 需要的話起 zenoh router → 起遙控容器
（容器裡自己 apt 裝套件、`colcon build`、`ros2 launch`）。

`zenoh-config` 已經一起放在這個 repo 裡（[zenoh-config/](zenoh-config/)），
不用另外 clone。

### run.sh 的其他用法

| 指令 | 做什麼 |
| --- | --- |
| `./run.sh` | 起 router（需要的話）+ 手把遙控 |
| `./run.sh shell` | 進容器 bash，手動 debug |
| `./run.sh joy` | 看 `/joy`，確認手把訊號 |
| `./run.sh vel` | 看送出去的速度指令 |
| `./run.sh logs` | 看 log |
| `./run.sh down` | 全部停掉 |

| 環境變數 | 預設 | 說明 |
| --- | --- | --- |
| `PAD` | `xbox` | `xbox` 或 `ps5` |
| `JOY_DEV` | `/dev/input/js0` | 手把裝置 |
| `OUTPUT_MODE` | `twist` | `twist` 或 `rc_override` |
| `NO_ROUTER` | — | 設 `1` 就不自己起 router |

```bash
PAD=ps5 JOY_DEV=/dev/input/js1 ./run.sh
```

### zenoh router 會不會重複起

`run.sh` 用 `/dev/tcp` 探 `127.0.0.1:7447`，有人在聽就沿用現有的 router，
不會起第二個去搶 port。IPC 上本來就有 router 在跑的話它會自己讓開。

### 第一次上機建議

先確認手把訊號正確再接飛控：

```bash
# Terminal 1
./run.sh

# Terminal 2
./run.sh joy      # 按住 RB 踩 RT，看 axes[5] 從 1.0 跑到 -1.0
./run.sh vel      # 再看速度指令對不對
```

手把 axis index 對不對是後面所有問題的根源，先確認這個再往下。

> **不要 `source docker/startup_bb_joy.sh`。** 那支腳本結尾是 `exec`，
> source 進互動 shell 會直接把你的 terminal 換掉，看起來就是「視窗突然消失」。
> 它是設計給 compose 當 `command` 用的。`run.sh` 不受影響，正常用就好。

### 依賴

`ros2-base` 映像裡沒有這兩包，`run.sh` 會自己補：

| 套件 | 需要嗎 |
| --- | --- |
| `ros-humble-joy-linux` | **必要**，沒有就讀不到手把 |
| `ros-humble-mavros-msgs` | 選配，只影響解鎖/切模式/`rc_override` |

**開船本身不需要 `mavros_msgs`。** 發速度指令只用到 `geometry_msgs/Twist`，
和你手打 `ros2 topic pub` 是一樣的東西 —— mavros 跑在別的節點（`apm.launch`），
我們只是往它的 topic 發。`mavros_msgs` 是給這些附加功能用的：

- 解鎖 / 上鎖（`CommandBool` service）
- 切模式（`SetMode` service）
- 讀 `/mavros/state`
- `output_mode: rc_override`

import 不到的話節點會印警告然後**照常跑**，只是那些按鍵沒作用。
要用的話進容器手動裝：

```bash
apt-get update && apt-get install -y ros-humble-mavros-msgs
```

容器裡 `/ros2_ws` 沒有掛出來，所以每次重啟都會重 build（ament_python 幾秒），
apt 裝的套件同樣不會留。之後嫌慢可以把 `joy_linux` 烤進映像。

## 手動建置（已經有 ROS2 環境的話）

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-joy-linux ros-humble-mavros-msgs

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
ln -s /path/to/js-perceptions/BB-joy bb_joy
cd ~/ros2_ws && colcon build --symlink-install --packages-select bb_joy
source install/setup.bash

ros2 launch bb_joy bb_joy.launch.py pad:=xbox
```

## 啟動順序（zenoh）

公司的 `zenoh-config` 是 `rmw_zenoh_cpp` + session `peer` 模式，
所以每個 terminal 都要先有這組環境變數（`setup_zenoh.sh` 會寫進 `.bashrc`）：

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI=~/zenoh-config/zenoh-session.json5
export ZENOH_ROUTER_CONFIG_URI=~/zenoh-config/zenoh-router.json5
```

順序：

```bash
# 1. zenoh router（放著不動，跨機器溝通靠它）
ros2 run rmw_zenoh_cpp rmw_zenohd

# 2. mavros（本來就在跑的話跳過）

# 3. build 一次
cd ~/ros2_ws && colcon build --symlink-install --packages-select bb_joy
source install/setup.bash

# 4. 跑 node（joy_linux + teleop 一起帶起來）
ros2 launch bb_joy bb_joy.launch.py pad:=xbox
```

幾點注意：

- **`RMW_IMPLEMENTATION` 要和 mavros 那邊一致。** 這是最常見的「topic list 看不到對方」
  的原因 —— 兩邊 RMW 不同就是兩個互不相通的網路。
- **build 之前要先 `source /opt/ros/humble/setup.bash`**，否則 colcon 找不到 ament。
- **router 沒起也可能會動**：session 是 `peer` + multicast scouting，同一台機器上
  peer 之間可以直接互相發現。但 log 會一直噴連不上 `tcp/localhost:7447`，
  而且跨機器（基地台）一定要 router。
- **`--symlink-install` 之下改 `.py` / `.yaml` 不用重 build**，重跑 launch 就好。
  只有新增檔案才需要再 `colcon build`。
- 卡住的時候用 `zenoh-config/ROS2_AED.sh`（`pkill -9 -f ros && ros2 daemon stop`）清乾淨再來。

## 測試順序

先確認手把有訊號，再接飛控：

```bash
# 1. 手把讀得到嗎（重點看 RT/LT 動的時候 axes[5] / axes[2] 有沒有變）
ros2 run joy_linux joy_linux_node
ros2 topic echo /joy

# 2. 按住 RB 踩 RT，看送出去的速度對不對
ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped

# 3. 確認飛控狀態（armed / mode 驗證按鍵有沒有生效）
ros2 topic echo /mavros/state
```

第 2 步在船架起來或螺旋槳淨空的狀況下做，確認數值符合上面那張表再下水。

## 常見狀況

**`No module named 'mavros_msgs'`** — 現在不會再讓節點死掉了，只會印警告並停用
解鎖/切模式。真的要那些功能就 `apt-get install -y ros-humble-mavros-msgs`。

**踩 RT 沒反應** — 先 `ros2 topic echo /joy` 看 `axes[5]` 會不會變。
有些 Xbox 手把（尤其藍牙模式）扳機的 index 不同，把實際的填回
`config/xbox.yaml` 的 `axis_throttle_forward`。

**扳機方向相反**（沒踩是 -1、踩到底是 +1）— 改 `trigger_idle: -1.0` 和
`trigger_full: 1.0`。

**轉向反了** — 把 `invert_steer` 設 `true`。

**推到底太快 / 太慢** — 調 `max_linear_speed`，或直接用慢速檔跑。

**PS5 手把按鍵對不上** — DualSense 在不同 kernel 版本排列不一樣。
`ros2 topic echo /joy` 一個一個按過，把實際 index 填回 `config/ps5.yaml`。
