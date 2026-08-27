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

上限 6 節 = 6 × 0.5144 ≈ **3.09 m/s**。

| 參數 | 值 | 意思 |
| --- | --- | --- |
| `max_linear_speed` | `3.09` | RT 踩到底的前進速度（6 節） |
| `max_reverse_speed` | `1.5` | LT 踩到底的後退速度（約 2.9 節，倒車保守一點） |
| `max_angular_speed` | `0.5` | 搖桿打滿的角速度 rad/s |
| `scale_low` | `0.5` | 慢速檔倍率（開機預設） |
| `scale_high` | `1.0` | 全速檔倍率，按 LB / L1 切換 |
| `invert_throttle` | `true` | 實測扳機方向是反的，翻過來 |

實際輸出：

| 操作 | 慢速檔（預設） | 全速檔 |
| --- | --- | --- |
| RT 踩到底 | `+1.55` m/s（3 節） | `+3.09` m/s（6 節） |
| LT 踩到底 | `-0.75` m/s | `-1.50` m/s |

要改上限直接調 `config/xbox.yaml` 的 `max_linear_speed`，不用動程式。
節數換算：`m/s = 節 × 0.5144`。

## 鬆手會倒車？—— twist 和 rc_override 的差別

**在 GUIDED 底下，`linear.x = 0` 不是「油門歸零」，是「把速度控制到 0 並維持住」。**
船還有前進慣性，ArduPilot 就主動下倒車推力去煞停 —— 表現出來就是鬆開扳機後
突然噴一段倒車。切到 AUTO/MANUAL 再切回 GUIDED 才恢復，也是同一個閉迴路
控制器重新接管時舊目標值和積分項還在。

這不是手把邏輯的問題，是閉迴路速度控制的必然行為。

| | `twist`（GUIDED） | `rc_override`（MANUAL） |
| --- | --- | --- |
| 控制方式 | 閉迴路，飛控算 PID 追速度 | 開迴路，PWM 直接對應推力 |
| 鬆手 | 主動煞停，會倒車 | 真的沒推力，靠水阻滑行 |
| 速度單位 | m/s，數字就是實際速度 | PWM，和推力的關係要自己抓 |
| 適合 | 自駕、定速航行 | **手動駕駛** |

### 手動駕駛建議改用 rc_override

```bash
OUTPUT_MODE=rc_override ./run.sh
```

飛控要切到 **MANUAL**（手把上按 A / ✕）。

要注意的是這條路徑**還沒在你們船上驗證過**：

- ArduPilot 的 `RC_OPTIONS` 要允許 RC override
- 通道對應預設是 ch1 轉向、ch3 油門，和你們的接線不同的話改
  `steer_channel` / `throttle_channel`
- PWM 範圍預設 `1100 / 1500 / 1900`，和你們的 ESC 行程不同要調

第一次測請把船架起來或確認螺旋槳淨空，先確認 `ros2 topic echo /mavros/rc/override`
的數字合理、推力方向正確，再下水。

### rc_override 在 MANUAL 沒反應？

先把「飛控不吃 override」和「我的節點有問題」分開。
`rctest` 不經過 bb_joy，直接對 topic 送一個固定值：

```bash
# Terminal 1 —— 容器要在跑（launch 起不起來都行）
./run.sh

# Terminal 2 —— 直接送 ch1 = 1600
./run.sh rctest

# Terminal 3 —— 看飛控收到什麼
./run.sh rcin
```

> ⚠️ 會持續送指令，確認螺旋槳淨空或船已架起。`CH=3 PWM=1600 ./run.sh rctest`
> 可以改通道和數值。

`rcin` 的 `channels[0]`（ch1）有沒有變成 1600？

#### `rc/in` 顯示 `channels: []`

表示飛控**沒有在送 RC_CHANNELS 訊息**，不是「送了但值沒變」。可能是：

- 船上根本沒接遙控接收機（純 MAVLink 控制）—— 這樣 `rc/in` 空是正常的
- MAVLink stream rate 被關掉（`SR0_RC_CHAN = 0`）—— `./run.sh rate` 可以調高

**`rc/in` 空不代表 override 沒作用。** `rc/in` 是「接收機收到什麼」，
override 則是繞過接收機直接注入。要確認 override 有沒有生效，看
**`/mavros/rc/out`** —— 那是飛控實際送給 ESC 的 PWM：

```bash
./run.sh servo
```

MANUAL + 已解鎖的狀態下送 override，`rc/out` 的對應通道應該跟著動。
動了就是通了，剩下的是通道對應；完全不動才是飛控在丟棄 override。

#### `rc/out` 停在 1510 之類的中立值

**ArduPilot 在 disarmed 狀態下，油門輸出一律鎖在中立 trim**，不管什麼模式、
也不管有沒有 override。所以「送了 override 但 `rc/out` 不動」最常見的原因
就是船沒解鎖，而不是 override 被丟棄。

完整的測試順序：

```bash
./run.sh state           # 先看 armed / mode
./run.sh mode MANUAL
./run.sh arm             # ⚠️ 螺旋槳淨空
CH=3 PWM=1600 ./run.sh rctest    # 另一個 terminal
./run.sh servo           # 再一個 terminal，看 ch3 有沒有離開 1510
./run.sh disarm          # 測完記得上鎖
```

`rc/out` 的通道排列對應 `SERVOn_FUNCTION`，一艘船通常是
ch1 = 轉向、ch3 = 油門，其餘沒指派功能所以是 `0`。

#### 沒有接收機的話還要注意 RC failsafe

船上沒有遙控接收機時，ArduPilot 可能一直處在 **radio failsafe**，
即使在 MANUAL 也不會給推力。`./run.sh fc` 看有沒有 `Radio failsafe` 之類的訊息，
有的話要調 `FS_THR_ENABLE`。

如果要連手把一起測：

```bash
OUTPUT_MODE=rc_override ./run.sh    # Terminal 1
./run.sh rcout                       # Terminal 2，我們送出去的
./run.sh rcin                        # Terminal 3，飛控看到的
```

按住 RB 踩扳機，看 `rcout` 的 `channels[2]`（ch3 油門）有沒有離開 1500。
然後看 `rcin`：

| `rcin` 的表現 | 意思 | 往哪查 |
| --- | --- | --- |
| **完全不跟著變** | ArduPilot 把 override 丟掉了 | 下面的 `SYSID_MYGCS` |
| 跟著變但船不動 | 收下了，是通道 / 模式 / 解鎖的問題 | 下面第 2、3 點 |

#### 1. `SYSID_MYGCS`（最常見）

ArduPilot 只接受**來自 `SYSID_MYGCS` 指定的那個 GCS** 的 RC override，
其他來源的直接丟掉、不回報任何錯誤。這是 MAVROS + ArduPilot 最典型的坑。

- ArduPilot 預設 `SYSID_MYGCS = 255`（給地面站用的）
- MAVROS 預設自己的 `system_id = 1`

兩邊對不上，override 就靜靜被忽略 —— 而 `twist`（GUIDED setpoint）**沒有**
這個限制，所以會出現「只有 GUIDED 能動」這個現象。

兩種改法，擇一：

```bash
# A. 把飛控的 SYSID_MYGCS 改成 mavros 的 system_id（通常是 1）
#    用 Mission Planner / QGC 改，或
ros2 run mavros mavparam set SYSID_MYGCS 1

# B. 把 mavros 的 system_id 改成 255（改 apm.launch 的參數）
```

> ArduPilot 4.7 之後這個參數改名叫 `MAV_GCS_SYSID`，行為一樣。
> `ros2 run mavros mavparam get SYSID_MYGCS` 可以先確認目前值。

#### 2. 通道對應

預設 ch1 轉向、ch3 油門，對應 ArduPilot 的 `RCMAP_ROLL` / `RCMAP_THROTTLE`。
你們如果是差速推進（skid steering）或改過 `RCMAP_*`，要跟著改
`steer_channel` / `throttle_channel`。

#### 3. 其他

- 船要**解鎖**（armed），MANUAL 模式下沒解鎖一樣不會動
- `RC_OPTIONS` 有幾個 bit 會影響 override 行為
- PWM 要落在該通道的 `RCn_MIN` / `RCn_MAX` 之內

### 切不進 GUIDED（MANUAL / HOLD 都可以）

`mode_sent=True` **只代表指令送到飛控了，不代表飛控接受**。
節點現在會在送出後 3 秒回頭確認 `/mavros/state` 的 mode 有沒有真的變，
沒變就警告，並把飛控的 STATUSTEXT 一起印在 log 裡 —— 拒絕的理由都在那。

```bash
./run.sh fc      # 飛控自己講的話
./run.sh gps     # GPS fix / 衛星數 / EKF
```

**最可能的原因：GUIDED 需要有效的位置估計。**
ArduRover 的 GUIDED 是位置/速度導引模式，要有 GPS 3D fix 而且 EKF 收斂；
MANUAL 和 HOLD 不需要位置，所以照樣進得去 —— 症狀正好是「只有 GUIDED 不行」。

依序確認：

1. `./run.sh fc` 有沒有 `Mode change failed`、`EKF3 waiting for GPS`、
   `PreArm: ...` 之類的訊息
2. `./run.sh gps` 看 `status.status >= 0`（有 fix）、衛星數夠不夠（一般要 6 顆以上）
3. 室內測試本來就進不去 GUIDED —— 沒有 GPS 訊號。要在室內驗證手把邏輯，
   用 `MANUAL` + `rc_override`，或先接模擬器

### 模式守門

節點現在會檢查飛控模式，不對就**不送指令**：

| `output_mode` | 需要的模式 |
| --- | --- |
| `twist` | `GUIDED` |
| `rc_override` | `MANUAL` |

模式不對時 log 會說 `飛控在 X，twist 需要 GUIDED，暫不送指令`。
這樣切回 GUIDED 的瞬間就不會有一筆舊的 setpoint 等在那裡被拿去執行。
用 `require_mode: ""` 可以關掉這個檢查（不建議）。

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

**鬆開扳機後船會噴一段倒車** — 見上面
[鬆手會倒車？](#鬆手會倒車--twist-和-rc_override-的差別)。
GUIDED 的閉迴路控制器把「速度 0」當成「主動煞停」。
手動駕駛改用 `OUTPUT_MODE=rc_override ./run.sh` + MANUAL 模式。

**能切模式、log 也正常，但船就是不動** — 依序排除：

1. `./run.sh vel` 看 `linear.x` 有沒有數字。**一直是 0** 就是手把端，往下看第 2 步；
   **有數字**就是飛控端，跳到第 4 步。
2. 節點每秒會印一行 `steer=… throttle=… <提示>`，直接看它卡在哪
   （沒按 deadman、扳機中立、還是沒收到 `/joy`）。
3. `./run.sh joy`，**手不要碰扳機**，看 `axes[2]` / `axes[5]` 停在多少。
   不是 `1.0` 的話要改 config：停在 `0.0` 就設 `trigger_idle: 0.0` +
   `trigger_full: 1.0`；停在 `-1.0` 就設 `trigger_idle: -1.0` + `trigger_full: 1.0`。
4. 飛控端：`twist` 要在 **GUIDED** 模式而且已解鎖。log 尾巴的
   `armed=… mode=…` 可以直接確認。
5. 速度太小推不動：慢速檔 RT 踩到底只有 0.25 m/s。按 LB 切全速試試，
   或把 `scale_low` 調大。

**踩 RT 沒反應** — 先 `ros2 topic echo /joy` 看 `axes[5]` 會不會變。
有些 Xbox 手把（尤其藍牙模式）扳機的 index 不同，把實際的填回
`config/xbox.yaml` 的 `axis_throttle_forward`。

**扳機方向相反**（沒踩是 -1、踩到底是 +1）— 改 `trigger_idle: -1.0` 和
`trigger_full: 1.0`。

**轉向反了** — 把 `invert_steer` 設 `true`。
**前進後退反了** — 把 `invert_throttle` 設 `true`（目前已經是 `true`）。

**推到底太快 / 太慢** — 調 `max_linear_speed`，或直接用慢速檔跑。

**PS5 手把按鍵對不上** — DualSense 在不同 kernel 版本排列不一樣。
`ros2 topic echo /joy` 一個一個按過，把實際 index 填回 `config/ps5.yaml`。
