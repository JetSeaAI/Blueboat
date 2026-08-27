# zenoh-config（vendored）

這是從 [JetSeaAI/zenoh-config](https://github.com/JetSeaAI/zenoh-config) 複製過來的一份，
因為那個 repo 不是 public，IPC 上沒辦法直接 clone。

只留實際會用到的三個檔：

| 檔案 | 用途 |
| --- | --- |
| `zenoh-session.json5` | 每個 ROS2 node 的 session 設定，`peer` 模式，connect `tcp/localhost:7447` |
| `zenoh-router.json5` | router 設定，listen `tcp/0.0.0.0:7447` |
| `ROS2_AED.sh` | 卡住的時候清乾淨：`pkill -9 -f ros && ros2 daemon stop` |

> ⚠️ 這是**複製品**，上游改了不會自動同步。連線出問題時先和上游 repo 對一次，
> 特別是 `connect.endpoints`（跨機器 / 基地台的位址）。
