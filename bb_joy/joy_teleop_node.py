#!/usr/bin/env python3
"""手把 -> MAVROS 遙控節點。

訂閱 /joy (joy_linux 出來的 sensor_msgs/Joy)，轉成 MAVROS 的速度指令：

    RT / R2  前進      LT / L2  後退      左搖桿 左右  轉向

輸出等同於這條已知可動的指令：

    ros2 topic pub -r 10 /mavros/setpoint_velocity/cmd_vel_unstamped \
        geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"

另外保留 output_mode=rc_override，直接覆寫 RC PWM，給 MANUAL 模式用。

安全機制：
  * deadman 鍵沒按住就送 0，放開手把船就停。
  * 收不到 /joy 超過 joy_timeout 秒同樣送 0。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy

# 開船只需要 geometry_msgs 的 Twist —— mavros 跑在別的節點，我們只是往它的
# topic 發東西。mavros_msgs 只有這些附加功能才用得到：解鎖/上鎖、切模式、
# 讀 /mavros/state、rc_override。沒裝就退化成「只發速度指令」，照樣能開。
try:
    from mavros_msgs.msg import OverrideRCIn, State
    from mavros_msgs.srv import CommandBool, SetMode
    HAVE_MAVROS_MSGS = True
except ImportError:
    OverrideRCIn = State = CommandBool = SetMode = None
    HAVE_MAVROS_MSGS = False

# MAVROS state 是 best effort 發的，QoS 不合就收不到
STATE_QOS = QoSProfile(
    depth=10,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def clamp(value, low, high):
    return max(low, min(high, value))


def apply_deadzone(value, deadzone):
    """把搖桿中央的漂移吃掉，並把剩下的區間重新拉回 [-1, 1]。"""
    if abs(value) <= deadzone:
        return 0.0
    sign = math.copysign(1.0, value)
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


class Edge:
    """按鍵上升緣偵測，避免按一下被當成連按。"""

    def __init__(self):
        self._prev = {}

    def rising(self, key, pressed):
        was = self._prev.get(key, False)
        self._prev[key] = pressed
        return pressed and not was


class TriggerReader:
    """把類比扳機讀成 [0, 1]。

    常見的兩種回報方式：
      idle=+1.0, full=-1.0  典型的 joy_linux / xpad
      idle= 0.0, full=+1.0  部分驅動（含某些藍牙模式）

    第一種有個陷阱：driver 在「開機後扳機第一次被踩下去」之前會一直回 0.0，
    而 0.0 剛好落在區間正中間，直接換算會變成油門半開。所以要等看到第一個
    非 0 的值才開始相信讀數。

    第二種不需要（也不能用）那個保護 —— 0.0 本來就是它的 idle 值，
    再等下去油門會永遠是 0。所以只在 idle 不是 0 的時候才啟用保護。
    """

    def __init__(self, idle=1.0, full=-1.0):
        self.idle = idle
        self.full = full
        # idle 就是 0 的話，0.0 是合法讀數，不能拿它當「還沒讀到」的哨兵
        self._needs_first_event = (idle != 0.0)
        self._seen_event = False

    def read(self, raw):
        if self._needs_first_event and not self._seen_event:
            if raw == 0.0:
                return 0.0
            self._seen_event = True
        span = self.idle - self.full
        if span == 0.0:
            return 0.0
        return clamp((self.idle - raw) / span, 0.0, 1.0)


class JoyTeleopNode(Node):

    def __init__(self):
        super().__init__('bb_joy_teleop')

        # --- 按鍵/軸對應，預設是 Linux 下 Xbox 手把的排列 ---
        self.declare_parameter('axis_steer', 0)             # 左搖桿 左右 -> 轉向
        self.declare_parameter('axis_throttle_forward', 5)  # RT -> 前進
        self.declare_parameter('axis_throttle_reverse', 2)  # LT -> 後退
        self.declare_parameter('trigger_idle', 1.0)         # 扳機沒踩時的原始值
        self.declare_parameter('trigger_full', -1.0)        # 扳機踩到底的原始值

        self.declare_parameter('button_deadman', 5)    # RB 按住才會動
        self.declare_parameter('button_turbo', 4)      # LB 切慢速/全速
        self.declare_parameter('button_arm', 7)        # Start / Options
        self.declare_parameter('button_disarm', 6)     # Back / Create
        self.declare_parameter('button_mode_manual', 0)   # A / Cross
        self.declare_parameter('button_mode_hold', 1)     # B / Circle
        self.declare_parameter('button_mode_guided', 2)   # X / Square
        self.declare_parameter('button_mode_auto', 3)     # Y / Triangle

        self.declare_parameter('invert_steer', False)
        self.declare_parameter('invert_throttle', False)
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('trigger_deadzone', 0.05)

        # --- 輸出設定 ---
        self.declare_parameter('output_mode', 'twist')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('joy_timeout', 0.5)

        # twist：扳機踩到底時的速度。目前船上限速 0.5 m/s
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_reverse_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.5)  # rad/s

        # rc_override：ArduRover 預設 ch1 轉向、ch3 油門
        self.declare_parameter('steer_channel', 1)
        self.declare_parameter('throttle_channel', 3)
        self.declare_parameter('pwm_min', 1100)
        self.declare_parameter('pwm_mid', 1500)
        self.declare_parameter('pwm_max', 1900)

        self.declare_parameter('scale_low', 0.5)    # 慢速檔倍率
        self.declare_parameter('scale_high', 1.0)   # 全速檔倍率

        p = self.get_parameter
        self.axis_steer = p('axis_steer').value
        self.axis_fwd = p('axis_throttle_forward').value
        self.axis_rev = p('axis_throttle_reverse').value
        self.btn_deadman = p('button_deadman').value
        self.btn_turbo = p('button_turbo').value
        self.btn_arm = p('button_arm').value
        self.btn_disarm = p('button_disarm').value
        self.mode_buttons = {
            p('button_mode_manual').value: 'MANUAL',
            p('button_mode_hold').value: 'HOLD',
            p('button_mode_guided').value: 'GUIDED',
            p('button_mode_auto').value: 'AUTO',
        }
        self.invert_steer = p('invert_steer').value
        self.invert_throttle = p('invert_throttle').value
        self.deadzone = p('deadzone').value
        self.trigger_deadzone = p('trigger_deadzone').value
        self.output_mode = p('output_mode').value
        self.joy_timeout = p('joy_timeout').value
        self.max_linear = p('max_linear_speed').value
        self.max_reverse = p('max_reverse_speed').value
        self.max_angular = p('max_angular_speed').value
        self.steer_channel = p('steer_channel').value
        self.throttle_channel = p('throttle_channel').value
        self.pwm_min = p('pwm_min').value
        self.pwm_mid = p('pwm_mid').value
        self.pwm_max = p('pwm_max').value
        self.scale_low = p('scale_low').value
        self.scale_high = p('scale_high').value

        if self.output_mode not in ('twist', 'rc_override'):
            raise ValueError(f"未知的 output_mode: {self.output_mode}")
        if self.output_mode == 'rc_override' and not HAVE_MAVROS_MSGS:
            raise RuntimeError(
                'output_mode=rc_override 需要 mavros_msgs，但 import 不到。'
                ' 請 apt install ros-humble-mavros-msgs，或改用 output_mode=twist。')

        idle, full = p('trigger_idle').value, p('trigger_full').value
        self.trig_fwd = TriggerReader(idle, full)
        self.trig_rev = TriggerReader(idle, full)

        self.edge = Edge()
        self.last_joy = None
        self.last_joy_time = None
        self.state = State() if HAVE_MAVROS_MSGS else None
        self.turbo = False

        self.twist_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)

        # 以下全部要 mavros_msgs。沒有的話就只剩下發速度指令的能力。
        self.rc_pub = None
        self.arm_cli = None
        self.mode_cli = None
        if HAVE_MAVROS_MSGS:
            self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
            self.create_subscription(State, '/mavros/state', self._on_state, STATE_QOS)
            self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
            self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        else:
            self.get_logger().warn(
                'import 不到 mavros_msgs：解鎖/上鎖、切模式、rc_override 都停用，'
                '只會發速度指令到 /mavros/setpoint_velocity/cmd_vel_unstamped。'
                ' 要用那些功能請 apt install ros-humble-mavros-msgs。')

        period = 1.0 / max(1.0, p('publish_rate').value)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'bb_joy teleop 啟動：output_mode={self.output_mode}, '
            f'前進上限={self.max_linear} m/s, 後退上限={self.max_reverse} m/s, '
            f'deadman=button[{self.btn_deadman}]')

    # ------------------------------------------------------------------ 回呼

    def _on_state(self, msg):
        self.state = msg

    def _on_joy(self, msg):
        if self.last_joy is None:
            self.get_logger().info(
                f'第一筆 /joy：{len(msg.axes)} 軸 {len(msg.buttons)} 鍵\n'
                f'  axes    = {[round(a, 3) for a in msg.axes]}\n'
                f'  buttons = {list(msg.buttons)}\n'
                f'  轉向=axes[{self.axis_steer}] 前進=axes[{self.axis_fwd}] '
                f'後退=axes[{self.axis_rev}] deadman=buttons[{self.btn_deadman}]\n'
                f'  扳機沒踩時應該是 {self.trig_fwd.idle}；不是的話請改 '
                f'config 的 trigger_idle / trigger_full')
        self.last_joy = msg
        self.last_joy_time = self.get_clock().now()
        self._handle_buttons(msg)

    def _handle_buttons(self, joy):
        def pressed(idx):
            return 0 <= idx < len(joy.buttons) and joy.buttons[idx] == 1

        if self.edge.rising('arm', pressed(self.btn_arm)):
            self._request_arm(True)
        if self.edge.rising('disarm', pressed(self.btn_disarm)):
            self._request_arm(False)

        for idx, mode in self.mode_buttons.items():
            if self.edge.rising(f'mode{idx}', pressed(idx)):
                self._request_mode(mode)

        if self.edge.rising('turbo', pressed(self.btn_turbo)):
            self.turbo = not self.turbo
            self.get_logger().info(f'速度檔位：{"全速" if self.turbo else "慢速"}')

    # ------------------------------------------------------------------ 服務

    def _request_arm(self, value):
        if self.arm_cli is None:
            self.get_logger().warn('沒有 mavros_msgs，無法解鎖/上鎖')
            return
        if not self.arm_cli.service_is_ready():
            self.get_logger().warn('/mavros/cmd/arming 還沒起來')
            return
        req = CommandBool.Request()
        req.value = value
        future = self.arm_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'{"解鎖" if value else "上鎖"} 結果: {f.result().success}'))

    def _request_mode(self, mode):
        if self.mode_cli is None:
            self.get_logger().warn('沒有 mavros_msgs，無法切模式')
            return
        if not self.mode_cli.service_is_ready():
            self.get_logger().warn('/mavros/set_mode 還沒起來')
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        future = self.mode_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'切換到 {mode} 結果: {f.result().mode_sent}'))

    # ------------------------------------------------------------------ 主迴圈

    def _joy_is_fresh(self):
        if self.last_joy is None or self.last_joy_time is None:
            return False
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        return age <= self.joy_timeout

    def _read_axes(self):
        """回傳 (steer, throttle)。

        steer 範圍 [-1, 1]；throttle 正的是前進、負的是後退，同樣 [-1, 1]。
        沒握住 deadman 或訊號過期就回 (0, 0)。
        """
        joy = self.last_joy
        if not self._joy_is_fresh():
            return 0.0, 0.0

        deadman_ok = (0 <= self.btn_deadman < len(joy.buttons)
                      and joy.buttons[self.btn_deadman] == 1)
        if not deadman_ok:
            return 0.0, 0.0

        def axis(idx):
            return joy.axes[idx] if 0 <= idx < len(joy.axes) else 0.0

        steer = apply_deadzone(axis(self.axis_steer), self.deadzone)
        if self.invert_steer:
            steer = -steer

        forward = self.trig_fwd.read(axis(self.axis_fwd))
        reverse = self.trig_rev.read(axis(self.axis_rev))
        if forward < self.trigger_deadzone:
            forward = 0.0
        if reverse < self.trigger_deadzone:
            reverse = 0.0
        # 兩邊同時踩就互相抵銷，不會突然衝出去
        throttle = forward - reverse
        # 船裝反了或推進器極性相反時用這個翻，翻完再套前進/後退的限速
        if self.invert_throttle:
            throttle = -throttle

        scale = self.scale_high if self.turbo else self.scale_low
        return clamp(steer * scale, -1.0, 1.0), clamp(throttle * scale, -1.0, 1.0)

    def _tick(self):
        steer, throttle = self._read_axes()
        self._log_state(steer, throttle)
        if self.output_mode == 'twist':
            self._publish_twist(steer, throttle)
        else:
            self._publish_rc(steer, throttle)

    def _log_state(self, steer, throttle):
        """每秒回報一次，讓「為什麼船不動」可以一眼看出卡在哪。"""
        if self.last_joy is None:
            self.get_logger().warn(
                '還沒收到任何 /joy —— joy_linux_node 有在跑嗎？',
                throttle_duration_sec=2.0)
            return

        deadman = (0 <= self.btn_deadman < len(self.last_joy.buttons)
                   and self.last_joy.buttons[self.btn_deadman] == 1)
        if not deadman:
            hint = f'未按住 deadman (buttons[{self.btn_deadman}])'
        elif throttle == 0.0 and steer == 0.0:
            hint = '按住 deadman 但搖桿/扳機都是中立'
        else:
            hint = f'檔位={"全速" if self.turbo else "慢速"}'

        armed = getattr(self.state, 'armed', None)
        mode = getattr(self.state, 'mode', None)
        fc = '' if armed is None else f'  飛控: armed={armed} mode={mode}'
        self.get_logger().info(
            f'steer={steer:+.2f} throttle={throttle:+.2f}  {hint}{fc}',
            throttle_duration_sec=1.0)

    def _publish_twist(self, steer, throttle):
        msg = Twist()
        # 前進和後退分開限速，倒車通常要更保守
        limit = self.max_linear if throttle >= 0.0 else self.max_reverse
        msg.linear.x = throttle * limit
        msg.angular.z = -steer * self.max_angular   # 搖桿右推 = 右轉 = 負角速度
        self.twist_pub.publish(msg)

    def _to_pwm(self, value):
        span = (self.pwm_max - self.pwm_mid) if value >= 0 else (self.pwm_mid - self.pwm_min)
        return int(round(self.pwm_mid + value * span))

    def _publish_rc(self, steer, throttle):
        msg = OverrideRCIn()
        # 其餘通道保持不變，實體遙控器的開關照常有效
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * len(msg.channels)
        msg.channels[self.steer_channel - 1] = self._to_pwm(steer)
        msg.channels[self.throttle_channel - 1] = self._to_pwm(throttle)
        self.rc_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
