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
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy

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

    joy_linux 的扳機沒踩是 +1.0、踩到底是 -1.0，但是在「開機後第一次踩下去」
    之前，driver 會一直回 0.0 —— 那個 0.0 是「還沒讀到」不是「踩一半」。
    所以在第一次看到非零值之前，一律當成沒踩。
    """

    def __init__(self, idle=1.0, full=-1.0):
        self.idle = idle
        self.full = full
        self._initialised = False

    def read(self, raw):
        if not self._initialised:
            if raw == 0.0:
                return 0.0
            self._initialised = True
        span = self.idle - self.full
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

        idle, full = p('trigger_idle').value, p('trigger_full').value
        self.trig_fwd = TriggerReader(idle, full)
        self.trig_rev = TriggerReader(idle, full)

        self.edge = Edge()
        self.last_joy = None
        self.last_joy_time = None
        self.state = State()
        self.turbo = False

        self.twist_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.create_subscription(State, '/mavros/state', self._on_state, STATE_QOS)

        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')

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

        scale = self.scale_high if self.turbo else self.scale_low
        return clamp(steer * scale, -1.0, 1.0), clamp(throttle * scale, -1.0, 1.0)

    def _tick(self):
        steer, throttle = self._read_axes()
        if self.output_mode == 'twist':
            self._publish_twist(steer, throttle)
        else:
            self._publish_rc(steer, throttle)

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
