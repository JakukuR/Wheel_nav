#!/usr/bin/env python3
"""
键盘模拟手柄测试节点
====================
在没有手柄的情况下，用键盘模拟手柄输入测试 cmd_vel。
使用与 gamepad_teleop 相同的安全控制管线（加速度限制、低通滤波等）。

按键映射:
  W/S       : 前进/后退（模拟左摇杆 Y 轴）
  A/D       : 左转/右转（模拟摇杆 X 轴）
  Q/E       : 原地左转/原地右转
  1/2/3     : 低速/中速/高速 档位
  空格      : 紧急停止切换
  Esc       : 退出

按键按下=摇杆推满，松开=归零，所有安全逻辑（加速度限制、滤波）照常生效。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import os
import time
import math
import termios
import tty
import select
import threading


class KeyboardSimTeleop(Node):
    """键盘模拟手柄遥控节点"""

    def __init__(self):
        super().__init__('keyboard_sim_teleop')

        # 参数
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('max_linear_decel', 1.0)
        self.declare_parameter('max_angular_accel', 1.5)
        self.declare_parameter('max_angular_decel', 3.0)
        self.declare_parameter('filter_alpha', 0.3)
        self.declare_parameter('steering_speed_factor', 0.6)
        self.declare_parameter('spin_angular_speed', 0.8)

        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.max_lin_accel = self.get_parameter('max_linear_accel').value
        self.max_lin_decel = self.get_parameter('max_linear_decel').value
        self.max_ang_accel = self.get_parameter('max_angular_accel').value
        self.max_ang_decel = self.get_parameter('max_angular_decel').value
        self.filter_alpha = self.get_parameter('filter_alpha').value
        self.steer_speed_factor = self.get_parameter('steering_speed_factor').value
        self.spin_speed = self.get_parameter('spin_angular_speed').value

        self.speed_profiles = {'low': 0.3, 'mid': 0.6, 'high': 1.0}
        self.current_profile = 'mid'
        self.estop_active = False

        # 模拟摇杆状态
        self.sim_linear = 0.0    # -1 ~ 1 模拟摇杆
        self.sim_angular = 0.0
        self.sim_spin_left = False
        self.sim_spin_right = False

        # 控制状态
        self.filtered_linear = 0.0
        self.filtered_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_time = time.time()

        # 按键状态
        self.keys_pressed = set()

        # 发布器
        topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_pub = self.create_publisher(Twist, topic, 10)
        self.status_pub = self.create_publisher(String, 'gamepad_status', 10)

        # 控制定时器
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        # 键盘读取线程
        self.running = True
        self.key_thread = threading.Thread(target=self.read_keyboard, daemon=True)
        self.key_thread.start()

        self.print_usage()

    def print_usage(self):
        """打印使用说明"""
        self.get_logger().info('='*50)
        self.get_logger().info('  键盘模拟手柄 - 测试节点')
        self.get_logger().info('='*50)
        self.get_logger().info('  W/S = 前进/后退')
        self.get_logger().info('  A/D = 左转/右转')
        self.get_logger().info('  Q/E = 原地左转/原地右转')
        self.get_logger().info('  1/2/3 = 低速/中速/高速')
        self.get_logger().info('  空格 = 紧急停止')
        self.get_logger().info('  Esc = 退出')
        self.get_logger().info(f'  当前档位: {self.current_profile}')
        self.get_logger().info('='*50)
        print('\n\033[1;32m>>> 按 W/A/S/D 控制，按住不放 = 持续运动，松开 = 停止 <<<\033[0m\n')

    def read_keyboard(self):
        """后台线程：读取键盘输入"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    ch = sys.stdin.read(1)
                    if ch == '\x1b':  # ESC
                        self.running = False
                        break
                    self.handle_key(ch.lower())
                else:
                    # 没有按键按下，清除所有方向键状态
                    self.sim_linear = 0.0
                    self.sim_angular = 0.0
                    self.sim_spin_left = False
                    self.sim_spin_right = False
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def handle_key(self, key):
        """处理按键"""
        if key == 'w':
            self.sim_linear = 1.0
            self.sim_angular = 0.0
        elif key == 's':
            self.sim_linear = -1.0
            self.sim_angular = 0.0
        elif key == 'a':
            self.sim_angular = 1.0
            self.sim_linear = 0.0 if abs(self.sim_linear) < 0.1 else self.sim_linear
        elif key == 'd':
            self.sim_angular = -1.0
            self.sim_linear = 0.0 if abs(self.sim_linear) < 0.1 else self.sim_linear
        elif key == 'q':
            self.sim_spin_left = True
            self.sim_spin_right = False
        elif key == 'e':
            self.sim_spin_right = True
            self.sim_spin_left = False
        elif key == '1':
            self.current_profile = 'low'
            self.get_logger().info('[档位] 低速 (30%)')
        elif key == '2':
            self.current_profile = 'mid'
            self.get_logger().info('[档位] 中速 (60%)')
        elif key == '3':
            self.current_profile = 'high'
            self.get_logger().info('[档位] 高速 (100%)')
        elif key == ' ':
            self.estop_active = not self.estop_active
            state = "激活" if self.estop_active else "解除"
            self.get_logger().warn(f'[紧急停止] {state}!')

    def apply_lowpass(self, current, target, alpha):
        return current + alpha * (target - current)

    def apply_accel_limit(self, current, target, max_accel, max_decel, dt):
        diff = target - current
        if abs(diff) < 1e-6:
            return target
        if abs(target) > abs(current) or (target * current < 0):
            max_change = max_accel * dt
        else:
            max_change = max_decel * dt
        if abs(diff) <= max_change:
            return target
        return current + math.copysign(max_change, diff)

    def control_loop(self):
        """控制主循环 - 与 gamepad_teleop 相同的安全管线"""
        now = time.time()
        dt = max(0.001, min(now - self.last_time, 0.1))
        self.last_time = now

        msg = Twist()

        if not self.running:
            rclpy.shutdown()
            return

        # 紧急停止
        if self.estop_active:
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.filtered_linear = 0.0
            self.filtered_angular = 0.0
            self.cmd_pub.publish(msg)
            return

        profile_scale = self.speed_profiles[self.current_profile]

        # 原地旋转
        if self.sim_spin_left or self.sim_spin_right:
            target_linear = 0.0
            target_angular = self.spin_speed * profile_scale * (1.0 if self.sim_spin_left else -1.0)
        else:
            # 正常输入
            target_linear = self.sim_linear * self.max_linear * profile_scale
            target_angular = self.sim_angular * self.max_angular * profile_scale

            # 转弯自动减速
            speed_ratio = abs(self.current_linear) / self.max_linear if self.max_linear > 0 else 0
            angular_limit = 1.0 - speed_ratio * (1.0 - self.steer_speed_factor)
            target_angular *= angular_limit

            # 后退减速
            if target_linear < 0:
                target_linear *= 0.6

        # 低通滤波
        self.filtered_linear = self.apply_lowpass(self.filtered_linear, target_linear, self.filter_alpha)
        self.filtered_angular = self.apply_lowpass(self.filtered_angular, target_angular, self.filter_alpha)

        # 加速度限制
        self.current_linear = self.apply_accel_limit(
            self.current_linear, self.filtered_linear,
            self.max_lin_accel, self.max_lin_decel, dt)
        self.current_angular = self.apply_accel_limit(
            self.current_angular, self.filtered_angular,
            self.max_ang_accel, self.max_ang_decel, dt)

        # 限幅 + 微小值归零
        self.current_linear = max(-self.max_linear, min(self.max_linear, self.current_linear))
        self.current_angular = max(-self.max_angular, min(self.max_angular, self.current_angular))
        if abs(self.current_linear) < 0.01:
            self.current_linear = 0.0
        if abs(self.current_angular) < 0.01:
            self.current_angular = 0.0

        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.cmd_pub.publish(msg)

        # 实时打印（每10帧）
        if not hasattr(self, '_cnt'):
            self._cnt = 0
        self._cnt += 1
        if self._cnt % 10 == 0:
            profile_pct = profile_scale * 100
            bar_lin = '█' * int(abs(self.current_linear) / self.max_linear * 20)
            bar_ang = '█' * int(abs(self.current_angular) / self.max_angular * 20)
            dir_lin = '↑' if self.current_linear > 0 else ('↓' if self.current_linear < 0 else '·')
            dir_ang = '←' if self.current_angular > 0 else ('→' if self.current_angular < 0 else '·')
            print(f'\r  [{self.current_profile}:{profile_pct:.0f}%] '
                  f'线速:{self.current_linear:+.3f} {dir_lin}{bar_lin:<20s} | '
                  f'角速:{self.current_angular:+.3f} {dir_ang}{bar_ang:<20s}  ', end='', flush=True)

    def destroy_node(self):
        self.running = False
        msg = Twist()
        self.cmd_pub.publish(msg)
        self.get_logger().info('键盘模拟节点已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardSimTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
