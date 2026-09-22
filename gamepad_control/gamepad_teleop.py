#!/usr/bin/env python3
"""
Flydigi Dune Fox 手柄遥控节点 - 差速驱动机器人
==================================================
专为高重心、大质量差速驱动底盘设计的安全遥控程序。

控制逻辑特点：
1. 死人开关 (Deadman Switch) - 必须按住才能运动，松开立即停止
2. 加速度限制 (Acceleration Limiting) - 平滑加减速，防止重心偏移/侧翻
3. 指数响应曲线 (Exponential Response) - 细腻低速控制 + 全速可达
4. 低通滤波 (Low-pass Filter) - 消除手柄抖动和突变输入
5. 转弯自动减速 (Speed-dependent Steering) - 高速时自动限制转弯角速度
6. 紧急停止按钮 (Emergency Stop) - 一键锁定停止
7. 多速度档位 (Speed Profiles) - 低速/中速/高速三档切换

手柄映射 (Flydigi Dune Fox - DInput 模式):
  左摇杆 Y 轴 (Axis 1) : 前进/后退 (线速度)
  右摇杆 X 轴 (Axis 2) : 左转/右转 (角速度)
  LT (Axis 5)           : 死人开关 (按下才能动)
  RT (Axis 4)           : 加速 (线性叠加)
  A  (Button 0)         : 紧急停止开关 (切换)
  X  (Button 2)         : 低速档
  Y  (Button 3)         : 中速档
  B  (Button 1)         : 高速档
  LB (Button 4)         : 原地左旋转
  RB (Button 5)         : 原地右旋转
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import struct
import os
import time
import math
import threading


class GamepadTeleop(Node):
    """手柄遥控节点"""

    # ═══════════════════════════════════════════════════════
    # JS 事件类型
    # ═══════════════════════════════════════════════════════
    JS_EVENT_BUTTON = 0x01
    JS_EVENT_AXIS = 0x02
    JS_EVENT_INIT = 0x80
    JS_EVENT_FORMAT = "IhBB"  # timestamp, value, type, number
    JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)

    def __init__(self):
        super().__init__('gamepad_teleop')

        # ═══════════════════════════════════════════════════
        # 声明参数
        # ═══════════════════════════════════════════════════
        self.declare_parameter('device', '/dev/input/js0')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('publish_rate', 50.0)  # Hz

        # 速度限制参数
        self.declare_parameter('max_linear_speed', 1.0)    # m/s forward
        self.declare_parameter('max_reverse_speed', 0.7)   # m/s reverse
        self.declare_parameter('max_angular_speed', 1.5)    # rad/s
        self.declare_parameter('min_linear_speed', 0.05)    # m/s 最小有效速度（消除蠕动）

        # 加速度限制参数 - 平滑加减速
        self.declare_parameter('max_linear_accel', 0.5)     # m/s² 最大线加速度
        self.declare_parameter('max_linear_decel', 1.0)     # m/s² 最大线减速度（刹车更快）
        self.declare_parameter('max_angular_accel', 1.5)    # rad/s² 最大角加速度
        self.declare_parameter('max_angular_decel', 3.0)    # rad/s² 最大角减速度

        # 响应曲线参数
        self.declare_parameter('expo_linear', 2.0)   # 指数曲线幂次（越大低速越细腻）
        self.declare_parameter('expo_angular', 1.8)
        self.declare_parameter('deadzone', 0.08)     # 摇杆死区

        # 低通滤波参数
        self.declare_parameter('filter_alpha', 0.3)  # 滤波系数 (0~1, 越小越平滑)

        # 转弯减速参数
        self.declare_parameter('steering_speed_factor', 0.6)  # 高速转弯时角速度缩减比例

        # 速度档位
        self.declare_parameter('speed_profile_low', 0.3)    # 低速档比例
        self.declare_parameter('speed_profile_mid', 0.6)    # 中速档比例
        self.declare_parameter('speed_profile_high', 1.0)   # 高速档比例

        # 原地旋转速度
        self.declare_parameter('spin_angular_speed', 0.8)   # rad/s

        # 功能开关
        self.declare_parameter('deadman_enabled', True)      # 是否启用死人开关
        self.declare_parameter('single_stick_mode', False)   # 单摇杆模式（左摇杆同时控制前后和转弯）

        # 轴映射参数（适配 Flydigi Dune Fox DInput 模式）
        self.declare_parameter('axis_linear', 1)     # 左摇杆Y轴 -> 线速度
        self.declare_parameter('axis_angular', 2)    # 右摇杆X轴 -> 角速度（单摇杆模式下自动切换为0）
        self.declare_parameter('axis_lt', 5)         # LT -> 死人开关
        self.declare_parameter('axis_rt', 4)         # RT -> 加速

        # 按钮映射
        self.declare_parameter('btn_estop', 0)       # A -> 紧急停止
        self.declare_parameter('btn_speed_low', 2)   # X -> 低速
        self.declare_parameter('btn_speed_mid', 3)   # Y -> 中速
        self.declare_parameter('btn_speed_high', 1)  # B -> 高速
        self.declare_parameter('btn_spin_left', 4)   # LB -> 原地左转
        self.declare_parameter('btn_spin_right', 5)  # RB -> 原地右转

        # ═══════════════════════════════════════════════════
        # 读取参数
        # ═══════════════════════════════════════════════════
        self.device = self.get_parameter('device').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_reverse = self.get_parameter('max_reverse_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.min_linear = self.get_parameter('min_linear_speed').value

        self.max_lin_accel = self.get_parameter('max_linear_accel').value
        self.max_lin_decel = self.get_parameter('max_linear_decel').value
        self.max_ang_accel = self.get_parameter('max_angular_accel').value
        self.max_ang_decel = self.get_parameter('max_angular_decel').value

        self.expo_lin = self.get_parameter('expo_linear').value
        self.expo_ang = self.get_parameter('expo_angular').value
        self.deadzone = self.get_parameter('deadzone').value
        self.filter_alpha = self.get_parameter('filter_alpha').value
        self.steer_speed_factor = self.get_parameter('steering_speed_factor').value

        self.speed_profiles = {
            'low': self.get_parameter('speed_profile_low').value,
            'mid': self.get_parameter('speed_profile_mid').value,
            'high': self.get_parameter('speed_profile_high').value,
        }
        self.spin_speed = self.get_parameter('spin_angular_speed').value

        # 功能开关
        self.deadman_enabled = self.get_parameter('deadman_enabled').value
        self.single_stick = self.get_parameter('single_stick_mode').value

        # 轴/按钮映射
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.axis_lt = self.get_parameter('axis_lt').value
        self.axis_rt = self.get_parameter('axis_rt').value

        # 单摇杆模式：角速度自动使用左摇杆 X 轴 (axis 0)
        if self.single_stick:
            self.axis_angular = 0
            self.get_logger().info('[模式] 单摇杆模式：左摇杆同时控制前后和转弯')

        self.btn_estop = self.get_parameter('btn_estop').value
        self.btn_speed_low = self.get_parameter('btn_speed_low').value
        self.btn_speed_mid = self.get_parameter('btn_speed_mid').value
        self.btn_speed_high = self.get_parameter('btn_speed_high').value
        self.btn_spin_left = self.get_parameter('btn_spin_left').value
        self.btn_spin_right = self.get_parameter('btn_spin_right').value

        # ═══════════════════════════════════════════════════
        # 状态变量
        # ═══════════════════════════════════════════════════
        self.axes = {}        # 摇杆轴原始值
        self.buttons = {}     # 按钮状态

        # 控制状态
        self.deadman_pressed = False   # 死人开关状态
        self.estop_active = False      # 紧急停止状态
        self.current_profile = 'mid'   # 当前速度档位
        self.spin_left = False         # 原地左转按下
        self.spin_right = False        # 原地右转按下

        # 滤波与速度状态
        self.filtered_linear = 0.0     # 滤波后的线速度指令
        self.filtered_angular = 0.0    # 滤波后的角速度指令
        self.current_linear = 0.0      # 当前实际发布的线速度（经加速度限制）
        self.current_angular = 0.0     # 当前实际发布的角速度
        self.last_publish_time = time.time()

        # 设备连接状态
        self.js_fd = None
        self.connected = False

        # ═══════════════════════════════════════════════════
        # ROS2 发布器与定时器
        # ═══════════════════════════════════════════════════
        topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_pub = self.create_publisher(Twist, topic, 10)
        self.status_pub = self.create_publisher(String, 'gamepad_status', 10)

        # 控制循环定时器
        dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(dt, self.control_loop)

        # 设备读取线程
        self.running = True
        self.read_thread = threading.Thread(target=self.read_device_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info('='*55)
        self.get_logger().info('  Gamepad Teleop 手柄遥控节点启动')
        self.get_logger().info(f'  设备: {self.device}')
        self.get_logger().info(f'  话题: {topic}')
        self.get_logger().info(f'  最大前进速度: {self.max_linear} m/s')
        self.get_logger().info(f'  最大倒车速度: {self.max_reverse} m/s')
        self.get_logger().info(f'  最大角速度: {self.max_angular} rad/s')
        self.get_logger().info(f'  当前档位: {self.current_profile}')
        self.get_logger().info(f'  死人开关: {"启用" if self.deadman_enabled else "禁用"}')
        self.get_logger().info(f'  操控模式: {"单摇杆(左手)" if self.single_stick else "双摇杆"}')
        self.get_logger().info('-'*55)
        self.get_logger().info('  操控方式:')
        if self.single_stick:
            self.get_logger().info('  左摇杆上下=前后 | 左摇杆左右=转弯')
        else:
            self.get_logger().info('  左摇杆=前后 | 右摇杆=转弯')
        if self.deadman_enabled:
            self.get_logger().info('  LT(按住)=使能(死人开关)')
        self.get_logger().info('  X=低速 Y=中速 B=高速 | A=紧急停止')
        self.get_logger().info('  LB=原地左转 RB=原地右转 | RT=加速')
        self.get_logger().info('='*55)

    # ═══════════════════════════════════════════════════════
    # 设备读取线程
    # ═══════════════════════════════════════════════════════
    def read_device_loop(self):
        """持续读取手柄输入的后台线程"""
        while self.running:
            if not self.connected:
                self.try_connect()
                if not self.connected:
                    time.sleep(1.0)
                    continue

            try:
                event = os.read(self.js_fd, self.JS_EVENT_SIZE)
                if len(event) < self.JS_EVENT_SIZE:
                    continue

                timestamp, value, etype, number = struct.unpack(
                    self.JS_EVENT_FORMAT, event)

                # 去除初始化标记
                etype &= ~self.JS_EVENT_INIT

                if etype == self.JS_EVENT_AXIS:
                    # 归一化到 [-1.0, 1.0]
                    self.axes[number] = value / 32767.0

                elif etype == self.JS_EVENT_BUTTON:
                    prev = self.buttons.get(number, 0)
                    self.buttons[number] = value

                    # 按钮事件处理（只在按下瞬间触发）
                    if value == 1 and prev == 0:
                        self.handle_button_press(number)

                    # 长按按钮状态更新
                    if number == self.btn_spin_left:
                        self.spin_left = bool(value)
                    elif number == self.btn_spin_right:
                        self.spin_right = bool(value)

            except OSError:
                self.get_logger().warn('手柄连接断开，尝试重新连接...')
                self.connected = False
                self.deadman_pressed = False
                if self.js_fd is not None:
                    try:
                        os.close(self.js_fd)
                    except OSError:
                        pass
                    self.js_fd = None

    def try_connect(self):
        """尝试连接手柄设备"""
        try:
            self.js_fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
            # 切换为阻塞模式以便线程阻塞等待
            import fcntl
            flags = fcntl.fcntl(self.js_fd, fcntl.F_GETFL)
            fcntl.fcntl(self.js_fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
            self.connected = True
            self.get_logger().info(f'手柄已连接: {self.device}')
        except OSError as e:
            self.connected = False
            self.js_fd = None

    def handle_button_press(self, button):
        """处理按钮按下事件（边沿触发）"""
        if button == self.btn_estop:
            self.estop_active = not self.estop_active
            state = "激活" if self.estop_active else "解除"
            self.get_logger().warn(f'[紧急停止] {state}!')

        elif button == self.btn_speed_low:
            self.current_profile = 'low'
            self.get_logger().info(f'[速度档位] 低速 ({self.speed_profiles["low"]*100:.0f}%)')

        elif button == self.btn_speed_mid:
            self.current_profile = 'mid'
            self.get_logger().info(f'[速度档位] 中速 ({self.speed_profiles["mid"]*100:.0f}%)')

        elif button == self.btn_speed_high:
            self.current_profile = 'high'
            self.get_logger().info(f'[速度档位] 高速 ({self.speed_profiles["high"]*100:.0f}%)')

    # ═══════════════════════════════════════════════════════
    # 输入处理函数
    # ═══════════════════════════════════════════════════════
    def apply_deadzone(self, value):
        """应用死区，消除摇杆微小漂移"""
        if abs(value) < self.deadzone:
            return 0.0
        # 重新映射：死区边缘到1.0 -> 0.0到1.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def apply_expo(self, value, expo):
        """
        应用指数响应曲线
        低输入 -> 非常小的输出（精细控制）
        高输入 -> 接近线性（全速可达）
        """
        sign = 1.0 if value >= 0 else -1.0
        return sign * (abs(value) ** expo)

    def apply_lowpass(self, current, target, alpha):
        """一阶低通滤波"""
        return current + alpha * (target - current)

    def apply_accel_limit(self, current, target, max_accel, max_decel, dt):
        """
        加速度限制器
        - 加速时限制加速度
        - 减速（刹车/松手）时允许更大减速度
        """
        diff = target - current
        if abs(diff) < 1e-6:
            return target

        # 判断是加速还是减速
        if abs(target) > abs(current) or (target * current < 0):
            # 加速或反向 -> 使用加速限制
            max_change = max_accel * dt
        else:
            # 减速 -> 使用减速限制（允许更快停下）
            max_change = max_decel * dt

        if abs(diff) <= max_change:
            return target
        else:
            return current + math.copysign(max_change, diff)

    # ═══════════════════════════════════════════════════════
    # 核心控制循环
    # ═══════════════════════════════════════════════════════
    def control_loop(self):
        """50Hz 控制主循环"""
        now = time.time()
        dt = now - self.last_publish_time
        self.last_publish_time = now

        # 防止 dt 异常
        dt = max(0.001, min(dt, 0.1))

        msg = Twist()

        # ───────────────────────────────────────────────
        # 1. 紧急停止检查
        # ───────────────────────────────────────────────
        if self.estop_active:
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.filtered_linear = 0.0
            self.filtered_angular = 0.0
            self.cmd_pub.publish(msg)
            self.publish_status("ESTOP")
            return

        # ───────────────────────────────────────────────
        # 2. 未连接或设备断开
        # ───────────────────────────────────────────────
        if not self.connected:
            self.current_linear = self.apply_accel_limit(
                self.current_linear, 0.0, self.max_lin_accel, self.max_lin_decel, dt)
            self.current_angular = self.apply_accel_limit(
                self.current_angular, 0.0, self.max_ang_accel, self.max_ang_decel, dt)
            msg.linear.x = self.current_linear
            msg.angular.z = self.current_angular
            self.cmd_pub.publish(msg)
            self.publish_status("DISCONNECTED")
            return

        # ───────────────────────────────────────────────
        # 3. 死人开关检查 (LT 扳机)
        # ───────────────────────────────────────────────
        if self.deadman_enabled:
            # LT 扳机: 未按=0 或 -1(取决于手柄), 完全按下=1
            lt_value = self.axes.get(self.axis_lt, 0.0)
            # Flydigi DInput: LT 范围通常从 -1(松开) 到 1(按下)
            # 有些手柄是0到1，兼容处理
            self.deadman_pressed = lt_value > 0.3
        else:
            # 禁用死人开关时始终视为按下
            self.deadman_pressed = True

        if not self.deadman_pressed:
            # 松开死人开关 -> 平滑刹车到零
            target_linear = 0.0
            target_angular = 0.0

            self.filtered_linear = self.apply_lowpass(
                self.filtered_linear, target_linear, 0.5)
            self.filtered_angular = self.apply_lowpass(
                self.filtered_angular, target_angular, 0.5)

            self.current_linear = self.apply_accel_limit(
                self.current_linear, self.filtered_linear,
                self.max_lin_accel, self.max_lin_decel, dt)
            self.current_angular = self.apply_accel_limit(
                self.current_angular, self.filtered_angular,
                self.max_ang_accel, self.max_ang_decel, dt)

            msg.linear.x = self.current_linear if abs(self.current_linear) > 0.01 else 0.0
            msg.angular.z = self.current_angular if abs(self.current_angular) > 0.01 else 0.0
            self.cmd_pub.publish(msg)
            self.publish_status("STANDBY")
            return

        # ───────────────────────────────────────────────
        # 4. 原地旋转模式
        # ───────────────────────────────────────────────
        if self.spin_left or self.spin_right:
            target_linear = 0.0
            target_angular = 0.0
            profile_scale = self.speed_profiles[self.current_profile]

            if self.spin_left:
                target_angular = self.spin_speed * profile_scale
            elif self.spin_right:
                target_angular = -self.spin_speed * profile_scale

            self.filtered_linear = self.apply_lowpass(
                self.filtered_linear, target_linear, self.filter_alpha)
            self.filtered_angular = self.apply_lowpass(
                self.filtered_angular, target_angular, self.filter_alpha)

            self.current_linear = self.apply_accel_limit(
                self.current_linear, self.filtered_linear,
                self.max_lin_accel, self.max_lin_decel, dt)
            self.current_angular = self.apply_accel_limit(
                self.current_angular, self.filtered_angular,
                self.max_ang_accel, self.max_ang_decel, dt)

            msg.linear.x = self.current_linear
            msg.angular.z = self.current_angular
            self.cmd_pub.publish(msg)
            self.publish_status("SPIN")
            return

        # ───────────────────────────────────────────────
        # 5. 正常遥控模式
        # ───────────────────────────────────────────────
        # 读取摇杆原始值
        raw_linear = -self.axes.get(self.axis_linear, 0.0)   # 反转Y轴（推杆向前为负）
        raw_angular = -self.axes.get(self.axis_angular, 0.0)  # 反转X轴（向左为正）

        # RT 加速叠加
        rt_value = self.axes.get(self.axis_rt, 0.0)
        # RT: -1(松开) -> 1(按下)，映射为 boost: 0 -> 0.3
        rt_boost = max(0.0, (rt_value + 1.0) / 2.0) * 0.3

        # (a) 死区
        raw_linear = self.apply_deadzone(raw_linear)
        raw_angular = self.apply_deadzone(raw_angular)

        # (b) 指数响应曲线
        raw_linear = self.apply_expo(raw_linear, self.expo_lin)
        raw_angular = self.apply_expo(raw_angular, self.expo_ang)

        # (c) 速度档位缩放
        profile_scale = self.speed_profiles[self.current_profile]

        # (d) 计算目标速度
        linear_limit = self.max_linear if raw_linear >= 0.0 else self.max_reverse
        target_linear = raw_linear * linear_limit * (profile_scale + rt_boost)
        target_angular = raw_angular * self.max_angular * profile_scale

        # (e) 转弯自动减速 —— 车速越快，角速度限制越严
        #     这对于高重心车辆非常重要，防止高速急转弯侧翻
        active_linear_limit = self.max_linear if self.current_linear >= 0.0 else self.max_reverse
        speed_ratio = abs(self.current_linear) / active_linear_limit if active_linear_limit > 0 else 0.0
        angular_limit_factor = 1.0 - speed_ratio * (1.0 - self.steer_speed_factor)
        target_angular *= angular_limit_factor

        # (f) 速度下限过滤（消除蠕动）
        if 0 < abs(target_linear) < self.min_linear:
            target_linear = 0.0

        # (g) 低通滤波 —— 平滑摇杆输入
        self.filtered_linear = self.apply_lowpass(
            self.filtered_linear, target_linear, self.filter_alpha)
        self.filtered_angular = self.apply_lowpass(
            self.filtered_angular, target_angular, self.filter_alpha)

        # (h) 加速度限制 —— 防止加速/减速过猛
        self.current_linear = self.apply_accel_limit(
            self.current_linear, self.filtered_linear,
            self.max_lin_accel, self.max_lin_decel, dt)
        self.current_angular = self.apply_accel_limit(
            self.current_angular, self.filtered_angular,
            self.max_ang_accel, self.max_ang_decel, dt)

        # (i) 最终安全限幅
        self.current_linear = max(-self.max_reverse, min(self.max_linear, self.current_linear))
        self.current_angular = max(-self.max_angular, min(self.max_angular, self.current_angular))

        # (j) 微小值归零
        if abs(self.current_linear) < 0.01:
            self.current_linear = 0.0
        if abs(self.current_angular) < 0.01:
            self.current_angular = 0.0

        # 发布
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.cmd_pub.publish(msg)
        self.publish_status("ACTIVE")

    def publish_status(self, state):
        """发布状态信息（低频率）"""
        # 只每10次循环发一次状态，减少带宽
        if not hasattr(self, '_status_counter'):
            self._status_counter = 0
        self._status_counter += 1

        if self._status_counter % 10 == 0:
            status_msg = String()
            profile_pct = self.speed_profiles[self.current_profile] * 100
            status_msg.data = (
                f"[{state}] "
                f"档位:{self.current_profile}({profile_pct:.0f}%) "
                f"线速:{self.current_linear:+.3f}m/s "
                f"角速:{self.current_angular:+.3f}rad/s "
                f"死人开关:{'ON' if self.deadman_pressed else 'OFF'}"
            )
            self.status_pub.publish(status_msg)

    def destroy_node(self):
        """清理资源"""
        self.running = False
        if self.js_fd is not None:
            try:
                os.close(self.js_fd)
            except OSError:
                pass

        # 发送零速停车指令
        msg = Twist()
        self.cmd_pub.publish(msg)
        self.get_logger().info('手柄遥控节点已关闭，已发送停车指令')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GamepadTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
