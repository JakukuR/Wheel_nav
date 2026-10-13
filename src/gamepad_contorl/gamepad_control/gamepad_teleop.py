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
import select
import ctypes
import errno
from pathlib import Path


class DevicePathEvents:
    """Wait for Linux device-path changes, including udev symlink/parent creation."""

    MASK = 0x00000004 | 0x00000040 | 0x00000080 | 0x00000100 | 0x00000200 | 0x00000400 | 0x00000800
    HEADER = struct.Struct('iIII')

    def __init__(self, device):
        self.device = Path(os.path.abspath(device))
        self.libc = ctypes.CDLL(None, use_errno=True)
        self.libc.inotify_init1.argtypes = [ctypes.c_int]
        self.libc.inotify_init1.restype = ctypes.c_int
        self.libc.inotify_add_watch.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_uint32]
        self.libc.inotify_add_watch.restype = ctypes.c_int
        self.libc.inotify_rm_watch.argtypes = [ctypes.c_int, ctypes.c_int]
        self.libc.inotify_rm_watch.restype = ctypes.c_int
        self.fd = self.libc.inotify_init1(os.O_NONBLOCK | os.O_CLOEXEC)
        if self.fd < 0:
            raise OSError(ctypes.get_errno(), 'cannot watch joystick device events')
        self.watches = {}
        try:
            self.refresh()
        except OSError:
            self.close()
            raise

    def refresh(self):
        wanted = {}
        # Also watch the symlink target: udev may grant access after creating it.
        for path in (self.device, Path(os.path.realpath(self.device))):
            while path.parent != path:
                wanted.setdefault(path.parent, set()).add(path.name)
                path = path.parent
        watches = {}
        for directory, names in wanted.items():
            wd = self.libc.inotify_add_watch(self.fd, os.fsencode(directory), self.MASK)
            if wd < 0:
                error = ctypes.get_errno()
                if error in (errno.ENOENT, errno.ENOTDIR):
                    continue
                raise OSError(error, 'cannot watch ' + str(directory))
            watches.setdefault(wd, set()).update(names)
        for wd in self.watches.keys() - watches.keys():
            self.libc.inotify_rm_watch(self.fd, wd)
        self.watches = watches

    def wait(self, stop_event):
        while not stop_event.is_set():
            # Timeout only checks shutdown; it never triggers a connection attempt.
            if not select.select([self.fd], [], [], .2)[0]:
                continue
            data = os.read(self.fd, 65536)
            offset = 0; changed = False
            while offset < len(data):
                wd, mask, _, length = self.HEADER.unpack_from(data, offset)
                offset += self.HEADER.size
                name = os.fsdecode(data[offset:offset + length].split(b'\0', 1)[0])
                offset += length
                names = self.watches.get(wd)
                if mask & 0x00004000 or (names is not None and
                        (name in names or mask & (0x00000400 | 0x00000800))):
                    changed = True
            if changed:
                self.refresh()
                return True
        return False

    def close(self):
        os.close(self.fd)


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
        self.input_lock = threading.RLock()
        self.stop_event = threading.Event()
        self.last_wait_log = -float("inf")

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
    def reset_input(self):
        """Clear commands across unplug/replug; preserve the emergency-stop latch."""
        self.axes.clear()
        self.buttons.clear()
        self.deadman_pressed = False
        self.spin_left = self.spin_right = False
        self.filtered_linear = self.filtered_angular = 0.0
        self.current_linear = self.current_angular = 0.0

    def disconnect_device(self):
        with self.input_lock:
            self.connected = False
            self.reset_input()
            if self.js_fd is not None:
                try:
                    os.close(self.js_fd)
                except OSError:
                    pass
                self.js_fd = None

    def read_device_loop(self):
        """Keep zero output while waiting for device events; no timed reconnect."""
        events = None
        try:
            events = DevicePathEvents(self.device)
            self._read_device_events(events)
        except OSError as error:
            self.disconnect_device()
            self.get_logger().error(f'手柄设备事件监听失败，保持零速: {error}')
            self.stop_event.wait()
        finally:
            if events is not None:
                events.close()

    def _read_device_events(self, events):
        attempt = True  # One initial open handles an already attached controller.
        while self.running and not self.stop_event.is_set():
            if not self.connected:
                if attempt:
                    self.try_connect()
                    attempt = False
                if not self.connected:
                    attempt = events.wait(self.stop_event)
                    continue
            try:
                ready, _, _ = select.select([self.js_fd], [], [], 0.2)
                if not ready:
                    continue
                event = os.read(self.js_fd, self.JS_EVENT_SIZE)
                if len(event) != self.JS_EVENT_SIZE:
                    raise OSError('joystick stream closed or incomplete')
                _, value, etype, number = struct.unpack(self.JS_EVENT_FORMAT, event)
                initial = bool(etype & self.JS_EVENT_INIT)
                etype &= ~self.JS_EVENT_INIT
                with self.input_lock:
                    if etype == self.JS_EVENT_AXIS:
                        self.axes[number] = value / 32767.0
                    elif etype == self.JS_EVENT_BUTTON:
                        prev = self.buttons.get(number, 0)
                        self.buttons[number] = value
                        # Initial button snapshots must not toggle the estop latch.
                        if not initial and value == 1 and prev == 0:
                            self.handle_button_press(number)
                        if number == self.btn_spin_left:
                            self.spin_left = bool(value)
                        elif number == self.btn_spin_right:
                            self.spin_right = bool(value)
            except BlockingIOError:
                continue
            except OSError:
                self.disconnect_device()
                if not self.stop_event.is_set():
                    self.get_logger().warn('手柄连接断开，输出归零，等待手柄重新接入。')

    def try_connect(self):
        """A missing or temporarily inaccessible device is a waiting state."""
        try:
            fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as error:
            self.disconnect_device()
            now = time.monotonic()
            if now - self.last_wait_log >= 10.0:
                self.get_logger().info(
                    f'等待手柄接入: {self.device} ({error.strerror})；等待设备接入或权限变化事件，输出保持零速。')
                self.last_wait_log = now
            return
        with self.input_lock:
            self.reset_input()
            self.js_fd = fd
            self.connected = True
        self.last_wait_log = -float("inf")
        self.get_logger().info(f'手柄已连接: {self.device}；请先松开摇杆回中。')

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
        with self.input_lock:
            self._control_loop()

    def _control_loop(self):
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
            self.reset_input()
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
        """Stop the reader before closing its descriptor and ROS publishers."""
        self.running = False
        self.stop_event.set()
        self.read_thread.join(timeout=2.0)
        self.disconnect_device()
        if rclpy.ok(context=self.context):
            self.cmd_pub.publish(Twist())
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
