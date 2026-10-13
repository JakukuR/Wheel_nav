"""Exercise real ROS timers and the input thread on a synthetic stream only."""
import importlib.util
import os
from pathlib import Path
import struct
import tempfile
import time
from unittest.mock import patch

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

spec = importlib.util.spec_from_file_location('candidate_gamepad', Path(__file__).resolve().parents[1] / 'gamepad_control/gamepad_teleop.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

assert os.environ.get('ROS_DOMAIN_ID') == '174', 'Run this synthetic probe in ROS_DOMAIN_ID=174 only'

with tempfile.TemporaryDirectory() as directory:
    device = Path(directory) / 'joystick'
    rclpy.init(args=['--ros-args', '-p', f'device:={device}',
                     '-p', 'cmd_vel_topic:=/test/gamepad_cmd', '-p', 'deadman_enabled:=false'])
    node = module.GamepadTeleop()
    messages, statuses = [], []
    node.create_subscription(Twist, '/test/gamepad_cmd', lambda m: messages.append((m.linear.x, m.angular.z)), 10)
    node.create_subscription(String, 'gamepad_status', lambda m: statuses.append(m.data), 10)

    def pump(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def until(predicate, timeout=4):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            pump(.04)
            if predicate():
                return
        raise AssertionError('Timed out waiting for expected state')

    def event(fd, value, kind, number):
        os.write(fd, struct.pack('IhBB', 0, value, kind, number))

    writer = None
    try:
        pump(1.5)
        assert not node.connected and node.read_thread.is_alive()
        assert messages and all(v == (0., 0.) for v in messages)
        assert statuses and statuses[-1].startswith('[DISCONNECTED]')
        print('PASS absent at startup: alive, waiting, zero output', flush=True)

        os.mkfifo(device)
        writer = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        until(lambda: node.connected)
        event(writer, 0, 0x82, node.axis_linear)
        event(writer, -32767, 0x82, node.axis_rt)
        event(writer, 1, 0x81, node.btn_estop)
        until(lambda: node.buttons.get(node.btn_estop) == 1)
        assert not node.estop_active
        event(writer, 1, 0x01, node.btn_spin_left)
        until(lambda: messages and abs(messages[-1][1]) > .01)
        print('PASS delayed connection: input resumes; INIT does not toggle estop', flush=True)

        device.unlink()
        os.close(writer)
        writer = None
        until(lambda: not node.connected)
        pump(.3)
        assert messages[-1] == (0., 0.) and statuses[-1].startswith('[DISCONNECTED]')
        assert not node.axes and not node.buttons and not node.spin_left
        assert node.current_linear == node.current_angular == 0.
        assert node.read_thread.is_alive()
        print('PASS unplug/EOF: zero output, old input cleared, reader alive', flush=True)

        # Exercise a temporarily inaccessible device without stopping the thread.
        with patch.object(module.os, 'open', side_effect=PermissionError(13, 'Permission denied')):
            pump(1.2)
            assert not node.connected and node.read_thread.is_alive()
        print('PASS inaccessible device: waiting continues', flush=True)

        os.mkfifo(device)
        writer = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        until(lambda: node.connected)
        pump(.4)
        assert messages[-1] == (0., 0.) and not node.spin_left
        event(writer, 0, 0x81, node.btn_estop)
        event(writer, 1, 0x01, node.btn_estop)
        until(lambda: node.estop_active)
        device.unlink()
        os.close(writer)
        writer = None
        until(lambda: not node.connected)
        os.mkfifo(device)
        writer = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        until(lambda: node.connected)
        event(writer, 1, 0x81, node.btn_estop)
        pump(.3)
        assert node.estop_active and messages[-1] == (0., 0.)
        print('PASS repeated reconnect: no stale rotation; estop latch preserved', flush=True)
    finally:
        if writer is not None:
            os.close(writer)
        # Reproduce SIGINT shutdown ordering: the context may already be gone.
        rclpy.shutdown()
        node.destroy_node()
        assert not node.read_thread.is_alive()
        print('PASS shutdown after ROS context closes', flush=True)
