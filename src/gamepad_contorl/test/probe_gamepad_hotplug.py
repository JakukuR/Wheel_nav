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
    device = Path(directory) / 'input' / 'by-id' / 'joystick'
    rclpy.init(args=['--ros-args', '-p', f'device:={device}',
                     '-p', 'cmd_vel_topic:=/test/gamepad_cmd', '-p', 'deadman_enabled:=false'])
    connect_attempts = []
    original_connect = module.GamepadTeleop.try_connect

    def counted_connect(self):
        connect_attempts.append(time.monotonic())
        return original_connect(self)

    module.GamepadTeleop.try_connect = counted_connect
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
        assert len(connect_attempts) == 1, 'no periodic open while absent'
        (Path(directory) / 'unrelated').touch()
        pump(1.2)
        assert len(connect_attempts) == 1, 'unrelated filesystem events must not retry'
        print('PASS absent at startup: alive, zero output, one open only; unrelated events ignored', flush=True)
        device.parent.mkdir(parents=True)

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

        # A failed event-triggered open waits for a new event, not a timer.
        with patch.object(module.os, 'open', side_effect=PermissionError(13, 'Permission denied')):
            before = len(connect_attempts)
            os.mkfifo(device)
            until(lambda: len(connect_attempts) > before)
            attempts = len(connect_attempts)
            pump(1.2)
            assert len(connect_attempts) == attempts, 'permission failure must not cause polling'
            assert not node.connected and node.read_thread.is_alive()
        print('PASS inaccessible device: no timed retries', flush=True)

        writer = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        os.chmod(device, 0o600)  # Permission event triggers the next connection.
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

        device.unlink(); os.close(writer); writer = None
        until(lambda: not node.connected)
        device.parent.rmdir(); device.parent.parent.rmdir()
        pump(.3)
        device.parent.mkdir(parents=True)
        # Exercise the production by-id symlink and target-permission path.
        target = device.parent.parent / 'js0'
        os.mkfifo(target)
        writer = os.open(target, os.O_RDWR | os.O_NONBLOCK)
        with patch.object(module.os, 'open', side_effect=PermissionError(13, 'Permission denied')):
            before = len(connect_attempts)
            device.symlink_to(target)
            until(lambda: len(connect_attempts) > before)
            pump(.3)
            assert not node.connected
        os.chmod(target, 0o600)
        until(lambda: node.connected)
        pump(.3)
        assert node.estop_active and messages[-1] == (0., 0.)
        print('PASS recreated parent directories, by-id symlink and target permission event', flush=True)
    finally:
        if writer is not None:
            os.close(writer)
        # Reproduce SIGINT shutdown ordering: the context may already be gone.
        rclpy.shutdown()
        node.destroy_node()
        assert not node.read_thread.is_alive()
        print('PASS shutdown after ROS context closes', flush=True)
