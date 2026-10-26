"""Kalo debug page in an isolated domain: the camera stream subscribes only
while viewed, keeps <=10 Hz, emits valid JPEG with the camera header, and the
static server answers its config. No camera, chassis or motion."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

BASE = Path(__file__).resolve().parents[1]
LIB = Path(os.environ.get('KALO_LIB', '/home/orin/ros2_ws/install/wla_r680_navigation/lib/wla_r680_navigation'))


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '73'), 'run in an isolated test domain'
    rclpy.init()
    node = rclpy.create_node('kalo_test_fixture')
    camera = node.create_publisher(Image, '/r680/d455/color/image_raw', qos_profile_sensor_data)
    jpeg = subprocess.Popen([str(LIB/'kalo_jpeg'), '--ros-args', '-r', '__node:=kalo_jpeg'],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    port = free_port()
    web = subprocess.Popen([sys.executable, str(BASE/'scripts/kalo_debug_web'), '--host', '127.0.0.1', '--port', str(port)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    frame = np.zeros((480, 640, 3), np.uint8); frame[:, :320] = (255, 120, 40)
    received = []

    def pump(seconds):
        end = time.monotonic() + seconds; due = 0.
        while time.monotonic() < end:
            if time.monotonic() >= due:
                due = time.monotonic() + 1 / 30
                msg = Image(); msg.header.stamp = node.get_clock().now().to_msg(); msg.header.frame_id = 'd455_color_optical_frame'
                msg.height, msg.width, msg.encoding, msg.step = 480, 640, 'rgb8', 1920; msg.data = frame.tobytes()
                camera.publish(msg)
            rclpy.spin_once(node, timeout_sec=.005)

    def camera_readers():
        return [i.node_name for i in node.get_subscriptions_info_by_topic('/r680/d455/color/image_raw')]

    try:
        pump(2.)
        assert 'kalo_jpeg' not in camera_readers(), 'camera subscribed with no viewer'
        sub = node.create_subscription(CompressedImage, '/r680/debug/color/compressed', received.append, qos_profile_sensor_data)
        pump(2.)
        assert 'kalo_jpeg' in camera_readers(), 'viewer did not start the camera subscription'
        received.clear(); start = time.monotonic(); pump(4.)
        rate = len(received) / (time.monotonic() - start)
        assert 7. <= rate <= 11., f'rate {rate:.1f} Hz'
        last = received[-1]
        assert last.format == 'jpeg' and last.header.frame_id == 'd455_color_optical_frame'
        image = cv2.imdecode(np.frombuffer(bytes(last.data), np.uint8), cv2.IMREAD_COLOR)
        assert image.shape == (480, 640, 3)
        left, right = image[:, :300].mean(axis=(0, 1)), image[:, 340:].mean(axis=(0, 1))
        assert abs(left[2] - 255) < 12 and abs(left[0] - 40) < 12 and right.max() < 12, (left, right)  # RGB kept (BGR decode)
        node.destroy_subscription(sub); pump(2.)
        assert 'kalo_jpeg' not in camera_readers(), 'camera still subscribed after the viewer left'
        with urllib.request.urlopen(f'http://127.0.0.1:{port}/config.json', timeout=2) as r:
            cfg = json.load(r)
        assert cfg['schema'] == 'kalo-debug/v1' and cfg['bridge_port'] == 8765 and cfg['domain'] == os.environ['ROS_DOMAIN_ID']
        for page in ('/', '/kalo.js', '/kalo.css', '/kalo.jpg'):
            with urllib.request.urlopen(f'http://127.0.0.1:{port}{page}', timeout=2) as r:
                assert r.status == 200 and r.read()
        print(f'PASS: lazy camera subscription, {rate:.1f} Hz JPEG 640x480 with camera header, static page and config')
    finally:
        for proc in (jpeg, web):
            os.killpg(proc.pid, signal.SIGINT)
        out = jpeg.communicate(timeout=10)[0]
        web.wait(timeout=10)
        print('kalo_jpeg log:', ' | '.join(l for l in out.splitlines() if 'NVJPG' in l or 'CPU' in l or 'viewer' in l)[:400])
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
