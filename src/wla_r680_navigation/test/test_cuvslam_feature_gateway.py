"""Isolated software-only integration: no camera, estimator, Nav2 or base."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request
from io import BytesIO

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from PIL import Image as PILImage, ImageDraw

root = Path('/home/orin/ros2_ws')
build = root / '.runtime/cuvslam_navigation/build/wla_cuvslam_navigation'
artifacts = root / '.runtime/cuvslam_navigation/feature-preview-test'
artifacts.mkdir(exist_ok=True)
subprocess.run([str(build / 'test_feature_overlay'), str(artifacts / 'fixture.ppm')], check=True)
fixture = PILImage.open(artifacts / 'fixture.ppm').convert('RGB')
draw = ImageDraw.Draw(fixture)
draw.rectangle((0, 0, fixture.width, 34), fill=(85, 20, 20))
draw.text((18, 10), 'SYNTHETIC SOFTWARE TEST - NOT CAMERA DATA', fill='white')
fixture.save(artifacts / 'fixture.png')
env = os.environ.copy()
env['ROS_DOMAIN_ID'] = '198'
env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ.update({k: env[k] for k in ['ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'RMW_IMPLEMENTATION']})
log = (artifacts / 'gateway.log').open('w')
preview = '--preview' in sys.argv
gateway = subprocess.Popen([sys.executable, str(root / 'src/wla_r680_navigation/scripts/web_gateway'),
    '--ros-args', '-p', 'host:=' + ('0.0.0.0' if preview else '127.0.0.1'),
    '-p', 'port:=18080'], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
rclpy.init()
node = rclpy.create_node('feature_preview_fixture_no_hardware')
pub = node.create_publisher(Image, '/r680_nav/vio/features/image',
                          QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
url = 'http://127.0.0.1:18080'

def get(path):
    with urllib.request.urlopen(url + path, timeout=2) as response:
        return response.read()

def status():
    return json.loads(get('/api/vio/status'))

def send():
    msg = Image()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'synthetic_fixture_not_camera'
    msg.width, msg.height = fixture.size
    msg.step = msg.width * 3
    msg.encoding = 'rgb8'
    msg.data = fixture.tobytes()
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.01)

try:
    deadline = time.monotonic() + 10
    while True:
        try:
            assert b'vio.js' in get('/vio.html')
            break
        except OSError:
            assert time.monotonic() < deadline, 'gateway did not start'
            time.sleep(0.2)
    assert b'/vio.html' in get('/'), 'navigation entry missing'
    assert b'pollFeatures' in get('/vio.js')
    assert pub.get_subscription_count() == 0, 'preview subscribed without viewer'
    first = status()
    assert not first['live'] and first['age_s'] is None
    deadline = time.monotonic() + 8
    while True:
        send()
        current = status()
        if current['live']:
            break
        assert time.monotonic() < deadline, 'no preview frame received'
        time.sleep(0.15)
    decoded = PILImage.open(BytesIO(get('/api/vio/image.jpg')))
    assert decoded.size == (1280, 480)
    assert current['frame_id'] == 'synthetic_fixture_not_camera'
    (artifacts / 'http-frame.jpg').write_bytes(get('/api/vio/image.jpg'))
    if preview:
        print('PREVIEW_READY http://172.31.18.156:18080/vio.html (synthetic only)', flush=True)
        until = time.monotonic() + 55
        while time.monotonic() < until:
            send()
            time.sleep(0.2)
    else:
        until = time.monotonic() + 2.4
        while time.monotonic() < until:
            current = status()
            time.sleep(0.2)
        assert not status()['live'], 'frozen frame shown as live'
        until = time.monotonic() + 4
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert pub.get_subscription_count() == 0, 'subscription not released after viewer leaves'
    print('PASS: HTML, viewer lease, ROS image -> JPEG, timestamp, stale state, subscription lifetime', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
    os.killpg(gateway.pid, signal.SIGINT)
    try:
        gateway.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(gateway.pid, signal.SIGTERM)
        gateway.wait(timeout=3)
    log.close()
