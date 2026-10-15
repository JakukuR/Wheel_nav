#!/usr/bin/env python3
"""Exercise visual priority and fallback input expiry on an isolated ROS graph."""
import os
from pathlib import Path
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from rclpy.qos import qos_profile_sensor_data


def test_visual_odom_sensor_independence():
    assert 100 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 232
    rclpy.init()
    node = rclpy.create_node('visual_odom_sensor_independence')
    prefix = Path(get_package_prefix('wla_r680_navigation'))
    binary = os.environ.get('WLA_CONTINUITY_EXECUTABLE', str(prefix/'lib/wla_r680_navigation/degraded_odometry'))
    pubs = {}
    for key, cls, topic, qos in [
        ('raw', Odometry, '/r680_nav/vio_raw_odom', qos_profile_sensor_data),
        ('wheel', Odometry, '/wheel/odom', qos_profile_sensor_data),
        ('imu', Imu, '/wheel/imu/data_raw', qos_profile_sensor_data),
        ('healthy', Bool, '/r680_nav/vio_tracking_healthy', 10),
        ('visual', Bool, '/r680_nav/vio_visual_observed', 10),
        ('inertial', Bool, '/r680_nav/vio_inertial_valid', 10),
        ('watchdog', Bool, '/r680_nav/vo_watchdog_healthy', 10),
        ('status', String, '/r680_nav/vio_status', 10),
    ]:
        pubs[key] = node.create_publisher(cls, topic, qos)
    health, states = [], []
    node.create_subscription(Bool, '/r680_nav/continuous_odom_healthy',
                             lambda m: health.append((time.monotonic(), m.data)), 10)
    node.create_subscription(String, '/r680_nav/continuous_odom_status',
                             lambda m: states.append((time.monotonic(), m.data)), 10)
    process = subprocess.Popen([binary, '--ros-args', '--params-file',
        str(prefix/'share/wla_r680_navigation/config/degraded_odometry.yaml')],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def phase(seconds, *, visual=True, imu=True, wheel=True):
        start = time.monotonic()
        while time.monotonic()-start < seconds:
            assert process.poll() is None
            stamp = node.get_clock().now().to_msg()
            for key, value in [('healthy', visual), ('visual', visual),
                               ('inertial', True), ('watchdog', True)]:
                pubs[key].publish(Bool(data=value))
            pubs['status'].publish(String(data='tracking_inertial_ready' if visual else 'tracking_lost_no_output'))
            if imu:
                msg = Imu(); msg.header.stamp = stamp; msg.header.frame_id = 'gyro_link'
                msg.linear_acceleration.z = 9.81
                pubs['imu'].publish(msg)
            if wheel:
                msg = Odometry(); msg.header.stamp = stamp
                msg.pose.pose.orientation.w = 1.
                pubs['wheel'].publish(msg)
            if visual:
                msg = Odometry(); msg.header.stamp = stamp
                msg.header.frame_id = 'd455_floor_odom'; msg.child_frame_id = 'r680_mapping_floor'
                msg.pose.pose.orientation.w = 1.
                for i in (0, 7, 35): msg.pose.covariance[i] = .0001
                pubs['raw'].publish(msg)
            for _ in range(8): rclpy.spin_once(node, timeout_sec=0)
            time.sleep(.01)
        return start

    def expect(label, mode, healthy, seconds=1., **inputs):
        start = phase(seconds, **inputs)
        cutoff = start + seconds - .2
        values = [v for t, v in health if t >= cutoff]
        messages = [v for t, v in states if t >= cutoff]
        assert len(values) >= 3 and messages, label + ': missing evidence'
        assert all(v == healthy for v in values), (label, values, messages)
        assert all(v.startswith(mode) for v in messages), (label, messages)
        print('PASS', label, flush=True)

    try:
        phase(2.5)
        expect('normal visual with stale auxiliary IMU', 'visual', True, imu=False)
        expect('normal visual with stale auxiliary wheel', 'visual', True, wheel=False)
        expect('normal visual with both auxiliary inputs stale', 'visual', True, imu=False, wheel=False)
        phase(.3)
        expect('visual loss enters bounded fallback', 'wheel_imu_degraded', True, seconds=.4, visual=False)
        expect('auxiliary IMU loss during fallback blocks', 'blocked', False, seconds=.5, visual=False, imu=False)
        expect('blocked visual return still needs stopped wheel evidence', 'blocked', False, imu=False, wheel=False)
        expect('stopped visual recovery does not need auxiliary IMU', 'visual', True, seconds=2.5, imu=False)
        phase(.3)
        expect('second visual loss enters fallback', 'wheel_imu_degraded', True, seconds=.4, visual=False)
        expect('auxiliary wheel loss during fallback blocks', 'blocked', False, seconds=.5, visual=False, wheel=False)
        for topic in ('/cmd_vel', '/wheel/cmd_vel', '/cmd_vel_nav', '/cmd_vel_safe'):
            assert not node.get_publishers_info_by_topic(topic)
    finally:
        process.terminate()
        try: process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait(timeout=2)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    test_visual_odom_sensor_independence()
