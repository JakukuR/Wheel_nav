#!/usr/bin/env python3
"""Run the installed watchdog against synthetic RGB-D/map evidence, no motors."""
import os
from pathlib import Path
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import Info, OdomInfo
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def test_rgbd_requires_map_localization():
    # This test owns synthetic navigation input names and must never join the
    # production ROS domain. The watchdog cannot terminate any frontend here.
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    assert 100 <= domain <= 232, 'set an isolated ROS_DOMAIN_ID in [100, 232]'
    rclpy.init()
    node = rclpy.create_node('rgbd_map_gate_regression')
    publishers = {
        'vo': node.create_publisher(Odometry, '/r680_nav/vo_odom', 10),
        'odom_info': node.create_publisher(OdomInfo, '/d455_slam/odom_info', 10),
        'map_info': node.create_publisher(Info, '/d455_slam/info', 10),
        'pose': node.create_publisher(PoseWithCovarianceStamped,
                                      '/d455_slam/localization_pose', 10),
    }
    broadcaster = TransformBroadcaster(node)
    received = []
    node.create_subscription(Bool, '/r680_nav/vo_watchdog_healthy',
                             lambda msg: received.append((time.monotonic(), msg.data)), 10)
    executable = (Path(get_package_prefix('wla_r680_navigation')) /
                  'lib/wla_r680_navigation/vo_watchdog')
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'frontend:=rgbd',
        '-p', 'restart_enabled:=false', '-p', 'startup_grace_s:=30.0',
        '-p', 'localization_stale_s:=0.4', '-p', 'stale_s:=1.5',
    ], stdout=subprocess.DEVNULL)

    def tick(*, matched=False, pose=False, transform=True):
        assert process.poll() is None, 'watchdog exited unexpectedly'
        stamp = node.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'd455_floor_odom'
        odometry.child_frame_id = 'r680_mapping_floor'
        odometry.pose.pose.orientation.w = 1.
        publishers['vo'].publish(odometry)
        quality = OdomInfo()
        quality.header.stamp = stamp
        quality.lost = False
        publishers['odom_info'].publish(quality)
        info = Info()
        info.header.stamp = stamp
        info.loop_closure_id = 1 if matched else 0
        publishers['map_info'].publish(info)
        if pose:
            location = PoseWithCovarianceStamped()
            location.header.stamp = stamp
            location.header.frame_id = 'map'
            location.pose.pose.position.x = 2.
            location.pose.pose.orientation.w = 1.
            publishers['pose'].publish(location)
        if transform:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'map'
            tf.child_frame_id = 'r680_mapping_floor'
            tf.transform.translation.x = 2.
            tf.transform.rotation.w = 1.
            broadcaster.sendTransform(tf)
        rclpy.spin_once(node, timeout_sec=.005)
        time.sleep(.02)

    def phase(seconds, **inputs):
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            tick(**inputs)
        return [value for stamp, value in received if stamp >= start + seconds / 2]

    def expect_closed(label, **inputs):
        evidence = phase(1.2, **inputs)
        assert len(evidence) >= 3, label + ': no health evidence'
        assert not any(evidence), label + ': admitted motion'

    def expect_open(label, **inputs):
        deadline = time.monotonic() + 5.
        start = time.monotonic()
        while time.monotonic() < deadline:
            tick(**inputs)
            if received and received[-1][0] >= start and received[-1][1]:
                return
        raise AssertionError(label + ': matching map did not admit navigation')

    try:
        # Discovery first, then assertions require actual received health.
        phase(1.0)
        expect_closed('local odometry and map TF without localization')
        expect_closed('pose and TF without geometric match', pose=True)
        phase(.7)  # expire the previous pose before introducing a match
        expect_closed('match Info without a localization pose', matched=True)
        expect_open('matched pose and TF', matched=True, pose=True)
        expect_closed('expired localization pose', matched=True)
        expect_open('fresh localization restored', pose=True)
        expect_closed('expired map TF', pose=True, transform=False)
        expect_open('fresh TF restored', pose=True)
        print('PASS RGB-D map admission: odom-only, pose-only, match-only, '
              'matched localization, pose expiry, TF expiry and recovery')
    finally:
        process.terminate()
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    test_rgbd_requires_map_localization()
