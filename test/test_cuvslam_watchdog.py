#!/usr/bin/env python3
"""Isolated ROS regression: no hardware, no motion publishers, no process killing."""
import os
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import Info, OdomInfo
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster


def run(frontend):
    if os.environ.get('ROS_DOMAIN_ID') != '74':
        raise SystemExit('Run this sensor-only regression in isolated ROS_DOMAIN_ID=74')
    process = subprocess.Popen([
        get_package_prefix('wla_r680_navigation') + '/lib/wla_r680_navigation/vo_watchdog', '--ros-args',
        '-p', 'frontend:=' + frontend, '-p', 'restart_enabled:=false',
        '-p', 'startup_grace_s:=30.0', '-p', 'stale_s:=0.3',
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    node = Node('watchdog_contract_probe')
    health = []
    status = []
    node.create_subscription(Bool, '/r680_nav/vo_watchdog_healthy', lambda m: health.append(m.data), 10)
    node.create_subscription(String, '/r680_nav/vo_watchdog_status', lambda m: status.append(m.data), 10)
    odom = node.create_publisher(Odometry, '/r680_nav/vo_odom', 10)
    wheel = node.create_publisher(Odometry, '/wheel/odom', 10)
    generic = node.create_publisher(Bool, '/r680_nav/vio_tracking_healthy', 10)
    info = node.create_publisher(OdomInfo, '/d455_slam/odom_info', 10)
    map_info = node.create_publisher(Info, '/d455_slam/info', 10)
    localization = node.create_publisher(PoseWithCovarianceStamped, '/d455_slam/localization_pose', 10)
    broadcaster = TransformBroadcaster(node)

    def drive(seconds, ready=False, matched=False, send=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = 'd455_floor_odom'
            msg.child_frame_id = 'r680_mapping_floor'
            msg.pose.pose.orientation.w = 1.0
            if send:
                odom.publish(msg)
                wheel.publish(msg)
                generic.publish(Bool(data=ready))
                info.publish(OdomInfo(lost=False))
            if matched:
                pose = PoseWithCovarianceStamped()
                pose.header.stamp = stamp
                pose.header.frame_id = 'map'
                pose.pose.pose.orientation.w = 1.0
                localization.publish(pose)
                match = Info()
                match.header.stamp = stamp
                match.loop_closure_id = 1
                map_info.publish(match)
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = 'map'
                tf.child_frame_id = 'r680_mapping_floor'
                tf.transform.rotation.w = 1.0
                broadcaster.sendTransform(tf)
            rclpy.spin_once(node, timeout_sec=0.025)
    try:
        drive(2.0)
        assert health, 'watchdog produced no health'
        if frontend == 'cuvslam':
            assert not any(health), 'uninitialized VIO opened health'
            assert any(s == 'waiting_for_inertial_initialization' for s in status)
            health.clear()
            drive(0.8, ready=True)
            assert not any(health), 'VIO opened before a map match'
            drive(1.0, ready=True, matched=True)
            assert health[-1], 'initialized and map-localized VIO did not open health'
            drive(0.6, ready=False, matched=True)
            assert not health[-1], 'lost inertial tracking failed to close health'
        else:
            assert health[-1], 'legacy OdomInfo compatibility was broken'
            drive(0.8, send=False)
            assert not health[-1], 'legacy stale input failed to close health'
        print(frontend + ': initialization, localization and loss gates passed')
    finally:
        node.destroy_node()
        process.terminate()
        try:
            out = process.communicate(timeout=5)[0]
        except subprocess.TimeoutExpired:
            process.kill()
            out = process.communicate()[0]
        print(out.decode(errors='replace')[-1200:])


if __name__ == '__main__':
    rclpy.init()
    try:
        run('rgbd')
        run('cuvslam')
    finally:
        rclpy.shutdown()
