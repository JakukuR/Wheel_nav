#!/usr/bin/env python3
"""Isolated startup sequencing regression; mock lifecycle server, no chassis."""
import importlib.util
import os
from pathlib import Path
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ManageLifecycleNodes
from tf2_ros import StaticTransformBroadcaster

assert os.environ.get('ROS_DOMAIN_ID') == '198'
path = Path(__file__).resolve().parents[1]/'scripts/openvins_nav_startup.py'
spec = importlib.util.spec_from_file_location('startup', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
rclpy.init()
fixture = Node('openvins_startup_fixture')
executor = SingleThreadedExecutor()
executor.add_node(fixture)
requests = []
success = True
def response(request, result):
    requests.append(request.command)
    result.success = success
    return result
service = fixture.create_service(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes', response)
odom_pub = fixture.create_publisher(Odometry, '/d455_slam/odom', qos_profile_sensor_data)
broadcaster = StaticTransformBroadcaster(fixture)
def send(stale=False):
    stamp = fixture.get_clock().now().to_msg()
    if stale:
        stamp.sec -= 5
    o = Odometry()
    o.header.stamp = stamp
    o.header.frame_id = 'd455_floor_odom'
    o.child_frame_id = 'r680_mapping_floor'
    o.pose.pose.orientation.w = 1.0
    odom_pub.publish(o)
def spin(seconds, callback=None):
    until = time.monotonic()+seconds
    while time.monotonic()<until:
        if callback:
            callback()
        executor.spin_once(timeout_sec=.01)
def node():
    n = module.Startup()
    executor.add_node(n)
    spin(.3)
    return n
def destroy(n):
    executor.remove_node(n)
    n.destroy_node()
n = node()
try:
    spin(.3)
    assert not requests
    spin(.4, lambda:send(stale=True))
    assert not requests, 'stale odometry must not activate Nav2'
    spin(.4, send)
    assert not requests, 'missing map TF must wait'
    t = TransformStamped()
    t.header.stamp = fixture.get_clock().now().to_msg()
    t.header.frame_id = 'map'
    t.child_frame_id = 'r680_mapping_floor'
    t.transform.rotation.w = 1.0
    broadcaster.sendTransform(t)
    spin(.6, send)
    assert requests == [ManageLifecycleNodes.Request.STARTUP] and n.done
    spin(.3, send)
    assert len(requests)==1, 'startup must be one-shot'
    destroy(n)
    requests.clear()
    success = False
    n = node()
    spin(.6, send)
    assert len(requests)==1 and n.failed
    spin(.3, send)
    assert len(requests)==1, 'failed startup must not auto retry'
    print('PASS waiting/stale/missing-TF, one-shot STARTUP and rejection')
finally:
    destroy(n)
    executor.remove_node(fixture)
    fixture.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
