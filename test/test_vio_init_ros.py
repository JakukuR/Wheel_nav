#!/usr/bin/env python3
"""Isolated synthetic closed-loop bootstrap test. Hardware output ALWAYS false."""
import math
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from std_msgs.msg import Bool, String
import struct


def run(case):
    prefix = Path(get_package_prefix('wla_r680_navigation')) / 'lib/wla_r680_navigation'
    result = Path('/tmp') / ('vio-init-regression-' + case + '.json')
    commands = [
        [str(prefix / 'vio_initializer'), '--ros-args', '-p', 'stationary_time_s:=0.2',
         '-p', 'stopping_time_s:=0.2', '-p', 'startup_timeout_s:=5.0',
         '-p', 'initialization_timeout_s:=3.0', '-p', 'motion_duration_s:=1.0',
         '-p', 'result_path:=' + str(result)],
        [str(prefix / 'command_guard'), '--ros-args', '-p', 'hardware_output_enabled:=false',
         '-p', 'initialization_mode_enabled:=true'],
    ]
    files = [open('/tmp/vio-init-' + case + '-' + str(i) + '.log', 'w') for i in range(2)]
    processes = [subprocess.Popen(c, stdout=f, stderr=subprocess.STDOUT) for c, f in zip(commands, files)]
    node = Node('vio_init_synthetic_' + case)
    state = {'name': '', 'preview': Twist(), 'raw': 0, 'moving_at': None, 'fault_at': None}
    states, bootstrap_values, normal_values = [], [], []
    def state_cb(m):
        state['name'] = m.data
        if not states or states[-1] != m.data: states.append(m.data)
        if m.data == 'moving' and state['moving_at'] is None: state['moving_at'] = time.monotonic()
    def preview_cb(m):
        state['preview'] = m
        if state['name'] == 'moving': bootstrap_values.append((m.linear.x, m.angular.z))
        if state['name'] == 'succeeded': normal_values.append(m.linear.x)
    node.create_subscription(String, '/r680_nav/vio_init_state', state_cb, 10)
    node.create_subscription(Twist, '/r680_nav/cmd_vel_safe_preview', preview_cb, 10)
    node.create_subscription(Twist, '/r680_nav/chassis_cmd_vel', lambda m: state.update(raw=state['raw']+1), 10)
    checked = node.create_publisher(Twist, '/r680_nav/cmd_vel_collision_checked', 1)
    def relay(m):
        # Deliberately retain a smoother residual at zero input; final guard must stop it.
        if state['name'] in ['stopping', 'waiting_localization', 'failed']:
            m = Twist(); m.linear.x = 0.04; m.angular.z = 0.10
        checked.publish(m)
    node.create_subscription(Twist, '/cmd_vel_nav', relay, 1)
    publishers = {name: node.create_publisher(typ, name, 10) for name, typ in [
        ('/d455_slam/odom', Odometry), ('/wheel/odom', Odometry), ('/wheel/imu/data_raw', Imu),
        ('/r680/d455/aligned_depth_to_color/image_raw', Image), ('/r680_nav/d455/points_safety', PointCloud2),
        ('/r680_nav/vio_status', String), ('/r680_nav/vio_tracking_healthy', Bool),
        ('/r680_nav/localization_ready', Bool), ('/r680_nav/mission_motion_allowed', Bool),
        ('/r680_nav/nav_command_input', Twist)]}
    def service_cb(req, res):
        active = not (case == 'lifecycle' and state['fault_at'] is not None)
        res.current_state.id = 3 if active else 2
        return res
    node.create_service(GetState, '/velocity_smoother/get_state', service_cb)
    node.create_service(GetState, '/collision_monitor/get_state', service_cb)
    x = y = yaw = 0.0
    begin = previous = time.monotonic()
    last_image = last_vo = 0.0
    held_failure = 0.0
    try:
        while time.monotonic() - begin < 7:
            current = time.monotonic(); dt = min(current-previous, 0.05); previous = current
            if any(p.poll() is not None for i,p in enumerate(processes)
                   if not (case == 'node_exit' and i == 0 and state['fault_at'] is not None)):
                raise AssertionError('node exited: inspect /tmp/vio-init logs')
            moving_time = 0 if state['moving_at'] is None else current-state['moving_at']
            if case not in ['success', 'timeout'] and state['moving_at'] and moving_time > 0.25:
                if state['fault_at'] is None:
                    state['fault_at'] = current
                    if case == 'node_exit': processes[0].terminate()
            fault = state['fault_at'] is not None
            preview = state['preview']
            x += preview.linear.x * math.cos(yaw)*dt
            y += preview.linear.x * math.sin(yaw)*dt
            yaw += preview.angular.z*dt
            stamp = node.get_clock().now().to_msg()
            imu = Imu(); imu.header.stamp = stamp; imu.header.frame_id = 'gyro_link'
            imu.linear_acceleration.z = 9.81; imu.angular_velocity.z = preview.angular.z
            if not (case == 'imu_gap' and fault): publishers['/wheel/imu/data_raw'].publish(imu)
            wheel = Odometry(); wheel.header.stamp = stamp; wheel.header.frame_id = 'wheel_odom'
            wheel.child_frame_id = 'r680_mapping_floor'
            wheel.pose.pose.position.x = x; wheel.pose.pose.position.y = y
            wheel.pose.pose.orientation.w = math.cos(yaw/2); wheel.pose.pose.orientation.z = math.sin(yaw/2)
            wheel.twist.twist.linear.x = preview.linear.x; wheel.twist.twist.angular.z = preview.angular.z
            publishers['/wheel/odom'].publish(wheel)
            if current-last_vo > 0.032:
                vo = Odometry(); vo.header.stamp = stamp; vo.header.frame_id = 'd455_floor_odom'
                vo.child_frame_id = 'r680_mapping_floor'; vo.pose = wheel.pose; vo.twist = wheel.twist
                if case == 'pose_jump' and fault: vo.pose.pose.position.x += 0.30
                publishers['/d455_slam/odom'].publish(vo)
                last_vo = current
            inertial = case == 'success' and state['moving_at'] is not None and moving_time > 0.5
            publishers['/r680_nav/vio_status'].publish(String(data='tracking_inertial_ready' if inertial else 'waiting_for_stable_inertial_initialization'))
            publishers['/r680_nav/vio_tracking_healthy'].publish(Bool(data=inertial))
            localized = inertial and state['name'] in ['waiting_localization', 'succeeded']
            publishers['/r680_nav/localization_ready'].publish(Bool(data=localized))
            publishers['/r680_nav/mission_motion_allowed'].publish(Bool(data=state['name'] == 'succeeded'))
            nav = Twist(); nav.linear.x = 0.10 if state['name'] == 'succeeded' else 0.60
            publishers['/r680_nav/nav_command_input'].publish(nav)
            if current-last_image > 0.08:
                image = Image(); image.header.stamp = stamp; image.header.frame_id = 'd455_color_optical_frame'
                image.width = image.height = 64; image.step = 128; image.encoding = '16UC1'
                image.data = (b'\0\0' if case == 'depth_unknown' and fault else b'\xe8\x03') * 4096
                publishers['/r680/d455/aligned_depth_to_color/image_raw'].publish(image)
                cloud = PointCloud2(); cloud.header.stamp = stamp; cloud.header.frame_id = 'r680_mapping_floor'
                cloud.height = 1; cloud.point_step = 12
                cloud.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i,n in enumerate(['x','y','z'])]
                if case == 'obstacle' and fault:
                    cloud.width = 1; cloud.row_step = 12; cloud.data = struct.pack('fff', 0.40, 0.0, 0.20)
                if case == 'cloud_malformed' and fault:
                    cloud.width = 1; cloud.row_step = 12; cloud.data = b'\0' # Incomplete point buffer.
                publishers['/r680_nav/d455/points_safety'].publish(cloud)
                last_image = current
            rclpy.spin_once(node, timeout_sec=0)
            if case == 'node_exit' and fault and current-state['fault_at'] > 0.6: break
            if state['name'] in ['failed', 'succeeded']:
                if held_failure == 0: held_failure = current
                if current-held_failure > 0.6: break
            time.sleep(max(0, 0.007-(time.monotonic()-current)))
        assert state['raw'] == 0, 'hardware-disabled test published chassis commands'
        assert bootstrap_values and any(v > 0 for v,w in bootstrap_values), 'bootstrap never moved in synthetic test'
        assert all(0 <= v <= 0.060001 and abs(w) <= 0.200001 for v,w in bootstrap_values)
        if case == 'success':
            assert state['name'] == 'succeeded', states
            assert normal_values and max(normal_values) > 0.09, 'normal control did not resume'
        else:
            if case != 'node_exit': assert state['name'] == 'failed', states
            assert abs(state['preview'].linear.x) < 1e-9 and abs(state['preview'].angular.z) < 1e-9, 'smoother residual bypassed failure stop'
        print(case, 'PASS', states, 'raw messages:', state['raw'], flush=True)
    finally:
        node.destroy_node()
        for p in processes:
            p.terminate()
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired: p.kill(); p.wait()
        for f in files: f.close()


def legacy_guard():
    executable = Path(get_package_prefix('wla_r680_navigation')) / 'lib/wla_r680_navigation/command_guard'
    with open('/tmp/vio-init-legacy-guard.log', 'w') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '-p', 'hardware_output_enabled:=false'],
                                   stdout=log, stderr=subprocess.STDOUT)
        node = Node('vio_init_legacy_guard_probe')
        preview, raw = [], []
        node.create_subscription(Twist, '/r680_nav/cmd_vel_safe_preview', lambda m: preview.append(m.linear.x), 10)
        node.create_subscription(Twist, '/r680_nav/chassis_cmd_vel', lambda m: raw.append(m), 10)
        checked = node.create_publisher(Twist, '/r680_nav/cmd_vel_collision_checked', 1)
        health = node.create_publisher(Bool, '/r680_nav/localization_ready', 1)
        permit = node.create_publisher(Bool, '/r680_nav/mission_motion_allowed', 1)
        try:
            for healthy in [True, False]:
                preview.clear(); end = time.monotonic()+1.0
                while time.monotonic() < end:
                    command = Twist(); command.linear.x = 0.1; checked.publish(command)
                    health.publish(Bool(data=healthy)); permit.publish(Bool(data=True))
                    rclpy.spin_once(node, timeout_sec=0.02)
                assert preview
                assert (max(preview) > 0.09 if healthy else abs(preview[-1]) < 1e-9)
            assert not raw
            print('legacy_guard PASS: default route unchanged; lost health stops', flush=True)
        finally:
            node.destroy_node(); process.terminate(); process.wait(timeout=5)


if __name__ == '__main__':
    assert os.environ.get('ROS_DOMAIN_ID') == '74', 'Synthetic tests require isolated ROS_DOMAIN_ID=74'
    rclpy.init()
    try:
        for case in ['success', 'obstacle', 'imu_gap', 'pose_jump', 'depth_unknown', 'cloud_malformed', 'lifecycle', 'timeout', 'node_exit']: run(case)
        legacy_guard()
    finally: rclpy.shutdown()
