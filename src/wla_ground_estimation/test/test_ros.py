#!/usr/bin/env python3
"""Synthetic ROS integration, must run on an isolated nonzero ROS domain."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, Range
from std_msgs.msg import String
from wla_ground_estimation.msg import GroundEstimate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--params', required=True)
    parser.add_argument('--fixture', required=True)
    args = parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        raise RuntimeError('Synthetic publishers require an isolated nonzero ROS domain')
    rclpy.init()
    node = rclpy.create_node('ground_integration_fixture')
    imu = node.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
    joints = node.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
    state = node.create_publisher(String, '/motor35_powered/state', 10)
    frames = ['front_tof_L', 'front_tof_R', 'back_tof_L', 'back_tof_R']
    topics = ['/tof/front_left/range', '/tof/front_right/range', '/tof/back_left/range', '/tof/back_right/range']
    ranges = [node.create_publisher(Range, t, qos_profile_sensor_data) for t in topics]
    messages = []
    sub = node.create_subscription(GroundEstimate, '/ground_estimation/plane', messages.append, 30)
    del sub  # Node retains the subscription.
    hip = 0.0
    contact = 1
    emit_imu = True
    emit_joints = True
    emit_ranges = False
    corrupt_range = False
    log_dir = Path('/tmp/wla-ground-ros-tests-20261112')
    log_dir.mkdir(exist_ok=True)
    process = None
    checks = []

    def drive(seconds):
        start = len(messages)
        end = time.monotonic()+seconds
        last_slow = 0.0
        while time.monotonic() < end:
            now = node.get_clock().now()
            if emit_imu:
                msg = Imu()
                msg.header.stamp = now.to_msg()
                msg.header.frame_id = 'base_Link'
                msg.orientation.w = 1.0
                msg.linear_acceleration.z = 9.80665
                imu.publish(msg)
            if time.monotonic()-last_slow > .025:
                last_slow = time.monotonic()
                stamp = rclpy.time.Time(nanoseconds=now.nanoseconds-7_000_000).to_msg()
                if emit_joints:
                    msg = JointState()
                    msg.header.stamp = stamp
                    msg.name = ['hip_L', 'hip_R', 'wheel_L', 'wheel_R']
                    msg.position = [hip, hip, 0.0, 0.0]
                    joints.publish(msg)
                state.publish(String(data=json.dumps({'session': 'synthetic_fixture', 'ready': False, 'gate': '',
                    'support_observer': {'enabled': 1, 'calibration_verified': 1, 'L_valid': 1,
                                         'R_valid': 1, 'L_state': contact, 'R_state': contact}})))
                if emit_ranges:
                    for i, frame in enumerate(frames):
                        msg = Range()
                        msg.header.stamp = stamp
                        msg.header.frame_id = frame
                        msg.min_range = .02
                        msg.max_range = 3.0
                        msg.field_of_view = .1
                        z = .035893 if i < 2 else (.026245 if i == 2 else .026244885463394)
                        pitch = [.40776, .40778, .785398163397441, .7854][i]
                        msg.range = float((.2405+z)/math.sin(pitch)+(0.3 if corrupt_range and i == 0 else 0.0))
                        ranges[i].publish(msg)
            rclpy.spin_once(node, timeout_sec=.002)
            time.sleep(.003)
        return messages[start:]

    def start(source):
        nonlocal process
        stream = open(log_dir/(source+'.log'), 'w')
        process = subprocess.Popen([args.executable, '--ros-args', '--params-file', args.params,
            '-p', 'model_urdf:='+args.fixture, '-p', 'ground_source:='+source,
            '-p', 'imu_axes_verified:=true', '-p', 'mount_geometry_verified:=true',
            '-p', 'sample_times_verified:=true', '-p', 'tof_axes_verified:=true'],
            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        stream.close()

    def stop():
        nonlocal process
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
                    raise RuntimeError('test node did not exit on SIGINT')
            assert process.returncode == 0, f'node exit: {process.returncode}, see {log_dir}'
            process = None

    try:
        start('kinematic_flat')
        batch = drive(2.0)
        good = [m for m in batch if m.valid]
        assert len(good) > 5, f'no valid analytic height: {[m.reason for m in batch[-4:]]}'
        assert abs(good[-1].vertical_height_m-.2405) < 1e-9
        assert len(good[-1].sensor_names) == 7 and 'upper_camera_L' not in good[-1].sensor_names
        checks.append('flat FK height, real ROS interfaces and seven sensor planes')
        hip = .4
        batch = drive(.6)
        good = [m for m in batch[-10:] if m.valid]
        assert good and abs(good[-1].vertical_height_m-(.2*math.cos(.4)+.0405)) < 1e-9
        checks.append('joint feedback changes height; no target height input')
        contact = 0
        batch = drive(.5)
        assert batch[-1].has_estimate and not batch[-1].valid
        checks.append('unknown contact stays explicitly invalid')
        contact = 2
        batch = drive(.5)
        assert not batch[-1].has_estimate and 'airborne' in batch[-1].reason
        checks.append('flight does not invent ground')
        contact = 1
        emit_imu = False
        batch = drive(.5)
        assert not batch[-1].valid and 'attitude_not_bracketed' in batch[-1].reason
        checks.append('missing IMU cannot reuse stale rotation')
        emit_imu = True
        batch = drive(.5)
        assert any(m.valid for m in batch)
        emit_joints = False
        batch = drive(.5)
        assert not batch[-1].valid
        checks.append('missing joints invalidate output')
        stop()
        # New node process owns a fresh history; no cross-backend state carryover.
        hip = 0.0
        contact = 0
        emit_joints = True
        emit_ranges = True
        start('tof_plane')
        batch = drive(2.0)
        good = [m for m in batch if m.valid]
        assert len(good) > 5, f'ToF fit failed: {[m.reason for m in batch[-4:]]}'
        assert abs(good[-1].vertical_height_m-.2405) < 1e-6
        checks.append('four ToF ground points recover local plane without wheel contact')
        corrupt_range = True
        batch = drive(.6)
        assert not batch[-1].valid and 'tof_plane_' in batch[-1].reason
        checks.append('inconsistent ToF points rejected without silent wheel fallback')
        stop()
        result = {'checks': checks, 'count': len(checks), 'domain': os.environ['ROS_DOMAIN_ID'], 'hardware_used': False}
        (log_dir/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, ensure_ascii=False))
    finally:
        stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
