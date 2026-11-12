#!/usr/bin/env python3
"""Read-only bridge contract tests using isolated synthetic ROS publishers."""
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
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--params', required=True)
    args = parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        raise RuntimeError('synthetic ROS input requires nonzero isolated domain')
    rclpy.init()
    node = rclpy.create_node('state_adapter_fixture')
    imu = node.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
    state = node.create_publisher(String, '/motor35_powered/state', 10)
    joints, attitudes, rpys, statuses = [], [], [], []
    node.create_subscription(JointState, '/joint_states', joints.append, 10)
    node.create_subscription(QuaternionStamped, '/robot_state/attitude', attitudes.append, qos_profile_sensor_data)
    node.create_subscription(Vector3Stamped, '/robot_state/rpy', rpys.append, qos_profile_sensor_data)
    node.create_subscription(String, '/robot_state/status', lambda m: statuses.append(json.loads(m.data)), 10)
    emitted = set()
    ready = True
    good_imu = True
    yaw = .7

    def drive(seconds, publish=True):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if publish:
                msg = Imu()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'base_Link'
                msg.orientation.w = 0.0  # Jazzy's Quaternion defaults w=1; invalid fixture must be explicit.
                emitted.add((msg.header.stamp.sec, msg.header.stamp.nanosec))
                if good_imu:
                    sr, cr, sp, cp, sy, cy = math.sin(.05), math.cos(.05), math.sin(-.1), math.cos(-.1), math.sin(yaw/2), math.cos(yaw/2)
                    msg.orientation.x = sr*cp*cy-cr*sp*sy
                    msg.orientation.y = cr*sp*cy+sr*cp*sy
                    msg.orientation.z = cr*cp*sy-sr*sp*cy
                    msg.orientation.w = cr*cp*cy+sr*sp*sy
                imu.publish(msg)
                state.publish(String(data=json.dumps({'ready': ready, 'gate': '', 'q_rad': list(range(8)),
                    'reference_q_rad': [99]*6, 'attitude_source': 'host_mahony'})))
            rclpy.spin_once(node, timeout_sec=.002)
            time.sleep(.008)

    log = open('/tmp/wla-state-adapter-ros-test.log', 'w')
    process = subprocess.Popen([args.executable, '--ros-args', '--params-file', args.params],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    checks = []
    try:
        drive(2.0)
        assert len(joints) > 5 and len(attitudes) > 5 and rpys and statuses
        assert list(joints[-1].position) == list(range(8))
        assert joints[-1].name[0] == 'abad_L_Joint' and joints[-1].name[-1] == 'wheel_R_Joint'
        assert not joints[-1].velocity and not joints[-1].effort
        checks.append('standard eight-joint feedback excludes reference targets and invented velocity/effort')
        assert (attitudes[-1].header.stamp.sec, attitudes[-1].header.stamp.nanosec) in emitted
        assert abs(rpys[-1].vector.x-.1) < 1e-9 and abs(rpys[-1].vector.y+.2) < 1e-9 and abs(rpys[-1].vector.z) < 1e-9
        checks.append('source IMU timestamps preserved; only startup yaw gauge changed')
        yaw = .85
        drive(.4)
        assert abs(rpys[-1].vector.z-.15) < 1e-9
        checks.append('existing fused IMU attitude forwarded without another filter')
        ready = False
        drive(.3)
        count = len(joints)
        drive(.2)
        assert len(joints) == count and not statuses[-1]['joint_state_available'] and statuses[-1]['attitude_available']
        checks.append('unready controller stops joint output while attitude remains independent')
        good_imu = False
        drive(.3)
        count = len(attitudes)
        drive(.2)
        assert len(attitudes) == count and not statuses[-1]['attitude_available']
        checks.append('invalid quaternion suppressed')
        ready, good_imu = True, True
        drive(.4)
        assert statuses[-1]['joint_state_available'] and statuses[-1]['attitude_available']
        drive(.4, publish=False)
        assert not statuses[-1]['joint_state_available'] and not statuses[-1]['attitude_available']
        checks.append('recovery and source timeout reported without fabricating a pose')
        result = {'domain': os.environ['ROS_DOMAIN_ID'], 'checks': checks, 'count': len(checks), 'hardware_used': False}
        Path('/tmp/wla-state-adapter-ros-test.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result))
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        assert process.returncode == 0, process.returncode


if __name__ == '__main__':
    main()
