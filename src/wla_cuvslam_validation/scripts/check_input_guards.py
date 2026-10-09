#!/usr/bin/env python3
"""Exercise installed C++ input guards with isolated synthetic ROS topics (no SDK initialization)."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import time
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    prefix = Path(get_package_prefix('wla_cuvslam_validation'))
    share = Path(get_package_share_directory('wla_cuvslam_validation'))
    directory = prefix / 'lib/wla_cuvslam_validation'
    spec = importlib.util.spec_from_file_location('run_validation', directory / 'run_validation.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        stats = Path(tmp) / 'stats.json'
        cmd = [str(directory / 'cuvslam_validation'), '--ros-args', '--params-file', str(share / 'config/validation.yaml')]
        for name, value in {'left_topic': '/r680_vio_guard_test/left', 'right_topic': '/r680_vio_guard_test/right',
                            'left_info_topic': '/r680_vio_guard_test/left_info', 'right_info_topic': '/r680_vio_guard_test/right_info',
                            'imu_topic': '/r680_vio_guard_test/imu', 'wheel_odom_topic': '/r680_vio_guard_test/wheel',
                            'output_topic': '/r680_vio_guard_test/odom', 'status_topic': '/r680_vio_guard_test/status',
                            'statistics_path': str(stats)}.items():
            cmd += ['-p', f'{name}:={value}']
        with (args.output.parent / 'input_guard.log').open('w') as log:
            child = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = Node('cuvslam_input_guard_regression')
            left = node.create_publisher(Image, '/r680_vio_guard_test/left', qos_profile_sensor_data)
            imu = node.create_publisher(Imu, '/r680_vio_guard_test/imu', qos_profile_sensor_data)
            try:
                deadline = time.monotonic() + 10
                while left.get_subscription_count() == 0 or imu.get_subscription_count() == 0:
                    if child.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('input guard node did not become ready')
                    rclpy.spin_once(node, timeout_sec=.1)
                def send(pub, msg):
                    pub.publish(msg)
                    rclpy.spin_once(node, timeout_sec=.1)
                bad = Image(width=2, height=2, step=6, encoding='rgb8', data=[0] * 12)
                send(left, bad)
                good = Image(width=2, height=2, step=2, encoding='mono8', data=[0] * 4)
                good.header.stamp = node.get_clock().now().to_msg()
                send(left, good)
                send(left, good)  # Same timestamp must be rejected.
                wrong = Imu()
                wrong.header.frame_id = 'wrong_imu_frame'
                wrong.header.stamp = node.get_clock().now().to_msg()
                send(imu, wrong)
                valid = Imu()
                valid.header.frame_id = 'gyro_link'
                valid.header.stamp = node.get_clock().now().to_msg()
                valid.linear_acceleration.z = 9.81
                send(imu, valid)
                send(imu, valid)  # Same timestamp must be rejected.
                invalid = Imu()
                invalid.header.frame_id = 'gyro_link'
                invalid.header.stamp = node.get_clock().now().to_msg()
                invalid.angular_velocity.z = float('nan')
                send(imu, invalid)
                time.sleep(.5)
            finally:
                node.destroy_node()
                rclpy.shutdown()
                helper.terminate(child)
        result = json.loads(stats.read_text())
        checks = {'invalid_images': result['invalid_images'] >= 1,
                  'out_of_order_images': result['out_of_order_images'] >= 1,
                  'invalid_imu': result['invalid_imu'] >= 2,
                  'out_of_order_imu': result['out_of_order_imu'] >= 1,
                  'no_pose_without_camera_info': result['published_frames'] == 0,
                  'no_sdk_fault': not result['faulted']}
        args.output.write_text(json.dumps({'checks': checks, 'statistics': result}, indent=2) + '\n')
        print(json.dumps(checks), flush=True)
        if not all(checks.values()):
            raise RuntimeError('input guard regression failed')


if __name__ == '__main__':
    main()
