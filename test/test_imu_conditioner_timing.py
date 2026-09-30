"""Sensor-only regression: physical-time calibration at 20/200 Hz and reset gates."""
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def run_case(binary, rate, duration=5.0, reset=False):
    node = rclpy.create_node(f'conditioner_timing_probe_{rate}')
    topic = f'/wla_imu_timing_test/rate_{rate}'
    pub = node.create_publisher(Imu, topic + '/input', qos_profile_sensor_data)
    outputs = []
    sub = node.create_subscription(Imu, topic + '/output',
                                   lambda msg: outputs.append((time.monotonic(), msg)),
                                   qos_profile_sensor_data)
    child = subprocess.Popen([str(binary), '--ros-args',
        '-r', f'__node:=conditioner_timing_{rate}',
        '-p', f'input_topic:={topic}/input', '-p', f'output_topic:={topic}/output',
        '-p', f'calibration_duration_s:={duration}'])
    try:
        deadline = time.monotonic() + 5
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert pub.get_subscription_count() == 1, 'conditioner not discovered'
        start = time.monotonic()
        next_tick = start
        end = start + duration + 1.1 + (1 if reset else 0)
        while time.monotonic() < end:
            msg = Imu()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'gyro_link'
            elapsed = time.monotonic() - start
            msg.angular_velocity.x = .01
            msg.angular_velocity.y = -.02
            msg.angular_velocity.z = .005
            msg.linear_acceleration.z = 9.81
            if reset and .8 <= elapsed < 1:
                msg.angular_velocity.z = 1.0
            pub.publish(msg)
            next_tick += 1 / rate
            while time.monotonic() < next_tick:
                rclpy.spin_once(node, timeout_sec=min(.002, max(0, next_tick - time.monotonic())))
        assert child.poll() is None, 'conditioner exited'
        assert outputs, 'conditioner never calibrated'
        delay = outputs[0][0] - start
        assert delay >= duration + (1 if reset else 0) - .03, f'premature calibration: {delay}'
        assert delay < duration + (1 if reset else 0) + .4, f'late calibration: {delay}'
        for _, msg in outputs:
            assert abs(msg.angular_velocity.x) < 1e-8
            assert abs(msg.angular_velocity.y) < 1e-8
            assert abs(msg.angular_velocity.z) < 1e-8
            assert msg.orientation_covariance[0] == -1
        assert len(outputs) >= rate * .7, 'post-calibration throughput too low'
        print(f'PASS rate={rate} reset={reset} first_output_s={delay:.3f} outputs={len(outputs)}', flush=True)
    finally:
        child.send_signal(signal.SIGINT)
        child.wait(timeout=5)
        node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try:
        binary = Path(os.environ.get('IMU_CONDITIONER_BINARY',
            '/home/orin/ros2_ws/install/wla_r680_navigation/lib/wla_r680_navigation/imu_conditioner'))
        run_case(binary, 20)
        run_case(binary, 200)
        run_case(binary, 200, duration=1., reset=True)
    finally:
        rclpy.shutdown()
