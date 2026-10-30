#!/usr/bin/env python3
"""odom_fusion node in an isolated graph: topics, states and TF, without motion."""
import math
import os
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray, String, UInt32
from tf2_msgs.msg import TFMessage


def main():
    assert 100 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 232
    rclpy.init()
    n = rclpy.create_node('odom_fusion_regression')
    pub = lambda kind, topic, qos=10: n.create_publisher(kind, topic, qos)
    imu = pub(Imu, '/wheel/imu/data_raw', qos_profile_sensor_data)
    wheel = pub(Odometry, '/wheel/odom', qos_profile_sensor_data)
    raw = pub(Odometry, '/r680_nav/vio_raw_odom', qos_profile_sensor_data)
    visual = pub(Bool, '/r680_nav/vio_visual_observed')
    tracking = pub(Bool, '/r680_nav/vio_tracking_healthy')
    quality = pub(Float32MultiArray, '/r680_nav/vio_visual_quality')
    out, healthy, degraded, states, epochs, tf = [], [], [], [], [], []
    n.create_subscription(Odometry, '/d455_slam/odom', out.append, 50)
    n.create_subscription(Bool, '/r680_nav/continuous_odom_healthy', lambda m: healthy.append(m.data), 50)
    n.create_subscription(Bool, '/r680_nav/odom_degraded', lambda m: degraded.append(m.data), 50)
    n.create_subscription(String, '/r680_nav/odom_state', lambda m: states.append(m.data), 50)
    n.create_subscription(UInt32, '/r680_nav/odom_epoch', lambda m: epochs.append(m.data), QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(TFMessage, '/tf', lambda m: tf.extend(
        t for t in m.transforms if t.child_frame_id == 'r680_mapping_floor'), 50)
    proc = subprocess.Popen([os.environ['WLA_FUSION_EXECUTABLE'], '--ros-args', '-p', 'keyframe_lag_s:=0.05'])
    pose = [0.0, 0.0, 0.0]
    flags = dict(visual=True, wheel=True, speed=0.0)

    def feed(seconds):
        end = time.monotonic() + seconds
        next_visual = 0.0
        last = time.monotonic()
        while time.monotonic() < end:
            assert proc.poll() is None, 'odom_fusion exited'
            stamp = n.get_clock().now().to_msg()
            # Truth advances with real elapsed time, as the wheel stamps do.
            now = time.monotonic()
            dt, last = now - last, now
            pose[0] += flags['speed'] * dt * math.cos(pose[2])
            pose[1] += flags['speed'] * dt * math.sin(pose[2])
            m = Imu()
            m.header.stamp, m.header.frame_id = stamp, 'gyro_link'
            m.linear_acceleration.z = 9.81
            imu.publish(m)
            if flags['wheel']:
                w = Odometry()
                w.header.stamp, w.header.frame_id, w.child_frame_id = stamp, 'odom', 'base'
                w.twist.twist.linear.x = flags['speed']
                wheel.publish(w)
            now = time.monotonic()
            if now >= next_visual:
                next_visual = now + 1 / 30
                visual.publish(Bool(data=flags['visual']))
                tracking.publish(Bool(data=True))
                quality.publish(Float32MultiArray(data=[150.0, 0.5]))
                r = Odometry()
                r.header.stamp, r.header.frame_id, r.child_frame_id = stamp, 'd455_floor_odom', 'r680_mapping_floor'
                r.pose.pose.position.x, r.pose.pose.position.y = pose[0] + 5.0, pose[1]  # cuVSLAM frame
                r.pose.pose.orientation.w = 1.0
                raw.publish(r)
            for _ in range(50):  # the node publishes ~400 msg/s; keep up with it
                rclpy.spin_once(n, timeout_sec=0)
            time.sleep(0.005)

    try:
        feed(3.0)
        assert states and states[-1] == 'GOOD', states[-5:]
        assert healthy[-1] and not degraded[-1] and epochs and epochs[-1] == 0
        assert abs(out[-1].pose.pose.position.x - 5.0) < 0.02, 'odometry not anchored on cuVSLAM'
        assert tf and abs(tf[-1].transform.translation.x - out[-1].pose.pose.position.x) < 0.05
        print('PASS anchors on cuVSLAM, GOOD, epoch 0, publishes odom and TF', flush=True)

        flags.update(speed=0.15, visual=False)
        feed(2.0)
        assert states[-1] == 'BRIDGING' and healthy[-1] and degraded[-1], states[-5:]
        moved = out[-1].pose.pose.position.x - 5.0
        assert abs(moved - pose[0]) < 0.05, (moved, pose[0])
        print('PASS vision loss while moving bridges on wheel/gyro (degraded, healthy)', flush=True)

        flags.update(visual=True)
        feed(1.5)
        assert states[-1] == 'GOOD' and epochs[-1] == 0
        print('PASS vision returns to GOOD without a new epoch', flush=True)

        flags.update(wheel=False)
        feed(1.0)
        assert states[-1] == 'LOST' and not healthy[-1], states[-5:]
        print('PASS stale wheel input is LOST and unhealthy', flush=True)

        flags.update(wheel=True, speed=0.0)
        feed(3.0)
        assert states[-1] == 'GOOD' and epochs[-1] == 1, (states[-5:], epochs)
        print('PASS re-anchored by vision with epoch 1', flush=True)
    finally:
        proc.terminate()
        proc.wait(timeout=5)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
