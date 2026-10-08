#!/usr/bin/env python3
"""Domain-isolated watchdog respawn test; mock frontend, no motor publishers."""
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import Info
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '74', 'test requires isolated domain 74'
    with tempfile.TemporaryDirectory(prefix='wla-prior-test-') as temporary:
        root = Path(temporary)/'wla_cuvslam_navigation'
        root.mkdir()
        source = root/'mock.cpp'
        source.write_text('#include <unistd.h>\nint main(){for(;;) usleep(10000);}\n')
        executable = root/'cuvslam_odometry'
        subprocess.run(['g++', str(source), '-o', str(executable)], check=True)
        def spawn():
            return subprocess.Popen([str(executable), '__ns:=/d455_vio', '__node:=cuvslam_odometry'])
        mock = spawn()
        rclpy.init()
        node = rclpy.create_node('prior_regression_inputs')
        pubs = {
            'odom': node.create_publisher(Odometry, '/d455_slam/odom', 10),
            'wheel': node.create_publisher(Odometry, '/wheel/odom', 10),
            'health': node.create_publisher(Bool, '/r680_nav/vio_tracking_healthy', 10),
            'info': node.create_publisher(Info, '/d455_slam/info', 10),
            'pose': node.create_publisher(PoseWithCovarianceStamped, '/d455_slam/localization_pose', 10)}
        tf = TransformBroadcaster(node)
        state = {'health': False, 'prior': [], 'fault': False, 'matched': True, 'moving': False}
        node.create_subscription(Bool, '/r680_nav/vo_watchdog_healthy',
                                 lambda msg: state.update(health=msg.data), 10)
        node.create_subscription(PoseWithCovarianceStamped, '/d455_slam/initialpose',
                                 lambda msg: state['prior'].append(msg), 10)
        watchdog = subprocess.Popen([str(Path(get_package_prefix('wla_r680_navigation'))/
            'lib/wla_r680_navigation/vo_watchdog'),
            '--ros-args', '-p', 'frontend:=cuvslam', '-p', 'startup_grace_s:=5.0',
            '-p', 'vo_topic:=/d455_slam/odom',
            '-p', 'recovery_stable_s:=0.5', '-p', 'prior_stop_stable_s:=0.2',
            '-p', 'prior_tracking_stable_s:=0.2'], stdout=subprocess.DEVNULL)
        def tick():
            stamp = node.get_clock().now().to_msg()
            o = Odometry(); o.header.stamp = stamp; o.header.frame_id = 'd455_floor_odom'
            o.child_frame_id = 'r680_mapping_floor'; o.pose.pose.orientation.w = 1.0
            pubs['odom'].publish(o)
            w = Odometry(); w.header.stamp = stamp; w.header.frame_id = 'wheel_odom'
            w.child_frame_id = 'r680_mapping_floor'; w.pose.pose.orientation.w = 1.0
            w.pose.pose.position.x = 0.1 if state['fault'] else 0.0
            w.twist.twist.linear.x = 0.1 if state['moving'] else 0.0
            pubs['wheel'].publish(w); pubs['health'].publish(Bool(data=True))
            t = TransformStamped(); t.header.frame_id = 'map'; t.header.stamp = stamp
            t.child_frame_id = 'r680_mapping_floor'; t.transform.translation.x = 3.0
            t.transform.translation.y = 4.1 if state['fault'] else 4.0
            t.transform.rotation.z = math.sin(math.pi/4); t.transform.rotation.w = math.cos(math.pi/4)
            tf.sendTransform(t)
            info = Info(); info.header.stamp = stamp
            info.loop_closure_id = 1 if state['matched'] else 0; pubs['info'].publish(info)
            if state['matched']:
                pose = PoseWithCovarianceStamped(); pose.header.frame_id = 'map'; pose.header.stamp = stamp
                pose.pose.pose.position.x = t.transform.translation.x
                pose.pose.pose.position.y = t.transform.translation.y
                pose.pose.pose.orientation = t.transform.rotation; pubs['pose'].publish(pose)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.02)
        def until(predicate, seconds):
            end = time.monotonic()+seconds
            while time.monotonic()<end:
                tick()
                if predicate(): return
            raise AssertionError('watchdog transition timed out')
        try:
            until(lambda: state['health'], 8)
            state['matched'] = False
            bad = Odometry(); bad.header.stamp = node.get_clock().now().to_msg()
            bad.pose.pose.position.x = float('nan'); bad.pose.pose.orientation.w = 1.0
            pubs['odom'].publish(bad)
            until(lambda: mock.poll() is not None, 4)
            state['fault'] = True; state['moving'] = True; state['health'] = False
            mock = spawn()
            end = time.monotonic()+1.5
            while time.monotonic()<end: tick()
            assert not state['prior'], 'hint was sent while wheel feedback showed motion'
            state['moving'] = False
            until(lambda: state['prior'], 4)
            prior = state['prior'][0].pose.pose
            assert abs(prior.position.x-3)<0.01 and abs(prior.position.y-4.1)<0.01
            assert not state['health'], 'prior incorrectly unlocked motion without a map match'
            end = time.monotonic()+1
            while time.monotonic()<end: tick()
            assert len(state['prior']) == 1, 'initialpose was repeatedly overwritten'
            state['matched'] = True
            until(lambda: state['health'], 4)
            print('PASS: stationary prior, SE2 wheel delta, one hint, map-match lock, stable unlock')
        finally:
            watchdog.terminate()
            try: watchdog.wait(timeout=4)
            except subprocess.TimeoutExpired: watchdog.kill(); watchdog.wait()
            mock.terminate(); mock.wait(timeout=3)
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
