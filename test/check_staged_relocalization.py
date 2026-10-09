#!/usr/bin/env python3
"""Domain-isolated watchdog respawn test; mock frontend, no motor publishers."""
import math
import sys
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
from std_msgs.msg import Bool, String
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
        node = rclpy.create_node('staged_regression_inputs')
        backend = rclpy.create_node('rtabmap', namespace='/d455_slam')
        baseline = {'RGBD/ProximityBySpace': 'true', 'RGBD/LocalRadius': '10',
                    'RGBD/ProximityAngle': '45', 'RGBD/ProximityMaxPaths': '3',
                    'Rtabmap/LoopThr': '0.11', 'RGBD/AggressiveLoopThr': '0.05',
                    'Reg/Strategy': '0', 'Vis/EstimationType': '1'}
        for key, value in baseline.items(): backend.declare_parameter(key, value)
        pubs = {
            'odom': node.create_publisher(Odometry, '/d455_slam/odom', 10),
            'wheel': node.create_publisher(Odometry, '/wheel/odom', 10),
            'health': node.create_publisher(Bool, '/r680_nav/vio_tracking_healthy', 10),
            'info': node.create_publisher(Info, '/d455_slam/info', 10),
            'pose': node.create_publisher(PoseWithCovarianceStamped, '/d455_slam/localization_pose', 10)}
        tf = TransformBroadcaster(node)
        state = {'health': False, 'prior': [], 'fault': False, 'matched': True, 'moving': False, 'stage': '', 'fixed_stamp': None, 'jitter': False, 'frame': 0}
        node.create_subscription(Bool, '/r680_nav/vo_watchdog_healthy',
                                 lambda msg: state.update(health=msg.data), 10)
        node.create_subscription(PoseWithCovarianceStamped, '/d455_slam/initialpose',
                                 lambda msg: state['prior'].append(msg), 10)
        node.create_subscription(String, '/r680_nav/relocalization_status',
                                 lambda msg: state.update(stage=msg.data.split()[0]), 10)
        watchdog = subprocess.Popen([str(Path(get_package_prefix('wla_r680_navigation'))/
            'lib/wla_r680_navigation/vo_watchdog'),
            '--ros-args', '-p', 'frontend:=cuvslam', '-p', 'staged_relocalization_enabled:=true',
            '-p', 'recovery_near_timeout_s:=2.0', '-p', 'recovery_expanded_timeout_s:=2.0',
            '-p', 'recovery_vpr_timeout_s:=15.0', '-p', 'startup_grace_s:=5.0',
            '-p', 'vo_topic:=/d455_slam/odom',
            '-p', 'recovery_stable_s:=0.5', '-p', 'prior_stop_stable_s:=0.2',
            '-p', 'prior_tracking_stable_s:=0.2'], stdout=subprocess.DEVNULL)
        def tick():
            stamp = state['fixed_stamp'] or node.get_clock().now().to_msg()
            o = Odometry(); o.header.stamp = stamp; o.header.frame_id = 'd455_floor_odom'
            o.child_frame_id = 'r680_mapping_floor'; o.pose.pose.orientation.w = 1.0
            pubs['odom'].publish(o)
            w = Odometry(); w.header.stamp = stamp; w.header.frame_id = 'wheel_odom'
            w.child_frame_id = 'r680_mapping_floor'; w.pose.pose.orientation.w = 1.0
            w.pose.pose.position.x = 0.1 if state['fault'] else 0.0
            w.twist.twist.linear.x = 0.1 if state['moving'] else 0.0
            pubs['wheel'].publish(w); pubs['health'].publish(Bool(data=True))
            t = TransformStamped(); t.header.frame_id = 'map'; t.header.stamp = stamp
            t.child_frame_id = 'r680_mapping_floor'; t.transform.translation.x = 5.0 if state['stage'] in ('vpr', 'verified') else 3.0
            if state['jitter']: t.transform.translation.x += 0.15 if int(time.monotonic()*5)%2 else 0.0
            t.transform.translation.y = 4.1 if state['fault'] else 4.0
            t.transform.rotation.z = math.sin(math.pi/4); t.transform.rotation.w = math.cos(math.pi/4)
            tf.sendTransform(t)
            info = Info(); info.header.stamp = stamp
            info.loop_closure_id = 1 if state['matched'] else 0
            info.ref_id = state['frame']; state['frame'] += 1
            info.posterior_keys = [1, 2]; info.posterior_values = [0.8, 0.1]
            pubs['info'].publish(info)
            if state['matched']:
                pose = PoseWithCovarianceStamped(); pose.header.frame_id = 'map'; pose.header.stamp = stamp
                pose.pose.pose.position.x = t.transform.translation.x
                pose.pose.pose.position.y = t.transform.translation.y
                pose.pose.pose.orientation = t.transform.rotation; pubs['pose'].publish(pose)
            rclpy.spin_once(backend, timeout_sec=0.001)
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
            bad = Odometry(); bad.header.stamp = state['fixed_stamp'] or node.get_clock().now().to_msg()
            bad.pose.pose.position.x = float('nan'); bad.pose.pose.orientation.w = 1.0
            pubs['odom'].publish(bad)
            until(lambda: mock.poll() is not None, 4)
            state['fault'] = True; state['moving'] = True; state['health'] = False
            mock = spawn()
            end = time.monotonic()+1.5
            while time.monotonic()<end: tick()
            assert not state['prior'], 'hint was sent while wheel feedback showed motion'
            state['moving'] = False
            until(lambda: state['stage'] == 'near', 6)
            assert not state['health'], 'near stage unlocked without geometry'
            until(lambda: state['stage'] == 'expanded', 6)
            assert backend.get_parameter('RGBD/LocalRadius').value == '3.000000'
            until(lambda: state['stage'] == 'vpr', 6)
            assert backend.get_parameter('Rtabmap/LoopThr').value == '0.11'
            assert backend.get_parameter('Vis/EstimationType').value == '0'
            assert not state['health'], 'VPR candidate scores unlocked without a verified match'
            if '--timeout' in sys.argv:
                until(lambda: state['stage'] == 'manual_required', 18)
                assert not state['health'], 'failed VPR unlocked motion'
                until(lambda: backend.get_parameter('Rtabmap/LoopThr').value == '0.11', 3)
                hint = PoseWithCovarianceStamped(); hint.header.frame_id = 'map'
                hint.header.stamp = node.get_clock().now().to_msg(); hint.pose.pose.orientation.w = 1.0
                state['prior'].clear()
                publisher = node.create_publisher(PoseWithCovarianceStamped, '/d455_slam/initialpose', 10)
                end = time.monotonic()+0.3
                while time.monotonic()<end: tick()
                hint.header.stamp = node.get_clock().now().to_msg(); publisher.publish(hint)
                until(lambda: state['stage'] == 'vpr', 3)
                assert not state['health'], 'manual hint bypassed geometry verification'
                print('PASS: exhausted search remains locked, parameters restored, manual hint retries without unlocking')
            else:
                end = time.monotonic()+1.2
                while time.monotonic()<end: tick()
                state['matched'] = True; state['jitter'] = True
                end = time.monotonic()+2.0
                while time.monotonic()<end: tick()
                assert not state['health'], 'unstable map poses unlocked motion'
                state['jitter'] = False
                until(lambda: state['health'], 5)
                for key, value in baseline.items():
                    assert backend.get_parameter(key).value == value, key + ' was not restored'
                print('PASS: near -> expanded -> VPR, candidate-only lock, unstable-pose lock, repeated geometry, stable unlock, parameter restoration')
        finally:
            watchdog.terminate()
            try: watchdog.wait(timeout=4)
            except subprocess.TimeoutExpired: watchdog.kill(); watchdog.wait()
            mock.terminate(); mock.wait(timeout=3)
            backend.destroy_node(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
