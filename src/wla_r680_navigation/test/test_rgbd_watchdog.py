#!/usr/bin/env python3
"""Run the installed watchdog against synthetic RGB-D/map evidence, no motors."""
import os
from pathlib import Path
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import Info, OdomInfo
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster


def exercise(frontend, bounded=False, initial_match=False, startup_timeout_case=None,
             continuity_recovery=False):
    # This test owns synthetic navigation input names and must never join the
    # production ROS domain. The watchdog cannot terminate any frontend here.
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    assert 100 <= domain <= 232, 'set an isolated ROS_DOMAIN_ID in [100, 232]'
    rclpy.init()
    node = rclpy.create_node('rgbd_map_gate_regression')
    publishers = {
        'vo': node.create_publisher(Odometry, '/r680_nav/vo_odom', 10),
        'vio': node.create_publisher(Bool, '/r680_nav/vio_tracking_healthy', 10),
        'continuous': node.create_publisher(Bool, '/r680_nav/continuous_odom_healthy', 10),
        'degraded': node.create_publisher(Bool, '/r680_nav/odom_degraded', 10),
        'prior': node.create_publisher(PoseWithCovarianceStamped, '/d455_slam/initialpose', 10),
        'odom_info': node.create_publisher(OdomInfo, '/d455_slam/odom_info', 10),
        'map_info': node.create_publisher(Info, '/d455_slam/info', 10),
        'pose': node.create_publisher(PoseWithCovarianceStamped,
                                      '/d455_slam/localization_pose', 10),
    }
    broadcaster = TransformBroadcaster(node)
    received = []
    statuses = []
    node.create_subscription(String, "/r680_nav/vo_watchdog_status",
                             lambda msg: statuses.append(msg.data), 10)
    node.create_subscription(Bool, '/r680_nav/vo_watchdog_healthy',
                             lambda msg: received.append((time.monotonic(), msg.data)), 10)
    executable = (Path(get_package_prefix('wla_r680_navigation')) /
                  'lib/wla_r680_navigation/vo_watchdog')
    executable = os.environ.get('WLA_WATCHDOG_EXECUTABLE',str(executable))
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'frontend:='+frontend,
        '-p', 'allow_bounded_degraded:='+str(bounded).lower(),
        '-p', 'lost_limit:='+('3' if continuity_recovery else '10000'),
        '-p', 'restart_enabled:=false', '-p', 'startup_grace_s:='+('1.0' if startup_timeout_case or continuity_recovery else '120.0'),
        '-p', 'localization_stale_s:=0.4', '-p', 'stale_s:=1.5',
    ], stdout=subprocess.DEVNULL)

    def tick(*, matched=False, pose=False, transform=True, vio=True, continuity=True,
             degraded=False, info_offset=0., pose_offset=0., tf_x=2., prior=False,
             rgbd_quality=True, odom_offset=0., publish_vo=True,
             publish_vio=True, publish_continuity=True):
        assert process.poll() is None, 'watchdog exited unexpectedly'
        stamp = node.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp = (node.get_clock().now()-rclpy.duration.Duration(seconds=odom_offset)).to_msg()
        odometry.header.frame_id = 'd455_floor_odom'
        odometry.child_frame_id = 'r680_mapping_floor'
        odometry.pose.pose.orientation.w = 1.
        if publish_vo: publishers['vo'].publish(odometry)
        quality = OdomInfo()
        quality.header.stamp = stamp
        quality.lost = False
        if rgbd_quality: publishers['odom_info'].publish(quality)
        if publish_vio: publishers['vio'].publish(Bool(data=vio))
        if publish_continuity: publishers['continuous'].publish(Bool(data=continuity))
        publishers['degraded'].publish(Bool(data=degraded))
        info = Info()
        info.header.stamp = (rclpy.time.Time.from_msg(stamp)-rclpy.duration.Duration(seconds=info_offset)).to_msg()
        info.loop_closure_id = 1 if matched else 0
        publishers['map_info'].publish(info)
        if pose:
            location = PoseWithCovarianceStamped()
            location.header.stamp = (rclpy.time.Time.from_msg(stamp)-rclpy.duration.Duration(seconds=pose_offset)).to_msg()
            location.header.frame_id = 'map'
            location.pose.pose.position.x = 2.
            location.pose.pose.orientation.w = 1.
            publishers['pose'].publish(location)
            if prior: publishers['prior'].publish(location)
        if transform:
            tf = TransformStamped()
            # Match the real backend's future-dated map->odom broadcast.
            tf.header.stamp = (rclpy.time.Time.from_msg(stamp)+rclpy.duration.Duration(seconds=.1)).to_msg()
            tf.header.frame_id = 'map'
            tf.child_frame_id = 'd455_floor_odom'
            tf.transform.translation.x = tf_x
            tf.transform.rotation.w = 1.
            base = TransformStamped()
            base.header.stamp = stamp
            base.header.frame_id = 'd455_floor_odom'
            base.child_frame_id = 'r680_mapping_floor'
            base.transform.rotation.w = 1.
            broadcaster.sendTransform([tf,base])
        rclpy.spin_once(node, timeout_sec=.005)
        time.sleep(.02)

    def phase(seconds, **inputs):
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            tick(**inputs)
        return [value for stamp, value in received if stamp >= start + max(0.,seconds-.5)]

    def expect_closed(label, **inputs):
        evidence = phase(2.1, **inputs)
        assert len(evidence) >= 3, label + ': no health evidence'
        assert not any(evidence), label + ': admitted motion'

    def expect_open(label, **inputs):
        deadline = time.monotonic() + 5.
        start = time.monotonic()
        while time.monotonic() < deadline:
            tick(**inputs)
            if received and received[-1][0] >= start and received[-1][1]:
                return
        raise AssertionError(label + ': matching map did not admit navigation')

    try:
        if continuity_recovery:
            assert frontend == 'cuvslam' and bounded
            phase(1.)
            expect_open('initially healthy', matched=True, pose=True)
            start = time.monotonic()
            phase(.4, continuity=False, publish_vo=False, matched=True, pose=True)
            closed = [v for t, v in received if t >= start + .2]
            assert closed and not any(closed), 'unavailable odometry admitted motion'
            assert not any('continuous odometry unhealthy' in s or 'timed out' in s
                           for s in statuses), statuses[-5:]
            expect_open('short interruption recovers without restart', matched=True, pose=True)
            expect_closed('continuity false with live frontend', continuity=False,
                          matched=True, pose=True)
            assert not any('relocalization_required' in s or 'restarting_vo' in s
                           for s in statuses), statuses[-5:]
            expect_open('continuity return recovers without restart', matched=True, pose=True)
            expect_closed('genuine odometry silence still detected', publish_vo=False, pose=True)
            assert 'timed out' in statuses[-1], statuses[-5:]
            assert all(not node.get_publishers_info_by_topic(t) for t in
                       ('/cmd_vel', '/wheel/cmd_vel', '/cmd_vel_nav', '/cmd_vel_safe'))
            print('PASS continuity loss does not latch/restart; recovery and real timeout checked', flush=True)
            return
        if startup_timeout_case:
            phase(.5, vio=False, continuity=False, publish_vo=False)
            if startup_timeout_case == 'live':
                expect_closed('live warmup stays closed without odometry', vio=False,
                              continuity=False, publish_vo=False, matched=True, pose=True)
                assert statuses and statuses[-1] == 'waiting_for_inertial_initialization', statuses[-3:]
                expect_open('live warmup can initialize without restart', matched=True, pose=True)
                expect_closed('odometry loss after readiness still faults', publish_vo=False, pose=True)
                assert 'timed out' in statuses[-1], statuses[-3:]
            else:
                expect_closed('missing warmup heartbeat faults', vio=False, continuity=False,
                    publish_vo=False, publish_vio=startup_timeout_case != 'frontend_dead',
                    publish_continuity=startup_timeout_case != 'continuity_dead')
                assert statuses and 'timed out' in statuses[-1], statuses[-3:]
            print('PASS startup timeout '+startup_timeout_case+' bounded='+str(bounded), flush=True)
            return
        if initial_match:
            assert frontend == 'cuvslam'
            phase(1.0, vio=False, continuity=False)
            phase(1.0, matched=True, pose=True, vio=False, continuity=False,
                  info_offset=2.)
            expect_closed('stale initial match rejected', pose=True)
            phase(1.0, matched=True, pose=True, vio=False, continuity=False)
            phase(.8, vio=False, continuity=False)
            expect_closed('early match still requires fresh pose')
            expect_closed('early match still requires consistent TF', pose=True, tf_x=5.)
            expect_open('early match survives inertial initialization', pose=True)
            expect_closed('initial admission expires without fresh pose')
            expect_closed('startup match cannot reopen after loss', pose=True)
            expect_open('fresh paired match recovers after loss', matched=True, pose=True)
            forbidden=('/cmd_vel','/wheel/cmd_vel','/cmd_vel_nav','/cmd_vel_safe')
            assert all(not node.get_publishers_info_by_topic(t) for t in forbidden)
            print('PASS cuVSLAM early map match bounded='+str(bounded), flush=True)
            return
        # Discovery first, then assertions require actual received health.
        phase(1.0)
        expect_closed('local odometry and map TF without localization')
        expect_closed('pose and TF without geometric match', pose=True)
        phase(.7)  # expire the previous pose before introducing a match
        expect_closed('match Info without a localization pose', matched=True)
        expect_open('matched pose and TF', matched=True, pose=True)
        expect_closed('expired localization pose', matched=True)
        phase(.6)  # let the last unpaired match candidate expire too
        expect_closed('pose only cannot reuse expired match', pose=True)
        expect_open('fresh match restores localization', matched=True, pose=True)
        expect_closed('expired map TF', pose=True, transform=False)
        expect_closed('TF return alone cannot reuse expired match', pose=True)
        expect_closed('stale match rejected', matched=True, pose=True, info_offset=2.)
        expect_closed('unpaired backend stamps rejected', matched=True, pose=True, info_offset=.2)
        expect_closed('pose/TF disagreement rejected', matched=True, pose=True, tf_x=5.)
        expect_closed('manual prior is not localization', matched=True, pose=True, prior=True)
        expect_open('new paired evidence restores localization', matched=True, pose=True)
        expect_open('continuous map tracking without repeated loops', pose=True)
        expect_closed('stale odometry source cannot stay healthy', pose=True, odom_offset=3.)
        expect_closed('returning odometry needs new map proof', pose=True)
        expect_open('recover from stale odometry', matched=True, pose=True)
        if frontend=='cuvslam':
            expect_open('RGBD OdomInfo not required by VIO', matched=True, pose=True, rgbd_quality=False)
            if bounded:
                assert any(phase(.3,vio=False,degraded=True)), 'bounded continuity was prematurely closed'
                expect_closed('bounded map grace expires', vio=False,degraded=True,transform=False)
                expect_closed('continuous output cannot initialize map', matched=True,pose=True,vio=False,degraded=True)
                expect_open('VIO and map recover',matched=True,pose=True)
                expect_closed('continuous health failure',matched=True,pose=True,continuity=False)
            expect_closed('RGBD info cannot override failed VIO', matched=True,pose=True,vio=False)
            expect_open('VIO resumes with fresh map proof',matched=True,pose=True)
        else:
            expect_closed('VIO bool cannot replace RGBD OdomInfo',matched=True,pose=True,rgbd_quality=False)
        forbidden=('/cmd_vel','/wheel/cmd_vel','/cmd_vel_nav','/cmd_vel_safe')
        assert all(not node.get_publishers_info_by_topic(t) for t in forbidden)
        print('PASS frontend='+frontend+' bounded='+str(bounded),flush=True)
        print('PASS RGB-D map admission: odom-only, pose-only, match-only, '
              'matched localization, pose expiry, TF expiry and recovery')
    finally:
        process.terminate()
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


def test_rgbd_requires_map_localization():
    for frontend,bounded in [('rgbd',False),('cuvslam',False),('cuvslam',True)]:
        exercise(frontend,bounded)


if __name__ == '__main__':
    test_rgbd_requires_map_localization()
