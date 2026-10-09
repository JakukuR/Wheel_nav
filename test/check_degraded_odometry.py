#!/usr/bin/env python3
"""Isolated ROS graph regression. Fake sensors, preview-only command guard, no hardware.
Run with ROS_DOMAIN_ID=174 and Jazzy/navigation overlays sourced.
"""
import math
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

assert os.environ.get('ROS_DOMAIN_ID') == '174', 'Use isolated ROS_DOMAIN_ID=174'
root=Path('/home/orin/ros2_ws/install/wla_r680_navigation/lib/wla_r680_navigation')
config=Path('/home/orin/ros2_ws/install/wla_r680_navigation/share/wla_r680_navigation/config/degraded_odometry.yaml')
logs=Path('/tmp/wla-degraded-odom-tests');logs.mkdir(exist_ok=True)

def run_case(case):
    commands=[
        [str(root/'degraded_odometry'),'--ros-args','--params-file',str(config)],
        [str(root/'vo_watchdog'),'--ros-args','-p','frontend:=cuvslam','-p','vo_topic:=/d455_slam/odom',
         '-p','frontend_health_topic:=/r680_nav/continuous_odom_healthy','-p','allow_bounded_degraded:=true',
         '-p','restart_enabled:=false','-p','startup_grace_s:=20.0'],
        [str(root/'interface_monitor'),'--ros-args','-p','require_chassis_odom:=true',
         '-p','require_vo_watchdog:=true','-p','require_vio_health:=true',
         '-p','vio_health_topic:=/r680_nav/continuous_odom_healthy'],
        [str(root/'command_guard'),'--ros-args','-p','hardware_output_enabled:=false',
         '-p','degraded_mode_enabled:=true'],
    ]
    if case=='disabled':commands[0].extend(['-p','enabled:=false'])
    files=[open(logs/f'{case}-{i}.log','w') for i in range(len(commands))]
    processes=[subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT) for cmd,f in zip(commands,files)]
    rclpy.init();node=rclpy.create_node('degraded_odom_regression_'+case)
    try:
        publishers={}
        def publisher(cls,topic,sensor=False):
            if topic not in publishers:
                publishers[topic]=node.create_publisher(cls,topic,qos_profile_sensor_data if sensor else 10)
            return publishers[topic]
        def send(msg,topic,sensor=False):publisher(type(msg),topic,sensor).publish(msg)
        observed={'health':False,'watchdog':False,'degraded':False,'ready':False,'command':Twist(),
                  'state':'','poses':[]}
        for key,topic in [('health','/r680_nav/continuous_odom_healthy'),('watchdog','/r680_nav/vo_watchdog_healthy'),
                          ('degraded','/r680_nav/odom_degraded'),('ready','/r680_nav/localization_ready')]:
            node.create_subscription(Bool,topic,lambda m,k=key:observed.__setitem__(k,m.data),10)
        node.create_subscription(String,'/r680_nav/continuous_odom_status',lambda m:observed.__setitem__('state',m.data),10)
        node.create_subscription(Twist,'/r680_nav/cmd_vel_safe_preview',lambda m:observed.__setitem__('command',m),10)
        node.create_subscription(Odometry,'/d455_slam/odom',lambda m:observed['poses'].append(m),10)
        tf=TransformBroadcaster(node)
        mode='startup';wheel=True;imu=True
        motion=(0.1,0.2) if case=='no_bias' else (0.0,0.0)
        collision=False;fresh_map=case!='no_anchor'
        x=y=heading=0.0;last=time.monotonic();iteration=0;held_raw_stamp=None
        def step():
            nonlocal x,y,heading,last,iteration,held_raw_stamp
            current=time.monotonic();dt=current-last;last=current;iteration+=1
            v,w=motion;half=w*dt/2;sinc=1 if abs(half)<1e-8 else math.sin(half)/half
            x+=v*dt*sinc*math.cos(heading+half);y+=v*dt*sinc*math.sin(heading+half);heading+=w*dt
            stamp=node.get_clock().now().to_msg()
            if imu:
                m=Imu();m.header.stamp=stamp;m.header.frame_id='gyro_link'
                m.angular_velocity.z=w+0.01;m.linear_acceleration.z=9.81
                send(m,'/wheel/imu/data_raw',True)
            if iteration%4==0:
                if wheel:
                    m=Odometry();m.header.stamp=stamp;m.header.frame_id='wheel_odom';m.child_frame_id='base_footprint'
                    m.pose.pose.position.x=x;m.pose.pose.position.y=y
                    m.pose.pose.orientation.z=math.sin(heading/2);m.pose.pose.orientation.w=math.cos(heading/2)
                    m.twist.twist.linear.x=v;m.twist.twist.angular.z=w;send(m,'/wheel/odom',True)
                send(Bool(data=True),'/r680_nav/mission_motion_allowed')
                c=Twist();c.linear.x=0.0 if collision else 0.4;c.angular.z=0.0 if collision else 0.6
                send(c,'/r680_nav/cmd_vel_collision_checked')
                send(Image(),'/r680/d455/color/image_raw',True)
                send(Image(),'/r680/d455/aligned_depth_to_color/image_raw',True)
                info=CameraInfo();info.k=[500.,0.,320.,0.,500.,240.,0.,0.,1.]
                send(info,'/r680/d455/color/camera_info',True)
                cloud=PointCloud2();cloud.header.stamp=stamp;cloud.header.frame_id='r680_mapping_floor'
                cloud.height=1;cloud.point_step=12;send(cloud,'/r680_nav/d455/points',True)
                transform=TransformStamped();transform.header.stamp=stamp;transform.header.frame_id='map'
                transform.child_frame_id='d455_floor_odom';transform.transform.rotation.w=1.0;tf.sendTransform(transform)
                if fresh_map:
                    p=PoseWithCovarianceStamped();p.header.stamp=stamp;p.header.frame_id='map'
                    p.pose.pose.position.x=x;p.pose.pose.position.y=y;p.pose.pose.orientation.w=1.0
                    p.pose.covariance[0]=p.pose.covariance[7]=p.pose.covariance[35]=0.01
                    send(p,'/d455_slam/localization_pose',True)
                    inf=Info();inf.header.stamp=stamp;inf.loop_closure_id=10;send(inf,'/d455_slam/info',True)
            if iteration%7==0:
                healthy=mode in ('visual','inertial_only','late','invalid_pose','future_pose')
                evidence=mode in ('visual','return','mismatch','duplicate','inertial_only','late','invalid_pose','future_pose')
                status='tracking_inertial_ready' if healthy else (
                    'tracking_lost_no_output' if mode=='lost' else 'invalid_pose_no_output' if mode=='fatal'
                    else 'waiting_for_stable_inertial_initialization')
                send(Bool(data=healthy),'/r680_nav/vio_tracking_healthy')
                send(Bool(data=evidence),'/r680_nav/vio_inertial_valid')
                send(Bool(data=evidence and mode!='inertial_only'),'/r680_nav/vio_visual_observed')
                send(String(data=status),'/r680_nav/vio_status')
                if mode not in ('lost','fatal'):
                    m=Odometry();m.header.stamp=stamp;m.header.frame_id='d455_floor_odom';m.child_frame_id='r680_mapping_floor'
                    if mode=='late':
                        m.header.stamp=Time(nanoseconds=node.get_clock().now().nanoseconds-192_000_000).to_msg()
                    if mode=='future_pose':
                        m.header.stamp=Time(nanoseconds=node.get_clock().now().nanoseconds+100_000_000).to_msg()
                    if mode=='duplicate':
                        if held_raw_stamp is None:held_raw_stamp=stamp
                        m.header.stamp=held_raw_stamp
                    m.pose.pose.position.x=x+(1 if mode=='mismatch' else 0);m.pose.pose.position.y=y
                    m.pose.pose.orientation.z=math.sin(heading/2);m.pose.pose.orientation.w=math.cos(heading/2)
                    if mode=='invalid_pose':m.pose.pose.orientation.w=2.0
                    m.twist.twist.linear.x=v;m.twist.twist.angular.z=w
                    for i in range(6):m.pose.covariance[7*i]=0.02 if case=='high_uncertainty' else 0.0001
                    send(m,'/r680_nav/vio_raw_odom',True)
            # Drain observations; one spin per IMU sample backlogs the 100 Hz TF/health stream.
            for _ in range(12):rclpy.spin_once(node,timeout_sec=0)
            time.sleep(0.004)
        def pump(seconds):
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                step()
                active=processes[1:] if case=='bridge_exit' else processes
                assert all(p.poll() is None for p in active), 'node exited'
        pump(1.6)
        assert not observed['health'] and not observed['degraded'], observed
        assert observed['poses'], 'warming TF/odom missing: initialization/costmap deadlock'
        assert abs(observed['command'].linear.x)<1e-8, 'startup bypass'
        mode='visual';pump(1.0)
        assert observed['health'],observed['state']
        if case=='no_anchor':
            assert not observed['watchdog'] and abs(observed['command'].linear.x)<1e-8
        else:
            assert observed['watchdog'] and observed['ready'],observed['state']
            assert observed['command'].linear.x>0.3, 'normal preview blocked'
        if case=='bridge_exit':
            processes[0].terminate();processes[0].wait(timeout=5);pump(0.3)
            assert abs(observed['command'].linear.x)+abs(observed['command'].angular.z)<1e-8
            print('PASS bridge_exit stale degradation heartbeat stops preview while new requests continue')
            return
        if case in ('fatal','disabled','no_anchor','no_bias','high_uncertainty','invalid_pose','future_pose'):
            mode=case if case in ('invalid_pose','future_pose') else 'fatal' if case=='fatal' else 'lost';pump(0.25)
        else:
            mode='late' if case in ('late_return','late_timeout') else 'inertial_only' if case=='inertial_only' else 'lost'
            motion=(0.10,0.20);fresh_map=False;pump(0.4)
            assert observed['degraded'] and observed['health'] and observed['watchdog'] and observed['ready'],observed['state']
            assert 0<observed['command'].linear.x<=0.15001
            assert abs(observed['command'].angular.z)<=0.30001
            assert observed['poses'][-1].pose.pose.position.x>0.02, 'wheel integration did not move'
            assert observed['poses'][-1].pose.covariance[0]>0.0001, 'fallback reduced covariance'
            if case in ('return','late_return'):
                if case=='late_return':
                    assert 'late_visual_frames=0' not in observed['state'], 'late frames not exercised'
                    assert 'late_visual_frames=' in observed['state'], observed['state']
                collision=True;pump(0.10);assert abs(observed['command'].linear.x)<1e-8,'collision zero bypassed'
                collision=False;mode='return';pump(0.35)
                assert observed['health'] and observed['watchdog'] and not observed['degraded'],observed['state']
                poses=observed['poses'];assert max(math.hypot(b.pose.pose.position.x-a.pose.pose.position.x,
                    b.pose.pose.position.y-a.pose.pose.position.y) for a,b in zip(poses,poses[1:]))<0.04,'return pose jump'
                mode='visual';pump(0.2)
                print('PASS',case,'continuous return, cap, collision stop; last pose',poses[-1].pose.pose.position.x)
                return
            if case=='imu_stale':imu=False;pump(0.3)
            elif case=='sdk_imu_fault':
                send(String(data='gyro_bias_out_of_bounds'),'/r680_nav/vio_health_reason');pump(0.3)
            elif case=='wheel_stale':wheel=False;pump(0.3)
            elif case=='mismatch':mode='mismatch';pump(0.25)
            elif case=='duplicate':
                mode='duplicate';pump(0.10);assert observed['degraded'],'duplicate frames recovered'
                mode='lost';pump(2.0)
            elif case=='distance':motion=(1.0,0.0);pump(0.4)
            elif case=='turn':motion=(0.0,2.0);pump(0.5)
            else:pump(2.0)
        assert not observed['health'] and not observed['watchdog'],observed['state']
        if case=='late_timeout':
            assert 'budget exhausted' in observed['state'], 'late frames reset budget or caused hard fault'
        if case=='invalid_pose':assert 'invalid visual pose' in observed['state']
        if case=='future_pose':assert 'visual timestamp in the future' in observed['state']
        assert abs(observed['command'].linear.x)+abs(observed['command'].angular.z)<1e-8,'fault command nonzero'
        print('PASS',case,observed['state'])
        if case=='timeout':
            mode='visual';motion=(0.0,0.0);fresh_map=True;pump(2.4)
            assert observed['health'] and not observed['watchdog'], 'continuity node bypassed latched map recovery'
            assert abs(observed['command'].linear.x)+abs(observed['command'].angular.z)<1e-8
            print('PASS stopped frontend return provides odom, watchdog motion latch remains enforced')
    finally:
        node.destroy_node();rclpy.shutdown()
        for p in processes:p.terminate()
        for p in processes:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for f in files:f.close()

if __name__=='__main__':
    import sys
    for name in sys.argv[1:] or ['return','timeout','imu_stale','wheel_stale','mismatch','duplicate','distance','turn','fatal',
                               'inertial_only','sdk_imu_fault','disabled','no_anchor','no_bias','high_uncertainty','bridge_exit',
                               'late_return','late_timeout','invalid_pose','future_pose']:
        run_case(name)
