#!/usr/bin/env python3
"""Isolated ROS contract test. No chassis, camera, or production-domain process."""
import os
import signal
import struct
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from std_msgs.msg import Bool, String


assert os.environ.get('ROS_DOMAIN_ID') == '198', 'Only isolated domain 198 is permitted'
rclpy.init()
node = Node('openvins_control_contract_fixture')
processes=[]
def start(binary, params):
    args=[str(Path(os.environ['WLA_NAV_BIN'])/binary),'--ros-args']
    for k,v in params.items(): args += ['-p',f'{k}:={str(v).lower() if isinstance(v,bool) else v}']
    p=subprocess.Popen(args,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,start_new_session=True)
    processes.append(p)
    return p
def stop(p):
    os.killpg(p.pid,signal.SIGINT)
    try: _,err=p.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGKILL);_,err=p.communicate()
    assert p.returncode==0,err.decode()
    processes.remove(p)
def pub(kind,topic): return node.create_publisher(kind,topic,qos_profile_sensor_data if kind in (Imu,Image,Odometry,PointCloud2) else 10)
def spin(duration,callback):
    until=time.monotonic()+duration
    while time.monotonic()<until:
        callback();rclpy.spin_once(node,timeout_sec=.002);time.sleep(.002)

try:
    checked=pub(Twist,'/r680_nav/cmd_vel_collision_checked')
    healthy=pub(Bool,'/r680_nav/localization_ready')
    permission=pub(Bool,'/openvins/motion_ready')
    preview=[];raw=[]
    node.create_subscription(Twist,'/r680_nav/cmd_vel_safe_preview',lambda m:preview.append(m.linear.x),10)
    node.create_subscription(Twist,'/r680_nav/chassis_cmd_vel',lambda m:raw.append(m.linear.x),10)
    guard=start('command_guard',dict(hardware_output_enabled=False,require_mission_permission=False,require_frontend_motion_permission=True))
    def send_guard(allow=None):
        m=Twist();m.linear.x=.12;checked.publish(m);healthy.publish(Bool(data=True))
        if allow is not None: permission.publish(Bool(data=allow))
    spin(.6,lambda:send_guard(False))
    assert preview and not any(preview),'Denied frontend forwarded a command'
    preview.clear();spin(.35,lambda:send_guard(True))
    assert any(x>.1 for x in preview),'Positive permission control did not pass'
    preview.clear();spin(.4,send_guard)
    assert preview[-5:]==[0.]*5,'Stale frontend permission did not stop'
    assert not raw,'Preview guard published chassis commands'
    stop(guard)

    imu_pub=pub(Imu,'/test/imu');wheel_pub=pub(Odometry,'/wheel/odom')
    image_pub=pub(Image,'/r680/d455/infra1/image_rect_raw')
    depth_pub=pub(Image,'/r680/d455/aligned_depth_to_color/image_raw')
    cloud_pub=pub(PointCloud2,'/r680_nav/d455/points_safety')
    contact_pub=pub(String,'/test/contact');manual_pub=pub(Twist,'/r680_manual/cmd_vel')
    states=[];requests=[]
    node.create_subscription(String,'/r680_nav/vio_init_state',lambda m:states.append(m.data),10)
    node.create_subscription(TwistStamped,'/r680_nav/vio_init_request',lambda m:requests.append(m.twist.linear.x),10)
    def active(req,res): res.current_state.id=3;res.current_state.label='active';return res
    services=[node.create_service(GetState,f'/{name}/get_state',active) for name in ('velocity_smoother','collision_monitor')]
    tick=0
    def sensors(contact=True,manual=False,invalid=False):
        global tick
        stamp=node.get_clock().now().to_msg()
        m=Imu();m.header.stamp=stamp;m.header.frame_id='d455_imu_optical_frame';m.linear_acceleration.z=9.81;imu_pub.publish(m)
        tick+=1
        if tick%8: return
        m=Odometry();m.header.stamp=stamp;m.header.frame_id='wheel_odom';m.child_frame_id='base';m.pose.pose.orientation.w=1.;wheel_pub.publish(m)
        image=Image();image.header.stamp=stamp
        if not invalid:image_pub.publish(image)
        m=Image();m.header.stamp=stamp;m.width=m.height=32;m.encoding='16UC1';m.step=64;m.data=struct.pack('<H',1000)*1024;depth_pub.publish(m)
        m=PointCloud2();m.header.stamp=stamp;m.header.frame_id='r680_mapping_floor';m.height=1;m.width=1;m.point_step=m.row_step=12
        m.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
        m.data=struct.pack('<fff',2.,0.,.1);cloud_pub.publish(m)
        if contact: contact_pub.publish(String(data='GROUNDED'))
        if manual:
            m=Twist();m.linear.x=.1;manual_pub.publish(m)
    with tempfile.TemporaryDirectory() as folder:
        params=dict(frontend='openvins',imu_topic='/test/imu',imu_frame='d455_imu_optical_frame',
            contact_topic='/test/contact',stationary_time_s=.12,result_path=folder+'/init.json')
        # Shared NAV/manual is the default (9a2631d); the takeover latch is the opt-out path.
        init=start('vio_initializer',dict(params,nav_manual_coexist=False))
        spin(.9,lambda:sensors(contact=False))
        assert requests and not any(requests),'Unknown contact permitted bootstrap'
        spin(.9,sensors)
        assert 'moving' in states and any(requests),'OpenVINS sensor input without odom must admit bounded preview'
        spin(.35,lambda:sensors(manual=True));requests.clear();spin(.3,sensors)
        assert states[-1]=='failed' and not any(requests),'Manual takeover must latch failure and zero requests'
        stop(init)
        states.clear();requests.clear()
        init=start('vio_initializer',params)
        spin(1.0,sensors);assert 'moving' in states
        spin(.5,lambda:sensors(contact=False));requests.clear();spin(.2,lambda:sensors(contact=False))
        assert states[-1]=='failed' and not any(requests),'Lost contact must latch failure'
        stop(init)
    print('PASS guard allow/deny/stale, no raw output; OpenVINS no-odom bootstrap, unknown/lost contact and manual takeover')
finally:
    for p in processes:
        if p.poll() is None: os.killpg(p.pid,signal.SIGINT);p.wait(timeout=5)
    node.destroy_node();rclpy.shutdown()
