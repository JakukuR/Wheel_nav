#!/usr/bin/env python3
"""Exercise final health aggregation in an isolated ROS graph, without motion."""
import os
import subprocess
import time
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def main():
    assert 100<=int(os.environ.get('ROS_DOMAIN_ID','0'))<=232
    rclpy.init();n=rclpy.create_node('frontend_interface_regression');samples=[]
    topics=[(Image,'/r680/d455/color/image_raw'),(Image,'/r680/d455/aligned_depth_to_color/image_raw'),
            (CameraInfo,'/r680/d455/color/camera_info'),(Odometry,'/d455_slam/odom'),
            (Bool,'/test/vio'),(Bool,'/test/permission'),(Bool,'/r680_nav/odom_degraded')]
    pubs=[n.create_publisher(t,name,10) for t,name in topics]
    n.create_subscription(Bool,'/r680_nav/localization_ready',lambda m:samples.append(m.data),10)
    proc=subprocess.Popen([os.environ['WLA_MONITOR_EXECUTABLE'],'--ros-args',
        '-p','require_obstacle_points:=false','-p','require_vio_health:=true',
        '-p','require_motion_permission:=true','-p','allow_bounded_degraded:=true',
        '-p','vio_health_topic:=/test/vio','-p','motion_permission_topic:=/test/permission'])
    def phase(vio,permission,degraded,expected):
        end=time.monotonic()+1.0;observed=[]
        while time.monotonic()<end:
            assert proc.poll() is None
            c=CameraInfo();c.k[0]=c.k[4]=100.
            o=Odometry();o.header.frame_id='odom';o.child_frame_id='base'
            msgs=[Image(),Image(),c,o,Bool(data=vio),Bool(data=permission),Bool(data=degraded)]
            for p,m in zip(pubs,msgs):p.publish(m)
            rclpy.spin_once(n,timeout_sec=.02);time.sleep(.02)
            if samples and time.monotonic()>end-.3:observed.append(samples[-1])
        assert observed and all(x==expected for x in observed),(vio,permission,degraded,observed)
    try:
        phase(False,True,False,False)  # motion aggregation cannot masquerade as VIO
        phase(True,False,False,False)  # VIO cannot override motion restrictions
        phase(True,True,False,True)
        phase(False,True,True,True)    # explicitly enabled bounded fallback
        phase(False,False,True,False)
        phase(False,True,False,False)
        print('PASS interface raw VIO / permission / bounded degradation separation',flush=True)
    finally:
        proc.terminate();proc.wait(timeout=5);n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
