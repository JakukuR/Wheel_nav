#!/usr/bin/env python3
"""Read-only readiness check. Never starts navigation, publishes TF, or sends motion."""
import argparse,json,time,math
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener

def main():
 p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=8);p.add_argument('--base-frame',default='r680_mapping_floor');p.add_argument('--wheel-topic',default='/wheel/odom_50hz');a=p.parse_args()
 if not 1<=a.seconds<=60:p.error('seconds must be between 1 and 60')
 rclpy.init();n=rclpy.create_node('las2_consumer_preflight');b=Buffer();listener=TransformListener(b,n);latest={};subs=[]
 def store(k,m):latest[k]=m
 topics={'depth':('/camera/stereo/depth/image_raw',Image),'depth_info':('/camera/stereo/depth/camera_info',CameraInfo),'aligned':('/camera/stereo/aligned_depth_to_color/image_raw',Image),'aligned_info':('/camera/stereo/aligned_depth_to_color/camera_info',CameraInfo),'color_info':('/camera/processed/color/camera_info',CameraInfo),'wheel':(a.wheel_topic,Odometry),'status':('/camera/stereo/status',String)}
 for k,(topic,ty) in topics.items():subs.append(n.create_subscription(ty,topic,lambda m,k=k:store(k,m),qos_profile_sensor_data))
 end=time.monotonic()+a.seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
 def fresh(m,limit=.3):return m is not None and stamp(m)>0 and -.05<=(n.get_clock().now().nanoseconds-stamp(m))*1e-9<=limit
 def pair(k):
  d=latest.get(k);i=latest.get(k+'_info')
  return bool(fresh(d) and fresh(i) and d.header.frame_id==i.header.frame_id and (d.width,d.height)==(i.width,i.height) and abs(stamp(d)-stamp(i))<=250000000 and d.encoding=='16UC1' and i.k[0]>0 and i.k[4]>0 and all(math.isfinite(x) for x in i.k))
 def tf_ok(k):
  d=latest.get(k)
  return bool(d and b.can_transform(a.base_frame,d.header.frame_id,rclpy.time.Time.from_msg(d.header.stamp)))
 wheel=latest.get('wheel');wok=bool(fresh(wheel,.15) and wheel.child_frame_id==a.base_frame)
 result={'base_frame':a.base_frame,'rectified_depth_ready':pair('depth'),'rgb_aligned_depth_ready':pair('aligned'),'rgb_camera_info_fresh':fresh(latest.get('color_info')),'navigation_mount_tf_ready':tf_ok('depth'),'follow_mount_tf_ready':tf_ok('depth'),'detection_mount_tf_ready':tf_ok('aligned'),'wheel_odometry_ready':wok,'navigation_motion_started':False}
 result['data_ready']=result['rectified_depth_ready'] and result['rgb_aligned_depth_ready'] and result['rgb_camera_info_fresh']
 result['robot_interfaces_ready']=result['data_ready'] and result['navigation_mount_tf_ready'] and result['follow_mount_tf_ready'] and result['detection_mount_tf_ready'] and wok
 print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
 return 0 if result['robot_interfaces_ready'] else 2
if __name__=='__main__':raise SystemExit(main())
