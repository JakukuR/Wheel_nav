"""Real navigation consumer, synthetic data in isolated domain 194. No motion nodes."""
def main():
 import os,time,subprocess,signal,json
 from pathlib import Path
 assert os.environ.get('ROS_DOMAIN_ID')=='194'
 import rclpy
 from sensor_msgs.msg import Image,CameraInfo,PointCloud2
 from geometry_msgs.msg import TransformStamped
 from tf2_ros import StaticTransformBroadcaster
 from rclpy.qos import qos_profile_sensor_data
 rclpy.init();n=rclpy.create_node('stereo_consumer_fixture')
 q=qos_profile_sensor_data
 pi=n.create_publisher(CameraInfo,'/camera/stereo/depth/camera_info',q)
 pd=n.create_publisher(Image,'/camera/stereo/depth/image_raw',q)
 clouds=[];sub=n.create_subscription(PointCloud2,'/test/points',clouds.append,q)
 tf=StaticTransformBroadcaster(n);t=TransformStamped();t.header.stamp=n.get_clock().now().to_msg();t.header.frame_id='test_floor';t.child_frame_id='test_camera';t.transform.rotation.w=1.;tf.sendTransform(t)
 exe='/home/onana/ros2_ws/install/wla_r680_navigation/lib/wla_r680_navigation/depth_to_points'
 params={'depth_topic':'/camera/stereo/depth/image_raw','camera_info_topic':'/camera/stereo/depth/camera_info','target_frame':'test_floor','output_topic':'/test/points','safety_output_topic':'/test/safety','clearing_output_topic':'/test/clear','visualization_output_topic':'/test/viz','ground_filter_enabled':False,'fallback_obstacle_min_height':0.1,'obstacle_max_height':2.,'clearing_max_height':2.,'isolated_cluster_min_cells':1,'stride':2,'max_rate':30.}
 cmd=[exe,'--ros-args']
 for k,v in params.items():cmd+=['-p',f'{k}:={str(v).lower() if isinstance(v,bool) else v}']
 Path('/home/onana/ros2_ws/.runtime/las2_nav_follow_integration').mkdir(parents=True,exist_ok=True)
 log=Path('/home/onana/ros2_ws/.runtime/las2_nav_follow_integration/consumer_test.log').open('w');p=subprocess.Popen(cmd,stdout=log,stderr=log)
 def spin(seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
 def send(mode):
  ns=n.get_clock().now().nanoseconds-(1000000000 if mode=='stale' else 0)
  d=Image();d.header.frame_id='test_camera';d.header.stamp.sec=ns//1000000000;d.header.stamp.nanosec=ns%1000000000;d.width=64;d.height=36;d.encoding='16UC1';d.step=128;d.data=bytes([232,3])*(64*36)
  if mode=='missing_tf':d.header.frame_id='unmounted_camera'
  ci=CameraInfo();ci.header=d.header;ci.width=d.width;ci.height=d.height;ci.k=[60.,0.,32.,0.,60.,18.,0.,0.,1.]
  if mode=='wrong_frame':
   import copy
   ci.header=copy.deepcopy(d.header);ci.header.frame_id='wrong_camera'
  if mode=='truncated':d.data=d.data[:4]
  pi.publish(ci);spin(.025);pd.publish(d)
 def batch(mode):
  before=len(clouds)
  for _ in range(15):send(mode);spin(.06)
  spin(.12);return len(clouds)-before
 try:
  spin(1.5);result={}
  result['valid']=batch('valid');assert result['valid']>3,result
  for mode in ['stale','wrong_frame','truncated','missing_tf']:
   result[mode]=batch(mode);assert result[mode]==0,result
  result['recovered']=batch('valid');assert result['recovered']>3,result
  assert clouds[-1].header.frame_id=='test_floor' and clouds[-1].width>0
  print(json.dumps(result));Path('/home/onana/ros2_ws/.runtime/las2_nav_follow_integration/consumer_test.json').write_text(json.dumps(result,indent=2))
 finally:
  p.send_signal(signal.SIGINT);p.wait(timeout=10);log.close();n.destroy_node();rclpy.shutdown()

if __name__=="__main__":main()
