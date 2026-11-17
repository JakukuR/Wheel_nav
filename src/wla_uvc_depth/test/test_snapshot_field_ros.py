"""Isolated-domain contract test: no real camera, TF or motion commands."""
import argparse,json,os,subprocess,time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--executable',required=True);a=p.parse_args()
assert int(os.environ.get('ROS_DOMAIN_ID','0'))>=190,'Use an isolated ROS domain'
rclpy.init();node=rclpy.create_node('snapshot_field_contract_test');received=[]
sub=node.create_subscription(String,'/r680/debug/stereo/field',lambda m:received.append(json.loads(m.data)),qos_profile_sensor_data)
dp=node.create_publisher(Image,'/camera/stereo/depth/image_raw',qos_profile_sensor_data);ip=node.create_publisher(CameraInfo,'/camera/stereo/depth/camera_info',qos_profile_sensor_data)
child=subprocess.Popen([a.executable],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
def run(mode,seconds=1.2):
 start=len(received);end=time.monotonic()+seconds
 while time.monotonic()<end:
  if mode!='stopped':
   d=Image();d.header.stamp=node.get_clock().now().to_msg();d.header.frame_id='sunplus_left_rect_optical_frame';d.width=384;d.height=216;d.encoding='16UC1';d.step=768;d.data=np.full((216,384),0 if mode=='empty' else 1000,dtype='<u2').tobytes()
   i=CameraInfo();i.header=d.header;i.width=d.width;i.height=d.height;i.distortion_model='plumb_bob';i.d=[0.]*5;i.k=[280.,0.,191.5,0.,280.,107.5,0.,0.,1.]
   if mode=='wrong_frame':i.header.frame_id='wrong_optical_frame'
   if mode=='truncated':d.data=d.data[:-2]
   if mode=='wrong_size':i.width=640
   dp.publish(d);ip.publish(i)
  until=time.monotonic()+.05
  while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
 return received[start:]
try:
 result={}
 samples=run('valid',3);assert samples and samples[-1]['available'] and samples[-1]['occupied_voxels']>0;result['valid']=True
 for mode in ['wrong_frame','truncated','wrong_size','stopped']:
  samples=run(mode);assert len(samples)>1 and all(not x['available'] for x in samples[-2:]),mode;result[mode]=True
 samples=run('empty');assert samples[-1]['available'] and samples[-1]['observed_voxels']==0 and samples[-1]['surface_voxels']==[];result['empty_is_unknown']=True
 samples=run('valid');assert samples[-1]['available'] and samples[-1]['occupied_voxels']>0;result['recovered']=True
 assert all(not x['temporal_fusion'] and not x['navigation_connected'] for x in received)
 print(json.dumps(result,indent=2))
finally:
 child.terminate();child.wait(timeout=5);node.destroy_node();rclpy.shutdown()
