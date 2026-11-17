"""Isolated ROS contract: ground map consumer, no hardware or motion."""
import argparse,copy,json,os,subprocess,time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
p=argparse.ArgumentParser();p.add_argument('--executable',required=True);a=p.parse_args()
assert int(os.environ.get('ROS_DOMAIN_ID','0'))>=190
rclpy.init();n=rclpy.create_node('ground_map_contract');rows=[];grids=[]
sub=n.create_subscription(String,'/r680/debug/stereo/ground_map',lambda m:rows.append(json.loads(m.data)),qos_profile_sensor_data)
gs=n.create_subscription(OccupancyGrid,'/r680/debug/stereo/ground_map/grid',lambda m:grids.append(m),10)
dp=n.create_publisher(Image,'/camera/stereo/depth/image_raw',qos_profile_sensor_data);ip=n.create_publisher(CameraInfo,'/camera/stereo/depth/camera_info',qos_profile_sensor_data);sp=n.create_publisher(String,'/robot_state/geometry',qos_profile_sensor_data)
broad=StaticTransformBroadcaster(n);t=TransformStamped();t.header.frame_id='sunplus_left_optical_frame';t.child_frame_id='sunplus_left_rect_optical_frame';t.transform.rotation.w=1.;t.header.stamp=n.get_clock().now().to_msg();broad.sendTransform(t)
angle=np.deg2rad(11);R=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])@np.array([[0,0,1],[-1,0,0],[0,-1,0]])
v,u=np.mgrid[:216,:384];rays=np.stack([(u-191.5)/280,(v-107.5)/280,np.ones_like(u)],-1)@R.T
z=np.divide(-.188482,rays[:,:,2],out=np.zeros_like(u,dtype=float),where=rays[:,:,2]!=0);depth=np.where((z>.15)&(z<3),np.round(z*1000),0).astype('<u2')
child=subprocess.Popen([a.executable],stdout=subprocess.DEVNULL)
def run(mode,duration=1.2):
 start=len(rows);end=time.monotonic()+duration
 while time.monotonic()<end:
  stamp=n.get_clock().now();seconds=stamp.nanoseconds/1e9
  d=Image();d.header.stamp=stamp.to_msg();d.header.frame_id='sunplus_left_rect_optical_frame';d.width=384;d.height=216;d.step=768;d.encoding='16UC1';d.data=(np.zeros_like(depth) if mode=='empty' else depth).tobytes()
  i=CameraInfo();i.header=copy.deepcopy(d.header);i.width=384;i.height=216;i.distortion_model='plumb_bob';i.d=[0.]*5;i.k=[280.,0.,191.5,0.,280.,107.5,0.,0.,1.]
  g={'schema':'robot-geometry/v1','frame':'base_Link','available':True,'joint_stamp':seconds,'imu_stamp':seconds,'ground':{'available':True,'normal':[0.,0.,1.],'offset_m':.15,'wheel_heights_m':[.15,.15],'reason':'operator_confirmed_both_contact_flat_ground'},'frames':[{'name':'bottom_camera_L','p':[.084995,.0185,.038482],'q':[0.,float(np.sin(angle/2)),0.,float(np.cos(angle/2))]}]}
  if mode=='stale_support':g['joint_stamp']=seconds-1;g['imu_stamp']=seconds-1
  if mode=='missing_source_time':del g['joint_stamp']
  if mode=='wrong_frame':d.header.frame_id='wrong_frame'
  if mode=='truncated':d.data=d.data[:-2]
  if mode=='wrong_height':g['ground']['offset_m']=.35
  if mode!='stopped':dp.publish(d);ip.publish(i)
  if mode!='no_support':sp.publish(String(data=json.dumps(g)))
  until=time.monotonic()+.05
  while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.01)
 return rows[start:]
try:
 out={};r=run('valid',6);assert r, ('no messages',child.poll());assert r[-1]['available'] and r[-1]['ground_points']>400;assert 0 in grids[-1].data and -1 in grids[-1].data;out['valid_grid']=True
 for mode in ['stale_support','missing_source_time','wrong_frame','truncated','stopped','no_support','empty','wrong_height']:
  r=run(mode);assert len(r)>1 and all(not x['available'] for x in r[-2:]),(mode,r[-1]);assert all(x==-1 for x in grids[-1].data),mode;out[mode]=True
 r=run('valid');assert r[-1]['available'];out['recovered']=True
 assert all(not x['navigation_connected'] and not x['temporal_fusion'] for x in rows)
 print(json.dumps(out,indent=2))
finally:
 child.terminate();child.wait(timeout=5);n.destroy_node();rclpy.shutdown()
