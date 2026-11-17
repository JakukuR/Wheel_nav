"""Isolated-domain real-model test: no RGB, exact stereo stamps, stale/mismatch rejection."""
import os,subprocess,time,signal,json
from pathlib import Path
import cv2,numpy as np,rclpy,yaml
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,PointCloud2,CameraInfo,CompressedImage
from std_msgs.msg import String
ROOT=Path('/home/onana/ros2_ws')
rclpy.init();node=rclpy.create_node('stereo_contract_test');qos=qos_profile_sensor_data
left_pub=node.create_publisher(Image,'/camera/sunplus/left/image_raw',qos)
right_pub=node.create_publisher(Image,'/camera/sunplus/right/image_raw',qos)
seen=[];clouds=[];status=[];colored=[];previews=[];jpeg_processes=[]
rgb_pub=node.create_publisher(Image,"/camera/processed/color/image_raw",qos)
info_pub=node.create_publisher(CameraInfo,"/camera/processed/color/camera_info",qos)
rig=yaml.safe_load((ROOT/"config/uvc_rig/calibrated_20261116/rig.yaml").read_text())
k=np.array(rig["cameras"]["rgb"]["k"]);a=np.array([[1/3,0,1/6-.5],[0,1/3,1/6-.5],[0,0,1]]);k=a@k
node.create_subscription(Image,'/camera/stereo/depth/image_raw',lambda m:seen.append(m),qos)
node.create_subscription(PointCloud2,'/camera/stereo/points',lambda m:clouds.append(m),qos)
node.create_subscription(PointCloud2,'/camera/stereo/points_color',lambda m:colored.append(m),qos)
node.create_subscription(CompressedImage,'/test/stereo/depth/compressed',lambda m:previews.append(m),qos)
node.create_subscription(String,'/camera/stereo/status',lambda m:status.append(json.loads(m.data)),1)
images=[cv2.imread(str(ROOT/'.runtime'/f'{name}.png')) for name in ('left','right')]
if any(a is None or a.shape!=(720,1280,3) for a in images):raise RuntimeError('missing full-resolution reference stereo pair')
cmd=[str(ROOT/'install/wla_uvc_depth/lib/wla_uvc_depth/uvc_stereo_trt'),'--engine',str(ROOT/'models/depth_comparison/c_fast_stereo_192x320_fp32.engine'),'--calibration',str(ROOT/'config/uvc_rig/calibrated_20261116/rig.yaml'),'--max-rate','2']
log=(ROOT/'.runtime/stereo_ros_test.log').open('w')
jpeg_processes.append(subprocess.Popen([str(ROOT/'install/wla_r680_navigation/lib/wla_r680_navigation/kalo_jpeg'),'--ros-args','-r','__node:=contract_depth_jpeg','-p','input_topic:=/camera/stereo/depth/preview','-p','output_topic:=/test/stereo/depth/compressed','-p','max_rate:=5.0'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def pump(duration,phase=None):
 end=time.monotonic()+duration;next_frame=0
 while time.monotonic()<end:
  if phase and time.monotonic()>next_frame:
   next_frame=time.monotonic()+.2;stamp=node.get_clock().now().nanoseconds
   if phase=='stale':stamp-=2_000_000_000
   if phase=='color':
    info=CameraInfo();info.header.stamp.sec=stamp//1_000_000_000;info.header.stamp.nanosec=stamp%1_000_000_000;info.header.frame_id="dicota_color_optical_frame";info.width=640;info.height=360;info.k=k.ravel().tolist();info.d=[0.]*5;info.distortion_model="plumb_bob";info_pub.publish(info)
    rgb=Image();rgb.header=info.header;rgb.width=640;rgb.height=360;rgb.encoding="bgr8";rgb.step=640*3;rgb.data=np.full((360,640,3),(0,0,255),dtype=np.uint8).tobytes();rgb_pub.publish(rgb)
   for i,pub in enumerate((left_pub,right_pub)):
    t=stamp+(1_000_000 if phase=='mismatch' and i==1 else 0)
    m=Image();m.header.stamp.sec=t//1_000_000_000;m.header.stamp.nanosec=t%1_000_000_000;m.header.frame_id=f'sunplus_{("left","right")[i]}_optical_frame';m.width=1280;m.height=720;m.encoding='bgr8';m.step=1280*3;m.data=images[i].tobytes();pub.publish(m)
  rclpy.spin_once(node,timeout_sec=.01)
  if proc.poll() is not None:raise RuntimeError(f'node exited:{proc.returncode}')
try:
 pump(3);pump(6,'valid');pump(.5)
 if not previews:raise AssertionError('network depth JPEG preview not produced')
 if not seen or not clouds:raise AssertionError('no metric depth/cloud without RGB')
 d=seen[-1];z=np.frombuffer(d.data,dtype='<u2').reshape(360,640)
 assert d.encoding=='16UC1' and d.header.frame_id=='sunplus_left_rect_optical_frame' and np.count_nonzero(z)>0
 c=clouds[-1];xyz=np.ndarray((c.width,3),dtype='<f4',buffer=c.data,strides=(c.point_step,4))
 assert c.header.frame_id=='sunplus_left_optical_frame' and np.isfinite(xyz).all() and np.all((xyz[:,2]>=.15)&(xyz[:,2]<=5))
 pump(3,'color');pump(.5)
 if not colored:raise AssertionError('no colored cloud for calibrated same-time RGB')
 cc=colored[-1];rgb_bits=np.ndarray((cc.width,),dtype='<u4',buffer=cc.data,offset=12,strides=(cc.point_step,))
 assert np.count_nonzero(rgb_bits==0xff0000)>0,'RGB did not color any points'
 colored_xyz=np.ndarray((cc.width,3),dtype='<f4',buffer=cc.data,strides=(cc.point_step,4))
 assert np.isfinite(colored_xyz).all(),'colored cloud has invalid geometry'
 baseline=len(seen);pump(2,'mismatch');assert len(seen)==baseline,'mismatched stereo timestamps accepted'
 pump(2,'stale');assert len(seen)==baseline,'stale source accepted'
 result={'depth_jpeg_frames':len(previews),'depth_without_rgb_passed':True,'coloring_passed':True,'colored_clouds':len(colored),'depth_frames':len(seen),'cloud_frames':len(clouds),'last_cloud_points':c.width,'valid_depth_fraction':float(np.count_nonzero(z)/z.size),'exact_pairing_passed':True,'stale_rejection_passed':True,'status':status[-1] if status else {}}
 (ROOT/'.runtime/stereo_ros_test.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
finally:
 os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=5)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 for j in jpeg_processes:
  os.killpg(j.pid,signal.SIGINT)
  try:j.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(j.pid,signal.SIGKILL);j.wait()
 log.close();node.destroy_node();rclpy.shutdown()
