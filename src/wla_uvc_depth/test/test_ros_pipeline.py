"""Isolated sensor-only native pipeline checks; always ROS domain 198/localhost."""
import os
os.environ['ROS_DOMAIN_ID']='198'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import argparse,json,signal,subprocess,time
from pathlib import Path
import cv2,numpy as np,yaml,rclpy
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

def main():
 p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 binary=a.workspace/'install/wla_uvc_depth/lib/wla_uvc_depth/uvc_depth_cpp';rig=a.workspace/'config/uvc_rig/calibrated_20261116/rig.yaml'
 invalid=subprocess.run([str(binary),'--calibration',str(a.output/'missing.yaml')],capture_output=True,text=True,timeout=10)
 assert invalid.returncode!=0,'missing file must not silently fall back'
 rclpy.init();n=rclpy.create_node('native_uvc_isolated_checks');values={'color':[],'depth':[],'info':[],'status':[]}
 subs=[]
 for key,topic,kind in [('color','/camera/processed/color/image_raw',Image),('depth','/camera/processed/aligned_depth_to_color/image_raw',Image),('info','/camera/processed/color/camera_info',CameraInfo),('status','/camera/processed/status',String)]:
  def callback(m,key=key):
   values[key].append(m)
   if len(values[key])>80:values[key].pop(0)
  subs.append(n.create_subscription(kind,topic,callback,qos_profile_sensor_data))
 pubs={eye:n.create_publisher(Image,topic,qos_profile_sensor_data) for eye,topic in [('rgb','/camera/dicota/color/image_raw'),('left','/camera/sunplus/left/image_raw'),('right','/camera/sunplus/right/image_raw')]}
 data={'rgb':bytes(1920*1080*3),'left':bytes(1280*720*3),'right':bytes(1280*720*3)}
 def spin(seconds,send=None,period=.1):
  end=time.monotonic()+seconds;next_send=0
  while time.monotonic()<end:
   if send and time.monotonic()>=next_send:send();next_send=time.monotonic()+period
   rclpy.spin_once(n,timeout_sec=.02)
 def send(eye,stamp):
  m=Image();m.header.stamp.sec=stamp//10**9;m.header.stamp.nanosec=stamp%10**9;m.header.frame_id='dicota_color_optical_frame' if eye=='rgb' else f'sunplus_{eye}_optical_frame';m.width,m.height=(1920,1080) if eye=='rgb' else (1280,720);m.encoding='bgr8';m.step=m.width*3;m.data=data[eye];pubs[eye].publish(m)
 def stop(child):
  os.killpg(child.pid,signal.SIGINT);child.wait(timeout=10);assert child.returncode==0,child.returncode
 result={'invalid_calibration_rejected':True,'domain':198,'discovery':'LOCALHOST'}
 with (a.output/'uncalibrated.log').open('w') as f:
  child=subprocess.Popen([str(binary),'--rgb-mode','crop'],stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
  try:
   spin(1.5);spin(2.,lambda:send('rgb',n.get_clock().now().nanoseconds));assert len(values['color'])>=3 and values['info'];assert not values['depth'];assert values['info'][-1].k[0]==0
   result['uncalibrated']={'color_received':len(values['color']),'depth_received':0,'fx':float(values['info'][-1].k[0])}
  finally:stop(child)
 for key in values:values[key].clear()
 with (a.output/'calibrated.log').open('w') as f:
  child=subprocess.Popen([str(binary),'--calibration',str(rig),'--rgb-mode','crop','--max-rate','10'],stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
  try:
   spin(1.5);spin(1.5,lambda:send('rgb',n.get_clock().now().nanoseconds));assert values['color'] and not values['depth'];result['missing_stereo_no_depth']=True
   def wrong():
    stamp=n.get_clock().now().nanoseconds;send('rgb',stamp-100_000_000);send('left',stamp);send('right',stamp)
   spin(1.5,wrong,period=.22);assert not values['depth'];result['skew_100ms_rejected']=True
   sent_depth=set();sent_color=set()
   def good():
    stamp=n.get_clock().now().nanoseconds;sent_depth.add(stamp);sent_color.add(stamp-8_000_000);send('rgb',stamp-8_000_000);send('left',stamp);send('right',stamp)
   spin(2.,good);assert len(values['depth'])>=3
   dm=values['depth'][-1];ci=values['info'][-1]
   assert (dm.width,dm.height,dm.encoding,dm.step)==(640,480,'16UC1',1280)
   assert dm.header.frame_id=='dicota_color_optical_frame';assert dm.header.stamp.sec*10**9+dm.header.stamp.nanosec in sent_depth
   cm=values['color'][-1];assert cm.header.stamp.sec*10**9+cm.header.stamp.nanosec in sent_color
   k=np.array(yaml.safe_load(rig.read_text())['cameras']['rgb']['k']);scale=4/9.;matrix=np.array([[scale,0,scale*(.5-240)-.5],[0,scale,scale*.5-.5],[0,0,1]])@k
   assert np.allclose(np.array(ci.k).reshape(3,3),matrix,atol=1e-10,rtol=0)
   assert np.array_equal(ci.d,np.zeros(5));assert np.array_equal(np.array(ci.r).reshape(3,3),np.eye(3));assert (ci.width,ci.height)==(640,480)
   assert np.allclose(np.array(ci.p).reshape(3,4),np.column_stack([matrix,np.zeros(3)]),rtol=0,atol=1e-10)
   result['calibrated']={'depth_received':len(values['depth']),'color_info_k':list(ci.k),'dimensions':[dm.width,dm.height],'encoding':dm.encoding,'depth_uses_stereo_stamp':True,'color_keeps_rgb_stamp':True,'zero_texture_depth_all_invalid':not any(dm.data)}
   spin(.5);before=(len(values['color']),len(values['depth']))
   def stale():
    stamp=n.get_clock().now().nanoseconds-1_000_000_000
    for eye in pubs:send(eye,stamp)
   spin(1.2,stale);assert (len(values['color']),len(values['depth']))==before;result['stale_inputs_do_not_publish']=True
  finally:stop(child)
 n.destroy_node();rclpy.shutdown();(a.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
