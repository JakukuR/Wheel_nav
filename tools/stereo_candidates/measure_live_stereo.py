"""Read-only live stream/resource measurement. Does not start camera or motion."""
import argparse,json,time,os
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,CompressedImage
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--pid',type=int,required=True);p.add_argument('--seconds',type=float,default=60);p.add_argument('--out',type=Path,required=True);p.add_argument('--aligned',action='store_true');p.add_argument('--extra-pid',type=int,action='append',default=[]);a=p.parse_args()
rclpy.init();n=rclpy.create_node('las2_resolution_measure');rows={};last={};status=[];subs=[];resources=[]
def receive(k,m):
 now=time.monotonic();age=None
 if hasattr(m,'header'):age=(n.get_clock().now().nanoseconds-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))*1e-6
 rows.setdefault(k,[]).append([now,age]);last[k]=m
names={'depth':('/camera/stereo/depth/image_raw',Image),'info':('/camera/stereo/depth/camera_info',CameraInfo),'left_preview':('/r680/debug/calibration/left/compressed',CompressedImage),'depth_preview':('/r680/debug/stereo/depth/compressed',CompressedImage)}
if a.aligned:names.update(aligned=('/camera/stereo/aligned_depth_to_color/image_raw',Image),aligned_info=('/camera/stereo/aligned_depth_to_color/camera_info',CameraInfo))
for k,(t,ty) in names.items():subs.append(n.create_subscription(ty,t,lambda m,k=k:receive(k,m),qos_profile_sensor_data))
subs.append(n.create_subscription(String,'/camera/stereo/status',lambda m:status.append(json.loads(m.data)),qos_profile_sensor_data))
def proc(pid=None):
 s=Path(f'/proc/{pid or a.pid}/stat').read_text().split();return (int(s[13])+int(s[14]))/os.sysconf('SC_CLK_TCK')
def quantile(v):
 if not v:return None
 return {k:float(np.quantile(v,q)) for k,q in [('p50',.5),('p95',.95),('p99',.99),('max',1)]}
extra0={pid:proc(pid) for pid in a.extra_pid};start=time.monotonic();c0=proc();tick=start
while time.monotonic()-start<a.seconds:
 rclpy.spin_once(n,timeout_sec=.02)
 if time.monotonic()>=tick:
  sample={'t':time.monotonic()-start,'cpu_seconds':proc()-c0}
  for key,path in [('cpu_c','/sys/class/thermal/thermal_zone0/temp'),('gpu_c','/sys/class/thermal/thermal_zone1/temp'),('gpu_load','/sys/devices/platform/bus@0/17000000.gpu/load')]:
   try:sample[key]=int(Path(path).read_text())/(1000 if key.endswith('_c') else 10)
   except (OSError,ValueError):pass
  resources.append(sample);tick+=1
elapsed=time.monotonic()-start
result={'duration_s':elapsed,'depth_pid':a.pid,'depth_cpu_one_core_percent':100*(proc()-c0)/elapsed,'streams':{},'resources':resources,'statuses':status}
result['extra_process_cpu_one_core_percent']={str(pid):100*(proc(pid)-value)/elapsed for pid,value in extra0.items()}
for k,v in rows.items():
 times=np.array([x[0] for x in v]);ages=[x[1] for x in v if x[1] is not None]
 result['streams'][k]={'count':len(v),'hz':float((len(v)-1)/(times[-1]-times[0])) if len(v)>1 else 0,'age_ms':quantile(ages),'interval_ms':quantile((np.diff(times)*1000).tolist()),'last_20s_hz':sum(t>=start+max(0,a.seconds-20) for t in times)/min(20,a.seconds)}
for k in ['depth','aligned']:
 if k not in last:continue
 m=last[k];i=last.get('info' if k=='depth' else 'aligned_info');data=np.frombuffer(m.data,dtype='<u2').reshape(m.height,m.step//2)[:,:m.width]
 result[k+'_contract']={'shape':[m.height,m.width],'encoding':m.encoding,'frame':m.header.frame_id,'valid_fraction':float(np.count_nonzero(data)/data.size),'fx':float(i.k[0]) if i else None,'info_shape_ok':bool(i and (i.width,i.height)==(m.width,m.height))}
a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(result,indent=2))
print(json.dumps({k:v for k,v in result.items() if k not in ['resources','statuses']},indent=2));print('last_status',json.dumps(status[-1] if status else {}))
n.destroy_node();rclpy.shutdown()
