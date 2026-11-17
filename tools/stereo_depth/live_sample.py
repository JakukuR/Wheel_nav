"""Read-only live geometry evidence; measures observed data rates, saves one depth preview."""
import json,time,os
from pathlib import Path
import numpy as np,cv2,rclpy
from sensor_msgs.msg import Image,PointCloud2
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
root=Path('/home/onana/ros2_ws/.runtime/stereo_live');root.mkdir(exist_ok=True)
rclpy.init();node=rclpy.create_node('stereo_live_sample');qos=qos_profile_sensor_data
rows={k:[] for k in ('depth','cloud','colored','status')};last={}
def receive(key,m):
 rows[key].append(time.monotonic());last[key]=m
 if key=='status':last['status_json']=json.loads(m.data)
for k,t,typ in [('depth','/camera/stereo/depth/image_raw',Image),('cloud','/camera/stereo/points',PointCloud2),('colored','/camera/stereo/points_color',PointCloud2),('status','/camera/stereo/status',String)]:node.create_subscription(typ,t,lambda m,k=k:receive(k,m),qos if k!='status' else 1)
start=time.monotonic();thermal=[];reason='completed'
while time.monotonic()-start<25:
 rclpy.spin_once(node,timeout_sec=.02)
 if not thermal or time.monotonic()-thermal[-1]['time']>1:
  temp=[int(Path(f'/sys/class/thermal/thermal_zone{i}/temp').read_text())/1000 for i in (0,1)]
  thermal.append({'time':time.monotonic(),'temperature_c':temp})
  if max(temp)>=97:reason='temperature_abort';break
result={'elapsed_s':time.monotonic()-start,'stop_reason':reason,'samples':{},'status':last.get('status_json'),'thermal':thermal}
for key,r in rows.items():result['samples'][key]={'count':len(r),'observed_hz':(len(r)-1)/(r[-1]-r[0]) if len(r)>1 else None}
if 'depth' in last:
 m=last['depth'];z=np.ndarray((m.height,m.width),dtype='<u2',buffer=m.data,strides=(m.step,2));valid=z[z>0]*.001
 cv2.imwrite(str(root/'depth.png'),z);gray=np.clip(255-z.astype(float)*255/5000,0,255).astype('uint8');preview=cv2.applyColorMap(gray,cv2.COLORMAP_TURBO);preview[z==0]=0;cv2.imwrite(str(root/'depth_preview.jpg'),preview)
 result['depth_valid_fraction']=float(np.count_nonzero(z)/z.size);result['depth_quantile_m']=np.percentile(valid,[5,50,95]).tolist() if valid.size else []
for key in ('cloud','colored'):
 if key in last:
  m=last[key];xyz=np.ndarray((m.width,3),dtype='<f4',buffer=m.data,strides=(m.point_step,4));result[key]={'points':m.width,'finite_xyz':bool(np.isfinite(xyz).all()),'min_z_m':float(xyz[:,2].min()) if m.width else None,'max_z_m':float(xyz[:,2].max()) if m.width else None}
  (root/(key+'.bin')).write_bytes(m.data)
(root/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='thermal'}));node.destroy_node();rclpy.shutdown()
