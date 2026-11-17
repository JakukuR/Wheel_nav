"""Subscribe to all depth-page JPEG feeds; save evidence without browser automation."""
import rclpy,time,json
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from pathlib import Path
root=Path('/home/onana/ros2_ws/.runtime/stereo_live');root.mkdir(exist_ok=True)
rclpy.init();n=rclpy.create_node('stereo_preview_verify');rows={};status={};subs=[]
def got(name,m):
 rows[name]=rows.get(name,0)+1;(root/(name+'.jpg')).write_bytes(bytes(m.data))
for name,topic in [('left','/r680/debug/calibration/left/compressed'),('right','/r680/debug/calibration/right/compressed'),('rgb','/r680/debug/depth/color/compressed'),('depth','/r680/debug/stereo/depth/compressed'),('aligned','/r680/debug/stereo/aligned/compressed')]:
 subs.append(n.create_subscription(CompressedImage,topic,lambda m,name=name:got(name,m),qos_profile_sensor_data))
def info(m):status.update(json.loads(m.data))
subs.append(n.create_subscription(String,'/camera/stereo/status',info,1));end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
result={'preview_frames':rows,'status':status};(root/'preview_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));n.destroy_node();rclpy.shutdown()
if any(rows.get(k,0)==0 for k in ('left','right','rgb','depth','aligned')):raise SystemExit('missing preview feed')
