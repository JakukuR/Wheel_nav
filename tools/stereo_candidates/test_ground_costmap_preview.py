"""Bounded, read-only sensor integration with an isolated Nav2 costmap instance.
Only launches costmap_2d, never planner/controller/behavior/motor nodes.
"""
import argparse,json,subprocess,time
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import ChangeState,GetState
from lifecycle_msgs.msg import Transition
from nav_msgs.msg import OccupancyGrid
p=argparse.ArgumentParser();p.add_argument('--params',required=True);p.add_argument('--out',required=True);a=p.parse_args()
rclpy.init();n=rclpy.create_node('ground_costmap_probe');received=[]
sub=n.create_subscription(OccupancyGrid,'/ground_map_preview/costmap',lambda m:received.append((time.monotonic(),m)),10)
out=Path(a.out);out.parent.mkdir(parents=True,exist_ok=True);log=open(out.with_suffix('.log'),'w')
child=subprocess.Popen(['/opt/ros/jazzy/lib/nav2_costmap_2d/nav2_costmap_2d','--ros-args','-r','__ns:=/ground_map_preview','-r','__node:=costmap','--params-file',a.params],stdout=log,stderr=subprocess.STDOUT)
def call(client,request):
 assert client.wait_for_service(timeout_sec=12),'lifecycle service missing'
 f=client.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=12);assert f.done(),'lifecycle timeout';return f.result()
try:
 change=n.create_client(ChangeState,'/ground_map_preview/costmap/change_state');state=n.create_client(GetState,'/ground_map_preview/costmap/get_state')
 for ident in [Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE]:
  request=ChangeState.Request();request.transition.id=ident;assert call(change,request).success,ident
 end=time.monotonic()+20
 while time.monotonic()<end:
  assert child.poll() is None,'costmap process exited';rclpy.spin_once(n,timeout_sec=.02)
 result={'state':call(state,GetState.Request()).current_state.label,'messages':len(received),'topics':n.get_topic_names_and_types()}
 if received:
  t,m=received[-1];result.update(hz=(len(received)-1)/(t-received[0][0]),frame=m.header.frame_id,shape=[m.info.width,m.info.height],resolution=m.info.resolution,cost_values=sorted(set(m.data)),unknown=sum(x==-1 for x in m.data),lethal=sum(x==100 for x in m.data))
 out.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='topics'},indent=2));assert result['messages']>20 and result['state']=='active';assert result['frame']=='stereo_local_ground_preview' and 100 in result['cost_values'] and -1 in result['cost_values']
 request=ChangeState.Request();request.transition.id=Transition.TRANSITION_DEACTIVATE;assert call(change,request).success
finally:
 child.terminate()
 try:child.wait(timeout=8)
 except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3)
 log.close();n.destroy_node();rclpy.shutdown()
