"""Non-motion regression tests; run with an isolated ROS_DOMAIN_ID."""
import unittest,runpy,threading,json,urllib.request,urllib.error,time,math,struct,tempfile
from pathlib import Path
from std_msgs.msg import Header,String
from sensor_msgs.msg import Image
from sensor_msgs_py import point_cloud2
import rclpy

ns=runpy.run_path(str(Path(__file__).resolve().parents[1]/'scripts/web_gateway'),run_name='gateway_under_test')
class GatewayTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  rclpy.init();cls.node=ns['GatewayNode']()
  cls.server=ns['ThreadingHTTPServer'](('127.0.0.1',0),ns['GatewayHandler']);cls.server.gateway=cls.node;cls.server.web_root=Path(__file__).resolve().parents[1]/'web'
  cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True);cls.thread.start();cls.url='http://127.0.0.1:'+str(cls.server.server_port)
 @classmethod
 def tearDownClass(cls):
  cls.server.shutdown();cls.server.server_close();cls.node.destroy_node();rclpy.shutdown()
 def request(self,path,body=None,origin=None):
  headers={'Content-Type':'application/json'}
  if origin:headers['Origin']=origin
  req=urllib.request.Request(self.url+path,data=body,headers=headers)
  try:
   with urllib.request.urlopen(req,timeout=3) as r:return r.status,r.read(),r.headers
  except urllib.error.HTTPError as e:return e.code,e.read(),e.headers
 def test_static_asset_and_traversal(self):
  self.assertEqual(self.request('/')[0],200)
  self.assertEqual(self.request('/vendor/bicmap-0.0.9.min.js')[0],200)
  self.assertEqual(self.request('/%2e%2e/scripts/web_gateway')[0],404)
  self.assertEqual(self.request('/vendor/../../scripts/web_gateway')[0],404)
 def test_state(self):
  code,data,_=self.request('/api/state');state=json.loads(data)
  self.assertEqual(code,200);self.assertEqual(state['frontend'],'r680-bicmap-v1');self.assertFalse(state['controls']['goal'])
 def test_colcon_individual_file_symlinks(self):
  original=self.server.web_root
  try:
   with tempfile.TemporaryDirectory() as directory:
    alias=Path(directory);(alias/'index.html').symlink_to(original/'index.html')
    self.server.web_root=alias
    self.assertEqual(self.request('/')[0],200)
    self.assertEqual(self.request('/coordinates.js')[0],200)
    self.assertEqual(self.request('/vendor/bicmap-0.0.9.min.js')[0],200)
    self.assertEqual(self.request('/%2e%2e/scripts/web_gateway')[0],404)
  finally:self.server.web_root=original
 def test_http_validation_and_disabled_endpoint(self):
  self.assertEqual(self.request('/api/goal',b'{"x":1,"y":2,"yaw":0}')[0],409)
  self.assertEqual(self.request('/api/goal',b'{"x":NaN,"y":2,"yaw":0}')[0],400)
  self.assertEqual(self.request('/api/goal',b'{}',origin='http://attacker.invalid')[0],403)
 def test_queue_acceptance_is_not_ros_execution(self):
  n=self.node;original=n.controls;n.controls=lambda:{'goal':True}
  try:
   code,data,_=self.request('/api/goal',b'{"x":1,"y":2,"yaw":0}')
   self.assertEqual(code,202);self.assertEqual(json.loads(data)['stage'],'queued')
   command,payload=n._commands.get_nowait();self.assertEqual(command,'goal');self.assertEqual(payload,{'x':1.,'y':2.,'yaw':0.})
  finally:n.controls=original
 def test_xyz_preserved_nonfinite_removed(self):
  n=self.node;h=Header();h.frame_id='map';h.stamp=n.get_clock().now().to_msg()
  msg=point_cloud2.create_cloud_xyz32(h,[[1,2,3],[2,3,float('inf')]])
  n._received.pop('near_obstacles',None);n._points_cb(msg,'near_obstacles')
  self.assertEqual(n._state['near_obstacles'],[[1.,2.,3.]])
 def test_stale_cloud_rejected(self):
  n=self.node;h=Header();h.frame_id='map';h.stamp.sec=1
  n._received.pop('near_obstacles',None);version=n._state['cloud_versions']['near_obstacles']
  n._points_cb(point_cloud2.create_cloud_xyz32(h,[[8,9,10]]),'near_obstacles')
  self.assertEqual(n._state['cloud_versions']['near_obstacles'],version)
 def test_camera_exact_match_stride_and_expiry(self):
  n=self.node;msg=Image();msg.header.frame_id='camera';msg.header.stamp=n.get_clock().now().to_msg();msg.width=16;msg.height=16;msg.encoding='bgr8';msg.step=50;msg.data=bytes([0,0,200]*16+[0,0])*16
  n._camera_frames.clear();n._camera_cb(msg);stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
  det={'stamp_ns':stamp,'frame_id':'camera','width':16,'height':16,'objects':[{'label':'cup','bbox_xyxy':[2,2,12,12]}]}
  m=String();m.data=json.dumps(det);n._detections_cb(m);body,matched=n.camera_jpeg();self.assertTrue(matched);self.assertTrue(body.startswith(b'\xff\xd8'))
  det['stamp_ns']=stamp-1;m.data=json.dumps(det);n._detections_cb(m);self.assertFalse(n.camera_jpeg()[1])
  n._camera_frames.clear();self.assertEqual(self.request('/api/camera.jpg')[0],503)
if __name__=='__main__':unittest.main(verbosity=2)
