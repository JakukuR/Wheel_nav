#!/usr/bin/env python3
"""Isolated, non-motion HTTP/ROS integration. Requires ROS_DOMAIN_ID=197."""
import argparse
import json
import os
from pathlib import Path as FilePath
import signal
import subprocess
import sys
import time
import urllib.request
import urllib.error

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster


class Fixture(Node):
    def __init__(self):
        super().__init__('debug_fixture')
        self.camera_enabled = True
        self.geometry_mode = 'ok'
        self.detection_mode = 'ok'
        self.info_mode = 'ok'
        self.pubs = {k:self.create_publisher(typ, topic, 2) for k, typ, topic in [
            ('rgb', Image, '/r680/d455/color/image_raw'), ('info', CameraInfo, '/r680/d455/color/camera_info'),
            ('raw', String, '/r680/perception/detections'), ('rel', String, '/r680/perception/relative_objects'),
            ('voxels', PointCloud2, '/r680_nav/d455/points_confirmed'), ('points', PointCloud2, '/r680_nav/d455/points'),
            ('path', Path, '/r680/object_follow/local_path'), ('grid', OccupancyGrid, '/r680/object_follow/local_obstacles')]}
        self.broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id = 'd455_floor_odom';tf.child_frame_id = 'd455_color_optical_frame'
        tf.header.stamp = self.get_clock().now().to_msg();tf.transform.rotation.w = 1.
        self.broadcaster.sendTransform(tf)
        # This drawing is deliberately synthetic, not a camera or field-test claim.
        img = np.zeros((480,640,3), np.uint8);img[:] = [35,42,51]
        cv2.rectangle(img,(0,300),(639,479),(55,62,65),-1)
        for x in range(0,641,80):cv2.line(img,(320,240),(x,479),(72,85,91),1)
        for y in range(320,480,40):cv2.line(img,(0,y),(639,y),(72,85,91),1)
        cv2.rectangle(img,(155,90),(265,390),(72,101,101),-1)
        cv2.circle(img,(210,125),27,(159,178,175),-1)
        cv2.rectangle(img,(181,160),(239,310),(102,141,138),-1)
        cv2.rectangle(img,(430,225),(462,310),(125,157,184),-1)
        cv2.putText(img,'SYNTHETIC FIXTURE / NO ROBOT MOTION',(18,28),cv2.FONT_HERSHEY_SIMPLEX,.5,(196,216,228),1)
        self.image_bytes = img.tobytes()
        self.create_timer(.05,self.publish)

    def publish(self):
        now = self.get_clock().now();ns=now.nanoseconds
        h=Header(stamp=now.to_msg(),frame_id='d455_color_optical_frame')
        if self.camera_enabled:
            img=Image(header=h,height=480,width=640,encoding='bgr8',step=1920,data=self.image_bytes)
            self.pubs['rgb'].publish(img)
            info=CameraInfo(header=h,height=480,width=640,distortion_model='plumb_bob',d=[0.]*5,
                            k=[300.,0.,320.,0.,300.,240.,0.,0.,1.])
            if self.info_mode=='mismatch':info.header.frame_id='other_camera'
            self.pubs['info'].publish(info)
        raw={'session':'fixture','epoch':1,'stamp_ns':ns,'frame_id':h.frame_id,'width':640,'height':480,
             'objects':[{'track_id':42,'label':'person','confidence':.94,'bbox_xyxy':[155,90,265,390]},
                        {'track_id':7,'label':'bottle','confidence':.87,'bbox_xyxy':[430,225,462,310]}]}
        rel={'session':'fixture','epoch':1,'stamp_ns':ns,'enabled':True,'objects':[
            {'id':'fixture:1:42','detected':True,'distance':2.1,'yaw':.2},
            {'id':'fixture:1:7','detected':True,'distance':1.4,'yaw':-.3}]}
        if self.detection_mode=='wrong_frame':raw['stamp_ns']+=1_000_000
        if self.detection_mode=='wrong_epoch':rel['epoch']=2
        self.pubs['raw'].publish(String(data=json.dumps(raw)))
        self.pubs['rel'].publish(String(data=json.dumps(rel)))
        gh=Header(stamp=now.to_msg(),frame_id='d455_floor_odom')
        # Deliberately different acquisition times exercise lookup_transform_full.
        earlier=ns-50_000_000;gh.stamp.sec,gh.stamp.nanosec=divmod(earlier,1_000_000_000)
        if self.geometry_mode=='missing_tf':gh.frame_id='unknown_frame'
        if self.geometry_mode=='stale':gh.stamp.sec-=5
        if self.geometry_mode=='zero':gh.stamp.sec=gh.stamp.nanosec=0
        pts=np.array([[x*.1,y*.1,2.+z*.1] for x in range(-9,-3) for y in range(-3,5) for z in range(2)],np.float32)
        cloud=point_cloud2.create_cloud_xyz32(gh,pts)
        self.pubs['voxels'].publish(cloud);self.pubs['points'].publish(cloud)
        path=Path(header=gh)
        for y,z in [(.8,.8),(.7,1.2),(.6,1.8),(.4,2.5),(.2,3.2)]:
            pose=PoseStamped(header=gh);pose.pose.position.y=y;pose.pose.position.z=z;pose.pose.orientation.w=1.;path.poses.append(pose)
        self.pubs['path'].publish(path)
        grid=OccupancyGrid(header=gh)
        grid.info.width=grid.info.height=4;grid.info.resolution=.1
        grid.info.origin.position.x=.4;grid.info.origin.position.z=2.;grid.info.origin.orientation.w=1.
        grid.data=[100,100,0,0]*4;self.pubs['grid'].publish(grid)


def main():
    p=argparse.ArgumentParser();p.add_argument('--hold',type=float,default=0);p.add_argument('--port',type=int,default=18082);args=p.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='197', 'Fixture must only run in isolated ROS domain 197'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    base=FilePath(__file__).resolve().parents[1]
    env=os.environ.copy();env['PYTHONPATH']=str(base)+':'+env.get('PYTHONPATH','')
    log=open('/tmp/wla-perception-debug-fixture.log','w')
    proc=subprocess.Popen([sys.executable,str(base/'scripts/nav_perception_debug'),'--port',str(args.port)],env=env,stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();fixture=Fixture()
    url=f'http://127.0.0.1:{args.port}'
    def get(view):
        with urllib.request.urlopen(url+'/api/'+view,timeout=2) as f:return json.load(f)
    def wait_for(check, timeout=8):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(fixture,timeout_sec=.03)
            try:
                d,g=get('detections'),get('planning')
                if check(d,g):return d,g
            except (OSError,urllib.error.URLError):pass
            if proc.poll() is not None:raise AssertionError(FilePath(log.name).read_text())
        raise AssertionError('Timed out: '+json.dumps({'d':{k:v for k,v in d.items() if k!='image'},'g':{k:v for k,v in g.items() if k!='image'}})[:4000])
    def layer(g,key):return next((x for x in g.get('layers',[]) if x['id']==key),{})
    try:
        d,g=wait_for(lambda d,g:d.get('reason')=='ok' and len(d.get('objects',[]))==2 and layer(g,'voxels').get('reason')=='ok' and len(layer(g,'object_path').get('segments',[]))>0)
        assert d['objects'][0]['distance']==2.1
        assert len(layer(g,'voxels')['segments'])>0 and len(layer(g,'object_grid')['segments'])>0
        assert layer(g,'object_grid')['count']==8
        print('PASS exact RGB/person/object pair; cross-time voxel, point, path and 2D-grid projection',flush=True)
        fixture.detection_mode='wrong_epoch'
        wait_for(lambda d,g:d.get('reason')=='ok' and len(d.get('objects',[]))==2 and all(x['distance'] is None for x in d['objects']))
        fixture.detection_mode='wrong_frame'
        wait_for(lambda d,g:d.get('reason')=='waiting_for_exact_detection_frame' and not d.get('objects'))
        fixture.detection_mode='ok'
        print('PASS epoch mismatch and absent exact RGB frame never borrow distances/boxes',flush=True)
        fixture.geometry_mode='missing_tf'
        wait_for(lambda d,g:'unknown_frame' in layer(g,'voxels').get('reason','') and not layer(g,'voxels').get('segments'))
        for mode in ['stale','zero']:
            fixture.geometry_mode=mode
            wait_for(lambda d,g:layer(g,'voxels').get('reason')=='source_stale_or_zero_stamp' and not layer(g,'voxels').get('segments'))
        fixture.geometry_mode='ok'
        print('PASS missing TF, replayed old and zero-stamped geometry suppressed',flush=True)
        fixture.info_mode='mismatch'
        wait_for(lambda d,g:g.get('reason')=='camera_info_stale_or_time_mismatch' and not g.get('layers'))
        fixture.info_mode='ok'
        fixture.camera_enabled=False
        wait_for(lambda d,g:not d.get('image') and not g.get('image'))
        fixture.camera_enabled=True
        wait_for(lambda d,g:d.get('reason')=='ok' and layer(g,'voxels').get('reason')=='ok')
        print('PASS invalid CameraInfo and camera dropout clear overlays; recovery works',flush=True)
        for page in ['/detections','/planning','/app.js','/style.css']:
            with urllib.request.urlopen(url+page) as f:assert f.status==200 and len(f.read())>100
        try:
            urllib.request.urlopen(urllib.request.Request(url+'/api/planning',data=b'{}',method='POST'))
            raise AssertionError('write unexpectedly accepted')
        except urllib.error.HTTPError as e:assert e.code==501
        print('PASS both HTTP windows/assets; HTTP mutation rejected',flush=True)
        summary={'detection_count':len(d['objects']), 'layers':[{k:v for k,v in l.items() if k not in ('segments','points')} for l in g['layers']], 'result':'PASS', 'domain':197}
        FilePath('/tmp/wla-perception-debug-test.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
        print(f'FIXTURE READY http://172.31.18.156:{args.port}/detections and /planning; hold={args.hold}s',flush=True)
        end=time.monotonic()+args.hold
        while time.monotonic()<end:rclpy.spin_once(fixture,timeout_sec=.05)
    finally:
        fixture.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close()


if __name__=='__main__':main()
