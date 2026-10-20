"""Exact-stamp sidecar and target-only workload checks, isolated ROS domain 198."""
import json,os,time
from types import SimpleNamespace
import numpy as np
import pytest
pytestmark=pytest.mark.skipif(os.environ.get('RUN_SIM_ROS')!='1',reason='isolated ROS opt-in')


def test_sidecar_does_not_extract_bystanders_or_idle_frames():
    assert os.environ.get('ROS_DOMAIN_ID')=='198'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from wla_person_follow.reid_node import ReIDNode
    batches=[]
    def encode(crops):
        if crops:batches.append(len(crops))
        features=np.zeros((len(crops),512));features[:,0]=1.
        return features
    rclpy.init();side=ReIDNode(SimpleNamespace(model_id='test',encode=encode));fixture=Node('reid_sidecar_fixture')
    ex=SingleThreadedExecutor();ex.add_node(side);ex.add_node(fixture)
    rgb=fixture.create_publisher(Image,'/r680/d455/color/image_raw',qos_profile_sensor_data)
    raw=fixture.create_publisher(String,'/r680/perception/detections',1)
    relative=fixture.create_publisher(String,'/r680/perception/relative_objects',1)
    request=fixture.create_publisher(String,'/r680/person_follow/reid_request',1)
    packets=[];fixture.create_subscription(String,'/r680/person_follow/reid_features',lambda m:packets.append(json.loads(m.data)),1)
    def pump(seconds,active=False,refresh=True,skew=False,urgent=False):
        until=time.monotonic()+seconds;next_at=0.
        while time.monotonic()<until:
            if time.monotonic()>=next_at:
                next_at=time.monotonic()+.04
                ns=fixture.get_clock().now().nanoseconds
                image=Image();image.header.stamp.sec,image.header.stamp.nanosec=divmod(ns,10**9)
                image.header.frame_id='d455_color_optical_frame';image.width=640;image.height=480;image.step=640*3;image.encoding='rgb8';image.data=bytes(640*480*3)
                rgb.publish(image)
                raw.publish(String(data=json.dumps(dict(stamp_ns=ns,session='s',epoch=1,objects=[
                    dict(track_id=1,label='person',confidence=.9,bbox_xyxy=[20,20,100,300]),
                    dict(track_id=2,label='person',confidence=.9,bbox_xyxy=[200,20,280,300])]))))
                relative.publish(String(data=json.dumps(dict(stamp_ns=ns+1 if skew else ns,session='s',epoch=1,enabled=True,base_frame='r680_mapping_floor',
                    objects=[dict(id='s:1:'+str(i),label='person',detected=True,yaw=0.,distance=2.) for i in [1,2]]))))
                if refresh:request.publish(String(data=json.dumps(dict(schema='wla-reid-request/v1',stamp_ns=ns,request_id='task',session='s',epoch=1,enabled=active,urgent=urgent,allowed_ids=['s:1:2']))))
            ex.spin_once(timeout_sec=.005)
    try:
        pump(1.);assert not batches and not packets
        pump(1.5,True);assert batches and max(batches)==1
        assert all([r['id'] for r in p['objects']]==['s:1:2'] for p in packets)
        assert all(p['request_id']=='task' for p in packets)
        assert all(p['diagnostics']=={'s:1:2':'feature_ready'} for p in packets)
        count=len(batches);pump(1.2,True,urgent=True);assert 7<=len(batches)-count<=13
        pump(.7,False);count=len(batches);pump(.5,False);assert len(batches)==count
        pump(.7,True,skew=True);count=len(batches);pump(.5,True,skew=True);assert len(batches)==count
        pump(.7,True,refresh=False);count=len(batches);pump(.5,True,refresh=False);assert len(batches)==count
        assert side.count_publishers('/r680_base/cmd_vel')==0
    finally:
        side.close();ex.remove_node(side);ex.remove_node(fixture);side.destroy_node();fixture.destroy_node();ex.shutdown();rclpy.try_shutdown()
