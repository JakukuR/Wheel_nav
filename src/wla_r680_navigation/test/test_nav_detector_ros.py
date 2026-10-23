"""Continuous worker and image-time robot geometry; no map/odom or chassis fixture."""
import importlib.machinery
import importlib.util
import json
import math
import os
import threading
import time
from types import SimpleNamespace as NS
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Transform
from sensor_msgs.msg import Image, CameraInfo


def module():
    path=os.environ.get('WLA_NAV_DETECTOR_SOURCE', '/home/orin/ros2_ws/install/wla_r680_navigation/lib/wla_r680_navigation/nav_yolo_detector')
    loader=importlib.machinery.SourceFileLoader('nav_detector_under_test',path)
    spec=importlib.util.spec_from_loader(loader.name,loader)
    mod=importlib.util.module_from_spec(spec);loader.exec_module(mod)
    return mod


class Model:
    def __init__(self):
        self.calls=0
        self.entered=threading.Event()
        self.release=threading.Event()
        self.release.set()
        self.fail=False
    def track(self,*args,**kwargs):
        self.calls+=1;self.entered.set()
        assert self.release.wait(2.)
        if self.fail:
            self.fail=False
            raise ValueError('fixture_inference_failure')
        return [NS(boxes=[],names={})]


@pytest.fixture
def fixture(monkeypatch):
    # These tests isolate ROS/queue/geometry contracts. Real TrackingRuntime and
    # BoxMOT tracking is tested separately with the actual YOLOE on GPU0.
    class FixtureRuntime:
        def __init__(self, model, config=None):
            self.model = model
            self.signature = ('fixture-warmed',)
            self.last_metrics = {}
            self.use_cmc=True;self.features={}
            self.reid=NS(model_id='fixture')
        def reset(self):pass
        def ensure_warm(self, *args, **kwargs):
            return False
        def process(self, *args, **kwargs):
            return self.model.track(*args, **kwargs)[0], []
    import wla_r680_navigation_py.boxmot_runtime as runtime_module
    monkeypatch.setattr(runtime_module, 'TrackingRuntime', FixtureRuntime)
    assert os.environ.get('ROS_DOMAIN_ID')=='198'
    rclpy.init()
    model=Model()
    node=module().NavDetector(NS(base_frame='r680_mapping_floor',
        device='cpu',imgsz=416,tracker='',hz=30.),model)
    raw=[];relative=[]
    class Pub:
        def __init__(self,rows):self.rows=rows
        def publish(self,m):self.rows.append(json.loads(m.data))
    node.raw_pub=Pub(raw);node.relative_pub=Pub(relative)
    try:yield node,model,raw,relative
    finally:
        model.release.set();node.stop();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def rgb(node):
    msg=Image();msg.header.stamp=node.get_clock().now().to_msg()
    msg.header.frame_id='d455_color_optical_frame'
    msg.width=64;msg.height=48;msg.encoding='bgr8';msg.step=192;msg.data=bytes(64*48*3)
    return msg


def drain_until(node,predicate,timeout=1.):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        node.drain()
        if predicate():return
        time.sleep(.005)
    assert predicate()


def test_fresh_frames_run_without_agent_or_enable_request(fixture):
    node,model,raw,relative=fixture
    time.sleep(.02)
    assert model.calls==0 and node.pending is None
    node.on_image(rgb(node))
    drain_until(node,lambda:bool(raw))
    assert model.calls==1 and relative[-1]['enabled'] is True
    assert '/r680/perception/set_enabled' not in {s.srv_name for s in node.services}


def test_camera_epoch_change_drops_inflight_results(fixture):
    node,model,raw,relative=fixture
    model.release.clear();node.on_image(rgb(node))
    assert model.entered.wait(1.)
    previous=node.epoch
    with node.condition:
        node.reset_epoch()
    assert node.epoch>previous
    node.on_image(rgb(node));model.release.set()
    drain_until(node,lambda:bool(raw))
    assert model.calls==2 and len(raw)==1 and raw[0]['epoch']==node.epoch


def test_shutdown_retires_pending_and_inflight_work(fixture):
    node,model,raw,relative=fixture
    model.release.clear();node.on_image(rgb(node))
    assert model.entered.wait(1.)
    node.on_image(rgb(node))
    shutdown=threading.Thread(target=node.stop)
    shutdown.start()
    deadline=time.monotonic()+1.
    while not node.stopping and time.monotonic()<deadline:
        time.sleep(.005)
    assert node.stopping
    model.release.set();shutdown.join(1.)
    assert not shutdown.is_alive() and not node.worker.is_alive()
    node.on_image(rgb(node));node.drain()
    assert node.pending is None and not raw and not relative


def test_failure_changes_epoch_and_next_fresh_frame_recovers(fixture):
    node,model,raw,relative=fixture
    epoch=node.epoch;model.fail=True
    node.on_image(rgb(node))
    drain_until(node,lambda:node.epoch>epoch)
    assert not raw
    node.on_image(rgb(node));drain_until(node,lambda:bool(raw))
    assert raw[-1]['epoch']==node.epoch


def geometry(node):
    stamp=node.ros_ns()
    depth=Image();depth.header.stamp=rclpy.time.Time(nanoseconds=stamp).to_msg()
    depth.header.frame_id='d455_color_optical_frame';depth.width=64;depth.height=48
    depth.encoding='16UC1';depth.step=128;depth.data=np.full((48,64),2000,dtype='<u2').tobytes()
    info=CameraInfo();info.header=depth.header;info.width=64;info.height=48
    info.k=[40.,0.,32.,0.,40.,24.,0.,0.,1.]
    node.depths.append(depth);node.infos.append(info)
    batch=dict(stamp_ns=stamp,session=node.session,epoch=node.epoch,
        frame_id=depth.header.frame_id,width=64,height=48,
        objects=[dict(track_id=7,label='person',confidence=.8,bbox_xyxy=[24,16,48,40])])
    return batch


def test_projection_only_robot_frame_at_acquisition_time(fixture):
    node,_,_,_=fixture
    b=geometry(node)
    # Optical (right,down,forward) -> base (forward,left,up).
    tf=Transform();tf.rotation.x=-.5;tf.rotation.y=.5;tf.rotation.z=-.5;tf.rotation.w=.5
    tf.translation.x=.1;tf.translation.z=.25
    requested=[]
    def lookup(target,source,when):
        requested.append((target,source,when.nanoseconds));return NS(transform=tf)
    node.tf.lookup_transform=lookup
    row=node.project(b)['objects'][0]
    assert set(row)=={'detected','label','id','yaw','distance'}
    assert row['id']==f'{node.session}:{node.epoch}:7'
    assert row['yaw']==pytest.approx(math.atan2(-.2,2.1))
    assert row['distance']==pytest.approx(math.hypot(2.1,.2))
    assert requested==[('r680_mapping_floor','d455_color_optical_frame',b['stamp_ns'])]
    names={name for name,_ in node.get_topic_names_and_types()}
    assert not {'/map','/d455_slam/odom','/r680_nav/localization_ready'} & names


def test_missing_or_unsynchronized_geometry_preserves_label_only(fixture):
    node,_,_,_=fixture
    b=geometry(node)
    node.tf.lookup_transform=lambda *args:(_ for _ in ()).throw(ValueError('no extrinsic'))
    row=node.project(b)['objects'][0]
    assert row['detected'] and row['label']=='person' and row['yaw'] is None and row['distance'] is None
    node.depths.clear()
    assert node.project(b)['objects'][0]['distance'] is None


def test_expired_frame_cannot_be_republished_with_new_stamp(fixture):
    node,_,raw,relative=fixture
    relative.clear()
    batch=geometry(node);batch['stamp_ns']-=400_000_000
    node.result=(batch,dict(stamp_ns=batch['stamp_ns']))
    node.drain()
    assert not raw and not relative


def test_person_stop_forwards_without_stopping_detection(fixture):
    from rclpy.executors import MultiThreadedExecutor
    from wla_behavior_interfaces.srv import PersonCommand
    path='/home/orin/ros2_ws/install/wla_behavior_bridge/lib/wla_behavior_bridge/agent_person_api'
    loader=importlib.machinery.SourceFileLoader('person_api_under_test',path)
    spec=importlib.util.spec_from_loader(loader.name,loader)
    api_module=importlib.util.module_from_spec(spec);loader.exec_module(api_module)
    node,model,raw,_=fixture
    api=api_module.AgentPersonAPI()
    backend=rclpy.create_node('test_person_backend')
    probe=rclpy.create_node('test_person_client')
    received=[]
    def stop(request,response):
        received.append((request.command,request.request_id))
        response.accepted=True;response.enabled=False
        response.request_id=request.request_id;response.state='stopped'
        return response
    backend.create_service(PersonCommand,'/r680/person_follow/command',stop)
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(api);executor.add_node(backend)
    spinning=threading.Thread(target=executor.spin)
    spinning.start()
    try:
        node.on_image(rgb(node));drain_until(node,lambda:len(raw)==1)
        endpoints={s.srv_name for s in api.services}
        assert '/wla_agent/person_command' in endpoints
        assert '/wla_agent/perception/set_enabled' not in endpoints
        client=probe.create_client(PersonCommand,'/wla_agent/person_command')
        assert client.wait_for_service(timeout_sec=2.)
        future=client.call_async(PersonCommand.Request(command='stop',request_id='chosen',lease_duration_s=.5))
        rclpy.spin_until_future_complete(probe,future,timeout_sec=2.)
        assert future.done() and future.result().accepted
        assert future.result().state=='stopped' and received==[('stop','chosen')]
        node.on_image(rgb(node));drain_until(node,lambda:len(raw)==2)
        assert model.calls==2
    finally:
        executor.shutdown(timeout_sec=2.);spinning.join(2.)
        probe.destroy_node();backend.destroy_node();api.destroy_node()


def test_native_association_has_no_custom_quality_gate_or_public_id_remap(fixture):
    node,model,raw,relative=fixture
    row=dict(track_id=7,label='person',confidence=.9,bbox_xyxy=[24.,16.,48.,40.],detection_index=0)
    node.runtime.process=lambda *a,**kw:(NS(masks=None),[row])
    tf=Transform();tf.rotation.x=-.5;tf.rotation.y=.5;tf.rotation.z=-.5;tf.rotation.w=.5
    node.tf.lookup_transform=lambda *a:NS(transform=tf)
    epoch=node.epoch
    for _ in range(3):
        geometry(node);count=len(raw);node.on_image(rgb(node));drain_until(node,lambda:len(raw)>count)
        assert raw[-1]['objects'][0]['track_id']==7
        assert raw[-1]['association']['mode']=='boxmot_native_appearance'
        assert relative[-1]['objects'][0]['distance'] is not None
        assert node.epoch==epoch


def test_native_features_reused_and_masks_keep_detector_index_and_stamp(fixture):
    from wla_r680_navigation_py.target_masks import decode_mask
    import torch
    node,model,raw,relative=fixture
    # Larger fixture crop meets the retained long-term identity quality policy.
    msg=rgb(node);msg.width=160;msg.height=120;msg.step=480;msg.data=bytes(160*120*3)
    b=geometry(node)
    depth=node.depths[-1];info=node.infos[-1]
    for item in (depth,info):item.header=msg.header;item.width=160;item.height=120
    depth.step=320;depth.data=np.full((120,160),2000,dtype='<u2').tobytes()
    info.k=[100.,0.,80.,0.,100.,60.,0.,0.,1.]
    tf=Transform();tf.rotation.w=1.
    node.tf.lookup_transform=lambda *a:NS(transform=tf)
    row=dict(track_id=4,label='person',confidence=.9,bbox_xyxy=[30.,20.,75.,110.],detection_index=1)
    masks=torch.zeros((2,120,160));masks[1,20:110,30:75]=1
    node.runtime.process=lambda *a,**kw:(NS(masks=NS(data=masks)),[row])
    node.runtime.features={4:np.ones(512,dtype=float)/np.sqrt(512)}
    node.runtime.last_metrics={'reid_ms':2.}
    ident=f'{node.session}:{node.epoch}:4'
    node.mask_interest=dict(schema='wla-reid-request/v1',stamp_ns=node.ros_ns(),enabled=True,
        request_id='mask-task',session=node.session,epoch=node.epoch,target_id=ident,allowed_ids=[ident])
    reid=[];mask=[]
    class Pub:
        def __init__(self,rows):self.rows=rows
        def publish(self,m):self.rows.append(json.loads(m.data))
    node.reid_pub=Pub(reid);node.mask_pub=Pub(mask)
    node.on_image(msg);drain_until(node,lambda:bool(mask))
    assert reid[-1]['objects'][0]['id']==ident
    assert len(reid[-1]['objects'][0]['feature'])==512
    assert reid[-1]['stamp_ns']==mask[-1]['depth_stamp_ns']==raw[-1]['stamp_ns']
    assert mask[-1]['request_id']=='mask-task' and mask[-1]['target_id']==ident
    assert mask[-1]['bbox_xyxy']==[30.,20.,75.,110.]
    assert decode_mask(mask[-1]['bits'],120,160).sum()==90*45
    assert '_reid' not in raw[-1] and '_target_mask' not in raw[-1]


def test_target_box_is_published_when_segmentation_is_missing(fixture):
    node,_,raw,_=fixture
    msg=rgb(node)
    geometry(node)
    for item in (node.depths[-1],node.infos[-1]):item.header=msg.header
    row=dict(track_id=4,label='person',confidence=.9,
             bbox_xyxy=[12.,8.,42.,44.],detection_index=0)
    node.runtime.process=lambda *a,**kw:(NS(masks=None),[row])
    ident=f'{node.session}:{node.epoch}:4'
    node.mask_interest=dict(schema='wla-reid-request/v1',stamp_ns=node.ros_ns(),enabled=True,
        request_id='mask-task',session=node.session,epoch=node.epoch,
        target_id=ident,allowed_ids=[ident])
    masks=[]
    class Pub:
        def publish(self,m):masks.append(json.loads(m.data))
    node.mask_pub=Pub()
    node.on_image(msg);drain_until(node,lambda:bool(masks))
    assert raw and masks[-1]['target_id']==ident
    assert masks[-1]['bbox_xyxy']==row['bbox_xyxy']
    assert masks[-1]['bits']==''
