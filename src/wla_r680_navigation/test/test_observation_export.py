from types import SimpleNamespace as NS
from pathlib import Path
import json
import time
import pytest
from wla_r680_navigation_py.observation_export import ObservationExporter
from wla_sim.perception.observation_window import ObservationPackage
from wla_sim.perception.observation_ipc import ObservationSender, ObservationReceiver


def source(stamp=123_000_000_000,number=10):
    h=NS(stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9),frame_id='camera_optical')
    image=NS(header=h,width=4,height=4,step=12,data=bytes(48),encoding='rgb8',is_bigendian=0)
    meta=NS(header=h,json_data=json.dumps(dict(frame_number=number,clock_domain='global_time',frame_timestamp=stamp/1e6)))
    batch=dict(stamp_ns=stamp,session='detector',epoch=1,objects=[dict(track_id=1,label='person',confidence=.9,bbox_xyxy=[0,0,3,3])])
    now=time.monotonic()
    return image,meta,(image,batch,None,now,stamp)


def test_exact_join_no_nearest_and_same_image_hash(tmp_path):
    e=ObservationExporter(str(tmp_path/'missing.sock'),sensor_id='test',provenance={})
    try:
        image,meta,item=source()
        with pytest.raises(ValueError,match='missing'):e.prepare(item)
        e.on_metadata(meta)
        header,pixels=e.prepare(item)
        p=ObservationPackage(image_ref=pixels,**header)
        assert p.image_ref==image.data and p.tracks[0]['source_frame_id']==p.source_frame_id
        assert p.localization_status['status']=='unavailable'
        image2,_,item2=source(123_001_000_000,11)
        with pytest.raises(ValueError,match='missing'):e.prepare(item2)
    finally:e.close()


def test_camera_counter_restart_creates_new_session(tmp_path):
    e=ObservationExporter(str(tmp_path/'missing.sock'),sensor_id='test',provenance={})
    try:
        _,m,i=source();e.on_metadata(m);a,_=e.prepare(i)
        _,m,i=source(123_100_000_000,1);e.on_metadata(m);b,_=e.prepare(i)
        assert a['source_session']!=b['source_session']
    finally:e.close()


def test_private_ipc_roundtrip_and_second_consumer_rejected(tmp_path):
    path=str(tmp_path/'frames.sock');received=[];errors=[]
    r=ObservationReceiver(path,lambda h,d,t:received.append((h,d)),errors.append)
    s=ObservationSender(path,lambda x:x)
    try:
        with pytest.raises(RuntimeError):ObservationReceiver(path,lambda *x:None,errors.append)
        s.offer(({'id':'exact'},b'abcdef'),6)
        deadline=time.monotonic()+2
        while not received and time.monotonic()<deadline:time.sleep(.01)
        assert received==[({'id':'exact'},b'abcdef')]
        assert Path(path).stat().st_mode&0o777==0o600
    finally:s.close();r.close()


def test_disconnected_receiver_does_not_prepare_or_block_nav(tmp_path):
    called=[];s=ObservationSender(str(tmp_path/'absent.sock'),lambda v:called.append(v),max_items=2,max_bytes=100)
    try:
        start=time.monotonic()
        for i in range(100):s.offer(i,10)
        assert time.monotonic()-start<.2
        time.sleep(.05)
        assert not called and len(s.queue)<=2 and s.bytes<=100
    finally:s.close()
