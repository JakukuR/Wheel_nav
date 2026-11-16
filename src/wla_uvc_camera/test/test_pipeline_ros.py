"""Synthetic ROS contracts, isolated domain only; no camera or chassis access."""
import os
import time
from types import SimpleNamespace as NS
import cv2
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from std_msgs.msg import Header
from wla_uvc_camera.pipeline import Pipeline,image,stamp
from test_stereo import rig_file

pytestmark=pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID')!='197',reason='requires isolated ROS domain 197')


@pytest.fixture(params=[('crop',(1280,720)),('full',(1280,720)),('crop',(720,1280)),('full',(720,1280))])
def pipeline(tmp_path,request):
    mode,size=request.param
    path,_=rig_file(tmp_path,size=size);rclpy.init()
    node=Pipeline(NS(calibration=str(path),algorithm='sgbm',disparities=64,block_size=9,max_skew_ms=25.,rgb_mode=mode))
    published={k:[] for k in ('color','depth','info')}
    for name in published:setattr(node,name+'_pub',NS(publish=published[name].append))
    try:yield node,published
    finally:node.close();node.destroy_node();rclpy.try_shutdown()


def packet(values,frame,ns):
    header=Header();header.frame_id=frame
    header.stamp.sec=ns//1_000_000_000;header.stamp.nanosec=ns%1_000_000_000
    return image(np.repeat(values[:,:,None],3,axis=2),header,'bgr8')


def test_uncalibrated_default_keeps_full_view_and_no_fake_intrinsics():
    rclpy.init()
    node=Pipeline(NS(calibration='',algorithm='sgbm',disparities=64,block_size=9,max_skew_ms=25.))
    out={k:[] for k in ('color','depth','info')}
    for name in out:setattr(node,name+'_pub',NS(publish=out[name].append))
    try:
        raw=np.zeros((1080,1920),np.uint8);raw[:,:200]=50;raw[:,-200:]=200
        ns=node.get_clock().now().nanoseconds
        node.rgb(packet(raw,'dicota_color_optical_frame',ns))
        assert len(out['color'])==len(out['info'])==1 and not out['depth']
        color,info=out['color'][0],out['info'][0]
        assert (color.width,color.height)==(info.width,info.height)==(640,360)
        values=np.frombuffer(color.data,np.uint8).reshape(360,640,3)
        assert values[180,10,0]==50 and values[180,-10,0]==200
        assert stamp(color)==stamp(info)==ns and not any(info.k)
    finally:node.close();node.destroy_node();rclpy.try_shutdown()


def test_metric_depth_keeps_stereo_time_and_rgb_keeps_its_own_time(pipeline):
    node,out=pipeline
    rng=np.random.default_rng(7)
    left=cv2.resize(rng.integers(0,256,(360,640),dtype=np.uint8),node.rig.input_size,interpolation=cv2.INTER_NEAREST)
    right=np.zeros_like(left);right[:,:-24]=left[:,24:]
    ns=node.get_clock().now().nanoseconds
    node.rgb(packet(np.zeros((1080,1920),np.uint8),'dicota_color_optical_frame',ns-5_000_000))
    node.eye('left',packet(left,'sunplus_left_optical_frame',ns-10_000_000))
    node.eye('right',packet(right,'sunplus_right_optical_frame',ns-10_000_000))
    end=time.monotonic()+2
    while not out['color'] and time.monotonic()<end:time.sleep(.005)
    assert out['color'] and out['depth'] and out['info']
    depth=out['depth'][0];color=out['color'][0];info=out['info'][0]
    assert stamp(depth)==ns-10_000_000 and stamp(color)==ns-5_000_000
    assert depth.header.frame_id==color.header.frame_id=='dicota_color_optical_frame'
    assert (depth.width,depth.height,depth.encoding)==(*node.output,'16UC1')
    assert (color.width,color.height)==(info.width,info.height)==node.output
    scale=1/3 if node.rgb_mode=='full' else 4/9
    assert info.k[0]==pytest.approx(1200*scale) and info.k[2]==pytest.approx(319.5)
    assert info.k[5]==pytest.approx((node.output[1]-1)/2)
    values=np.frombuffer(depth.data,dtype='<u2');assert np.median(values[values>0])==pytest.approx(2000,abs=30)


def test_skewed_stereo_rgb_and_stale_input_cannot_produce_geometry(pipeline):
    node,out=pipeline;ns=node.get_clock().now().nanoseconds
    eye=np.zeros(node.rig.input_size[::-1],np.uint8)
    node.rgb(packet(np.zeros((1080,1920),np.uint8),'dicota_color_optical_frame',ns-100_000_000))
    node.eye('left',packet(eye,'sunplus_left_optical_frame',ns))
    node.eye('right',packet(eye,'sunplus_right_optical_frame',ns))
    assert node.pending is None and not any(out.values())
    node.rgb(packet(np.zeros((1080,1920),np.uint8),'dicota_color_optical_frame',ns-1_000_000_000))
    assert len(node.colors)==1 and not any(out.values())
