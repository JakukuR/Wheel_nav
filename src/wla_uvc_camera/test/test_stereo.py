"""Synthetic geometry fixtures; no physical calibration or accuracy claims."""
import cv2
import numpy as np
import pytest
import yaml
from wla_uvc_camera.stereo import crop_matrix,raw_crop,project_depth,load_rig,StereoRig


def rig_file(tmp_path,size=(1280,720)):
    k=[[800.,0.,(size[0]-1)/2],[0.,800.,(size[1]-1)/2],[0.,0.,1.]]
    rgb=[[1200.,0.,959.5],[0.,1200.,539.5],[0.,0.,1.]]
    right=np.eye(4);right[0,3]=-.06
    data=dict(schema='wla-uvc-rig/v1',units='m',cameras={
        'left':dict(width=size[0],height=size[1],k=k,d=[0.]*5,model='pinhole-radtan'),
        'right':dict(width=size[0],height=size[1],k=k,d=[0.]*5,model='pinhole-radtan'),
        'rgb':dict(width=1920,height=1080,k=rgb,d=[0.]*5,model='pinhole-radtan')},
        T_right_left=right.tolist(),T_rgb_left=np.eye(4).tolist())
    p=tmp_path/'fixture.yaml';p.write_text(yaml.safe_dump(data));return p,data


def test_center_crop_has_correct_pixel_centers():
    roi,a=crop_matrix()
    assert roi==(240,0,1440,1080)
    k=np.array([[1200.,0.,959.5],[0.,1200.,539.5],[0.,0.,1.]])
    out=a@k
    assert out[0,2]==pytest.approx(319.5)
    assert out[1,2]==pytest.approx(239.5)
    assert out[0,0]==pytest.approx(1200*4/9)


def test_full_view_retains_both_edges_and_updates_intrinsics(tmp_path):
    raw=np.zeros((1080,1920,3),np.uint8)
    raw[:,:200]=[0,0,255];raw[:,-200:]=[255,0,0]
    full=raw_crop(raw,output=(640,360))
    assert full.shape==(360,640,3)
    assert np.array_equal(full[180,10],[0,0,255])
    assert np.array_equal(full[180,-10],[255,0,0])
    assert not np.any(raw_crop(raw))  # Legacy central crop discards both edge targets.
    path,_=rig_file(tmp_path)
    rig=StereoRig(path,rgb_mode='full')
    assert rig.roi==(0,0,1920,1080) and rig.output==(640,360)
    assert rig.k[0,0]==pytest.approx(400.)
    assert rig.k[0,2]==pytest.approx(319.5)
    assert rig.k[1,2]==pytest.approx(179.5)
    assert rig.color(raw).shape==full.shape
    # Pick a projection on an integer pixel, avoiding ambiguous half-pixel ties.
    depth=project_depth([[.0125,.0225,2]],rig.k,np.eye(4),rig.output)
    assert depth.shape==(360,640) and depth[184,322]==2000


def test_z_buffer_and_nonfinite_or_out_of_range_points():
    points=np.array([[0,0,2.],[0,0,1.],[0,0,-2.],[np.nan,0,1.],[0,0,6.]])
    out=project_depth(points,np.eye(3),np.eye(4),(3,3))
    assert out[0,0]==1000 and np.count_nonzero(out)==1
    t=np.eye(4);t[0,3]=1
    out=project_depth([[0,0,1.]],np.eye(3),t,(3,3))
    assert out[0,1]==1000


@pytest.mark.parametrize('bad',['size','baseline','rotation','intrinsics','distortion','units'])
def test_rejects_unusable_calibration(tmp_path,bad):
    path,data=rig_file(tmp_path)
    if bad=='size':data['cameras']['rgb']['width']=640
    if bad=='baseline':data['T_right_left']=np.eye(4).tolist()
    if bad=='rotation':data['T_rgb_left'][0][0]=2
    if bad=='intrinsics':data['cameras']['left']['k'][0][0]=0
    if bad=='distortion':data['cameras']['left']['d']=[float('nan')]*5
    if bad=='units':data['units']='mm'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):load_rig(path)


@pytest.mark.parametrize('algorithm',['sgbm','bm'])
@pytest.mark.parametrize('rgb_mode',['crop','full'])
def test_textured_plane_recovers_two_metre_depth(tmp_path,algorithm,rgb_mode):
    path,_=rig_file(tmp_path);rig=StereoRig(path,algorithm,64,9,rgb_mode=rgb_mode)
    # f=800px, B=.06m, Z=2m -> 24px raw disparity, 12px at half size.
    rng=np.random.default_rng(7)
    texture=rng.integers(0,256,(360,640),dtype=np.uint8)
    left=cv2.resize(texture,(1280,720),interpolation=cv2.INTER_NEAREST)
    right=np.zeros_like(left);right[:,:-24]=left[:,24:]
    depth,disparity=rig.depth(left,right)
    valid=depth[depth>0]
    assert valid.size>depth.size*.25
    assert np.median(valid)==pytest.approx(2000,abs=30)
    assert np.median(disparity[disparity>0])==pytest.approx(12.,abs=.15)


@pytest.mark.parametrize('algorithm',['sgbm','bm'])
def test_upright_calibrated_stereo_recovers_metric_depth(tmp_path,algorithm):
    path,data=rig_file(tmp_path,size=(720,1280));rig=StereoRig(path,algorithm,64,9,rgb_mode='full')
    assert rig.input_size==(720,1280)
    rng=np.random.default_rng(8)
    left=cv2.resize(rng.integers(0,256,(640,360),dtype=np.uint8),rig.input_size,interpolation=cv2.INTER_NEAREST)
    right=np.zeros_like(left);right[:,:-24]=left[:,24:]
    depth,_=rig.depth(left,right);valid=depth[depth>0]
    assert valid.size>depth.size*.25
    assert np.median(valid)==pytest.approx(2000,abs=40)
    with pytest.raises(ValueError):rig.depth(left.T,right.T)
    data['cameras']['right']['width']=1280;data['cameras']['right']['height']=720
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):load_rig(path)
