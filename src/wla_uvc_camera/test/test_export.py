"""Kalibr chain direction and distortion model conversion using fixtures."""
import numpy as np
import pytest
import yaml
from wla_uvc_camera.export import export_chain,TOPICS
from wla_uvc_camera.geometry import calibration
from wla_uvc_camera.stereo import load_rig


def chain_file(tmp_path):
    data={}
    for i in range(3):
        data['cam'+str(i)]=dict(rostopic=TOPICS[i],camera_model='pinhole',distortion_model='radtan',
            intrinsics=[800.,800.,639.5,359.5] if i<2 else [1200.,1200.,959.5,539.5],
            distortion_coeffs=[.001,-.002,.003,-.004],resolution=[1280,720] if i<2 else [1920,1080])
    t=np.eye(4);t[0,3]=-.06;data['cam1']['T_cn_cnm1']=t.tolist()
    t=np.eye(4);t[0,3]=.08;t[1,3]=.01;data['cam2']['T_cn_cnm1']=t.tolist()
    p=tmp_path/'fixture-camchain.yaml';p.write_text(yaml.safe_dump(data));return p,data


def test_exports_chain_composition_and_raw_camera_info(tmp_path):
    source,_=chain_file(tmp_path);out=tmp_path/'export';export_chain(source,out)
    cams,right,rgb=load_rig(out/'rig.yaml')
    assert np.allclose(rgb[:3,3],[.02,.01,0.])
    assert np.allclose(right[:3,3],[-.06,0.,0.])
    assert np.allclose(cams['left'][1],[.001,-.002,.003,-.004,0.])
    assert calibration(out/'sunplus_left.yaml',1280,720)['k'][0]==800
    assert calibration(out/'sunplus_right.yaml',1280,720)['p'][3]<0
    assert calibration(out/'dicota.yaml',1920,1080)['k'][0]==1200
    assert sorted(yaml.safe_load((out/'stereo-camchain.yaml').read_text()))==['cam0','cam1']
    with pytest.raises(FileExistsError):export_chain(source,out)


@pytest.mark.parametrize('bad',['order','model','transform'])
def test_invalid_chain_never_writes_a_runtime_calibration(tmp_path,bad):
    source,data=chain_file(tmp_path)
    if bad=='order':data['cam0']['rostopic']=TOPICS[1]
    if bad=='model':data['cam2']['distortion_model']='equidistant'
    if bad=='transform':data['cam2']['T_cn_cnm1'][0][0]=2.
    source.write_text(yaml.safe_dump(data));out=tmp_path/'export'
    with pytest.raises(ValueError):export_chain(source,out)
    assert not out.exists()


def test_exports_upright_physical_eye_calibration_without_swapping_again(tmp_path):
    source,data=chain_file(tmp_path)
    for eye in ('cam0','cam1'):
        data[eye]['resolution']=[720,1280]
        data[eye]['intrinsics']=[800.,800.,359.5,639.5]
    source.write_text(yaml.safe_dump(data));out=tmp_path/'portrait-export'
    export_chain(source,out)
    _,right,_,size=load_rig(out/'rig.yaml',include_size=True)
    assert size==(720,1280) and right[0,3]==pytest.approx(-.06)
    assert calibration(out/'sunplus_left.yaml',720,1280)['k'][2]==359.5
    with pytest.raises(ValueError):calibration(out/'sunplus_left.yaml',1280,720)
