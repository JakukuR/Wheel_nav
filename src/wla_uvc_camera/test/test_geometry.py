import numpy as np
import pytest
import yaml
from wla_uvc_camera.geometry import split_frame,calibration


def test_layout_keeps_exact_left_right_pixels():
    a=np.zeros((3,8,3),np.uint8);a[:,:4]=23;a[:,4:]=157
    left,right=split_frame(a,'side_by_side')
    assert left.shape==right.shape==(3,4,3)
    assert np.all(left==23) and np.all(right==157)
    assert len(split_frame(a,'mono'))==1
    with pytest.raises(ValueError):split_frame(a[:,:7],'side_by_side')


def record():
    return dict(image_width=640,image_height=480,distortion_model='plumb_bob',
        camera_matrix=dict(data=[500,0,320,0,500,240,0,0,1]),
        rectification_matrix=dict(data=np.eye(3).reshape(-1).tolist()),
        projection_matrix=dict(data=[500,0,320,0,0,500,240,0,0,0,1,0]),
        distortion_coefficients=dict(data=[0]*5))


def test_no_calibration_is_explicit_and_wrong_resolution_fails(tmp_path):
    assert calibration('',640,480) is None
    p=tmp_path/'camera.yaml';p.write_text(yaml.safe_dump(record()))
    assert calibration(p,640,480)['k'][0]==500
    with pytest.raises(ValueError):calibration(p,1280,720)


@pytest.mark.parametrize('field,value',[('camera_matrix',[float('nan')]*9),
    ('camera_matrix',[0]*9),('rectification_matrix',[0]*9)])
def test_invalid_intrinsics_or_rotation_cannot_be_loaded(tmp_path,field,value):
    data=record();data[field]['data']=value;p=tmp_path/'camera.yaml';p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):calibration(p,640,480)
