"""Image layout and ROS calibration validation, independent of ROS and devices."""
from pathlib import Path
import numpy as np
import yaml


def split_frame(frame, layout):
    image=np.asarray(frame)
    if image.ndim!=3 or image.shape[2]!=3 or image.dtype!=np.uint8:
        raise ValueError('camera_requires_uint8_bgr')
    if layout=='mono':return (np.ascontiguousarray(image),)
    if layout!='side_by_side' or image.shape[1]%2:
        raise ValueError('stereo_requires_even_side_by_side_width')
    mid=image.shape[1]//2
    return (np.ascontiguousarray(image[:,:mid]),np.ascontiguousarray(image[:,mid:]))


def calibration(path, width, height):
    if not path:return None
    data=yaml.safe_load(Path(path).expanduser().read_text())
    if (data['image_width'],data['image_height'])!=(width,height):
        raise ValueError('calibration_dimensions_must_match_capture_eye')
    out={}
    for key,size,target in [('camera_matrix',9,'k'),('rectification_matrix',9,'r'),
                            ('projection_matrix',12,'p')]:
        values=np.asarray(data[key]['data'],dtype=float)
        if values.shape!=(size,) or not np.isfinite(values).all():
            raise ValueError('invalid_calibration_'+key)
        out[target]=values.tolist()
    coeff=np.asarray(data['distortion_coefficients']['data'],dtype=float)
    if coeff.ndim!=1 or not np.isfinite(coeff).all():raise ValueError('invalid_distortion')
    model=data['distortion_model']
    if model not in ('plumb_bob','rational_polynomial','equidistant'):
        raise ValueError('unsupported_distortion_model')
    expected={'plumb_bob':5,'rational_polynomial':8,'equidistant':4}[model]
    if len(coeff)!=expected or out['k'][0]<=0 or out['k'][4]<=0 or out['p'][0]<=0 or out['p'][5]<=0:
        raise ValueError('invalid_calibration_intrinsics')
    if not np.allclose(out['k'][6:],[0,0,1]):raise ValueError('invalid_calibration_last_row')
    rotation=np.asarray(out['r']).reshape(3,3)
    if not np.allclose(rotation@rotation.T,np.eye(3),atol=1e-4) or not np.isclose(np.linalg.det(rotation),1.,atol=1e-4):
        raise ValueError('invalid_rectification_rotation')
    return dict(out,d=coeff.tolist(),distortion_model=model)
