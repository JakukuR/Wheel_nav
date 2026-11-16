"""Image layout and ROS calibration validation, independent of ROS and devices."""
from pathlib import Path
import cv2
import numpy as np
import yaml


ROTATIONS={'none':None,'cw90':cv2.ROTATE_90_CLOCKWISE,
           '180':cv2.ROTATE_180,'ccw90':cv2.ROTATE_90_COUNTERCLOCKWISE}


def output_size(width,height,rotation):
    if rotation not in ROTATIONS:raise ValueError('invalid_eye_rotation')
    return (height,width) if rotation in ('cw90','ccw90') else (width,height)


def split_frame(frame, layout, *, swap_eyes=False,left_rotation='none',right_rotation='none'):
    image=np.asarray(frame)
    if image.ndim!=3 or image.shape[2]!=3 or image.dtype!=np.uint8:
        raise ValueError('camera_requires_uint8_bgr')
    for rotation in (left_rotation,right_rotation):
        if rotation not in ROTATIONS:raise ValueError('invalid_eye_rotation')
    if layout=='mono':
        if swap_eyes or right_rotation!='none':raise ValueError('mono_has_no_right_eye')
        images=(image,)
    else:
        if layout!='side_by_side' or image.shape[1]%2:
            raise ValueError('stereo_requires_even_side_by_side_width')
        mid=image.shape[1]//2;images=(image[:,:mid],image[:,mid:])
        if swap_eyes:images=images[::-1]
    # Rotation is applied AFTER physical-eye assignment. cv2.rotate is a lossless
    # pixel permutation, not a mirror, interpolation, crop or browser transform.
    return tuple(np.ascontiguousarray(part) if ROTATIONS[rotation] is None
                 else cv2.rotate(part,ROTATIONS[rotation])
                 for part,rotation in zip(images,(left_rotation,right_rotation)))


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
