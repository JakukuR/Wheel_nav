"""Export a measured Kalibr left/right/RGB chain; never estimates calibration."""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import cv2
import numpy as np
import yaml
from .stereo import load_rig,StereoRig

TOPICS=['/camera/sunplus/left/image_raw','/camera/sunplus/right/image_raw','/camera/dicota/color/image_raw']


def export_chain(source,output):
    source=Path(source);output=Path(output);data=yaml.safe_load(source.read_text())
    if sorted(data)!=['cam0','cam1','cam2']:raise ValueError('requires_exactly_three_cameras_in_left_right_rgb_order')
    cameras={}
    for index,name in enumerate(('left','right','rgb')):
        c=data['cam'+str(index)]
        if c.get('rostopic')!=TOPICS[index]:raise ValueError('camera_topic_order_mismatch_'+name)
        if c.get('camera_model')!='pinhole' or c.get('distortion_model')!='radtan':
            raise ValueError('only_kalibr_pinhole_radtan_supported')
        intrinsics=np.asarray(c['intrinsics'],float);d=list(c['distortion_coeffs'])
        if intrinsics.shape!=(4,) or len(d)!=4:raise ValueError('invalid_kalibr_intrinsics_or_distortion')
        fx,fy,cx,cy=intrinsics
        cameras[name]=dict(width=c['resolution'][0],height=c['resolution'][1],model='pinhole-radtan',
            k=[[float(fx),0.,float(cx)],[0.,float(fy),float(cy)],[0.,0.,1.]],d=[float(v) for v in d]+[0.])
    right=np.asarray(data['cam1']['T_cn_cnm1'],float)
    rgb_right=np.asarray(data['cam2']['T_cn_cnm1'],float)
    # Validate each chain link before multiplication; invalid links must not
    # cancel into an apparently valid product.
    from .stereo import rigid
    right=rigid(right);rgb_right=rigid(rgb_right)
    rig=dict(schema='wla-uvc-rig/v1',units='m',cameras=cameras,
        T_right_left=right.tolist(),T_rgb_left=(rgb_right@right).tolist())
    with tempfile.TemporaryDirectory() as folder:
        temporary=Path(folder)/'rig.yaml';temporary.write_text(yaml.safe_dump(rig,sort_keys=False))
        cams,_,_=load_rig(temporary)
        StereoRig(temporary)  # Also check horizontal eye order / disparity sign.
    kl,dl=cams['left'];kr,dr=cams['right'];kc,dc=cams['rgb']
    r1,r2,p1,p2,_,_,_=cv2.stereoRectify(kl,dl,kr,dr,(1280,720),right[:3,:3],right[:3,3],
        flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
    files={}
    for name,frame,r,p in [('left','sunplus_left',r1,p1),('right','sunplus_right',r2,p2),
            ('rgb','dicota',np.eye(3),np.column_stack([kc,np.zeros(3)]))]:
        k,d=cams[name];c=cameras[name]
        files[frame+'.yaml']=dict(image_width=c['width'],image_height=c['height'],camera_name=frame,
            distortion_model='plumb_bob',camera_matrix=dict(rows=3,cols=3,data=k.reshape(-1).tolist()),
            distortion_coefficients=dict(rows=1,cols=5,data=d.tolist()),
            rectification_matrix=dict(rows=3,cols=3,data=r.reshape(-1).tolist()),
            projection_matrix=dict(rows=3,cols=4,data=p.reshape(-1).tolist()))
    # Existing calibration is never overwritten by a converter invocation.
    output.mkdir(parents=True,exist_ok=False)
    (output/'rig.yaml').write_text(yaml.safe_dump(rig,sort_keys=False))
    for name,values in files.items():(output/name).write_text(yaml.safe_dump(values,sort_keys=False))
    stereo_chain=yaml.safe_load(yaml.safe_dump({k:data[k] for k in ('cam0','cam1')}))
    for c in stereo_chain.values():
        if 'cam_overlaps' in c:c['cam_overlaps']=[v for v in c['cam_overlaps'] if v in (0,1)]
    (output/'stereo-camchain.yaml').write_text(yaml.safe_dump(stereo_chain,sort_keys=False))
    provenance=dict(schema='wla-kalibr-export/v1',source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        topics=TOPICS,baseline_m=float(np.linalg.norm(right[:3,3])),
        T_rgb_left='cam2.T_cn_cnm1 @ cam1.T_cn_cnm1',
        distortion='Kalibr [k1,k2,p1,p2] -> OpenCV [k1,k2,p1,p2,0]; k3 fixed at zero in original model',
        independent_hardware_acceptance=False)
    (output/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    return rig


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('camchain');parser.add_argument('--output',required=True)
    args=parser.parse_args();export_chain(args.camchain,args.output)
    print(str(Path(args.output)/'rig.yaml'))


if __name__=='__main__':main()
