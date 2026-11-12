"""Calibrated horizontal stereo and projection to undistorted cropped RGB.

All transforms map source coordinates to destination coordinates, in metres.
No default intrinsics, baseline or extrinsics are supplied.
"""
from pathlib import Path
import cv2
import numpy as np
import yaml


def crop_matrix(width=1920, height=1080, output=(640,480)):
    # Largest central ROI with the output aspect ratio, without stretching.
    ow,oh=output
    if min(width,height,ow,oh)<=0:raise ValueError('invalid_crop_size')
    rw=min(width,int(height*ow/oh));rh=min(height,int(width*oh/ow))
    x=(width-rw)//2;y=(height-rh)//2
    sx=ow/rw;sy=oh/rh
    a=np.array([[sx,0,sx*(.5-x)-.5],[0,sy,sy*(.5-y)-.5],[0,0,1.]])
    return (x,y,rw,rh),a


def raw_crop(image, output=(640,480)):
    (x,y,w,h),_=crop_matrix(image.shape[1],image.shape[0],output)
    return cv2.resize(image[y:y+h,x:x+w],output,interpolation=cv2.INTER_AREA)


def rigid(value):
    t=np.asarray(value,dtype=float)
    if (t.shape!=(4,4) or not np.isfinite(t).all()
            or not np.allclose(t[3],[0,0,0,1])
            or not np.allclose(t[:3,:3]@t[:3,:3].T,np.eye(3),atol=1e-5)
            or not np.isclose(np.linalg.det(t[:3,:3]),1,atol=1e-5)):
        raise ValueError('invalid_rigid_transform')
    return t


def load_rig(path):
    data=yaml.safe_load(Path(path).read_text())
    if data.get('schema')!='wla-uvc-rig/v1' or data.get('units')!='m':
        raise ValueError('rig_schema_or_units')
    cams={}
    for name,size in [('left',(1280,720)),('right',(1280,720)),('rgb',(1920,1080))]:
        c=data['cameras'][name];k=np.asarray(c['k'],float);d=np.asarray(c['d'],float)
        if ((c['width'],c['height'])!=size or k.shape!=(3,3)
                or not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0
                or not np.allclose(k[2],[0,0,1]) or not np.allclose(k[[0,1],[1,0]],0)
                or d.shape not in ((5,),(8,)) or not np.isfinite(d).all()
                or c.get('model')!='pinhole-radtan'):
            raise ValueError('invalid_camera_'+name)
        cams[name]=(k,d)
    right=rigid(data['T_right_left']);rgb=rigid(data['T_rgb_left'])
    if np.linalg.norm(right[:3,3])<1e-4:raise ValueError('missing_stereo_baseline')
    return cams,right,rgb


def project_depth(points, k, transform, size, near=.15, far=5.):
    """Nearest optical Z at each RGB pixel, zero means no measurement."""
    ow,oh=size
    points=np.asarray(points).reshape(-1,3)
    points=points[np.isfinite(points).all(axis=1)]
    points=points@transform[:3,:3].T+transform[:3,3]
    z=points[:,2]
    valid=(z>=near)&(z<=far);points=points[valid];z=z[valid]
    uv=points@k.T
    uv=np.rint(uv[:,:2]/uv[:,2:]).astype(np.int64)
    good=(uv[:,0]>=0)&(uv[:,0]<ow)&(uv[:,1]>=0)&(uv[:,1]<oh)
    depth=np.full(ow*oh,np.inf,dtype=np.float32)
    np.minimum.at(depth,uv[good,1]*ow+uv[good,0],z[good])
    depth[~np.isfinite(depth)]=0
    return np.rint(depth.reshape(oh,ow)*1000).astype('<u2')


class StereoRig:
    def __init__(self,path,algorithm='sgbm',disparities=128,block=5,rgb_mode='crop'):
        if algorithm not in ('sgbm','bm') or disparities<16 or disparities%16:
            raise ValueError('invalid_matcher')
        if block<5 or block%2==0 or block>51:raise ValueError('invalid_block_size')
        cams,right,self.rgb_from_left=load_rig(path)
        kl,dl=cams['left'];kr,dr=cams['right'];kc,dc=cams['rgb']
        if rgb_mode not in ('full','crop'):raise ValueError('invalid_rgb_mode')
        self.size=(640,360);self.output=(640,360) if rgb_mode=='full' else (640,480)
        r1,r2,p1,p2,self.q,_,_=cv2.stereoRectify(kl,dl,kr,dr,(1280,720),
            right[:3,:3],right[:3,3],flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0,newImageSize=self.size)
        if p2[0,3]>=0 or abs(p2[1,3])>1e-5:
            raise ValueError('requires_horizontal_left_to_right_positive_disparity_check_eye_order')
        self.r1=r1
        self.left_maps=cv2.initUndistortRectifyMap(kl,dl,r1,p1,self.size,cv2.CV_32FC1)
        self.right_maps=cv2.initUndistortRectifyMap(kr,dr,r2,p2,self.size,cv2.CV_32FC1)
        self.roi,a=crop_matrix(output=self.output);self.k=a@kc
        self.rgb_maps=cv2.initUndistortRectifyMap(kc,dc,np.eye(3),self.k,self.output,cv2.CV_32FC1)
        if algorithm=='sgbm':
            self.matcher=cv2.StereoSGBM_create(minDisparity=0,numDisparities=disparities,
                blockSize=block,P1=8*block*block,P2=32*block*block,
                disp12MaxDiff=1,uniquenessRatio=12,speckleWindowSize=100,
                speckleRange=2,mode=cv2.STEREO_SGBM_MODE_SGBM)
        else:
            self.matcher=cv2.StereoBM_create(numDisparities=disparities,blockSize=block)
            self.matcher.setUniquenessRatio(12);self.matcher.setSpeckleWindowSize(100)
            self.matcher.setSpeckleRange(2);self.matcher.setDisp12MaxDiff(1)

    def color(self,rgb):
        return cv2.remap(rgb,*self.rgb_maps,cv2.INTER_LINEAR)

    def depth(self,left,right):
        if left.shape[:2]!=(720,1280) or right.shape[:2]!=(720,1280):
            raise ValueError('raw_stereo_size_changed_recalibration_required')
        l=cv2.remap(left,*self.left_maps,cv2.INTER_LINEAR)
        r=cv2.remap(right,*self.right_maps,cv2.INTER_LINEAR)
        if l.ndim==3:l=cv2.cvtColor(l,cv2.COLOR_BGR2GRAY)
        if r.ndim==3:r=cv2.cvtColor(r,cv2.COLOR_BGR2GRAY)
        disparity=self.matcher.compute(l,r).astype(np.float32)/16.
        points=cv2.reprojectImageTo3D(disparity,self.q)
        good=(disparity>0)&np.isfinite(points).all(axis=2)&(points[:,:,2]>.0)
        # reprojectImageTo3D returns rectified left coordinates. Invert R1
        # before applying the measured raw-left -> raw-RGB transform.
        points=points[good]@self.r1
        return project_depth(points,self.k,self.rgb_from_left,self.output),disparity
