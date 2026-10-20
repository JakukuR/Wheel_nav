"""Lightweight OSNet descriptor extraction; RGB crops, no control/identity logic."""
from pathlib import Path
import hashlib
import cv2
import numpy as np
import torch
from .vendor.osnet import osnet_x0_25


class AppearanceEncoder:
    def __init__(self,path):
        torch.set_num_threads(1)
        self.model_id='osnet-x025-msmt17:'+hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16]
        weights=torch.load(path,map_location='cpu',weights_only=True)
        weights=weights.get('state_dict',weights)
        weights={k.removeprefix('module.'):v for k,v in weights.items() if not k.removeprefix('module.').startswith('classifier.')}
        self.model=osnet_x0_25(num_classes=1,pretrained=False).eval()
        loaded=self.model.load_state_dict(weights,strict=False)
        if set(loaded.missing_keys)!={'classifier.weight','classifier.bias'} or loaded.unexpected_keys:
            raise ValueError('unexpected_reid_weights')
        # Freeze eval-only batch normalization into convolutions on the CPU.
        # Same FP32 weights/input/descriptor space; no GPU contention with YOLO.
        with torch.inference_mode():
            self.model=torch.jit.freeze(torch.jit.trace(self.model,torch.zeros(1,3,256,128)))
        self.mean=np.array([.485,.456,.406],np.float32)[None,None,:]
        self.std=np.array([.229,.224,.225],np.float32)[None,None,:]
        self.encode([np.zeros((256,128,3),np.uint8)])

    def encode(self,crops):
        if not crops:return np.empty((0,512),np.float32)
        if len(crops)>4:raise ValueError('reid_batch_too_large')
        x=np.stack([((cv2.resize(c,(128,256)).astype(np.float32)/255.-self.mean)/self.std).transpose(2,0,1) for c in crops])
        with torch.inference_mode():result=self.model(torch.from_numpy(x)).numpy()
        result/=np.maximum(np.linalg.norm(result,axis=1,keepdims=True),1e-8)
        if result.shape!=(len(crops),512) or not np.isfinite(result).all():raise ValueError('invalid_reid_features')
        return result


def person_crops(rgb,objects,limit=4,allowed_ids=None,prefix='',diagnostics=None):
    """Reject bad crops; report only requested IDs without encoding bystanders."""
    height,width=rgb.shape[:2];valid=[]
    if diagnostics is not None and allowed_ids is not None:
        diagnostics.update({ident:'target_not_detected' for ident in allowed_ids})
    def report(row,reason):
        if diagnostics is None or not isinstance(row,dict):return
        ident=prefix+str(row.get('track_id',''))
        if allowed_ids is None or ident in allowed_ids:diagnostics[ident]=reason
    for row in objects:
        try:
            if row['label']!='person':report(row,'not_person');continue
            if row['confidence']<.5:report(row,'low_confidence');continue
            b=np.asarray(row['bbox_xyxy'],dtype=float)
            if b.shape!=(4,) or not np.isfinite(b).all():report(row,'invalid_bbox');continue
            x1,y1,x2,y2=np.rint(b).astype(int)
            if x1<0 or y1<0 or x2>width or y2>height:report(row,'bbox_out_of_image');continue
            if x2-x1<32 or y2-y1<64:report(row,'crop_too_small');continue
            edges=sum([x1<=1,y1<=1,x2>=width-1,y2>=height-1])
            if edges>=2:report(row,'crop_clipped');continue
            if (x2-x1)*(y2-y1)>width*height*.7:report(row,'crop_too_large');continue
            valid.append((row,(x1,y1,x2,y2)))
        except (ValueError,TypeError,KeyError):report(row,'invalid_detection');continue
    selected=[]
    for i,(row,b) in enumerate(valid):
        area=(b[2]-b[0])*(b[3]-b[1]);overlap=False
        for j,(_,o) in enumerate(valid):
            if i==j:continue
            inter=max(0,min(b[2],o[2])-max(b[0],o[0]))*max(0,min(b[3],o[3])-max(b[1],o[1]))
            if inter/max(1,min(area,(o[2]-o[0])*(o[3]-o[1])))>.25:overlap=True;break
        if overlap:report(row,'crop_overlap');continue
        if allowed_ids is None or prefix+str(row['track_id']) in allowed_ids:
            selected.append((row,rgb[b[1]:b[3],b[0]:b[2]].copy()))
    selected.sort(key=lambda r:r[0]['confidence']*r[1].shape[0]*r[1].shape[1],reverse=True)
    for index,(row,_) in enumerate(selected):report(row,'selected' if index<limit else 'crop_batch_limit')
    return selected[:limit]
