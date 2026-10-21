"""Real BoxMOT association with synthetic detections; no camera or motion."""
from types import SimpleNamespace as NS
import numpy as np
import pytest
import torch
from wla_r680_navigation_py.boxmot_runtime import TrackingRuntime

class Encoder:
    model_id='fixture'
    last_ms=0.
    def warm(self):self.last_features=np.empty((0,512),np.float32)
    def get_features(self,boxes,image):
        self.last_features=np.full((len(boxes),512),1/np.sqrt(512),np.float32)
        return self.last_features

class Detector:
    def __init__(self):
        self.rows=[[10,10,50,90,.9,0],[55,10,95,90,.9,1]]
        self.predictor=NS(model=NS(device=NS(type='cpu')))
    def predict(self,frame,**opts):
        assert opts['retina_masks']
        return [NS(boxes=NS(data=torch.tensor(self.rows,dtype=torch.float32).reshape(-1,6)),names={0:'person',1:'chair'})]
    def track(self,*a,**kw):raise AssertionError('Ultralytics tracker must never run')


def test_real_boxmot_id_stability_no_predictions_disguised_as_detections():
    m=Detector();r=TrackingRuntime(m,reid_backend=Encoder());frame=np.zeros((100,100,3),np.uint8)
    assert r.process(frame,device='cpu',imgsz=416) is None
    first=r.process(frame,device='cpu',imgsz=416)[1]
    assert len(first)==2 and len({v['track_id'] for v in first})==2
    for _ in range(4):
        rows=r.process(frame,device='cpu',imgsz=416)[1]
        assert [v['track_id'] for v in rows]==[v['track_id'] for v in first]
        for row in rows:assert row['bbox_xyxy']==m.rows[row['detection_index']][:4]
    m.rows=[]
    assert r.process(frame,device='cpu',imgsz=416)[1]==[]
    assert r.tracker.cmc is not None and r.tracker.use_embeddings
    assert not r.object_tracker.use_embeddings
    assert not hasattr(m.predictor,'trackers')


def test_shape_change_requires_new_runtime():
    r=TrackingRuntime(Detector(),reid_backend=Encoder());r.process(np.zeros((100,100,3),np.uint8),device='cpu')
    with pytest.raises(ValueError):r.process(np.zeros((101,100,3),np.uint8),device='cpu')
