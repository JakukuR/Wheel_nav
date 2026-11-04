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


@pytest.fixture(params=['cpp','python'])
def backend(request,monkeypatch):
    monkeypatch.setenv('WLA_TRACKER_BACKEND',request.param)
    return request.param


def test_real_boxmot_id_stability_no_predictions_disguised_as_detections(backend):
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
    assert r.tracker_backend==backend and r.tracker.use_embeddings and r.use_cmc
    if backend=='python':assert r.tracker.cmc is not None and not r.object_tracker.use_embeddings
    assert not hasattr(m.predictor,'trackers')


def test_objects_are_tracked_per_class_with_unique_ids_and_frame_indices(backend):
    m=Detector();r=TrackingRuntime(m,reid_backend=Encoder());frame=np.zeros((100,100,3),np.uint8)
    # A chair and a table on the same pixels must not associate with each other.
    m.rows=[[5,5,20,20,.9,1],[55,10,95,90,.9,1],[55,10,95,90,.9,2],[10,10,50,90,.9,0]]
    names={0:'person',1:'chair',2:'table'}
    predict=m.predict
    def named(image,**options):
        output=predict(image,**options);output[0].names=names;return output
    m.predict=named
    r.process(frame,device='cpu')
    for _ in range(4):rows=r.process(frame,device='cpu')[1]
    assert sorted(v['detection_index'] for v in rows)==[0,1,2,3]
    assert len({v['track_id'] for v in rows})==4
    for row in rows:
        source=m.rows[row['detection_index']]
        assert row['bbox_xyxy']==source[:4] and row['label']==names[source[5]]
    objects=[v['track_id'] for v in rows if v['label']!='person']
    assert all(v%2 for v in objects) and all(v['track_id']%2==0 for v in rows if v['label']=='person')
    before={v['detection_index']:v['track_id'] for v in rows}
    m.rows=m.rows[:3]
    for row in r.process(frame,device='cpu')[1]:assert before[row['detection_index']]==row['track_id']
    r.reset()
    assert len(r.process(frame,device='cpu')[1])<=3


def test_person_features_follow_the_detection_index(backend):
    class Distinct(Encoder):
        def get_features(self,boxes,image):
            self.last_features=np.zeros((len(boxes),512),np.float32)
            self.last_features[np.arange(len(boxes)),(boxes[:,0]/5).astype(int)]=1
            return self.last_features
    m=Detector();m.rows=[[50,10,90,90,.9,0],[5,10,45,90,.9,0]]
    r=TrackingRuntime(m,reid_backend=Distinct());frame=np.zeros((100,100,3),np.uint8)
    r.process(frame,device='cpu')
    for _ in range(4):rows=r.process(frame,device='cpu')[1]
    assert len(rows)==2
    for row in rows:
        assert int(np.argmax(r.features[row['track_id']]))==int(m.rows[row['detection_index']][0]/5)


def test_backend_falls_back_to_python_when_native_cannot_load(monkeypatch,capsys):
    import boxmot.trackers.box.botsort.native as native
    def missing(*args,**kwargs):raise OSError('no library')
    monkeypatch.setenv('WLA_TRACKER_BACKEND','cpp');monkeypatch.setattr(native,'NativeBotSortTracker',missing)
    r=TrackingRuntime(Detector(),reid_backend=Encoder())
    assert r.tracker_backend=='python' and 'no library' in capsys.readouterr().err


def test_shape_change_requires_new_runtime():
    r=TrackingRuntime(Detector(),reid_backend=Encoder());r.process(np.zeros((100,100,3),np.uint8),device='cpu')
    with pytest.raises(ValueError):r.process(np.zeros((101,100,3),np.uint8),device='cpu')
