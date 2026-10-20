"""Real ORB and real BoT-SORT, synthetic images/detections; no robot I/O."""
from pathlib import Path
from types import SimpleNamespace as NS
import copy
import cv2
import numpy as np
import pytest
import torch
from ultralytics.engine.results import Results
from ultralytics.trackers.bot_sort import BOTSORT
from ultralytics.trackers.basetrack import BaseTrack
from ultralytics.trackers.track import on_predict_postprocess_end
from wla_r680_navigation_py.gmc_recovery import (
    PreparedORB, RecoverableTrackingRuntime, AssociationIds)


def image():
    return np.random.default_rng(42).integers(0, 256, (480, 640, 3), dtype=np.uint8)


class Model:
    """Replace only detector inference, retain upstream tracking callbacks."""
    def __init__(self):
        self.callbacks=[];self.fail=False;self.offset=0

    def add_callback(self, name, callback):
        assert name=='on_predict_postprocess_end'
        self.callbacks.append(callback)

    def track(self, frame, **kwargs):
        if self.fail:raise ValueError('inference_fault')
        if not hasattr(self, 'predictor'):
            args=NS(track_high_thresh=.4,track_low_thresh=.1,new_track_thresh=.5,
                    track_buffer=5,match_thresh=.8,fuse_score=True,gmc_method='orb',
                    with_reid=False,proximity_thresh=.5,appearance_thresh=.8)
            self.predictor=NS(trackers=[BOTSORT(args)], args=NS(task='detect'),
                              dataset=NS(mode='image'),save_dir=Path('/tmp'),vid_path=[None])
        x=self.offset
        boxes=torch.tensor([[250+x,100,350+x,400,.9,0.]])
        self.predictor.results=[Results(frame,'frame.jpg',{0:'person'},boxes=boxes)]
        for callback in self.callbacks:callback(self.predictor)
        on_predict_postprocess_end(self.predictor,persist=True)
        return self.predictor.results


def ready():
    model=Model();runtime=RecoverableTrackingRuntime(model,cuda_graph=False)
    frame=image()
    assert runtime.process(frame,device='cpu') is None
    result=runtime.process(frame,device='cpu')
    assert result[0].boxes.id.tolist()==[1.]
    return model,runtime,frame


def test_bad_background_continues_real_tracker_and_recovers():
    model,runtime,frame=ready();tracker=model.predictor.trackers[0]
    frame_id=tracker.frame_id;signature=runtime.signature
    for index in range(4):
        result=runtime.process(np.zeros_like(frame),device='cpu')
        assert result[0].boxes.id.tolist()==[1.]
        assert tracker.frame_id==frame_id+index+1
        assert runtime.signature==signature
        assert runtime.last_metrics['association_mode']=='uncompensated'
        assert runtime.last_metrics['gmc'][0]['fallback']=='identity_transform'
    # First clear frame re-establishes background features; second matches.
    for _ in range(2):result=runtime.process(frame,device='cpu')
    assert result[0].boxes.id.tolist()==[1.]
    assert runtime.last_metrics['association_mode']=='visual'


def test_blank_initial_frame_is_allowed_without_claiming_valid_gmc():
    gmc=PreparedORB();blank=np.zeros((480,640,3),np.uint8)
    gmc.prepare(blank,[])
    assert not gmc.diagnostics['valid']
    np.testing.assert_array_equal(gmc.apply(blank,[]),np.eye(2,3))
    for _ in range(2):gmc.prepare(image(),[]);gmc.apply(image(),[])
    assert gmc.diagnostics['valid']


def test_public_id_retired_on_degradation_but_stable_during_flapping():
    ids=AssociationIds();ids.begin_frame('visual');old=ids.resolve(7)
    ids.begin_frame('uncompensated');candidate=ids.resolve(7)
    assert candidate!=old
    revision=ids.revision
    for mode in ('visual','uncompensated')*12:
        ids.begin_frame(mode);assert ids.resolve(7)==candidate
    assert ids.revision==revision and ids.mode=='uncompensated'
    for _ in range(10):ids.begin_frame('visual')
    assert ids.mode=='visual' and ids.resolve(7)==candidate
    ids.begin_frame('uncompensated')
    assert ids.resolve(7) not in (old,candidate)
    assert ids.resolve(8)!=ids.resolve(7)


def test_unverified_fallback_candidate_needs_reid_confirmation():
    from wla_person_follow.reid_identity import TargetIdentity
    f=np.zeros(512);f[0]=1.
    def row(ident,feature=f):return dict(id=ident,feature=feature,position=np.array([2.,0.]))
    identity=TargetIdentity('s:1:1',model_id='test')
    identity.consume([row('s:1:1')],1.,'test');identity.consume([row('s:1:1')],1.2,'test')
    wrong=np.zeros(512);wrong[1]=1.
    identity.consume([row('s:1:2',wrong)],1.3,'test')
    assert identity.resolved_id=='s:1:1'
    for t in (1.4,1.5):
        identity.consume([row('s:1:2')],t,'test');assert identity.resolved_id=='s:1:1'
    identity.consume([row('s:1:2')],1.6,'test')
    assert identity.resolved_id=='s:1:2' and identity.locked_id=='s:1:1'


def test_good_preflight_matches_original_and_is_consumed_once():
    from wla_sim.perception.yoloe_runtime import BoundedORB
    plain=BoundedORB();prepared=PreparedORB();frame=image()
    for dx in (0,4,8):
        shifted=cv2.warpAffine(frame,np.float32([[1,0,dx],[0,1,0]]),(640,480))
        expected=plain.apply(shifted,[])
        prepared.prepare(shifted,[])
        np.testing.assert_allclose(prepared.apply(shifted,[]),expected,atol=1e-6)
        assert prepared._prepared is None


def test_real_inference_fault_still_resets_tracks():
    model,runtime,frame=ready();model.fail=True
    with pytest.raises(ValueError,match='inference_fault'):runtime.process(frame,device='cpu')
    assert not model.predictor.trackers[0].tracked_stracks
    assert runtime.signature is None


def test_unexpected_opencv_exception_is_not_quality_degradation(monkeypatch):
    model,runtime,frame=ready()
    def broken(*args):raise RuntimeError('opencv_fault')
    monkeypatch.setattr(model.predictor.trackers[0].gmc,'prepare',broken)
    with pytest.raises(RuntimeError,match='opencv_fault'):runtime.process(frame,device='cpu')
    assert runtime.signature is None
    assert not model.predictor.trackers[0].tracked_stracks
