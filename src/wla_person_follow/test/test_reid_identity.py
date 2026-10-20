import numpy as np
import pytest
from wla_person_follow.reid_identity import TargetIdentity


def feature(index=0):
    x=np.zeros(512);x[index]=1.;return x


def row(ident='s:1:1',p=(2.,0.),f=None):
    return dict(id=ident,position=np.array(p),feature=feature() if f is None else f)


def seeded():
    i=TargetIdentity('s:1:1',model_id='test')
    i.consume([row()],1.,'test');i.consume([row()],1.2,'test')
    assert i.state=='tracking';return i


def current(ident='s:1:1',yaw=0.,distance=1.72):
    return {ident:dict(detected=True,label='person',yaw=yaw,distance=distance)}


def test_same_person_new_id_requires_three_distinct_frames():
    i=seeded()
    for t in [2.,2.2]:i.consume([row('s:1:2')],t,'test');assert i.resolved_id=='s:1:1'
    i.consume([row('s:1:2')],2.2,'test');assert i.pending_count==2
    i.consume([row('s:1:2')],2.4,'test')
    assert i.resolved_id=='s:1:2' and i.reacquired==1 and i.locked_id=='s:1:1'
    assert i.select(current('s:1:2'),np.zeros(3),2.4,2.45)=='s:1:2'


@pytest.mark.parametrize('kind',['wrong_appearance','far','late','model','nan'])
def test_wrong_or_untrusted_candidates_never_rebind(kind):
    i=seeded();p=(4.,0.) if kind=='far' else (2.,0.)
    f=feature(1) if kind=='wrong_appearance' else feature()
    if kind=='nan':f[0]=np.nan
    for t in [2.,2.2,2.4]:i.consume([row('s:1:2',p,f)],t+(10 if kind=='late' else 0),'other' if kind=='model' else 'test')
    assert i.resolved_id=='s:1:1' and i.reacquired==0


def test_ambiguous_people_never_rebind_or_poison_gallery():
    i=seeded();before=list(i.gallery)
    for t in [2.,2.2,2.4]:i.consume([row('s:1:2'),row('s:1:3',(2.1,0))],t,'test')
    assert i.state=='ambiguous' and i.reacquired==0 and i.pending_count==0
    np.testing.assert_array_equal(i.gallery,before)


def test_confirmations_must_be_consecutive_and_fresh():
    i=seeded();i.consume([row('s:1:2')],2.,'test');i.consume([],2.2,'test')
    i.consume([row('s:1:2')],2.4,'test');assert i.pending_count==1
    i.consume([row('s:1:2')],3.2,'test');assert i.pending_count==1
    i.consume([row('s:1:3')],3.4,'test');assert i.pending_count==1


def test_current_geometry_and_recent_features_required():
    i=seeded()
    assert i.select(current(),np.zeros(3),1.2,1.25)=='s:1:1'
    assert i.select(current(),np.zeros(3),1.2,1.6) is None
    assert i.select(current(distance=5),np.zeros(3),1.3,1.35) is None
    assert i.select(current(),np.zeros(3),3.,3.) is None
    assert i.select(current(),np.zeros(3),10.,10.) is None and i.state=='expired'


def test_prediction_moves_with_measured_target_and_body_rotation():
    i=seeded();i.consume([row(p=(2.2,0))],1.4,'test')
    assert i.prediction(2.)[0]>2.2
    # Same world point: robot shifted +1 m and turned left 90 degrees.
    assert i.select(current(yaw=-np.pi/2,distance=.92),np.array([1.,0.,np.pi/2]),1.4,1.45)=='s:1:1'
    for t in [2.,2.2,2.4]:i.consume([row('s:1:2',p=i.prediction(t))],t,'test')
    assert i.resolved_id=='s:1:2'


def test_enrollment_cannot_adopt_a_different_person():
    i=TargetIdentity('s:1:1',model_id='test')
    i.consume([row('s:1:2')],1.,'test');assert not i.gallery
    i.consume([row()],1.2,'test')
    for t in [2.,2.2,2.4]:i.consume([row('s:1:2')],t,'test')
    assert i.seed_count==1 and i.reacquired==0


def test_feature_requests_only_target_then_spatial_candidates_on_loss():
    i=seeded();objects=current();objects.update(current('s:1:2',distance=1.8));objects.update(current('s:1:3',distance=8.))
    assert i.candidate_ids(objects,np.zeros(3),1.3,1.3)==['s:1:1']
    del objects['s:1:1']
    assert i.candidate_ids(objects,np.zeros(3),1.4,1.4)==['s:1:2']
    assert i.candidate_ids(objects,np.zeros(3),10.,10.)==[]
    i=TargetIdentity('s:1:1',model_id='test')
    assert i.candidate_ids(objects,np.zeros(3),1.,1.)==[]
