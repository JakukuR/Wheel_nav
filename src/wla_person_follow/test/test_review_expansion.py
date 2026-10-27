"""Current recovery-only gallery quality and ownership regressions.

The removed supervisor used a two-second expansion timer. RecoveryIdentity uses
current accepted geometry, no neighbor and similarity >= .90 instead.
"""
import numpy as np
from wla_person_follow.follow_session import FollowTracker, RecoveryIdentity


def feature(similarity=1.):
    value=np.zeros(512);value[0]=similarity;value[1]=np.sqrt(1-similarity**2)
    return value


def observe(identity,tracker,row,stamp,*,neighbor=False,state='tracking'):
    if state=='tracking':
        assert tracker.observe({'target':dict(detected=True,label='person',yaw=0.,distance=2.)},
                               np.zeros(3),stamp,stamp)
    snapshot=tracker.snapshot(stamp,np.zeros(3))
    if state!='tracking':assert snapshot['state']==state
    identity.observe([row],stamp,{},tracker,snapshot,neighbor=neighbor)


def enrolled():
    tracker=FollowTracker('target',0.);identity=RecoveryIdentity()
    row=dict(id='target',feature=feature(),position=np.array([2.28,0.]))
    for stamp in (1.,1.2):observe(identity,tracker,row,stamp)
    assert identity.complete and len(identity.gallery)==2
    return identity,tracker,row


def test_coasting_cannot_expand_view():
    identity,tracker,row=enrolled();row['feature']=feature(.95)
    observe(identity,tracker,row,1.6,state='coasting')
    assert len(identity.gallery)==2


def test_expansion_requires_high_similarity_and_unambiguous_current_geometry():
    identity,tracker,row=enrolled();seeds=[x.copy() for x in identity.gallery]
    row['feature']=feature(.75)
    for stamp in (1.4,1.6,1.8):observe(identity,tracker,row,stamp)
    assert len(identity.gallery)==2
    row['feature']=feature(.95)
    observe(identity,tracker,row,2.,neighbor=True)
    assert len(identity.gallery)==2
    for stamp in np.arange(2.2,4.2,.2):observe(identity,tracker,row,float(stamp))
    assert len(identity.gallery)==8
    for actual,expected in zip(identity.gallery[:2],seeds):np.testing.assert_array_equal(actual,expected)


def test_delayed_or_foreign_target_features_cannot_train_gallery():
    identity,tracker,row=enrolled();row['feature']=feature(.95);row['id']='other'
    observe(identity,tracker,row,1.4)
    assert len(identity.gallery)==2
    row['id']='target'
    assert tracker.observe({'target':dict(detected=True,label='person',yaw=0.,distance=2.)},np.zeros(3),2.,2.)
    identity.observe([row],1.5,{},tracker,tracker.snapshot(2.,np.zeros(3)))
    assert len(identity.gallery)==2


def test_enrollment_restarts_pair_after_feature_gap():
    tracker=FollowTracker('target',0.);identity=RecoveryIdentity()
    row=dict(id='target',feature=feature(),position=np.array([2.28,0.]))
    observe(identity,tracker,row,1.)
    observe(identity,tracker,row,1.9)
    assert not identity.complete and identity.provisional[1]==1.9
    observe(identity,tracker,row,2.1)
    assert identity.complete
