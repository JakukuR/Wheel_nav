import numpy as np
from wla_person_follow.identity_supervisor import IdentitySupervisor

def frame():return dict(id='s:1:1',position=np.array([2.,0.]),feature=np.eye(1,512).ravel())
def test_late_enrollment_on_continuous_binding_allowed():
    s=IdentitySupervisor('s:1:1',model_id='test',start_time=0.)
    for t in (40.,40.2):s.observe([frame()],t,{},geometry_state='bound_untracked')
    assert s.complete

def test_ambiguity_invalidates_enrollment_pair():
    s=IdentitySupervisor('s:1:1',model_id='test',start_time=0.)
    s.observe([frame()],1.,{},geometry_state='bound_untracked')
    s.observe([frame()],1.1,{},geometry_state='suspect',neighbor_ambiguity=True)
    assert not s.complete
    s.observe([frame()],1.2,{},geometry_state='bound_untracked')
    assert not s.complete
    s.observe([frame()],1.3,{},geometry_state='bound_untracked')
    assert s.complete

def test_different_binding_generation_cannot_complete_pair():
    s=IdentitySupervisor('s:1:1',model_id='test',start_time=0.)
    for t,breaks in ((1.,0),(1.1,1)):
        s.observe([frame()],t,{},geometry_state='bound_untracked',binding_snapshot=dict(locked_id='s:1:1',breaks=breaks))
    assert not s.complete
