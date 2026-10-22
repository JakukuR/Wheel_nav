import numpy as np
from test_review_clock import enrolled

def test_coasting_cannot_expand_view():
    s,row=enrolled();row['feature']=.75*row['feature']+np.sqrt(1-.75**2)*np.eye(1,512,1).ravel()
    s.observe([row],3.,{},geometry_state='bound_coasting')
    assert len(s.identity.gallery)==2

def test_expansion_requires_two_seconds_clear_geometry_and_own_radius():
    s,row=enrolled();row['feature']=.75*row['feature']+np.sqrt(1-.75**2)*np.eye(1,512,1).ravel()
    for t in (2.,3.9):
        s.observe([row],t,{},geometry_state='bound_tracking',expansion_neighbor=False)
        assert len(s.identity.gallery)==2
    s.observe([row],4.,{},geometry_state='bound_tracking',expansion_neighbor=True)
    assert len(s.identity.gallery)==2
    for t in (4.1,6.09):
        s.observe([row],t,{},geometry_state='bound_tracking',expansion_neighbor=False)
        assert len(s.identity.gallery)==2
    s.observe([row],6.11,{},geometry_state='bound_tracking',expansion_neighbor=False)
    assert len(s.identity.gallery)==3
