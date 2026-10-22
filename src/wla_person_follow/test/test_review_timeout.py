import pytest
from test_review_clock import enrolled
from wla_person_follow.identity_supervisor import IdentitySupervisor

@pytest.mark.parametrize('complete',[True,False])
def test_continuous_neighbor_expires_at_twelve_seconds(complete):
    s,_=enrolled() if complete else (IdentitySupervisor('s:1:1',model_id='test',start_time=0.),None)
    for t in (2.,5.,13.99):
        s.observe([],t,{},geometry_state='bound_tracking',neighbor_ambiguity=True)
        assert s.end_reason is None
    s.advance_time(14.,geometry_state='suspect')
    assert s.state=='ended' and s.end_reason=='suspect_timeout_'+('after' if complete else 'before')+'_enrollment'
    s.observe([],30.,{},geometry_state='bound_tracking',neighbor_ambiguity=True)
    assert s.state=='ended'
