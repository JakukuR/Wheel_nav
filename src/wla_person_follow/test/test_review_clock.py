import numpy as np
from wla_person_follow.identity_supervisor import IdentitySupervisor

def enrolled():
    s=IdentitySupervisor('s:1:1',model_id='test',start_time=0.)
    row=dict(id='s:1:1',position=np.array([2.,0.]),feature=np.eye(1,512).ravel())
    for t in (1.,1.2):s.observe([row],t,{},geometry_state='bound_tracking')
    return s,row

def test_timer_does_not_discard_delayed_feature():
    s,row=enrolled();snap=dict(state='lost',lost_since=2.,lost_s=1.,prediction=[2.,0.])
    s.advance_time(3.,geometry_state='lost',binding_snapshot=snap)
    assert s.last_frame_stamp==1.2
    s.observe([dict(row,id='s:1:2')],2.95,{},geometry_state='lost',binding_snapshot=snap)
    assert s.pending_count==1 and s.last_frame_stamp==2.95
    s.advance_time(3.1,geometry_state='lost',binding_snapshot=snap)
    assert s.pending_count==1 and s.last_frame_stamp==2.95
