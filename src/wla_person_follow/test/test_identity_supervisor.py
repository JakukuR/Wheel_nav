import numpy as np
from wla_person_follow.identity_supervisor import IdentitySupervisor, IdentitySupervisorConfig


def feature(x=1.,y=0.):
    v=np.zeros(512);v[:2]=[x,y];return v/np.linalg.norm(v)


def rows(*items):
    return [dict(id=i,position=np.asarray(p,dtype=float),feature=feature(*f)) for i,p,f in items]


def seeded(config=IdentitySupervisorConfig()):
    s=IdentitySupervisor('s:1:1',model_id='test',start_time=0.,config=config)
    good=rows(('s:1:1',(2.,0.),(1.,0.)))
    s.observe(good,1.,{'s:1:1':'feature_ready'},geometry_state='bound_untracked',
              binding_snapshot={'state':'bound_untracked','resolved_id':'s:1:1','prediction':[2.,0.]})
    s.observe(good,1.2,{'s:1:1':'feature_ready'},geometry_state='bound_untracked',
              binding_snapshot={'state':'bound_untracked','resolved_id':'s:1:1','prediction':[2.,0.]})
    assert s.complete and s.state=='tracking'
    return s


def healthy(snapshot=None):
    return snapshot or {'state':'bound_tracking','resolved_id':'s:1:1','prediction':[2.,0.],'lost_s':None}


def test_low_quality_is_no_evidence_and_does_not_drive_suspect():
    s=seeded()
    for i in range(12):
        t=2.+i*.1
        s.observe([],t,{'s:1:1':'low_confidence'},geometry_state='bound_tracking',binding_snapshot=healthy())
    assert s.state=='tracking' and s.no_evidence_frames==12 and not s.window


def test_neighbor_ambiguity_freezes_templates_and_requires_clear_hold_plus_reverify():
    s=seeded();before=len(s.identity.gallery)
    ambiguous=rows(('s:1:1',(2.,0.),(1.,0.)),('s:1:2',(2.2,.1),(1.,0.)))
    s.observe(ambiguous,2.,{'s:1:1':'feature_ready','s:1:2':'feature_ready'},
              geometry_state='bound_tracking',neighbor_ambiguity=True,binding_snapshot=healthy())
    assert s.state=='suspect_ambiguous' and len(s.identity.gallery)==before
    assert s.last_template_update['reason']=='suspect_ambiguous'
    one=rows(('s:1:1',(2.,0.),(1.,0.)))
    s.observe(one,2.1,{'s:1:1':'feature_ready'},geometry_state='bound_tracking',binding_snapshot=healthy())
    assert s.state=='suspect_ambiguous'
    s.observe(one,2.6,{'s:1:1':'feature_ready'},geometry_state='bound_tracking',binding_snapshot=healthy())
    assert s.state=='tracking' and s.reason=='ambiguity_cleared_and_reverified'


def test_neighbor_during_enrollment_holds_but_keeps_collecting_frames():
    s=IdentitySupervisor('s:1:1',model_id='test',start_time=0.)
    good=rows(('s:1:1',(2.,0.),(1.,0.)))
    s.observe(good,1.,{'s:1:1':'feature_ready'},geometry_state='bound_untracked',
              neighbor_ambiguity=True,
              binding_snapshot={'state':'bound_untracked','resolved_id':'s:1:1','prediction':[2.,0.]})
    assert not s.complete and s.state=='suspect_ambiguous'
    s.observe(good,1.2,{'s:1:1':'feature_ready'},geometry_state='bound_untracked',
              neighbor_ambiguity=True,
              binding_snapshot={'state':'bound_untracked','resolved_id':'s:1:1','prediction':[2.,0.]})
    assert s.complete and s.state=='suspect_ambiguous'
    assert s.suspect_trigger=='neighbor_within_radius'

def test_enrollment_timeout_during_ambiguity_still_holds_without_speed_cap():
    c=IdentitySupervisorConfig(enrollment_timeout_s=1.)
    s=IdentitySupervisor('s:1:1',model_id='test',start_time=0.,config=c)
    s.observe([],1.01,{'s:1:1':'low_confidence'},geometry_state='bound_untracked',
              neighbor_ambiguity=True,
              binding_snapshot={'state':'bound_untracked','resolved_id':'s:1:1','prediction':[2.,0.]})
    assert s.enrollment_timed_out and not s.complete
    assert s.state=='suspect_ambiguous'
    assert s.reason=='suspect_ambiguous_after_enrollment_timeout'
    assert s.active_speed_cap(.30) is None

def test_negative_window_triggers_suspect_and_can_recover():
    s=seeded();orth=rows(('s:1:1',(2.,0.),(0.,1.)))
    for i in range(10):
        s.observe(orth,2.+i*.1,{'s:1:1':'feature_ready'},geometry_state='bound_tracking',binding_snapshot=healthy())
    assert s.state=='suspect_negative' and s.suspect_trigger=='negative_frames_in_window'
    good=rows(('s:1:1',(2.,0.),(1.,0.)))
    for i in range(6):
        s.observe(good,3.1+i*.1,{'s:1:1':'feature_ready'},geometry_state='bound_tracking',binding_snapshot=healthy())
    assert s.state=='tracking' and s.reason=='negative_evidence_window_recovered'


def test_view_expansion_learns_a_moderately_similar_view():
    s=seeded();mixed=rows(('s:1:1',(2.,0.),(.8,.6)))
    s.observe(mixed,2.,{'s:1:1':'feature_ready'},geometry_state='bound_tracking',binding_snapshot=healthy())
    assert len(s.identity.gallery)==2
    s.observe(mixed,4.01,{'s:1:1':'feature_ready'},geometry_state='bound_tracking',binding_snapshot=healthy())
    assert len(s.identity.gallery)==3
    assert s.template_origins[:2]==['initial','initial'] and s.template_origins[-1]=='expansion'
    assert s.last_template_update['reason']=='view_expansion'


def test_suspect_timeout_reports_reason():
    c=IdentitySupervisorConfig(suspect_timeout_s=1.)
    s=seeded(c);s._start_suspect(2.,'ambiguous','fixture')
    s.observe(rows(),3.1,{},geometry_state='bound_tracking',neighbor_ambiguity=False,binding_snapshot=healthy())
    assert s.state=='ended' and s.end_reason=='suspect_timeout_after_enrollment'


def test_lost_phase_and_four_thresholds_reacquire_new_track_id():
    s=seeded()
    snap={'state':'lost','resolved_id':None,'prediction':[2.,0.],'lost_s':1.}
    s.observe([],3.,{},geometry_state='lost',binding_snapshot=snap)
    assert s.state=='searching'
    snap['lost_s']=9.;s.observe([],11.,{},geometry_state='lost',binding_snapshot=snap)
    assert s.state=='passive_reacquire'
    candidate=rows(('s:1:2',(2.,0.),(1.,0.)))
    for t in (11.1,11.3,11.5):
        s.observe(candidate,t,{'s:1:2':'feature_ready'},geometry_state='lost',binding_snapshot=snap)
    assert s.reacquire_proposal['id']=='s:1:2' and s.resolved_id=='s:1:1'
    s.rebind('s:1:2',11.5)
    assert s.reacquired==1
    snap['lost_s']=30.;s2=seeded();s2.observe([],31.,{},geometry_state='lost',binding_snapshot=snap)
    assert s2.state=='ended' and s2.end_reason=='lost_task_release'
