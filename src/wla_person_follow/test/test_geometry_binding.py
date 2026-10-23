import numpy as np
from wla_person_follow.follow_config import target_freshness_decision
from wla_person_follow.geometry_binding import GeometryBinding, GeometryBindingConfig


def row(ident='s:1:1',yaw=0.,distance=1.72):
    return {ident:dict(detected=True,label='person',id=ident,yaw=yaw,distance=distance)}


def test_enrollment_does_not_gate_same_id_geometry():
    b=GeometryBinding('s:1:1',0.)
    assert b.update(row(),np.zeros(3),1.,1.)=='s:1:1'
    assert b.state=='bound_untracked'


def test_single_missing_frame_keeps_binding_after_freshness_grace():
    c=GeometryBindingConfig(freshness_s=.30,missing_grace_s=.20)
    b=GeometryBinding('s:1:1',0.,c)
    b.update(row(),np.zeros(3),1.,1.)
    assert b.update({},np.zeros(3),1.31,1.31)=='s:1:1'
    assert b.state=='bound_coasting'
    assert b.update({},np.zeros(3),1.51,1.51) is None
    assert b.state=='lost'


def test_position_jump_has_no_grace():
    b=GeometryBinding('s:1:1',0.)
    b.update(row(distance=1.72),np.zeros(3),1.,1.)
    assert b.update(row(yaw=.8,distance=1.72),np.zeros(3),1.1,1.1) is None
    assert b.state=='lost' and b.reason=='current_position_jump'


def test_recovery_phase_and_release_are_explicit():
    b=GeometryBinding('s:1:1',0.)
    b.update(row(),np.zeros(3),1.,1.)
    b.update({},np.zeros(3),1.6,1.6)
    assert b.recovery_phase(1.6)=='active_search'
    assert b.recovery_phase(9.6)=='passive_reacquire'
    assert b.recovery_phase(31.7)=='release_due'


def test_rebind_restarts_track_history_and_source():
    b=GeometryBinding('s:1:1',0.)
    b.update(row(),np.zeros(3),1.,1.)
    b.update({},np.zeros(3),1.6,1.6)
    b.rebind('s:1:2',row('s:1:2',distance=1.5),np.zeros(3),2.,2.,confirmation=dict(id='s:1:2',lost_since=b.lost_since,count=3,stamp=2.))
    assert b.resolved_id=='s:1:2' and b.source=='reid_reacquired'
    assert b.state=='bound_tracking' and b.lost_since is None


def test_only_complete_compatibility_and_first_batch_presets_are_accepted():
    from wla_person_follow.follow_config import NEW_PRESET_FLAGS, validate_follow_preset
    p=dict(follow_preset='compatibility',geometry_binding_v2=False,
        front_check_truncation_enabled=False,identity_supervisor_v2=False,
        lost_task_release=False,mask_binding_v2=False,reverse_policy_v2=False,
        diagnostic_v2=False,reid_enabled=False,reid_model_id='')
    assert validate_follow_preset(p)=='compatibility'
    p['follow_preset']='first_batch_v2';p['reid_enabled']=True;p['reid_model_id']='fixture'
    for name in NEW_PRESET_FLAGS:
        p[name]=True
    assert validate_follow_preset(p)=='first_batch_v2'
    for name in NEW_PRESET_FLAGS:
        p[name]=True
        broken=dict(p);broken[name]=False
        try:validate_follow_preset(broken)
        except ValueError as exc:assert str(exc)=='invalid_follow_preset_configuration'
        else:raise AssertionError('partial first-batch preset must be rejected: '+name)
    p['lost_task_release']=False
    try:validate_follow_preset(p)
    except ValueError as exc:assert str(exc)=='invalid_follow_preset_configuration'
    else:raise AssertionError('partial first-batch preset must be rejected')
    p['lost_task_release']=True
    p['reid_model_id']=''
    try:validate_follow_preset(p)
    except ValueError as exc:assert str(exc)=='invalid_follow_preset_configuration'
    else:raise AssertionError('first batch requires an appearance model')
    p['reid_model_id']='fixture'
    p['follow_preset']='compatibility';p['geometry_binding_v2']=True
    try:validate_follow_preset(p)
    except ValueError as exc:assert str(exc)=='invalid_follow_preset_configuration'
    else:raise AssertionError('compatibility cannot enable v2 behavior')


def test_target_missing_grace_allows_only_freshness_plus_grace():
    kwargs=dict(geometry_binding_v2=True,binding_state='bound_coasting',
                max_sensor_age_s=.30,missing_grace_s=.20)
    assert target_freshness_decision(source_fresh=False,observation_age_s=.31,**kwargs)==(
        True,'geometry_binding_missing_grace')
    assert target_freshness_decision(source_fresh=False,observation_age_s=.50,**kwargs)==(
        True,'geometry_binding_missing_grace')
    assert target_freshness_decision(source_fresh=False,observation_age_s=.501,**kwargs)==(
        False,'target_observation_missing_grace_expired')
    kwargs['geometry_binding_v2']=False
    assert not target_freshness_decision(source_fresh=False,observation_age_s=.31,**kwargs)[0]
    kwargs['geometry_binding_v2']=True;kwargs['binding_state']='bound_tracking'
    assert not target_freshness_decision(source_fresh=False,observation_age_s=.31,**kwargs)[0]
