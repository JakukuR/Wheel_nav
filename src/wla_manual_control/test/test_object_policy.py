import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'scripts'))
from control_policy import Policy

def feed(p,t,v=0.,source='object',request='job'):
    stamp=int(t*1e9)
    assert getattr(p,source+'_status_input')(dict(schema='wla-'+source+'-status/v1',
        stamp_ns=stamp,follow_session=source,request_id=request,enabled=True,
        candidate_only=False,motion_permitted=True),t,stamp)
    assert getattr(p,source+'_input')(dict(schema='wla-'+source+'-candidate/v1',
        stamp_ns=stamp,follow_session=source,request_id=request,linear=v,angular=0.),t,stamp)

def ready():
    p=Policy();p.wheel_at=10.;p.wheel=(0.,0.)
    assert p.set_object_source(True,10.,int(10e9))[0]
    feed(p,10.01)
    assert p.set_auto(10.02,int(10.02e9))[0]
    return p

def test_only_selected_object_channel_reaches_output():
    p=ready();feed(p,10.03,.05);feed(p,10.04,.2,'person')
    assert p.output(10.05)==(.05,0.)
    assert not p.person_input(dict(schema='wla-object-candidate/v1'),10.06,int(10.06e9))
    assert p.output(10.07)==(.05,0.)

def test_switch_to_person_clears_object_and_requires_explicit_auto():
    p=ready();feed(p,10.03,.05);p.wheel_at=10.1
    assert p.set_person_source(True,10.1,int(10.1e9))[0]
    assert p.mode=='MANUAL' and p.object_channel.person_revoke_generation==1
    assert p.output(10.11)==(0.,0.)
    assert not p.set_auto(10.12,int(10.12e9))[0]

def test_object_expiry_and_manual_takeover_revoke_task():
    p=ready();feed(p,10.03,.05)
    assert p.output(10.3)==(0.,0.) and p.mode=='MANUAL'
    assert p.object_channel.person_revoke_generation==1
    p=ready();feed(p,10.03,.05);p.gamepad_status_input('ACTIVE',10.04)
    assert p.manual_input((.1,0.),10.05)
    assert p.object_channel.person_revoke_generation==1

def test_object_cannot_select_while_estopped_or_moving():
    p=Policy();p.wheel_at=10.;p.wheel=(.1,0.)
    assert not p.set_object_source(True,10.,int(10e9))[0]
    p.wheel=(0.,0.);p.set_estop(True,10.)
    assert not p.set_object_source(True,10.,int(10e9))[0]
