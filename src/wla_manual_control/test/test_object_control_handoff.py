import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'scripts'))
from control_policy import Policy
from test_object_policy import feed

def ready():
    p=Policy();p.wheel_at=10.;p.wheel=(0.,0.);p.automatic_input((0.,0.),10.)
    assert p.set_auto(10.01,int(10.01e9))[0];return p

def command(p,kind,t=10.02,generation=None,revoke=None,request='job',session='object'):
    return p.object_control(kind,p.source_generation if generation is None else generation,
        p.object_channel.person_revoke_generation if revoke is None else revoke,request,session,t,int(t*1e9))

def test_select_then_bound_zero_candidate_activation_and_release():
    p=ready();assert command(p,'select')[0]
    assert p.mode=='MANUAL' and p.automatic_source=='OBJECT' and p.output(10.03)==(0.,0.)
    assert not command(p,'activate',10.03)[0]
    feed(p,10.04);assert command(p,'activate',10.05)[0]
    feed(p,10.06,.12);assert p.output(10.07)==(.12,0.)
    assert not command(p,'release',10.08,request='another')[0]
    assert command(p,'release',10.09)[0];assert p.output(10.10)==(0.,0.)
    assert p.object_channel.person_revoke_generation==1

def test_select_from_modeless_nav_requires_stationary_wheels():
    p=Policy();assert not command(p,'select')[0]
    p.wheel_at=10.;p.wheel=(0.,0.);assert command(p,'select')[0]
    assert not command(p,'activate',10.03)[0]
    p=ready();p.wheel=(.1,0.);assert not command(p,'select')[0]

def test_old_generations_and_other_task_cannot_activate():
    p=ready();assert command(p,'select')[0];feed(p,10.04)
    assert not command(p,'activate',10.05,generation=0)[0]
    assert not command(p,'activate',10.05,request='wrong')[0]
    assert not command(p,'activate',10.05,session='wrong')[0]
    assert command(p,'activate',10.05)[0]
    p.set_manual();assert not command(p,'activate',10.06,revoke=0)[0]

def test_same_source_select_does_not_steal_an_active_object():
    p=ready();assert command(p,'select')[0];feed(p,10.04);assert command(p,'activate',10.05)[0]
    assert not command(p,'select',10.06)[0]
