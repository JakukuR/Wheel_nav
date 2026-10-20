"""Client lifecycle tests use only a fake ROS object; no graph or robot I/O."""
import importlib.util
import json
from pathlib import Path
import signal
import sys
from types import SimpleNamespace as NS
import pytest


@pytest.fixture
def trial(monkeypatch):
    path=Path(__file__).resolve().parents[3]/'person_follow_trial.py'
    spec=importlib.util.spec_from_file_location('trial_client_under_test',path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    clock=NS(now=0.,started=None,request='',active=False,commands=[],handlers={},callbacks=[],
             interrupt_at=None,interrupt_signal=signal.SIGINT,fail_heartbeat=False,backend_end=False,stale=False,initialized=False,target_ids=[],module=m)
    def status():
        return dict(stamp_ns=int(clock.now*1e9),candidate_only=True,enabled=clock.active,
                    request_id=clock.request,reason='target_lost_stop' if not clock.active else 'started')
    def command(req):
        clock.commands.append((clock.now,req.command,req.lease_duration_s))
        clock.target_ids.append(req.target_id)
        if req.command=='start':
            clock.started=clock.now;clock.active=True;clock.request=req.request_id
        if req.command=='stop':clock.active=False
        accepted=not (req.command=='heartbeat' and clock.fail_heartbeat)
        return NS(done=lambda:True,result=lambda:NS(accepted=accepted,reason='test_reply'))
    client=NS(call_async=command,wait_for_service=lambda **_:True)
    node=NS(create_subscription=lambda typ,topic,cb,qos:clock.callbacks.append((topic,cb)),
            create_client=lambda *_:client,get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock.now*1e9))),destroy_node=lambda:None)
    def spin(*_,timeout_sec=0):
        clock.now+=timeout_sec
        if clock.backend_end and clock.started is not None and clock.now-clock.started>1:clock.active=False
        if not(clock.stale and clock.started is not None):
            for topic,cb in clock.callbacks:
                data=status() if topic.endswith('/status') else dict(
                    schema='wla-relative-detections/v1',stamp_ns=int(clock.now*1e9),session='fixture',epoch=1,
                    enabled=True,objects=[dict(id='fixture:1:1',label='person',detected=True,distance=2.,yaw=0.)])
                cb(NS(data=json.dumps(data)))
        if clock.interrupt_at and clock.now>=clock.interrupt_at:
            clock.handlers[clock.interrupt_signal]()
    def init():clock.initialized=True
    monkeypatch.setattr(m,'rclpy',NS(init=init,create_node=lambda _:node,spin_once=spin,
                     spin_until_future_complete=lambda *a,**k:None,try_shutdown=lambda:None))
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock.now))
    monkeypatch.setattr(m,'signal',NS(SIGINT=signal.SIGINT,SIGTERM=signal.SIGTERM,
                                    signal=lambda sig,handler:clock.handlers.__setitem__(sig,handler)))
    def run(*args):
        monkeypatch.setattr(sys,'argv',['trial','follow','--target-id','fixture:1:1',*args])
        return m.main()
    return clock,run


@pytest.mark.parametrize('sig',[signal.SIGINT,signal.SIGTERM])
def test_continuous_outlives_timed_limit_and_stops_on_signal(trial,sig):
    c,run=trial;c.interrupt_at=136.;c.interrupt_signal=sig
    assert run('--continuous')==0
    assert c.now-c.started>120
    assert c.commands[0][1]=='start' and c.commands[-1][1]=='stop'
    assert sum(kind=='heartbeat' for _,kind,_ in c.commands)>250
    assert all(lease==1.5 for _,_,lease in c.commands)
    assert c.now-c.commands[-2][0]<.5


def test_default_still_expires_after_thirty_seconds(trial):
    c,run=trial
    assert run()==0
    assert 30<=c.now-c.started<30.1
    assert c.commands[-1][1]=='stop'


@pytest.mark.parametrize('failure',['fail_heartbeat','backend_end','stale'])
def test_continuous_failure_exits_and_requests_stop(trial,failure):
    c,run=trial;setattr(c,failure,True)
    assert run('--continuous')==1
    assert c.now-c.started<2
    assert c.commands[-1][1]=='stop'


def test_duration_flags_are_mutually_exclusive(trial):
    c,run=trial
    with pytest.raises(SystemExit) as error:run('--continuous','--seconds','15')
    assert error.value.code==2 and not c.initialized and not c.commands


def test_short_id_is_resolved_once_for_start_heartbeats_and_stop(trial):
    c,run=trial
    assert run('--target-id','1','--seconds','1')==0
    assert c.target_ids and set(c.target_ids)=={'fixture:1:1'}


def test_unknown_short_id_never_attempts_start_or_stop(trial):
    c,run=trial
    assert run('--target-id','999','--seconds','1')==1
    assert not c.commands


def test_cancel_picker_never_attempts_start_or_stop(trial,monkeypatch):
    c,run=trial
    monkeypatch.setattr(c.module,'choose_interactively',lambda *a:None)
    assert run('--target-id','')==0
    assert not c.commands


def test_picker_rechecks_other_task_before_start(trial,monkeypatch):
    c,run=trial
    def pick(people_now,pump,is_stopped):
        c.active=True;pump();return 'fixture:1:1'
    monkeypatch.setattr(c.module,'choose_interactively',pick)
    assert run('--target-id','')==1
    assert not c.commands


def test_noninteractive_missing_id_gives_error_without_motion(trial,monkeypatch):
    c,run=trial
    monkeypatch.setattr(sys,'stdin',NS(isatty=lambda:False))
    assert run('--target-id','')==1 and not c.commands


def test_visible_people_rejects_stale_wrong_generation_and_invalid_geometry(trial):
    c,_=trial;m=c.module
    row=dict(id='s:1:190',label='person',detected=True,distance=2.,yaw=.1)
    packet=dict(schema='wla-relative-detections/v1',session='s',epoch=1,enabled=True,stamp_ns=10**9,objects=[row])
    assert m.resolve_target('190',m.visible_people(packet,10**9))=='s:1:190'
    assert m.resolve_target('1:190',[row])=='s:1:190'
    for bad in (dict(packet,stamp_ns=0),dict(packet,enabled=False)):
        with pytest.raises(RuntimeError):m.visible_people(bad,10**9)
    for badrow in (dict(row,label='chair'),dict(row,detected=False),dict(row,id='s:2:190'),dict(row,distance=None),dict(row,yaw=float('nan'))):
        assert m.visible_people(dict(packet,objects=[badrow]),10**9)==[]
    with pytest.raises(RuntimeError):m.visible_people(dict(packet,objects=[row,row]),10**9)
    with pytest.raises(RuntimeError):m.resolve_target('190',[row,dict(row,id='other:1:190')])


def test_picker_keeps_displayed_full_id_despite_reordering(trial,monkeypatch):
    c,_=trial;m=c.module
    a=dict(id='s:1:190',distance=2.,yaw=0.);b=dict(id='s:1:191',distance=3.,yaw=.5)
    snapshots=iter([[a,b],[b,a]]);stdin=NS(isatty=lambda:True,readline=lambda:'1\n')
    monkeypatch.setattr(m.select,'select',lambda *a:([stdin],[],[]))
    assert m.choose_interactively(lambda:next(snapshots),lambda:None,lambda:False,stdin)=='s:1:190'


def test_picker_never_reuses_short_id_after_detector_restart(trial,monkeypatch):
    c,_=trial;m=c.module
    a=dict(id='s:1:190',distance=2.,yaw=0.);new=dict(a,id='s:2:190')
    snapshots=iter([[a],[new]]);lines=iter(['1\n','q\n']);stdin=NS(isatty=lambda:True,readline=lambda:next(lines))
    monkeypatch.setattr(m.select,'select',lambda *a:([stdin],[],[]))
    assert m.choose_interactively(lambda:next(snapshots),lambda:None,lambda:False,stdin) is None


def test_picker_refresh_and_eof(trial,monkeypatch):
    c,_=trial;m=c.module
    a=dict(id='s:1:190',distance=2.,yaw=0.)
    snapshots=iter([[],[a],[a]]);lines=iter(['r\n','1\n']);stdin=NS(isatty=lambda:True,readline=lambda:next(lines))
    monkeypatch.setattr(m.select,'select',lambda *a:([stdin],[],[]))
    assert m.choose_interactively(lambda:next(snapshots),lambda:None,lambda:False,stdin)=='s:1:190'
    stdin.readline=lambda:''
    assert m.choose_interactively(lambda:[a],lambda:None,lambda:False,stdin) is None
