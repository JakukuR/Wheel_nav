"""L3 software chain: Agent facade -> C++ transport/Python MPC -> C++ mux.

Synthetic depth, camera, wheel, detections, masks and ReID; private command sink
in domain 198. No chassis driver, detector inference or physical motion.
"""
import os
import signal
import time
import pytest
from test_hybrid_ros import hybrid

pytestmark=pytest.mark.skipif(os.environ.get('RUN_SIM_ROS')!='1',reason='isolated ROS opt-in required')


def assert_zero_after(f,when,budget):
    f.pump(budget+.15)
    tail=[row for row in f.outputs if row[0]>=when+budget]
    moving=[t for t,v,w in f.outputs if t>=when and (v!=0. or w!=0.)]
    prior=[row for row in f.outputs if row[0]<=when]
    f.checks.append(dict(event_at=when,budget_s=budget,tail_samples=len(tail),
                        pre_event_command=list(prior[-1][1:]) if prior else None,
                        last_nonzero_after_event_s=max(moving)-when if moving else None,
                        tail_all_zero=bool(tail) and all(v==0. and w==0. for _,v,w in tail)))
    assert tail and all(v==0. and w==0. for _,v,w in tail), tail


def assert_real_motion(f):
    assert any(abs(v)+abs(w)>0 for _,v,w in f.outputs), f.states[-1]
    assert any(s['output_source']=='PERSON' for s in f.controls)
    # The final sink receives actual solver candidates through the real mux.
    assert any(abs(v)+abs(w)>0 and any(0<=t-ct<.3 and v==c['linear'] and w==c['angular']
               for ct,c in f.candidates) for t,v,w in f.outputs)


@pytest.mark.parametrize('hybrid,mode,label',[
    (dict(full_chain=True,focal=60.),'0','person'),
    (dict(full_chain=True,focal=60.),'1','person'),
    (dict(full_chain=True,focal=20.),'2','chair')],indirect=['hybrid'])
def test_three_modes_sensor_to_mux_and_agent_cancel(hybrid,mode,label):
    f=hybrid
    end=time.monotonic()+2.
    while time.monotonic()<end and not any(s.get('planner_activity')=='idle_map_only' for s in f.states):
        f.pump(.05)
    assert all(v==0. and w==0. for _,v,w in f.outputs)
    assert any(s.get('planner_activity')=='idle_map_only' for s in f.states)
    f.task_mode=mode;f.target_label=label;f.pump(.15)
    f.run_task('chain-mode-'+mode);assert_real_motion(f)
    assert f.states[-1]['request_id']=='chain-mode-'+mode
    if mode=='2':assert any(s.get('approach_goal') for s in f.states)
    # Person modes also replan between depth frames on the newest map.
    else:assert any(s.get('planner_source')=='control_tick' for s in f.states)
    stopped=time.monotonic()
    response=f.call('stop','chain-mode-'+mode)
    assert response.accepted and response.request_id=='chain-mode-'+mode
    assert_zero_after(f,stopped,.3)
    assert not f.call('heartbeat','chain-mode-'+mode).accepted
    assert not f.call('start','chain-mode-'+mode).accepted


@pytest.mark.parametrize('hybrid,fault',[(dict(full_chain=True),fault) for fault in
    ('depth','wheel','lease','python_hang','cpp_crash')],indirect=['hybrid'])
def test_chain_faults_expire_actual_mux_output(hybrid,fault):
    f=hybrid;f.run_task('chain-fault');assert_real_motion(f)
    # Inject while the final sink is actually receiving nonzero output. An
    # earlier nonzero followed by a scheduling-induced zero is insufficient.
    end=time.monotonic()+3.;next_heartbeat=time.monotonic()+.3
    while not (abs(f.outputs[-1][1])+abs(f.outputs[-1][2])>0 and time.monotonic()-f.outputs[-1][0]<.1):
        assert time.monotonic()<end, 'no current nonzero mux output before fault'
        f.pump(.01)
        if time.monotonic()>=next_heartbeat:
            assert f.call('heartbeat','chain-fault').accepted
            next_heartbeat=time.monotonic()+.3
    when=time.monotonic();budget=.5
    if fault=='depth':f.depths=False
    elif fault=='wheel':f.wheels=False
    elif fault=='lease':budget=1.25
    elif fault=='python_hang':os.kill(f.worker,signal.SIGSTOP)
    else:
        f.expect_transport_exit=True;f.proc.kill();assert f.proc.wait(timeout=2)!=0
    assert_zero_after(f,when,budget)
    if fault=='python_hang':
        # Stop travels through the real Agent facade while Python is frozen.
        start=time.monotonic();assert f.call('stop','chain-fault').accepted
        assert time.monotonic()-start<.5
    if fault=='lease':
        assert not f.states[-1]['transport_task_active']
        assert not f.call('heartbeat','chain-fault').accepted


@pytest.mark.parametrize('hybrid',[dict(full_chain=True)],indirect=True)
def test_chain_manual_priority_and_mux_estop(hybrid):
    from std_srvs.srv import SetBool
    f=hybrid;f.run_task('chain-priority');assert_real_motion(f)
    f.manual_velocity=.08;f.pump(.3)
    assert f.outputs[-1][1:]==(.08,0.)
    assert f.controls[-1]['output_source']=='MANUAL'
    f.manual_velocity=0.;f.pump(.2)
    assert f.call('heartbeat','chain-priority').accepted
    client=f.node.create_client(SetBool,'/r680_base/set_estop')
    assert client.wait_for_service(timeout_sec=1.)
    def estop(value):
        future=client.call_async(SetBool.Request(data=value));end=time.monotonic()+1.
        while not future.done() and time.monotonic()<end:f.pump(.01)
        assert future.done() and future.result().success
    when=time.monotonic();estop(True);assert_zero_after(f,when,.3)
    assert f.controls[-1]['estop_latched']
    assert not f.states[-1]['transport_task_active']
    estop(False);f.pump(.25)
    assert not f.call('heartbeat','chain-priority').accepted
    assert f.outputs[-1][1:]==(0.,0.)
    f.node.destroy_client(client)


@pytest.mark.parametrize('hybrid',[dict(full_chain=True)],indirect=True)
def test_chain_reid_ambiguity_recovery_keeps_task_identity(hybrid):
    f=hybrid;f.run_task('chain-reid');assert_real_motion(f)
    assert any((s.get('reid') or {}).get('completed_enrollment') for s in f.states)
    f.detected_ids=('hybrid:0:2','hybrid:0:3')
    for _ in range(4):f.pump(.2);assert f.call('heartbeat','chain-reid').accepted
    ambiguous=[s for s in f.states if (s.get('reid') or {}).get('reason')=='no_unambiguous_recovery_candidate']
    assert ambiguous and all(s['resolved_target_id']=='hybrid:0:1' for s in ambiguous)
    f.detected_ids=('hybrid:0:2',)
    for _ in range(6):f.pump(.2);assert f.call('heartbeat','chain-reid').accepted
    recovered=[s for s in f.states if s.get('resolved_target_id')=='hybrid:0:2']
    assert recovered and all(s['request_id']=='chain-reid' and s['target_id']=='hybrid:0:1' for s in recovered)
    assert any((s.get('reid') or {}).get('reacquired')==1 for s in recovered)
    assert any(s.get('motion_permitted') for s in recovered)
    when=time.monotonic();assert f.call('stop','chain-reid').accepted;assert_zero_after(f,when,.3)


@pytest.mark.parametrize('hybrid',[dict(full_chain=True)],indirect=True)
def test_person_arrival_is_terminal_through_agent_and_mux(hybrid):
    f=hybrid;f.task_mode='1';f.run_task('chain-arrival');assert_real_motion(f)
    # Synthetic person approaches the stationary robot; depth and detection agree.
    for distance in (1.25,1.10,.95):
        f.target_distance=distance;f.pump(.2)
        reply=f.call('heartbeat','chain-arrival')
        assert reply.accepted or reply.state=='arrived'
    # Arrival needs three fresh solver frames; expired frames must reset the
    # confirmation. Allow bounded scheduling variation without extending any
    # production freshness/command deadline.
    end=time.monotonic()+8.
    while time.monotonic()<end and not f.states[-1].get('terminal_arrival'):
        f.pump(.15)
        reply=f.call('heartbeat','chain-arrival')
        if reply.state=='arrived':break
        assert reply.accepted
    f.pump(.1)
    assert f.states[-1]['state']=='arrived' and f.states[-1]['terminal_arrival']
    assert f.states[-1]['arrival']['reason']=='person_surface_distance_reached'
    assert not f.states[-1]['transport_task_active']
    reply=f.call('heartbeat','chain-arrival')
    assert not reply.accepted and reply.state=='arrived' and not reply.enabled
    assert f.call('status','chain-arrival').state=='arrived'
    f.target_distance=1.4
    assert_zero_after(f,time.monotonic(),.3)
    assert f.call('stop','chain-arrival').state=='arrived'
    assert not f.call('start','chain-arrival').accepted
