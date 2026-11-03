"""Exploration primitives: mode 3 point goal and mode 4 in-place orient."""
import math
import numpy as np
import pytest
from wla_person_follow.sim_runtime import SimulationEngine
from wla_person_follow.task_modes import parse_goal, normalize_mode
from wla_person_follow.task_session import TaskSession


def _box_ahead_map():
    from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
    m=RollingDepthMap(resolution=.05,size=160);m.slide(np.zeros(2))
    floor=np.array([[x,y,0.] for x in np.arange(-1.,3.5,.05) for y in np.arange(-2.,2.01,.05)
                    if not (.95<x<1.45 and abs(y)<.45)])
    box=np.array([[x,y,z] for x in np.arange(1.,1.41,.05) for y in np.arange(-.4,.41,.05) for z in (.2,.45)])
    points=np.r_[floor,box]
    m.update(dict(points=points,points_odom=points,dynamic=np.zeros(len(points),bool),
                  origin=np.array([0,0,.3]),origin_odom=np.array([0,0,.3])),np.zeros(3),0.)
    return m


def _session(now=1.):
    s=TaskSession()
    s.gate_enabled=True;s.expires_at=now+.3
    return s


def test_goal_target_id_parsing():
    assert normalize_mode('3')=='3' and normalize_mode(4)=='4'
    assert parse_goal('3','1.5,-0.5')==dict(kind='point',relative=[1.5,-.5])
    assert parse_goal('4','6.28')['relative']==pytest.approx(6.28)
    for mode,target in (('3','4,0'),('3','0,0'),('3','1'),('3','a,b'),('3','nan,1'),('4','7.5'),('4','0'),('4','1,2')):
        with pytest.raises(ValueError):parse_goal(mode,target)


def test_goal_start_needs_live_detector_gate_but_no_target():
    s=_session()
    assert s.command('start','p1','3','1.0,0.5',1.,1.)==(True,'started')
    assert s.goal==dict(kind='point',relative=[1.,.5]) and s.target_label==''
    s=_session();s.gate_enabled=False
    assert s.command('start','p2','3','1.0,0.5',1.,1.)==(False,'detection_disabled')
    s=_session()
    assert s.command('start','p3','3','1.0,0.5',1.,2.)==(False,'observations_stale')
    assert s.command('start','p4','4','9',1.,1.)[0] is False


def test_point_goal_routes_round_a_box_and_drives():
    e=SimulationEngine(np.zeros(3),0.);e.mapping=_box_ahead_map()
    out=e._plan(np.zeros(3),np.zeros(2),None,.1,None,None,'3',None,goal=dict(kind='point',point=[2.5,0.]))
    assert out['reason']=='point_goal' and out['planner_feasible']
    path=out['path']
    # Reference bends past the box instead of through it and ends at the goal.
    assert np.max(np.abs(path[:,1]))>.45 and np.allclose(path[-1],[2.5,0.])
    assert out['command'][0]>=0 and out['tuning']['max_v']==pytest.approx(.30)


def test_point_goal_route_is_reused_then_refreshed():
    e=SimulationEngine(np.zeros(3),0.);e.mapping=_box_ahead_map()
    goal=dict(kind='point',point=[2.5,0.])
    e._plan(np.zeros(3),np.zeros(2),None,.1,None,None,'3',None,goal=goal)
    first=e.point_route[1]
    e._plan(np.zeros(3),np.zeros(2),None,.3,None,None,'3',None,goal=goal)
    assert e.point_route[1]==first
    e._plan(np.zeros(3),np.zeros(2),None,.7,None,None,'3',None,goal=goal)
    assert e.point_route[1]==pytest.approx(.7)


@pytest.mark.parametrize('remaining,sign',[(math.pi,1.),(-1.,-1.)])
def test_orient_turns_in_place_towards_the_remaining_angle(remaining,sign):
    e=SimulationEngine(np.zeros(3),0.)
    out=e._plan(np.zeros(3),np.zeros(2),None,.1,None,None,'4',None,goal=dict(kind='orient',remaining=remaining))
    assert out['command'][0]==0. and np.sign(out['command'][1])==sign
    assert out['reason']=='orient' and out['planner_activity']=='in_place_turn'
    # Repeated plans ramp up to the turn rate and never exceed it.
    for k in range(2,40):
        out=e._plan(np.zeros(3),out['command'],None,.1*k,None,None,'4',None,goal=dict(kind='orient',remaining=remaining))
    assert .35<=abs(out['command'][1])<=.5


def test_orient_stops_inside_the_done_band():
    e=SimulationEngine(np.zeros(3),0.)
    out=e._plan(np.zeros(3),np.zeros(2),None,.1,None,None,'4',None,goal=dict(kind='orient',remaining=.03))
    assert np.allclose(out['command'],0.)
