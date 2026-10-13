import math
import numpy as np
import pytest
from _wla_follow_local import close_object
from wla_object_follow.control import ObjectFollower
from wla_object_follow.node import reference_points
from wla_person_follow.local_avoidance import LocalAvoidance,scene_from_points
from wla_person_follow.relative_control import Safety
from wla_person_follow.target_tracking import TargetEstimate

def surface(x,y=0.):
    return np.array([[x,y+dy,z] for dy in [-.03,0.,.03] for z in [.1,.15,.2]],np.float32)

def test_fifteen_centimeters_is_from_front_edge_not_robot_center():
    p=surface(.39);result=close_object(p,p,0.)
    assert result.valid and result.arrived
    assert result.gap==pytest.approx(.15,abs=1e-6)
    assert result.v==result.w==0.
    assert not close_object(surface(.15),surface(.15),0.).arrived

def test_close_speed_and_turn_limits():
    p=surface(.8);result=close_object(p,p,0.)
    assert  .05<result.v<=.20 and result.w==0.
    turned=close_object(p,p,.3)
    assert turned.v==0 and 0<turned.w<=.25

def test_foreground_and_side_obstacles_are_not_erased_with_target():
    target=surface(.8)
    for blocker in [surface(.32),surface(.1,.30)]:
        result=close_object(np.vstack([target,blocker]),target,0.)
        assert result.v==result.w==0. and not result.arrived
    # A distinct foreground obstacle stops the approach without claiming arrival.
    foreground=close_object(np.vstack([target,surface(.39)]),target,0.)
    assert foreground.v==0 and not foreground.arrived
    assert foreground.reason=='foreground_obstacle'

def test_invalid_or_insufficient_surface_and_minimum_clearance():
    assert not close_object(surface(.8),np.zeros((0,3),np.float32),0.).valid
    with pytest.raises(ValueError):close_object(surface(.8),surface(.8),0.,desired_clearance=.10)
    with pytest.raises(ValueError):close_object(surface(.8),surface(.8),float('nan'))

def ready(mode='approach_object',label='bottle'):
    core=ObjectFollower(local_planner=LocalAvoidance(),require_estimate=True)
    def feed(t,d=.39,identifier='det:0:1',label=label):
        assert core.receive(dict(schema='wla-relative-detections/v1',base_frame='r680_mapping_floor',
            session='det',epoch=0,enabled=True,stamp_ns=int(t*1e9),objects=[dict(detected=True,
            label=label,id=identifier,yaw=0.,distance=d)]),t,int(t*1e9))
        p=surface(d);core.geometry=(p,p,int(t*1e9));core.geometry_until=t+.3
        return TargetEstimate(identifier,0.,d,0.,.08,True)
    estimate=feed(10.)
    accepted=core.command('start','task',mode,'det:0:1',1.,10.)
    return core,feed,estimate,accepted

def scene():return scene_from_points([[2.,0.,0.],[2.,1.,0.],[2.,-1.,0.]])

def test_approach_latches_after_three_distinct_surface_frames():
    core,feed,estimate,accepted=ready();assert accepted[0]
    for t in [10.01,10.02]:
        assert core.tick(t,Safety(True,.39,True),scene(),estimate)==(0.,0.)
    assert core.active and core.arrival_count==1
    for t in [10.05,10.10]:
        estimate=feed(t);core.tick(t,Safety(True,.39,True),scene(),estimate)
    assert core.state=='arrived' and not core.active

def test_follow_maintains_then_restarts_as_object_moves_away():
    core,feed,estimate,accepted=ready('follow_object');assert accepted[0]
    for t in [10.05,10.10,10.15]:
        estimate=feed(t);core.tick(t,Safety(True,.39,True),scene(),estimate)
    assert core.active
    estimate=feed(10.20,.8)
    v,w=core.tick(10.20,Safety(True,.8,True),scene(),estimate)
    assert 0<v<=.20 and w==0

def test_object_prediction_loss_stale_depth_and_expired_lease_stop():
    core,feed,estimate,accepted=ready('follow_object');assert accepted[0]
    predicted=TargetEstimate('det:0:1',0.,.8,.1,.1,False)
    assert core.tick(10.1,Safety(True,.8,True),scene(),predicted)==(0.,0.)
    assert core.reason=='object_estimate_unavailable'
    estimate=feed(10.15,.8);core.geometry_until=10.16
    assert core.tick(10.2,Safety(True,.8,True),scene(),estimate)==(0.,0.)
    assert core.reason=='object_surface_unavailable'
    assert core.tick(11.1,Safety(True,.8,True),scene(),estimate)==(0.,0.)
    assert core.state=='expired'

def test_person_is_rejected_and_rejected_switch_keeps_current_label():
    core,feed,estimate,accepted=ready(label='person');assert not accepted[0]
    core,feed,estimate,accepted=ready();assert accepted[0]
    feed(10.1,label='chair',identifier='det:0:2')
    assert not core.command('start','other','invalid','det:0:2',1.,10.1)[0]
    assert core.target_label=='bottle' and core.target_id=='det:0:1'

def test_sensor_points_round_trip_through_independent_reference():
    points=surface(.8);pose=(.4,-.2,.7)
    assert reference_points(reference_points(points,pose),pose,True)==pytest.approx(points,abs=1e-6)


def test_speed_tapers_through_five_centimeters_per_second_to_zero():
    velocities=[]
    for gap in [.56,.35,.23,.19,.15]:
        p=surface(.24+gap);result=close_object(p,p,0.)
        velocities.append(result.v)
    assert .05<velocities[0]<=.20
    assert 0<velocities[-2]<.05
    assert velocities[-1]==0
    assert all(a>=b for a,b in zip(velocities,velocities[1:]))


def test_measured_motion_cannot_be_reported_as_arrived():
    p=surface(.39)
    result=close_object(p,p,0.,measured_v=.1)
    assert not result.arrived and result.v==0.
    assert not close_object(p,p,0.,measured_w=.1).arrived
    p=surface(.8)
    assert close_object(p,p,0.,max_speed=.12).v<=.12
    for kwargs in [dict(max_speed=.21),dict(reaction_s=0.),dict(deceleration=0.),dict(measured_v=float('nan'))]:
        with pytest.raises(ValueError):close_object(p,p,0.,**kwargs)


def test_simulated_delayed_chassis_converges_without_crossing_clearance():
    # Synthetic 0.7 s transport delay, 0.3 m/s^2 acceleration/braking.
    # This is an explicit plant assumption, not hardware stopping validation.
    from collections import deque
    dt=.05;pending=deque([0.]*14);gap=.8;velocity=0.;commands=[];gaps=[]
    for _ in range(1200):
        p=surface(.24+gap)
        result=close_object(p,p,0.,measured_v=velocity)
        pending.append(result.v);desired=pending.popleft()
        velocity+=max(-.3*dt,min(.3*dt,desired-velocity))
        gap-=velocity*dt;commands.append(result.v);gaps.append(gap)
        if result.arrived and velocity<.001 and not any(pending):break
    assert min(gaps)>=.13
    assert .13<=gap<=.17 and velocity<.001
    assert max(commands)>.05 and any(0<v<.05 for v in commands)
