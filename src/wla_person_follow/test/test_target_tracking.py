"""Native target Kalman regression: frame compensation, identity and limits."""
import math
import numpy as np
import pytest
from _wla_follow_local import TargetKalman
from wla_person_follow.target_tracking import TargetTracker,TargetEstimate
from wla_person_follow.relative_control import RelativeFollower,Safety
from wla_person_follow.local_avoidance import LocalAvoidance,scene_from_points

def test_constant_velocity_prediction_and_horizon():
    k=TargetKalman(.8,.4)
    for i in range(30):assert k.update(10.+i*.05,2.+.3*i*.05,.2)
    result=k.estimate(11.65)
    assert result and result[0]==pytest.approx(2.+.3*1.65,abs=.06)
    assert result[2]==pytest.approx(.3,abs=.06)
    assert not k.estimate(12.3)
    assert not k.estimate(11.)

def test_filter_reduces_measurement_jitter():
    k=TargetKalman();raw=[];filtered=[]
    for i in range(80):
        noise=.06*math.sin(i*2.)
        assert k.update(10.+i*.04,2.+noise,.3-noise)
        result=k.estimate(10.+i*.04)
        if i>10:raw.append(noise);filtered.append(result[0]-2.)
    assert np.std(filtered)<np.std(raw)*.7

def test_outlier_and_replay_do_not_refresh_accepted_measurement():
    k=TargetKalman();assert k.update(10.,2.,0.)
    assert not k.update(10.05,8.,0.) and k.reason=='innovation_rejected'
    assert k.stamp==10.
    assert not k.update(10.,2.,0.)
    assert k.estimate(10.1)[0]==pytest.approx(2.)

def test_covariance_limit_rejects_long_prediction_even_before_horizon():
    k=TargetKalman(.8,.1);k.update(10.,2.,0.)
    assert k.estimate(10.01)
    assert not k.estimate(10.2)

def test_robot_rotation_and_translation_are_not_target_motion():
    tracker=TargetTracker()
    for i in range(30):
        x=i*.005;theta=i*.01
        assert tracker.update(10.+i*.04,-theta,2.-x,(x,0.,theta))
    result=tracker.estimate(11.16,(.145,0.,.29),'id',11.16)
    assert result.yaw==pytest.approx(-.29,abs=.005)
    assert result.distance==pytest.approx(1.855,abs=.01)
    assert abs(tracker.filter.estimate(11.16)[2])<.01
    tracker.reset(('another','identity'))
    assert tracker.estimate(11.2,(0.,0.,0.),'new',0.) is None

def ready(mode='follow'):
    core=RelativeFollower(local_planner=LocalAvoidance(),require_estimate=True)
    def frame(t,objects):
        assert core.receive(dict(schema='wla-relative-detections/v1',base_frame='r680_mapping_floor',session='test',epoch=0,enabled=True,stamp_ns=int(t*1e9),objects=objects),t,int(t*1e9))
    frame(10.,[dict(detected=True,label='person',id='test:0:1',yaw=.3,distance=2.)])
    assert core.command('start','task',mode,'test:0:1',1.,10.)[0]
    x,y=np.meshgrid(np.arange(.1,3.,.1),np.arange(-3.,3.,.1))
    scene=scene_from_points(np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)]))
    return core,frame,scene

def test_lost_target_moves_to_last_seen_and_still_checks_obstacles():
    core,frame,scene=ready();frame(10.1,[])
    estimate=TargetEstimate('test:0:1',.3,2.,.1,.1,False,0.,2.)
    v,w=core.tick(10.1,Safety(True,10.,True),scene,estimate)
    assert v>0 and abs(w)<1e-6 and core.reason=='target_prediction_last_seen'
    assert core.tick(10.15,Safety(True,.3,True),scene,estimate)==(0.,0.)
    assert core.reason=='emergency_obstacle'

def test_prediction_never_bypasses_stale_packets_or_identity():
    core,frame,scene=ready();frame(10.1,[])
    wrong=TargetEstimate('test:0:2',.3,2.,.1,.1,False)
    assert core.tick(10.15,Safety(True,10.,True),scene,wrong)==(0.,0.)
    right=TargetEstimate('test:0:1',.3,2.,.4,.1,False)
    assert core.tick(10.5,Safety(True,10.,True),scene,right)==(0.,0.)
    assert core.reason=='observations_stale'

def test_predictions_do_not_complete_approach():
    core,frame,scene=ready('approach_interact')
    for i in range(1,5):
        frame(10.+i*.05,[])
        estimate=TargetEstimate('test:0:1',0.,1.,i*.05,.1,False)
        assert core.tick(10.+i*.05,Safety(True,10.,True),scene,estimate)==(0.,0.)
    assert core.active and core.arrival_count==0


def test_last_seen_position_is_fixed_while_predicted_person_moves():
    tracker=TargetTracker()
    for i in range(25):assert tracker.update(10.+i*.05,0.,2.+i*.02,(0.,0.,0.))
    result=tracker.estimate(11.4,(.1,0.,0.),'id',11.4)
    assert result and not result.observed
    assert result.last_seen_distance==pytest.approx(2.+24*.02-.1)
    assert result.distance>result.last_seen_distance+.03

def test_arrived_at_last_seen_turns_to_predicted_bearing_without_finishing_task():
    core,frame,scene=ready();frame(10.1,[])
    estimate=TargetEstimate('test:0:1',.4,2.,.1,.1,False,0.,.35)
    v,w=core.tick(10.1,Safety(True,10.,True),scene,estimate)
    assert v==0 and w>0 and core.reason=='target_prediction_align'
    assert core.active

def test_two_second_budget_is_configurable_and_does_not_renew_on_prediction():
    k=TargetKalman(2.,1.)
    for i in range(30):assert k.update(10.+i*.04,2.,0.)
    assert k.estimate(12.16)
    assert not k.estimate(13.17)
    assert not k.estimate(13.18)
    with pytest.raises(ValueError):TargetKalman(2.1,1.)


def test_robot_and_camera_move_but_stationary_person_stays_in_reference():
    from wla_person_follow.depth_proc import points_to_base
    from wla_person_follow.local_memory import camera_in_reference
    tracker=TargetTracker();world_person=np.array([3.,.4,.6])
    optical_basis=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    camera_positions=[]
    for i in range(30):
        pose=(i*.005,i*.002,i*.008)
        pan=.12*math.sin(i*.15)
        cp,sp=math.cos(pan),math.sin(pan)
        mount_rotation=np.array([[cp,-sp,0.],[sp,cp,0.],[0.,0.,1.]]) @ optical_basis
        mount_position=np.array([.20,.02*math.sin(i*.2),.30+.01*math.cos(i*.2)])
        rotation,position=camera_in_reference(pose,mount_rotation,mount_position)
        camera_positions.append(position)
        optical=rotation.T @ (world_person-position)
        fx=fy=300.;cx=320.;cy=240.
        u=int(round(cx+fx*optical[0]/optical[2]));v=int(round(cy+fy*optical[1]/optical[2]))
        assert 0<=u<640 and 0<=v<480
        depth=np.zeros((480,640),np.float32);depth[v,u]=optical[2]
        points=points_to_base(depth,(fx,fy,cx,cy),mount_rotation,mount_position,
                             height_min=-3.,height_max=3.,stride=1)
        assert len(points)==1
        x,y=points[0,:2]
        assert tracker.update(10.+i*.04,math.atan2(y,x),math.hypot(x,y),pose)
    estimate=tracker.filter.estimate(11.16)
    assert estimate[:2]==pytest.approx(world_person[:2],abs=.01)
    assert math.hypot(*estimate[2:4])<.025
    assert np.linalg.norm(camera_positions[-1]-camera_positions[0])>.1

def test_target_velocity_is_world_motion_not_robot_plus_camera_motion():
    tracker=TargetTracker()
    for i in range(60):
        dt=i*.04;pose=(.12*dt,-.05*dt,.2*dt)
        world=(3.+.25*dt,.4-.1*dt)
        dx,dy=world[0]-pose[0],world[1]-pose[1]
        assert tracker.update(10.+dt,math.atan2(dy,dx)-pose[2],math.hypot(dx,dy),pose)
    estimate=tracker.filter.estimate(12.36)
    assert estimate[2:4]==pytest.approx((.25,-.1),abs=.02)
