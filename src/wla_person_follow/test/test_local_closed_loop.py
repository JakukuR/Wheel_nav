"""Kinematic fixtures only: observed rays, camera target loss, no chassis."""
import math
import numpy as np
from wla_person_follow.local_avoidance import LocalAvoidance,scene_from_points
from wla_person_follow.local_memory import LocalMemory,LocalMotion,advance
from wla_person_follow.relative_control import RelativeFollower,Safety

def run_scene(fov, box_start=1.2, target_x=3.5):
    x,y=np.meshgrid(np.arange(-3.,9.01,.05),np.arange(-5.,5.01,.05))
    ground=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
    x,y=np.meshgrid(np.arange(box_start,box_start+.26,.025),np.arange(-.25,.26,.025))
    box=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,.20)])
    world=np.vstack([ground,box]);target=np.array([target_x,0.]);pose=np.zeros(3)
    planner=LocalAvoidance();core=RelativeFollower(local_planner=planner)
    memory=LocalMemory();motion=LocalMotion();prior=(0.,0.,0.)
    min_clearance=math.inf;max_lateral=0.;detoured=False;lost=False
    for step in range(450):
        now=10.+step*.1
        motion.update(now,prior,now)
        c,s=math.cos(pose[2]),math.sin(pose[2]);rot=np.array([[c,-s],[s,c]])
        local=(world[:,:2]-pose[:2])@rot
        seen=(local[:,0]>.05)&(np.linalg.norm(local,axis=1)<4.)&(np.abs(np.arctan2(local[:,1],local[:,0]))<math.radians(fov))
        points=np.column_stack([local[seen],world[seen,2]])
        scene=memory.apply(scene_from_points(points),motion.pose_at(now),now,motion.generation)
        relative=(target-pose[:2])@rot;distance=float(np.linalg.norm(relative));yaw=math.atan2(relative[1],relative[0])
        visible=abs(yaw)<math.radians(fov)
        rows=[dict(detected=True,label='person',id='sim:0:1',yaw=yaw,distance=distance)] if visible else []
        assert core.receive(dict(schema='wla-relative-detections/v1',base_frame='r680_mapping_floor',session='sim',epoch=0,enabled=True,stamp_ns=int(now*1e9),objects=rows),now,int(now*1e9))
        if step==0: assert core.command('start','task','follow','sim:0:1',1.,now)[0]
        else: assert core.command('heartbeat','task',lease_duration_s=1.,now=now)[0]
        obstacles=points[points[:,2]>.01]
        nearest=float(np.linalg.norm(obstacles[:,:2],axis=1).min()) if len(obstacles) else math.inf
        v,w=core.tick(now,Safety(True,nearest,False),scene)
        clearance=float(np.linalg.norm(box[:,:2]-pose[:2],axis=1).min())
        min_clearance=min(min_clearance,clearance);max_lateral=max(max_lateral,abs(pose[1]))
        assert clearance>.60,(step,pose,clearance)
        detoured |= core.reason=='local_detour'
        if not visible:
            assert (v,w)==(0.,0.) and core.reason=='target_not_visible'
            lost=True;break
        assert scene.command_clear(v,w) or (v,w)==(0.,0.)
        pose=np.array(advance(pose,(v,0.,w),.1));prior=(v,0.,w)
        if distance<=1.50 and abs(yaw)<.12: break
    return dict(distance=distance,min_clearance=min_clearance,max_lateral=max_lateral,detoured=detoured,lost=lost,steps=step+1,reason=core.reason)

def test_wide_view_closed_loop_bypasses_box():
    result=run_scene(85.)
    assert result['detoured'] and not result['lost'],result
    assert result['distance']<=1.5 and result['max_lateral']>.5,result

def test_narrow_view_does_not_continue_on_invisible_person():
    result=run_scene(43.)
    assert result['detoured'] and result['lost'],result
    assert result['distance']>1.5,result
