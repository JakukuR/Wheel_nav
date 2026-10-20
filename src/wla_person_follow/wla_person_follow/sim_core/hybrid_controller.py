"""Hybrid visual following / rolling depth map controller, differential drive.

Engineering sampling MPC; NOT the paper's ACADOS NMPC or upstream ROG-Map.
No scene, true obstacle geometry, or invisible target truth is accepted here.
"""
import time
import numpy as np
from .common import Tuning, wrap
from .common import rotation
from .blocked_recovery import BlockedRecovery
from .close_follow import rear_yield_plan,rear_cell_clearance
from .smooth_follow import smooth_reference,comfort_allowed
from .wide_turn import outside_reference


def approach_tangent(pose, command, tracks, target):
    """OP-Follower-inspired approach-aware tangential bias (weak, bounded).

    Relative approach rate increases its weight. Tangent sign favors target
    progress. This biases candidate headings BEFORE collision checking; it
    never blends an unchecked lateral velocity into a differential command.
    """
    result=0.;robot_v=rotation(pose[2])@np.array([command[0],0.])
    direction=target-pose[:2]
    for o in tracks:
        delta=pose[:2]-o['p'];distance=np.linalg.norm(delta)
        gap=max(.08,distance-o['r']-.24)
        if gap>=1.2:continue
        normal=delta/max(distance,1e-9)
        approaching=max(0.,-np.dot(normal,robot_v-o['v']))
        tangent=np.array([-normal[1],normal[0]])
        if np.dot(tangent,direction)<0:tangent=-tangent
        cross=np.cos(pose[2])*tangent[1]-np.sin(pose[2])*tangent[0]
        result+=.08*approaching*(1/gap-1/1.2)*cross
    return float(np.clip(result,-.18,.18))


class HybridFollower:
    def __init__(self, config=Tuning(), enable_recovery=True):
        self.c = config
        self.fallbacks = 0
        self.calls = 0
        self.times = []
        self.failure_times = []
        self.failure_reasons = []
        self.recovery=BlockedRecovery(config) if enable_recovery else None
        self.last_infeasible=False

    def command(self, pose, previous, target, velocity, history, local_map, obstacles, status):
        start = time.perf_counter()
        c = self.c
        recovery=self.recovery
        wide=None
        if getattr(c,'wide_turn',False) and status!='lost':
            source=recovery.route if recovery is not None and recovery.state=='detour' and recovery.route is not None else history
            wide=outside_reference(pose,source,target,local_map,obstacles,c)
        travel_history=wide['path'] if wide is not None else history
        if recovery is not None:
            recovery.last_infeasible=self.last_infeasible
            recovery.update(pose,previous,target,velocity,local_map,obstacles,status,self.calls*c.step,
                            history=travel_history)
        state='follow' if recovery is None else recovery.state
        retreat=state=='retreat'
        holding=state in ('wait','hold')
        scanning=state=='scan'
        detour=state=='detour' and recovery.route is not None
        if state!='follow':wide=None
        self.last_wide=wide
        self.last_infeasible=False
        lost = target is None or status == 'lost'
        search=recovery.search if recovery is not None else None
        if lost and search is not None:target=search['point']
        if target is None:
            target = pose[:2] + rotation(pose[2]) @ np.array([1.6, 0.])
        if lost:
            velocity = np.zeros(2)
        yielding=rear_yield_plan(pose,target,local_map,obstacles,c) if not lost and wide is None and state in ('follow','wait','hold') else None
        self.last_yield=bool(yielding)
        if yielding:holding=False
        comfort=getattr(c,'smooth_reference',False) and comfort_allowed(pose,previous,obstacles,c)
        path = np.array(wide['path'] if wide is not None else (recovery.route if detour else history))
        if len(path) < 2:
            path = np.vstack((path, target))
        # Append only the current estimate, never the future prescribed route.
        if np.linalg.norm(path[-1]-target)>.01:
            path = np.vstack((path, target))
        if len(path) == 0:
            path = np.vstack((pose[:2], target))
        if comfort and not (retreat or yielding or lost):
            path=smooth_reference(path,local_map,obstacles,c)
        segments = np.diff(path, axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        arcs = np.r_[0., np.cumsum(lengths)]
        alpha = np.clip(np.sum((pose[:2]-path[:-1])*segments, axis=1)/np.maximum(lengths**2, 1e-12), 0., 1.)
        projection = path[:-1]+alpha[:,None]*segments
        idx = np.argmin(np.linalg.norm(projection-pose[:2], axis=1))
        s0 = arcs[idx]+alpha[idx]*lengths[idx]
        target_speed = np.linalg.norm(velocity)
        cruise = np.clip(target_speed+.65*(arcs[-1]-s0-c.desired_gap), 0., c.max_v)
        times = np.arange(1,c.horizon+1)*c.step
        ref_s = np.maximum(s0, np.minimum(s0+cruise*times, arcs[-1]+target_speed*times-c.desired_gap))
        refs = np.column_stack([np.interp(ref_s, arcs, path[:,j]) for j in range(2)])
        beyond = np.maximum(ref_s-arcs[-1],0.)
        refs += beyond[:,None]*velocity/max(target_speed,1e-9)
        if retreat:
            remaining=max(0.,recovery.tuning.retreat_m-(recovery.retreat_start-pose[:2])@recovery.heading)
            refs=pose[:2]-np.minimum(remaining,recovery.tuning.reverse_speed*times)[:,None]*recovery.heading
        if yielding:refs=pose[:2]+np.minimum(yielding['distance'],.15*times)[:,None]*yielding['heading']
        # A family of feedback trajectories: rate-limited speed and heading bias.
        speeds, offsets = np.meshgrid(np.r_[-.25,-.15,-.075,np.linspace(0.,c.max_v,10)], np.linspace(-1.2,1.2,25))
        speeds, offsets = speeds.ravel(), offsets.ravel()
        if retreat:
            speeds,offsets=np.meshgrid(np.linspace(-recovery.tuning.reverse_speed,0.,7),np.linspace(-.3,.3,9))
            speeds,offsets=speeds.ravel(),offsets.ravel()
        if holding or scanning:speeds[:]=0.
        if (hasattr(c,'target_clearance_max') and not yielding and state=='follow'
                and np.linalg.norm(velocity)<.05
                and c.target_hard_gap+.01<=np.linalg.norm(target-pose[:2])<=c.radius+c.target_radius+c.target_clearance_max):
            speeds[:]=0.
        if yielding:speeds=np.linspace(0.,.15,10);offsets=np.zeros(10)
        if lost:speeds[:]=0.;offsets[:]=0.
        count = len(speeds)
        states = np.tile(pose,(count,1)); controls = np.tile(previous,(count,1))
        score = np.zeros(count); safe = np.ones(count,dtype=bool)
        reasons=np.ones((3,count),dtype=bool)
        near_fov = np.ones(count,dtype=bool)
        minimum = np.full(count,np.inf)
        first = None
        rear_previous=np.full(count,yielding['static_floor']) if yielding else None
        rear_track_previous=np.full(count,yielding['track_floor']) if yielding else None
        op = np.array([o['p'] for o in obstacles]).reshape(-1,2)
        ov = np.array([o['v'] for o in obstacles]).reshape(-1,2)
        radii = np.array([o['r'] for o in obstacles])
        apf = approach_tangent(pose, previous, obstacles, target)
        self.last_apf = apf
        for k,tau in enumerate(times):
            goal = target+velocity*tau
            # Route lookahead prevents cutting straight through furniture.
            look_s = np.minimum(arcs[-1], s0+cruise*tau+(c.corner_lookahead if wide is not None else (c.reference_lookahead if comfort else .7)))
            look = np.array([np.interp(look_s,arcs,path[:,j]) for j in range(2)])
            route_heading = np.arctan2(look[1]-states[:,1], look[0]-states[:,0])
            target_heading = np.arctan2(goal[1]-states[:,1],goal[0]-states[:,0])
            heading = route_heading+(0. if detour or wide is not None else .25)*wrap(target_heading-route_heading)
            if lost:heading = target_heading
            if lost and search is not None:heading=search['heading']
            heading += apf*np.exp(-tau)
            if retreat:heading=np.arctan2(recovery.heading[1],recovery.heading[0])
            if yielding:heading=pose[2]
            want_w = np.clip(2.*wrap(heading-states[:,2])+offsets, -c.max_w,c.max_w)
            if holding and not lost:want_w=np.zeros(count)
            if lost and search is not None:
                want_w=np.zeros(count) if search['expired'] else np.clip(want_w,-.3,.3)
            if scanning and not lost:
                scan_offset=recovery.scan_direction*.60*np.sin(2*np.pi*(self.calls*c.step-recovery.since)/recovery.tuning.scan_s)
                want_w=np.clip(2.*wrap(target_heading+scan_offset-states[:,2]),-.3,.3)
            desired_v = np.minimum(speeds, np.maximum(0., .8*(np.linalg.norm(goal-states[:,:2],axis=1)-getattr(c,'target_stop_gap',1.25))))
            if retreat:
                left=np.maximum(0.,remaining-(pose[:2]-states[:,:2])@recovery.heading)
                desired_v=np.maximum(speeds,-1.2*left)
            if yielding:
                left=np.maximum(0.,yielding['distance']-(states[:,:2]-pose[:2])@yielding['heading'])
                desired_v=np.minimum(speeds,1.2*left)
            old_controls=controls.copy()
            controls[:,0] += np.clip(desired_v-controls[:,0],-c.accel*c.step,c.accel*c.step)
            controls[:,1] += np.clip(want_w-controls[:,1],-c.yaw_accel*c.step,c.yaw_accel*c.step)
            if k==0:first=controls.copy()
            old=states[:,:2].copy()
            turn=controls[:,1]*c.step
            distance=controls[:,0]*c.step*np.sinc(turn/(2*np.pi))
            states[:,0]+=distance*np.cos(states[:,2]+turn/2)
            states[:,1]+=distance*np.sin(states[:,2]+turn/2)
            states[:,2]+=turn
            # Static grid clearance at start/middle/end of each swept chord.
            # Arc deviation at these bounds < 0.001 m.
            midpoint = (old + states[:,:2])*.5
            points = np.stack((old, midpoint, states[:,:2]),axis=1)
            clearance = np.min(local_map.query(points),axis=1)-c.radius-.001
            unknown_clear = np.min(local_map.query(points,unknown=True),axis=1)-c.radius-.001
            # Require observed free space over at least the current braking
            # time plus one control period. More distant unknown adds cost.
            stationary=np.linalg.norm(states[:,:2]-pose[:2],axis=1)<1e-8
            if retreat or detour or yielding or tau <= max(.5,abs(previous[0])/c.accel+c.step):
                safe &= (unknown_clear >= 0.) | stationary
                reasons[0] &= (unknown_clear >= 0.) | stationary
            static_safe=clearance>=c.margin
            if yielding:
                rear_clear=np.min(rear_cell_clearance(points,yielding['rear_centers'],local_map.res,c.radius),axis=1)
                other_clear=np.min(local_map.query(points,field=yielding['other_field']),axis=1)-c.radius-.001
                static_safe=(other_clear>=c.margin)&(rear_clear>=rear_previous-1e-7)
                rear_previous=np.minimum(c.margin,rear_clear)
            reasons[1] &= static_safe
            safe &= static_safe
            score += 15*np.maximum(.15-unknown_clear,0.)**2
            if len(obstacles):
                d0=old[:,None,:]-(op+(tau-c.step)*ov)[None,:,:]
                d1=states[:,None,:2]-(op+tau*ov)[None,:,:]
                delta=d1-d0
                u=np.clip(-np.sum(d0*delta,axis=2)/np.maximum(np.sum(delta**2,axis=2),1e-12),0.,1.)
                all_person_clear=np.linalg.norm(d0+u[:,:,None]*delta,axis=2)-radii[None,:]-c.radius-.025*tau
                person_clear=np.min(all_person_clear,axis=1)
                limits=np.where(yielding['rear_tracks'],yielding['track_floor'],c.margin) if yielding else c.margin
                person_safe=np.all(all_person_clear>=limits,axis=1)
                if yielding and yielding['rear_tracks'].any():
                    rear_clear=np.min(all_person_clear[:,yielding['rear_tracks']],axis=1)
                    person_safe &= rear_clear>=rear_track_previous-1e-7
                    rear_track_previous=np.minimum(c.margin,rear_clear)
                reasons[2] &= person_safe
                safe &= person_safe
                clearance=np.minimum(clearance,person_clear)
            minimum=np.minimum(minimum,clearance)
            angle=wrap(np.arctan2(goal[1]-states[:,1],goal[0]-states[:,0])-states[:,2])
            if tau<=.6:near_fov &= np.abs(angle)<=(c.camera_half_angle if detour or wide is not None else c.central_half_angle)
            gap=np.linalg.norm(goal-states[:,:2],axis=1)
            safe &= gap >= getattr(c,'target_hard_gap',.8)
            route_weight=35. if retreat or yielding else (60. if detour else (30. if wide is not None else 6.))
            gap_weight=0. if retreat else (.25 if detour else 2.)
            score += route_weight*np.sum((states[:,:2]-refs[k])**2,axis=1)+gap_weight*(gap-c.desired_gap)**2
            score += (1. if wide is not None else 4.)*angle**2+(15. if detour or wide is not None else 60.)*np.maximum(np.abs(angle)-c.central_half_angle,0.)**2
            score += 2*np.maximum(.35-clearance,0.)**2 + .15*controls[:,1]**2
            score += .25*np.sum((controls-old_controls)**2,axis=1)
            if wide is not None:
                moving=np.clip((np.abs(controls[:,0])-.04)/.08,0.,1.)
                excess=np.maximum(np.abs(controls[:,1])-np.abs(controls[:,0])/c.corner_radius,0.)
                score += 6.*moving*excess**2
            if comfort:score += c.yaw_change_weight*(controls[:,1]-old_controls[:,1])**2
            if comfort and c.preferred_turn_radius and not (lost or retreat or yielding or holding or scanning):
                # Soft comfort preference, never a post-MPC velocity filter.
                # Stopped search/turning remains possible in a tight corridor.
                moving=np.clip((np.abs(controls[:,0])-.04)/.08,0.,1.)
                excess=np.maximum(np.abs(controls[:,1])-np.abs(controls[:,0])/c.preferred_turn_radius,0.)
                score += c.curvature_weight*moving*excess**2
            if not retreat:score += 4*np.maximum(-controls[:,0],0.)**2
        # A detour can briefly move the target out of the central image area.
        # Actual lost observations still trigger zero translation above.
        feasible=safe if detour else safe&near_fov
        if not feasible.any():feasible=safe
        self.last_candidates=dict(safe=int(np.sum(safe)),feasible=int(np.sum(feasible)),
            forward=int(np.sum(feasible&(first[:,0]>.01))),
            reasons=np.sum(reasons,axis=1).tolist())
        if not feasible.any():
            self.last_infeasible=True
            self.fallbacks += 1
            self.failure_times.append(self.calls*.1)
            self.failure_reasons.append(np.sum(reasons,axis=1).tolist())
            # Report infeasibility; brake with command limits. Not a proof of
            # safety when all sampled trajectories fail.
            self.calls+=1;self.times.append(time.perf_counter()-start)
            return previous+np.clip(-previous,[-c.accel*c.step,-c.yaw_accel*c.step],
                                    [c.accel*c.step,c.yaw_accel*c.step])
        else:
            chosen=int(np.argmin(np.where(feasible,score,np.inf)))
        self.calls+=1;self.times.append(time.perf_counter()-start)
        return first[chosen]
