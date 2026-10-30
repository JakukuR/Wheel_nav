"""One sampled differential MPC for tracker following and finite approach goals."""
from dataclasses import replace
import numpy as np
from .common import Tuning, wrap, rotation, ROBOT_RADIUS_M
from .target_reference import bezier_reference
from .detour import DetourPlanner


def approach_tangent(pose, command, tracks, target, radius=ROBOT_RADIUS_M):
    result=0.; robot_v=rotation(pose[2])@np.array([command[0],0.])
    direction=target-pose[:2]
    for o in tracks:
        delta=pose[:2]-o['p']; distance=np.linalg.norm(delta)
        gap=max(.08,distance-o['r']-radius)
        if gap>=1.2: continue
        normal=delta/max(distance,1e-9)
        approaching=max(0.,-np.dot(normal,robot_v-o['v']))
        tangent=np.array([-normal[1],normal[0]])
        if np.dot(tangent,direction)<0:tangent=-tangent
        cross=np.cos(pose[2])*tangent[1]-np.sin(pose[2])*tangent[0]
        result+=.08*approaching*(1/gap-1/1.2)*cross
    return float(np.clip(result,-.18,.18))


def seed_command(measured, last, last_at, now, *, max_age=.3, max_dv=.15, max_dw=.3):
    """Rollout start: the last issued command while the chassis is tracking it.

    Seeding from the measured wheel twist under chassis lag re-starts every plan
    below the commanded speed (each step only +accel*dt above a lagging value).
    A large mismatch (blocked, slipping, external stop) falls back to measured.
    """
    measured=np.asarray(measured,dtype=float)
    if last is None or last_at is None or not 0<=now-last_at<=max_age:return measured
    last=np.asarray(last,dtype=float)
    if abs(last[0]-measured[0])>max_dv or abs(last[1]-measured[1])>max_dw:return measured
    return last


def limit_step(command, seed, dt, c):
    """First command change bounded by the real control interval, not the 0.1 s model step."""
    dt=float(np.clip(dt,.02,c.step))
    seed=np.asarray(seed,dtype=float)
    return np.array([np.clip(command[0],seed[0]-c.accel*dt,seed[0]+c.accel*dt),
                     np.clip(command[1],seed[1]-c.yaw_accel*dt,seed[1]+c.yaw_accel*dt)])


def chassis_yaw(w, c):
    """Yaw rate the chassis settles to for a commanded w (deadband model)."""
    w=np.asarray(w,dtype=float);a=np.abs(w);lo,hi=c.chassis_deadband_lo,c.chassis_deadband_hi
    floor=.2*lo
    eff=np.where(a<lo,.2*a,np.where(a<hi,floor+(a-lo)/max(hi-lo,1e-9)*(hi-floor),a))
    return np.sign(w)*eff


class HybridFollower:
    def __init__(self, config=Tuning()):
        self.c=config
        self.last_infeasible=False
        self.last_candidates={}
        self.last_reference_path=np.empty((0,2))
        self.detour=DetourPlanner()
        self.stuck_cycles=0  # consecutive plans without a feasible forward candidate
        self.trail=[]  # robot positions >= 5 cm apart (swept, so known free), newest last
        self.backed=0.  # distance backed up in the current stuck episode
        self.last_pose=None;self.reversing=False

    def reset(self):
        """Forget detour side/look state between tasks."""
        self.detour=DetourPlanner();self.stuck_cycles=0;self.trail=[];self.backed=0.;self.last_pose=None;self.reversing=False

    def _remember(self, pose):
        p=np.asarray(pose[:2],dtype=float)
        if self.reversing and self.last_pose is not None:
            self.backed+=float(np.linalg.norm(p-self.last_pose))
        self.last_pose=p.copy()
        if self.reversing:
            self.reversing=False;return
        if not self.trail or np.linalg.norm(p-self.trail[-1])>=.05:
            self.trail.append(p.copy())
            # Keep about 3 m of path: enough to back up and long-gone space is stale.
            while len(self.trail)>60:self.trail.pop(0)

    def _backup(self, pose, previous, local_map, c):
        """Reverse along the swept path while stuck; None when not possible."""
        p=np.asarray(pose[:2],dtype=float)
        if self.backed>=c.backup_max_m or len(self.trail)<3:return None
        # Aim 0.3 m back along the trail (the entry nearest the robot is its own spot).
        back=[q for q in reversed(self.trail[:-1]) if np.linalg.norm(q-p)>=.3]
        if not back:return None
        goal=back[0]
        # Known obstacles only: the swept path was free; unknown is just unseen.
        points=p+np.linspace(0.,1.,8)[:,None]*(goal-p)
        # Leaving a spot that is already tight is allowed as long as the path
        # never gets closer to known obstacles than the robot is now.
        clear=local_map.query(points)
        if np.min(clear[1:])<min(c.radius+.02,clear[0]):return None
        rear=pose[2]+np.pi
        err=float(wrap(np.arctan2(goal[1]-p[1],goal[0]-p[0])-rear))
        w=float(np.clip(1.5*err,-.5,.5))
        w=float(np.clip(w,previous[1]-c.yaw_accel*c.step,previous[1]+c.yaw_accel*c.step))
        v=-c.backup_speed*max(0.,np.cos(err))
        self.reversing=True
        self.last_candidates=dict(safe=1,feasible=1,forward=0,backing=True,backed_m=round(self.backed,2),
                                  detour=dict(self.detour.last or {}))
        self.last_reference_path=np.vstack((p,goal))
        return np.array([v,w])

    def recover(self, pose, previous, target, local_map, c):
        """Never forward: back out along the swept path, else turn to the target in
        place when the footprint is clear, else stop."""
        back=self._backup(pose,previous,local_map,c)
        if back is not None:return back
        self.last_candidates=dict(safe=1,feasible=1,forward=0,recovery=True,backed_m=round(self.backed,2))
        err=float(wrap(np.arctan2(target[1]-pose[1],target[0]-pose[0])-pose[2]))
        clear=float(np.min(local_map.query(np.asarray(pose[:2],dtype=float).reshape(1,2))))-c.radius
        if abs(err)<=.15 or clear<0.:return np.zeros(2)
        w=float(np.clip(1.5*err,-.6,.6))
        return np.array([0.,float(np.clip(w,previous[1]-c.yaw_accel*c.step,previous[1]+c.yaw_accel*c.step))])

    def command(self, pose, previous, target, velocity, history, local_map, obstacles,
                status, policy=None, policy_time=None, *, reference_path=None, face_point=None, measured=None,
                recovery=False):
        c=self.c
        self._remember(pose)
        searching=status in ('search_last_seen','search_heading')
        if searching:
            arrival=policy.search_arrival_m if policy is not None else .35
            c=replace(c,desired_gap=arrival,target_stop_gap=arrival,target_hard_gap=0.,
                      target_clearance_max=0.,max_v=min(c.max_v,.20))
        self.last_infeasible=False
        self.last_candidates={}
        self.last_reference_path=np.empty((0,2))
        if target is None or status in ('lost','binding','holding'):
            return np.zeros(2)
        if recovery:
            # The newest depth frame shows too little (e.g. the nose at a desk
            # inside the camera's minimum range): do not drive forward on it.
            return self.recover(pose,previous,target,local_map,c)
        path=bezier_reference(history) if reference_path is None else np.asarray(reference_path,dtype=float).reshape(-1,2)
        detouring=False
        if reference_path is None and not searching and self.detour is not None:
            # Blocked straight route: steer the sampled family around an obstacle edge.
            detour=self.detour.plan(pose,np.asarray(target,dtype=float),local_map,c,reference=path)
            if detour is not None:path=detour;detouring=True
            if detour is not None and self.detour.last.get('looking') and self.stuck_cycles>=3:
                back=self._backup(pose,previous,local_map,c)
                if back is not None:return back
            if detour is not None and self.detour.last.get('looking') and self.stuck_cycles>=3:
                # Turning to observe an unseen flank is an explicit in-place rotation
                # (bounded rate and yaw acceleration), not a trade-off against the
                # keep-the-person-centred cost, which otherwise stalls it part way.
                err=wrap(np.arctan2(detour[1][1]-pose[1],detour[1][0]-pose[0])-pose[2])
                if not self.detour.look_reached(abs(err)<=.15):
                    w=float(np.clip(1.5*err,-.6,.6))
                    w=float(np.clip(w,previous[1]-c.yaw_accel*c.step,previous[1]+c.yaw_accel*c.step))
                    clear=np.min(local_map.query(np.asarray(pose[:2],dtype=float).reshape(1,2)))-c.radius
                    self.last_candidates=dict(safe=1,feasible=1,forward=0,looking=True,detour=dict(self.detour.last))
                    self.last_reference_path=np.asarray(detour)
                    if clear>=0.:return np.array([0.,w])
        # The backing budget lasts for a whole stuck episode (no forward option).
        if (not detouring or not self.detour.last.get('looking')) and self.stuck_cycles==0:self.backed=0.
        if not len(path):path=np.asarray(target,dtype=float).reshape(1,2)
        if np.linalg.norm(path[-1]-target)>.01:path=np.vstack((path,target))
        if len(path)<2:path=np.vstack((pose[:2],path))
        edge=np.diff(path,axis=0)
        fraction=np.clip(np.sum((pose[:2]-path[:-1])*edge,axis=1)/np.maximum(np.sum(edge*edge,axis=1),1e-12),0.,1.)
        nearest=int(np.argmin(np.linalg.norm(path[:-1]+fraction[:,None]*edge-pose[:2],axis=1)))
        if nearest==0 and fraction[0]<=1e-6 and np.linalg.norm(path[0]-pose[:2])>.03:
            path=np.vstack((pose[:2],path))
        self.last_reference_path=path.copy()
        segments=np.diff(path,axis=0);lengths=np.linalg.norm(segments,axis=1)
        arcs=np.r_[0.,np.cumsum(lengths)]
        alpha=np.clip(np.sum((pose[:2]-path[:-1])*segments,axis=1)/np.maximum(lengths**2,1e-12),0.,1.)
        idx=np.argmin(np.linalg.norm(path[:-1]+alpha[:,None]*segments-pose[:2],axis=1))
        s0=arcs[idx]+alpha[idx]*lengths[idx]
        target_speed=np.linalg.norm(velocity)
        # Person off the image centre at close range (sidestep, passing the robot):
        # follow the person rather than their footprints and slow down so the turn
        # keeps up. Off on detours and finite goals, which own their reference.
        lateral=0.
        if reference_path is None and not searching and not detouring:
            bearing=abs(float(wrap(np.arctan2(target[1]-pose[1],target[0]-pose[0])-pose[2])))
            lateral=float(np.clip((bearing-c.lateral_bearing_start)/max(c.fov_soft_angle-c.lateral_bearing_start,1e-6),0.,1.))
        cap=c.max_v
        if reference_path is None and not searching:
            # Turn first: the point steered to is the detour waypoint, else the person.
            aim=path[1] if detouring and len(path)>2 and not self.detour.last.get('looking') else target
            off=abs(float(wrap(np.arctan2(aim[1]-pose[1],aim[0]-pose[0])-pose[2])))
            cap=c.max_v*float(np.clip((c.turn_first_angle-off)/max(c.turn_first_angle-c.turn_slow_angle,1e-6),0.,1.))
        # The trail's arc length overstates the remaining gap once the person curves.
        remaining=min(arcs[-1]-s0,float(np.linalg.norm(target-pose[:2])))
        cruise=np.clip(target_speed+.65*(remaining-c.desired_gap),0.,min(c.max_v*(1.-c.lateral_slowdown*lateral),cap))
        times=np.arange(1,c.horizon+1)*c.step
        ref_s=np.maximum(s0,np.minimum(s0+cruise*times,arcs[-1]+target_speed*times-c.desired_gap))
        refs=np.column_stack([np.interp(ref_s,arcs,path[:,j]) for j in range(2)])
        refs+=np.maximum(ref_s-arcs[-1],0.)[:,None]*velocity/max(target_speed,1e-9)
        speeds,offsets=np.meshgrid(np.r_[-.25,-.15,-.075,np.linspace(0.,c.max_v,10)],np.linspace(-1.2,1.2,25))
        speeds,offsets=speeds.ravel(),offsets.ravel()
        if lateral>0.:speeds=np.minimum(speeds,c.max_v*(1.-c.lateral_slowdown*lateral))
        speeds=np.minimum(speeds,cap)
        if (not searching and c.target_clearance_max>0 and np.linalg.norm(velocity)<.05 and
                c.target_hard_gap+.01<=np.linalg.norm(target-pose[:2])<=c.radius+c.target_radius+c.target_clearance_max):
            speeds[:]=0.
        search_heading=None
        if status=='search_heading' and policy is not None and policy.search_heading is not None:
            phase=0. if policy.search_started is None else max(0.,policy_time-policy.search_started)
            search_heading=policy.search_heading+.60*np.sin(2*np.pi*phase/8.)
        policy_stop=policy is not None and not policy.translation_allowed(policy_time)
        if policy_stop:speeds[:]=0.
        count=len(speeds); states=np.tile(pose,(count,1));controls=np.tile(previous,(count,1))
        # Actual chassis twist per rollout, starting from the measured one.
        actual=np.tile(np.asarray(previous if measured is None else measured,dtype=float),(count,1))
        score=np.zeros(count);safe=np.ones(count,dtype=bool);reasons=np.ones((3,count),dtype=bool)
        near_fov=np.ones(count,dtype=bool); first=None
        op=np.array([o['p'] for o in obstacles]).reshape(-1,2)
        ov=np.array([o['v'] for o in obstacles]).reshape(-1,2)
        radii=np.array([o['r'] for o in obstacles])
        apf=approach_tangent(pose,previous,obstacles,target,c.radius)
        for k,tau in enumerate(times):
            goal=target+velocity*tau
            look_s=min(arcs[-1],s0+cruise*tau+.7)
            look=np.array([np.interp(look_s,arcs,path[:,j]) for j in range(2)])
            route_heading=np.arctan2(look[1]-states[:,1],look[0]-states[:,0])
            target_heading=np.arctan2(goal[1]-states[:,1],goal[0]-states[:,0])
            heading=route_heading+(.25+.75*lateral)*wrap(target_heading-route_heading)
            if reference_path is not None:
                heading=route_heading
                if face_point is not None:
                    face=np.arctan2(face_point[1]-states[:,1],face_point[0]-states[:,0])
                    heading=np.where(np.linalg.norm(goal-states[:,:2],axis=1)<.12,face,heading)
            heading+=apf*np.exp(-tau)
            want_w=np.clip(2.*wrap(heading-states[:,2])+offsets,-c.max_w,c.max_w)
            if search_heading is not None:want_w=np.clip(2.*wrap(search_heading-states[:,2])+.25*offsets,-.3,.3)
            if policy_stop and not policy.active_search(policy_time):want_w[:]=0.
            desired_v=np.minimum(speeds,np.maximum(0.,.8*(np.linalg.norm(goal-states[:,:2],axis=1)-c.target_stop_gap)))
            old_controls=controls.copy()
            controls[:,0]+=np.clip(desired_v-controls[:,0],-c.accel*c.step,c.accel*c.step)
            controls[:,1]+=np.clip(want_w-controls[:,1],-c.yaw_accel*c.step,c.yaw_accel*c.step)
            if policy_stop:
                controls[:,0]=0.
                if not policy.active_search(policy_time):controls[:,1]=0.
            if k==0:first=controls.copy()
            if c.chassis_tau_w>0:
                actual[:,1]+=(chassis_yaw(controls[:,1],c)-actual[:,1])*min(1.,c.step/c.chassis_tau_w)
                actual[:,0]+=(controls[:,0]-actual[:,0])*min(1.,c.step/max(c.chassis_tau_v,1e-6))
            else:
                actual[:]=controls
            old=states[:,:2].copy(); turn=actual[:,1]*c.step
            distance=actual[:,0]*c.step*np.sinc(turn/(2*np.pi))
            states[:,0]+=distance*np.cos(states[:,2]+turn/2)
            states[:,1]+=distance*np.sin(states[:,2]+turn/2);states[:,2]+=turn
            points=np.stack((old,(old+states[:,:2])*.5,states[:,:2]),axis=1)
            clearance=np.min(local_map.query(points),axis=1)-c.radius-.001
            unknown=np.min(local_map.query(points,unknown=True),axis=1)-c.radius-.001
            stationary=np.linalg.norm(states[:,:2]-pose[:2],axis=1)<1e-8
            # Same as person follow/approach for every mode: only the near horizon
            # (stopping distance) must lie in known free space.
            if tau<=max(.5,abs(previous[0])/c.accel+c.step):
                allowed=(unknown>=0.)|stationary;safe&=allowed;reasons[0]&=allowed
            static_safe=clearance>=0.;safe&=static_safe;reasons[1]&=static_safe
            score+=c.margin_weight*np.maximum(c.margin-clearance,0.)**2
            score+=15*np.maximum(.15-unknown,0.)**2
            if len(obstacles):
                d0=old[:,None,:]-(op+(tau-c.step)*ov)[None,:,:]
                d1=states[:,None,:2]-(op+tau*ov)[None,:,:];delta=d1-d0
                u=np.clip(-np.sum(d0*delta,axis=2)/np.maximum(np.sum(delta**2,axis=2),1e-12),0.,1.)
                person_clear=np.min(np.linalg.norm(d0+u[:,:,None]*delta,axis=2)-radii[None,:]-c.radius-.025*tau,axis=1)
                person_safe=person_clear>=c.margin;safe&=person_safe;reasons[2]&=person_safe
                clearance=np.minimum(clearance,person_clear)
            bearing_goal=goal if face_point is None else face_point
            angle=wrap(np.arctan2(bearing_goal[1]-states[:,1],bearing_goal[0]-states[:,0])-states[:,2])
            if search_heading is not None:angle=wrap(search_heading-states[:,2])
            # Going round an obstacle may briefly face away; the FOV stays a cost then.
            visible=c.camera_half_angle-c.fov_edge_margin
            if tau<=.6 and not detouring:near_fov&=np.abs(angle)<=visible
            gap=np.linalg.norm(goal-states[:,:2],axis=1);safe&=gap>=c.target_hard_gap
            score+=6*(1.-.75*lateral)*np.sum((states[:,:2]-refs[k])**2,axis=1)+2*(gap-c.desired_gap)**2
            score+=c.centering_weight*angle**2+60*np.maximum(np.abs(angle)-c.fov_soft_angle,0.)**2
            score+=2*np.maximum(.35-clearance,0.)**2+.15*controls[:,1]**2
            score+=.25*np.sum((controls-old_controls)**2,axis=1)+4*np.maximum(-controls[:,0],0.)**2
        feasible=safe&near_fov
        if not feasible.any():feasible=safe
        self.stuck_cycles=self.stuck_cycles+1 if not np.any(feasible&(first[:,0]>.01)) else 0
        self.last_candidates=dict(safe=int(safe.sum()),feasible=int(feasible.sum()),
            forward=int(np.sum(feasible&(first[:,0]>.01))),reasons=np.sum(reasons,axis=1).tolist(),
            forward_total=int(np.sum(first[:,0]>.01)),forward_known=int(np.sum((first[:,0]>.01)&reasons[0])),
            forward_static=int(np.sum((first[:,0]>.01)&reasons[1])),forward_people=int(np.sum((first[:,0]>.01)&reasons[2])))
        if self.detour is not None and self.detour.last and self.detour.last.get('active'):
            self.last_candidates['detour']=dict(self.detour.last)
        if not feasible.any():
            self.last_infeasible=True
            if self.stuck_cycles>=3 and reference_path is None and not searching:
                # Nothing safe for three plans (e.g. inflated obstacles round the
                # body): back out along the swept path instead of waiting.
                back=self._backup(pose,previous,local_map,c)
                if back is not None:
                    self.last_infeasible=False
                    return back
            return np.zeros(2)
        return first[int(np.argmin(np.where(feasible,score,np.inf)))]
