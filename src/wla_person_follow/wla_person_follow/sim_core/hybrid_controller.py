"""One sampled differential MPC for tracker following and finite approach goals."""
from dataclasses import replace
import numpy as np
from .common import Tuning, wrap, rotation, ROBOT_RADIUS_M
from .target_reference import bezier_reference


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


class HybridFollower:
    def __init__(self, config=Tuning()):
        self.c=config
        self.last_infeasible=False
        self.last_candidates={}
        self.last_reference_path=np.empty((0,2))

    def command(self, pose, previous, target, velocity, history, local_map, obstacles,
                status, policy=None, policy_time=None, *, reference_path=None, face_point=None):
        c=self.c
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
        path=bezier_reference(history) if reference_path is None else np.asarray(reference_path,dtype=float).reshape(-1,2)
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
        cruise=np.clip(target_speed+.65*(arcs[-1]-s0-c.desired_gap),0.,c.max_v)
        times=np.arange(1,c.horizon+1)*c.step
        ref_s=np.maximum(s0,np.minimum(s0+cruise*times,arcs[-1]+target_speed*times-c.desired_gap))
        refs=np.column_stack([np.interp(ref_s,arcs,path[:,j]) for j in range(2)])
        refs+=np.maximum(ref_s-arcs[-1],0.)[:,None]*velocity/max(target_speed,1e-9)
        speeds,offsets=np.meshgrid(np.r_[-.25,-.15,-.075,np.linspace(0.,c.max_v,10)],np.linspace(-1.2,1.2,25))
        speeds,offsets=speeds.ravel(),offsets.ravel()
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
            heading=route_heading+.25*wrap(target_heading-route_heading)
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
            old=states[:,:2].copy(); turn=controls[:,1]*c.step
            distance=controls[:,0]*c.step*np.sinc(turn/(2*np.pi))
            states[:,0]+=distance*np.cos(states[:,2]+turn/2)
            states[:,1]+=distance*np.sin(states[:,2]+turn/2);states[:,2]+=turn
            points=np.stack((old,(old+states[:,:2])*.5,states[:,:2]),axis=1)
            clearance=np.min(local_map.query(points),axis=1)-c.radius-.001
            unknown=np.min(local_map.query(points,unknown=True),axis=1)-c.radius-.001
            stationary=np.linalg.norm(states[:,:2]-pose[:2],axis=1)<1e-8
            if reference_path is not None or tau<=max(.5,abs(previous[0])/c.accel+c.step):
                allowed=(unknown>=0.)|stationary;safe&=allowed;reasons[0]&=allowed
            static_safe=clearance>=c.margin;safe&=static_safe;reasons[1]&=static_safe
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
            if tau<=.6:near_fov&=np.abs(angle)<=c.central_half_angle
            gap=np.linalg.norm(goal-states[:,:2],axis=1);safe&=gap>=c.target_hard_gap
            score+=6*np.sum((states[:,:2]-refs[k])**2,axis=1)+2*(gap-c.desired_gap)**2
            score+=4*angle**2+60*np.maximum(np.abs(angle)-c.central_half_angle,0.)**2
            score+=2*np.maximum(.35-clearance,0.)**2+.15*controls[:,1]**2
            score+=.25*np.sum((controls-old_controls)**2,axis=1)+4*np.maximum(-controls[:,0],0.)**2
        feasible=safe&near_fov
        if not feasible.any():feasible=safe
        self.last_candidates=dict(safe=int(safe.sum()),feasible=int(feasible.sum()),
            forward=int(np.sum(feasible&(first[:,0]>.01))),reasons=np.sum(reasons,axis=1).tolist(),
            forward_total=int(np.sum(first[:,0]>.01)),forward_known=int(np.sum((first[:,0]>.01)&reasons[0])),
            forward_static=int(np.sum((first[:,0]>.01)&reasons[1])),forward_people=int(np.sum((first[:,0]>.01)&reasons[2])))
        if not feasible.any():
            self.last_infeasible=True
            return np.zeros(2)
        return first[int(np.argmin(np.where(feasible,score,np.inf)))]
