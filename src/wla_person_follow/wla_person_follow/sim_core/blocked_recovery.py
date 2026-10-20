"""Wait, bounded retreat, then local replan using observed odom data only.

No scene geometry, target truth, or future routes enter this module. Unknown
cells are blocked for recovery. All returned routes still go through MPC.
"""
from dataclasses import dataclass
import heapq
import numpy as np


@dataclass(frozen=True)
class RecoveryTuning:
    wait_s: float = 2.
    retreat_m: float = .35
    reverse_speed: float = .15
    attempt_timeout_s: float = 8.
    route_timeout_s: float = 18.
    retry_s: float = 1.
    scan_s: float = 6.
    recovery_timeout_s: float = 30.
    prediction_s: float = 1.5
    search_s: float = 8.
    search_amplitude: float = .30


def route_in_known_space(pose,target,mapping,tracks,c,allow_partial=False):
    """8-connected A* to a standoff point. No diagonal corner cutting.

The .08 m cell-edge reserve is > half a 10 cm cell diagonal. A route is a
reference, not a motion authorization: MPC checks the swept differential arc.
"""
    xy=(mapping.cells+.5)*mapping.res
    allowed=(mapping.query(xy)>=c.radius+c.margin+.08)&(mapping.query(xy,unknown=True)>=c.radius+.08)
    for o in tracks:
        allowed &= np.linalg.norm(xy-o['p'],axis=-1)>=o['r']+c.radius+c.margin+.08
    start=tuple(np.floor(pose[:2]/mapping.res).astype(int)-mapping.lower)
    if not all(0<=i<mapping.n for i in start) or not allowed[start]:return None
    toward=target-pose[:2];dist=np.linalg.norm(toward)
    if dist<c.desired_gap+.1:return None
    preferred=target-c.desired_gap*toward/dist
    # Endpoint needs to remain in the target's following-distance band.
    gap=np.linalg.norm(xy-target,axis=-1)
    goals=allowed&(gap>=c.desired_gap-.12)&(gap<=c.desired_gap+.12)
    goals &= np.linalg.norm(xy-preferred,axis=-1)<1.4
    goals &= np.linalg.norm(xy-pose[:2],axis=-1)>.3
    goal_xy=xy[goals] if goals.any() else preferred[None,:]
    heuristic=lambda a:float(np.min(np.linalg.norm(goal_xy-xy[a],axis=1)))
    queue=[(heuristic(start),0.,start)];cost={start:0.};parent={};found=None
    steps=[(dx,dy) for dx in (-1,0,1) for dy in (-1,0,1) if dx or dy]
    while queue:
        _,g,a=heapq.heappop(queue)
        if g>cost[a]+1e-9:continue
        if goals[a]:found=a;break
        for dx,dy in steps:
            b=(a[0]+dx,a[1]+dy)
            if not all(0<=i<mapping.n for i in b) or not allowed[b]:continue
            if dx and dy and not (allowed[a[0]+dx,a[1]] and allowed[a[0],a[1]+dy]):continue
            new=g+mapping.res*np.hypot(dx,dy)
            if new>=cost.get(b,np.inf):continue
            cost[b]=new;parent[b]=a;heapq.heappush(queue,(new+heuristic(b),new,b))
    if found is None:
        if not allow_partial:return None
        # A forward RGB-D camera may not yet see a complete bypass. Move only
        # to a reachable, known-free observation position; never bridge unknown.
        candidates=[a for a in cost if np.linalg.norm(xy[a]-pose[:2])>.25
                    and np.linalg.norm(xy[a]-target)<dist-.15]
        if not candidates:return None
        found=min(candidates,key=lambda a:np.linalg.norm(xy[a]-preferred)+.25*cost[a])
    indices=[found]
    while indices[-1]!=start:indices.append(parent[indices[-1]])
    path=np.array([pose[:2],*[xy[a] for a in indices[::-1][1:]]])
    # The last segment preserves the existing MPC standoff convention.
    return np.vstack((path,target))


class BlockedRecovery:
    def __init__(self,config,tuning=RecoveryTuning()):
        self.c=config;self.tuning=tuning;self.state='follow';self.reason=''
        self.since=0.;self.anchor=None;self.retreat_start=None;self.heading=None
        self.route=None;self.events=[];self.attempts=0;self.last_retry=-np.inf
        self.last_goal=None;self.last_infeasible=False;self.samples=[]
        self.scan_direction=1.
        self.stall_since=None;self.episode_since=None
        self.last_seen=None;self.search=None

    def update_search(self,pose,target,velocity,status,t):
        """Bounded bearing search from the last real observation, never truth."""
        self.search=None
        if status=='visible' and target is not None:
            self.last_seen=(t,target.copy(),velocity.copy(),float(pose[2]))
        if status!='lost' or self.last_seen is None:return
        seen,point,speed,heading=self.last_seen
        age=max(0.,t-seen)
        predicted=point+speed*min(age,self.tuning.prediction_s)
        delta=predicted-pose[:2]
        bearing=float(np.arctan2(delta[1],delta[0])) if np.linalg.norm(delta)>.1 else heading
        # First turn toward the predicted bearing, then sweep around it.
        phase=max(0.,age-1.6)
        cross=delta[0]*speed[1]-delta[1]*speed[0]
        sign=-1. if cross<0 else 1.
        offset=sign*self.tuning.search_amplitude*np.sin(2*np.pi*phase/4.)
        self.search=dict(point=predicted,heading=bearing+offset,
                         expired=age>self.tuning.search_s,age=age)

    def transition(self,state,t,reason):
        if state in ('retreat','scan','detour') and self.episode_since is None:self.episode_since=t
        if state=='follow':self.episode_since=None;self.samples=[]
        if state!=self.state:
            self.events.append(dict(t=round(t,3),state=state,reason=reason))
            self.state=state;self.since=t
        self.reason=reason

    def _path_samples(self,pose,target,route=None):
        """The local route the controller is actually trying to reach.

        Follow measured target history or the active detour, projecting onto
        segments rather than skipping a corner to the nearest vertex. Only
        the next 1.4 m is a supervisory blockage test; MPC validates motion.
        """
        pose=np.asarray(pose);start=pose[:2].copy()
        if route is not None and len(route)>=2:
            raw=np.asarray(route,dtype=float)[:,:2]
            if np.linalg.norm(raw[-1]-target)>.01:raw=np.vstack((raw,target))
            segments=np.diff(raw,axis=0);length2=np.sum(segments**2,axis=1)
            u=np.clip(np.sum((start-raw[:-1])*segments,axis=1)/np.maximum(length2,1e-12),0.,1.)
            projected=raw[:-1]+u[:,None]*segments
            i=int(np.argmin(np.linalg.norm(projected-start,axis=1)))
            points=np.vstack((start,projected[i],raw[i+1:]))
        else:
            delta=np.asarray(target,dtype=float)-start;distance=np.linalg.norm(delta)
            if distance<1e-9:return np.empty((0,2))
            points=np.vstack((start,start+delta*min(1.4,distance)/distance))
        # Remove duplicate vertices and truncate by arc length.
        keep=np.r_[True,np.linalg.norm(np.diff(points,axis=0),axis=1)>1e-8]
        points=points[keep]
        if len(points)<2:return np.empty((0,2))
        seg=np.linalg.norm(np.diff(points,axis=0),axis=1);arc=np.r_[0.,np.cumsum(seg)]
        limit=min(1.4,arc[-1]);s=np.unique(np.r_[np.arange(0.,limit,.05),arc[arc<=limit],limit])
        if len(s)==0:return np.empty((0,2))
        return np.column_stack([np.interp(s,arc,points[:,j]) for j in range(2)])

    def forward_blocked(self,pose,mapping,tracks,target=None,route=None,speed=None):
        """Return true only when a measured obstacle intersects the travel path."""
        if target is None:target=np.asarray(pose[:2])+np.array([np.cos(pose[2]),np.sin(pose[2])])
        samples=self._path_samples(pose,target,route)
        if not len(samples):return False
        # Occupancy distance is measured to the robot footprint, so this gate
        # is about the path itself rather than all points in front of the body.
        if np.any(mapping.query(samples)<=self.c.radius+self.c.margin+.026):return True
        if not tracks:return False
        # Couple obstacle prediction to distance along the path.  A person
        # beside the path is ignored; a person crossing the path is a blocker.
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(samples,axis=0),axis=1))]
        times=arc/np.clip(self.c.max_v if speed is None else speed,.15,self.c.max_v)
        for o in tracks:
            relative=samples-(np.asarray(o['p'])+times[:,None]*np.asarray(o['v']))
            a=relative[:-1];delta=np.diff(relative,axis=0)
            u=np.clip(-np.sum(a*delta,axis=1)/np.maximum(np.sum(delta**2,axis=1),1e-12),0.,1.)
            distance=np.linalg.norm(a+u[:,None]*delta,axis=1)
            if np.any(distance<=o['r']+self.c.radius+self.c.margin+.025*times[1:]):return True
        return False

    def rear_clear(self,pose,mapping,tracks):
        heading=np.array([np.cos(pose[2]),np.sin(pose[2])])
        points=pose[:2]-np.linspace(0,self.tuning.retreat_m+.05,20)[:,None]*heading
        if np.any(mapping.query(points)<self.c.radius+self.c.margin+.01):return False
        if np.any(mapping.query(points,unknown=True)<self.c.radius+.02):return False
        for o in tracks:
            # Conservative moving-track sweep during a complete retreat.
            for tau in np.linspace(0,4.,17):
                if np.any(np.linalg.norm(points-(o['p']+tau*o['v']),axis=1)<o['r']+self.c.radius+self.c.margin+.025*tau):return False
        return True

    def update(self,pose,previous,target,velocity,mapping,tracks,status,t,history=None):
        c=self.c;r=self.tuning
        self.update_search(pose,target,velocity,status,t)
        if self.state in ('retreat','scan','detour') and self.episode_since is not None and t-self.episode_since>r.recovery_timeout_s:
            self.route=None;self.transition('hold',t,'recovery_timeout');return
        if target is None or status=='lost':
            if self.state=='hold':return
            if target is not None and self.state in ('detour','scan'):
                # Keep the local route while MPC stops translation and turns
                # back to the last target bearing. Never advance on stale truth.
                self.reason='target_lost_stop';return
            self.route=None;self.anchor=None
            self.transition('follow',t,'target_lost_stop');return
        gap=np.linalg.norm(target-pose[:2])
        demand=gap>c.desired_gap+.25 or np.linalg.norm(velocity)>.15
        active_route=self.route if self.state=='detour' else history
        blocked=self.forward_blocked(pose,mapping,tracks,target=target,route=active_route,
                                     speed=max(abs(previous[0]),np.linalg.norm(velocity),.15))
        if not demand:
            self.route=None;self.anchor=None
            self.transition('follow',t,'standoff_reached');return
        if self.state=='retreat':
            distance=(self.retreat_start-pose[:2])@self.heading
            if distance>=r.retreat_m-.04 and abs(previous[0])<.06:
                self.route=route_in_known_space(pose,target,mapping,tracks,c)
                self.last_goal=target.copy();self.last_retry=t
                points=(mapping.cells[mapping.occupied]+.5)*mapping.res-pose[:2]
                nearby=points[np.linalg.norm(points,axis=1)<2.]
                lateral=nearby@np.array([-np.sin(pose[2]),np.cos(pose[2])])
                self.scan_direction=-1. if len(lateral) and np.mean(lateral)>0 else 1.
                self.transition('detour' if self.route is not None else 'scan',t,
                                'known_route_found' if self.route is not None else 'no_known_route')
                self.stall_since=None
            elif t-self.since>=r.attempt_timeout_s:
                self.transition('hold',t,'retreat_timeout')
            return
        if self.state=='scan':
            if t-self.last_retry>=r.retry_s:
                self.last_retry=t;self.route=route_in_known_space(pose,target,mapping,tracks,c,allow_partial=t-self.since>=3.)
                if self.route is not None:
                    self.last_goal=target.copy();self.stall_since=None;self.transition('detour',t,'known_route_found');return
            if t-self.since>=r.scan_s:self.transition('hold',t,'no_known_route')
            return
        if self.state=='detour':
            if abs(previous[0])<.03:
                if self.stall_since is None:self.stall_since=t
                if t-self.stall_since>=1.5:
                    self.last_retry=t;self.transition('scan',t,'need_more_observation');return
            else:self.stall_since=None
            if t-self.since>r.route_timeout_s:
                self.route=None;self.transition('hold',t,'detour_timeout');return
            at_frontier=self.route is not None and np.linalg.norm(pose[:2]-self.route[-2])<.35
            if (at_frontier or np.linalg.norm(target-self.last_goal)>.4) and t-self.last_retry>=r.retry_s:
                self.route=route_in_known_space(pose,target,mapping,tracks,c,allow_partial=True)
                self.last_retry=t;self.last_goal=target.copy()
                if self.route is None:self.transition('scan',t,'need_more_observation')
            if self.last_infeasible:
                self.samples.append(t)
                self.samples=[a for a in self.samples if t-a<1.]
                if len(self.samples)>=8:
                    self.route=None;self.transition('hold',t,'detour_blocked')
            return
        if self.state=='hold':
            if not blocked:
                self.anchor=None;self.transition('follow',t,'obstacle_cleared');return
            if self.reason in ('detour_timeout','detour_blocked','retreat_timeout','recovery_timeout'):return
            if t-self.last_retry>=r.retry_s:
                self.last_retry=t
                self.route=route_in_known_space(pose,target,mapping,tracks,c,allow_partial=True)
                if self.route is not None:
                    self.last_goal=target.copy();self.samples=[];self.stall_since=None
                    self.transition('detour',t,'known_route_found')
            return
        stalled=abs(previous[0])<.06
        if blocked and stalled:
            if self.anchor is None or np.linalg.norm(pose[:2]-self.anchor)>.08:
                self.anchor=pose[:2].copy();self.transition('wait',t,'front_blocked');self.since=t
            if t-self.since>=r.wait_s:
                if self.rear_clear(pose,mapping,tracks):
                    self.retreat_start=pose[:2].copy();self.heading=np.array([np.cos(pose[2]),np.sin(pose[2])])
                    self.episode_since=t
                    self.attempts+=1;self.transition('retreat',t,'persistent_front_block')
                else:self.last_retry=t;self.transition('hold',t,'rear_unknown_or_blocked')
        else:
            self.anchor=None;self.transition('follow',t,'progress_or_clear')
