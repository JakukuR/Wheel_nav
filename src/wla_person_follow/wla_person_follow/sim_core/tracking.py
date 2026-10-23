import numpy as np
from .target_reference import TargetHistory

class VisualTrack:
    """Constant-velocity Kalman estimate; no invisible truth updates."""
    def __init__(self,timeout=.8,*,history_window_s=5.,history_sample_period_s=.4,
                 history_max_gap_s=.6):
        self.x=None;self.P=np.eye(4);self.time=None;self.seen=None
        self.timeout=timeout
        self.trail=TargetHistory(history_window_s,history_sample_period_s,history_max_gap_s)

    @property
    def history(self):
        return self.trail.points()

    def update(self,z,t):
        if self.x is None:
            if z is None:return None,np.zeros(2),'lost'
            self.x=np.r_[z,0.,0.];self.P=np.diag([.01,.01,.2,.2])
            self.seen=t;self.time=t
        prediction_time=t if z is not None else min(t,self.seen+self.timeout)
        dt=max(0.,prediction_time-self.time);self.time=prediction_time
        F=np.eye(4);F[:2,2:]=np.eye(2)*dt
        G=np.vstack((np.eye(2)*dt*dt/2,np.eye(2)*dt))
        self.x=F@self.x;self.P=F@self.P@F.T+.7**2*G@G.T
        if z is not None:
            S=self.P[:2,:2]+np.eye(2)*.025**2
            K=np.linalg.solve(S,self.P[:,:2].T).T
            self.x+=K@(z-self.x[:2]);self.P-=K@self.P[:2,:]
            self.seen=t
            vnorm=np.linalg.norm(self.x[2:]);self.x[2:]*=min(1.,1./max(vnorm,1e-9))
            self.trail.update(self.x[:2],t)
        if t-self.seen>self.timeout:
            self.x[2:]=0.
            return self.x[:2].copy(),np.zeros(2),'lost'
        return self.x[:2].copy(),self.x[2:].copy(),'visible' if z is not None else 'coast'

class PeopleTracks:
    def __init__(self):self.tracks={}

    def update(self,detections,pose,t):
        for key,z in detections.items():
            if key not in self.tracks:self.tracks[key]=VisualTrack(timeout=1.5)
        result=[]
        for key,track in list(self.tracks.items()):
            z=detections[key][:2] if key in detections else None
            p,v,state=track.update(z,t)
            age=t-track.seen
            if age>1.5:
                del self.tracks[key];continue
            result.append(dict(p=p,v=v,r=.28+.035+.15*age,age=age,id=key))
        return result
