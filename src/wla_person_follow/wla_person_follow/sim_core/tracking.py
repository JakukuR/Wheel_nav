import math
from copy import copy
import numpy as np
from .target_reference import TargetHistory

class VisualTrack:
    """Constant-velocity Kalman estimate; no invisible truth updates.

    No speed limit: the old 1 m/s clamp held 24 % of the 2026-11-06 walk below
    its speed. Instead a measurement whose innovation exceeds `gate` (squared
    Mahalanobis, 2 dof) is skipped, at most `gate_max_skips` in a row so a real
    jump still lands; the skipped ones were the 0.2-0.5 m range jumps of a
    person leaving the image edge at 4-5 m. `coast_decay=(delay_s, tau_s)`
    lets the extrapolated velocity fade once the target is unseen for delay_s.
    """
    def __init__(self,timeout=.8,*,history_window_s=5.,history_sample_period_s=.4,
                 history_max_gap_s=.6,meas_std=.025,accel_std=.7,gate=None,gate_max_skips=2,
                 coast_decay=None):
        self.x=None;self.P=np.eye(4);self.time=None;self.seen=None
        self.timeout=timeout;self.meas_std=meas_std;self.accel_std=accel_std
        self.gate=gate;self.gate_max_skips=gate_max_skips;self.coast_decay=coast_decay
        self.skips=0;self.rejected=0
        self.trail=TargetHistory(history_window_s,history_sample_period_s,history_max_gap_s)

    @property
    def history(self):
        return self.trail.points()

    def predicted_position(self,t):
        """The position update(None,t) returns, without touching the track.

        Constant-velocity extrapolation (with the coast decay), as update()
        does it; the covariance is not needed for the position.
        """
        if self.x is None:return None
        dt=max(0.,min(t,self.seen+self.timeout)-self.time)
        free=dt if self.coast_decay is None else max(0.,min(dt,self.seen+self.coast_decay[0]-self.time))
        if dt>free:
            tau=self.coast_decay[1]
            return self.x[:2]+self.x[2:]*(free+tau*(1.-math.exp(-(dt-free)/tau)))
        return self.x[:2]+self.x[2:]*dt

    def update(self,z,t):
        if self.x is None:
            if z is None:return None,np.zeros(2),'lost'
            self.x=np.r_[z,0.,0.];self.P=np.diag([.01,.01,.2,.2])
            self.seen=t;self.time=t
        prediction_time=t if z is not None else min(t,self.seen+self.timeout)
        dt=max(0.,prediction_time-self.time);x0=self.x
        free=dt if z is not None or self.coast_decay is None else max(0.,min(dt,self.seen+self.coast_decay[0]-self.time))
        self.time=prediction_time
        F=np.eye(4);F[:2,2:]=np.eye(2)*dt
        G=np.vstack((np.eye(2)*dt*dt/2,np.eye(2)*dt))
        self.x=F@self.x;self.P=F@self.P@F.T+self.accel_std**2*G@G.T
        if dt>free:
            # Rebinds x (never in place): prediction() runs on a shallow copy.
            tau=self.coast_decay[1];k=math.exp(-(dt-free)/tau)
            self.x=np.r_[x0[:2]+x0[2:]*(free+tau*(1.-k)),x0[2:]*k]
        if z is not None:
            S=self.P[:2,:2]+np.eye(2)*self.meas_std**2
            y=z-self.x[:2]
            if self.gate is not None and self.skips<self.gate_max_skips and float(y@np.linalg.solve(S,y))>self.gate:
                self.skips+=1;self.rejected+=1
            else:
                self.skips=0
                K=np.linalg.solve(S,self.P[:,:2].T).T
                self.x+=K@y;self.P-=K@self.P[:2,:]
                self.seen=t
                self.trail.update(self.x[:2],t)
        if t-self.seen>self.timeout:
            self.x[2:]=0.
            return self.x[:2].copy(),np.zeros(2),'lost'
        return self.x[:2].copy(),self.x[2:].copy(),'visible' if z is not None else 'coast'

class PeopleTracks:
    def __init__(self):self.tracks={}

    def update(self,detections,pose,t):
        for key,z in detections.items():
            if key not in self.tracks:self.tracks[key]=VisualTrack(timeout=1.5,gate=13.8)
        result=[]
        for key,track in list(self.tracks.items()):
            z=detections[key][:2] if key in detections else None
            p,v,state=track.update(z,t)
            age=t-track.seen
            if age>1.5:
                del self.tracks[key];continue
            result.append(dict(p=p,v=v,r=.28+.035+.15*age,age=age,id=key))
        return result

    def predict(self,t):
        """What update({},pose,t) would return, leaving every track as it is.

        Replaces deepcopy(self).update(...): update(None) only rebinds x/P/time
        (never in place), so a shallow copy of each track is enough.
        """
        result=[]
        for key,track in self.tracks.items():
            p,v,_=copy(track).update(None,t)
            age=t-track.seen
            if age>1.5:continue
            result.append(dict(p=p,v=v,r=.28+.035+.15*age,age=age,id=key))
        return result
