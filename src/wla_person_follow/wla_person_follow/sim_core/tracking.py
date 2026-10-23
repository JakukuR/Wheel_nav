import numpy as np

class VisualTrack:
    """Constant-velocity Kalman estimate; no invisible truth updates."""
    def __init__(self,start,timeout=.8):
        self.x=None;self.P=np.eye(4);self.time=None;self.seen=None
        self.timeout=timeout;self.history=[np.array(start,dtype=float)]

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
            if np.linalg.norm(self.x[:2]-self.history[-1])>.075:
                self.history.append(self.x[:2].copy())
                self.history=self.history[-400:]
        if t-self.seen>self.timeout:
            self.x[2:]=0.
            return self.x[:2].copy(),np.zeros(2),'lost'
        return self.x[:2].copy(),self.x[2:].copy(),'visible' if z is not None else 'coast'

class PeopleTracks:
    def __init__(self):self.tracks={}

    def update(self,detections,pose,t):
        for key,z in detections.items():
            if key not in self.tracks:self.tracks[key]=VisualTrack(pose[:2],timeout=1.5)
        result=[]
        for key,track in list(self.tracks.items()):
            z=detections[key][:2] if key in detections else None
            p,v,state=track.update(z,t)
            age=t-track.seen
            if age>1.5:
                del self.tracks[key];continue
            result.append(dict(p=p,v=v,r=.28+.035+.15*age,age=age,id=key))
        return result
