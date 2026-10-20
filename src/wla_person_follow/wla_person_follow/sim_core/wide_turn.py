"""Observed-history outside-corner references, without scene geometry access."""
from dataclasses import dataclass
import numpy as np
from .close_follow import CloseFollowTuning


@dataclass(frozen=True)
class WideTurnTuning(CloseFollowTuning):
    wide_turn: bool = True
    corner_offset: float = .30
    corner_radius: float = .65
    corner_lookahead: float = .45


def outside_reference(pose, history, target, mapping, tracks, config):
    """Try successively smaller outside offsets only in observed free space.

    Detect a bend from measured history over a 70 cm window. Offset away from
    its inside, taper at the ends, and check edges through the current standoff
    point at <= 2.5 cm. The suffix to the person is a reference extension only.
    Return None when no bend/valid offset exists. This is a reference proposal;
    the differential MPC must still validate swept motion and moving people.
    """
    if target is None or len(history)<3:return None
    raw=np.asarray(history,dtype=float)
    if np.linalg.norm(raw[-1]-target)>.01:raw=np.vstack((raw,target))
    raw=raw[np.r_[True,np.linalg.norm(np.diff(raw,axis=0),axis=1)>1e-6]]
    if len(raw)<3:return None
    lengths=np.linalg.norm(np.diff(raw,axis=0),axis=1)
    arc=np.r_[0.,np.cumsum(lengths)];segments=np.diff(raw,axis=0)
    alpha=np.clip(np.sum((pose[:2]-raw[:-1])*segments,axis=1)/lengths**2,0.,1.)
    projection=raw[:-1]+alpha[:,None]*segments
    idx=int(np.argmin(np.linalg.norm(projection-pose[:2],axis=1)))
    s0=arc[idx]+alpha[idx]*lengths[idx]
    def interp(s):return np.column_stack([np.interp(s,arc,raw[:,j]) for j in (0,1)])
    probe=np.arange(s0+.15,min(s0+1.7,arc[-1]-.20),.05)
    if not len(probe):return None
    before=interp(probe)-interp(probe-.35);after=interp(probe+.35)-interp(probe)
    turn=np.arctan2(before[:,0]*after[:,1]-before[:,1]*after[:,0],np.sum(before*after,axis=1))
    j=int(np.argmax(abs(turn)))
    if abs(turn[j])<np.deg2rad(25):return None
    corner=probe[j];sign=np.sign(turn[j])
    # Preserve original vertices, the observed endpoint, and progress position.
    s=np.unique(np.r_[s0,arc[arc>s0],np.arange(s0,arc[-1],.05)])
    base=interp(s)
    tangent=interp(s+.15)-interp(s-.15)
    tangent/=np.maximum(np.linalg.norm(tangent,axis=1),1e-9)[:,None]
    outside=-sign*np.column_stack((-tangent[:,1],tangent[:,0]))
    taper=np.minimum(np.clip((s-(corner-1.2))/.45,0,1),np.clip((arc[-1]-s)/.35,0,1))
    profile=np.exp(-((s-corner)/.80)**2)*taper
    for offset in (config.corner_offset,config.corner_offset*2/3,config.corner_offset/3):
        proposal=base+offset*profile[:,None]*outside
        # Connect the robot to the reference; never teleport across an obstacle.
        route=np.vstack((pose[:2],proposal))
        edge=np.diff(route,axis=0);dist=np.linalg.norm(edge,axis=1)
        # The endpoint at the observed person is a standoff reference, not a
        # robot destination. Check only through the current standoff position;
        # do not demand observed free robot-sized space inside/behind the person.
        progress=np.r_[0.,np.cumsum(dist)]
        limit=max(0.,progress[-1]-config.desired_gap)
        if limit<.25:continue
        points=[]
        for i,(a,b,d) in enumerate(zip(route[:-1],edge,dist)):
            remaining=limit-progress[i]
            if remaining<=0:break
            length=min(d,remaining)
            points.append(a+np.linspace(0,length/max(d,1e-9),max(2,int(np.ceil(length/.025))+1))[:,None]*b)
        points=np.vstack(points)
        if np.any(mapping.query(points)<config.radius+config.margin+.014):continue
        if np.any(mapping.query(points,unknown=True)<config.radius+.014):continue
        if any(np.any(np.linalg.norm(points-o['p'],axis=1)<config.radius+o['r']+config.margin+.014)
               for o in tracks):continue
        return dict(path=route,offset=float(offset),turn=float(turn[j]),corner=interp([corner])[0])
    return None
