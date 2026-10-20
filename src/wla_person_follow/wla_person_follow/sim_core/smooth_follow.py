"""Optional smooth following experiment; only measured map/track inputs."""
from dataclasses import dataclass
import numpy as np
from .close_follow import CloseFollowTuning


@dataclass(frozen=True)
class SmoothFollowTuning(CloseFollowTuning):
    # Keep the original maneuvering authority for collision avoidance. Comfort
    # is an objective inside MPC, not a low-pass filter after safety checking.
    max_w: float = .90
    yaw_accel: float = 1.50
    smooth_reference: bool = True
    reference_lookahead: float = .95
    preferred_turn_radius: float = .55
    yaw_change_weight: float = 8.
    curvature_weight: float = 6.


def comfort_allowed(pose,previous,tracks,config):
    """Suspend extra comfort costs around a measured predicted close encounter.

    Uses a constant-velocity relative sweep over the existing MPC horizon;
    this switches preferences only, never relaxes collision constraints.
    """
    rv=previous[0]*np.array([np.cos(pose[2]),np.sin(pose[2])])
    for track in tracks:
        delta=np.asarray(track['p'])-pose[:2];relative=np.asarray(track['v'])-rv
        tau=np.clip(-delta@relative/max(relative@relative,1e-9),0.,config.horizon*config.step)
        gap=np.linalg.norm(delta+tau*relative)-track['r']-config.radius
        if gap<.8:return False
    return True


def smooth_reference(path, mapping, tracks, config):
    """Bounded elastic smoothing with every changed edge checked in known space.

    Resampling alone preserves geometry. Move at most 20 cm from the original
    polyline, fix endpoints, and reject an edit if ANY old/new neighbour pairing
    enters unknown or occupied space. MPC still checks the actual swept motion.
    """
    path=np.asarray(path,dtype=float)
    if len(path)<3:return path.copy()
    path=path[np.r_[True,np.linalg.norm(np.diff(path,axis=0),axis=1)>1e-6]]
    if len(path)<3:return path.copy()
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    # Include vertices so resampling cannot cut an unchecked corner.
    sample=np.unique(np.r_[arc,np.arange(0,arc[-1],.12)])
    anchor=np.column_stack([np.interp(sample,arc,path[:,j]) for j in range(2)])
    current=anchor.copy()
    def clear(a,b):
        # All segments <= .52 m after the displacement bound; 13 samples keep
        # spacings below .044 m. Reserve 3 cm beyond the required clearance.
        p=a[:,None,:]+(b-a)[:,None,:]*np.linspace(0,1,13)[None,:,None]
        safe=(mapping.query(p)>=config.radius+config.margin+.03)
        safe &= mapping.query(p,unknown=True)>=config.radius+.03
        for track in tracks:
            safe &= np.linalg.norm(p-track['p'],axis=-1)>=track['r']+config.radius+config.margin+.03
        return safe.all(axis=1)
    for _ in range(8):
        proposal=current.copy()
        proposal[1:-1]=.4*current[1:-1]+.3*(current[:-2]+current[2:])
        shift=proposal-anchor
        proposal=anchor+shift*np.minimum(1.,.20/np.maximum(np.linalg.norm(shift,axis=1),1e-9))[:,None]
        old_new=clear(current[:-1],proposal[1:])
        new_old=clear(proposal[:-1],current[1:])
        new_new=clear(proposal[:-1],proposal[1:])
        accept=np.r_[False,old_new[:-1]&new_new[:-1]&new_old[1:]&new_new[1:],False]
        current[accept]=proposal[accept]
    return current
