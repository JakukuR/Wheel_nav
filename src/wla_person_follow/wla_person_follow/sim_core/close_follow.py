"""Close following uses body-to-person clearance, not center distance."""
from dataclasses import dataclass
from .common import Tuning
import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass(frozen=True)
class CloseFollowTuning(Tuning):
    # User-selected body-to-obstacle margin for the current local demo.
    margin: float = .10
    # Circular footprint and fitted person-radius assumptions of this demo.
    target_radius: float = .28
    desired_gap: float = .24+.28+.40
    target_stop_gap: float = .24+.28+.36
    # A temporary close approach is allowed; normal reference/stop stay 40/36 cm.
    target_hard_gap: float = .24+.28+.10
    target_clearance_min: float = .30
    target_clearance_max: float = .50
    rear_yield: bool = True


def rear_cell_clearance(points,centers,resolution,radius):
    """Exact distance to occupied cell squares; avoids EDT cell-bound jumps."""
    points=np.asarray(points)
    if not len(centers):return np.full(points.shape[:-1],np.inf)
    delta=np.maximum(np.abs(points[...,None,:]-centers)-resolution/2,0.)
    return np.min(np.linalg.norm(delta,axis=-1),axis=-1)-radius-.001


def rear_yield_plan(pose,target,mapping,tracks,c):
    """Short forward relief, authorized only by observed rear/forward geometry.

    Anonymous rear ToF hits need not be classified as people. If a rear
    obstacle already violates the desired margin, keep its clearance from
    decreasing while preserving the full margin to every other obstacle.
    """
    if not getattr(c,'rear_yield',False) or target is None:return None
    heading=np.array([np.cos(pose[2]),np.sin(pose[2])])
    delta=target-pose[:2];gap=np.linalg.norm(delta)
    if delta@heading<0 or abs(np.cross(heading,delta))>gap*.4:return None
    xy=(mapping.cells+.5)*mapping.res
    delta=xy-pose[:2]
    rear=mapping.occupied&(delta@heading<-.08)&(np.linalg.norm(delta,axis=-1)<1.4)
    def field(mask,pad):
        if not mask.any() and not pad:return np.full(mask.shape,np.inf)
        return distance_transform_edt(np.pad(~mask,1,constant_values=not pad))[1:-1,1:-1]*mapping.res
    rear_field=field(rear,False);other_field=field(mapping.occupied&~rear,True)
    rear_centers=xy[rear]
    static_clear=float(rear_cell_clearance(pose[:2],rear_centers,mapping.res,c.radius))
    rear_tracks=np.array([(o['p']-pose[:2])@heading<-.08 for o in tracks],dtype=bool)
    track_clear=min((np.linalg.norm(o['p']-pose[:2])-o['r']-c.radius for o,is_rear in zip(tracks,rear_tracks) if is_rear),default=np.inf)
    current=min(static_clear,track_clear)
    if not .02<current<.40:return None
    # The usual 30-50 cm band is a preference, not a minimum for rear relief.
    # A comfortable closer stop need not trigger a reverse distance correction.
    distance=min(.15,gap-c.target_hard_gap-.02)
    if distance<.025:return None
    points=pose[:2]+np.linspace(0,distance,12)[:,None]*heading
    if np.any(mapping.query(points,unknown=True)<c.radius+.02):return None
    if np.any(mapping.query(points,field=other_field)<c.radius+c.margin+.01):return None
    return dict(distance=distance,heading=heading,rear_field=rear_field,other_field=other_field,
                rear_centers=rear_centers,static_floor=min(c.margin,static_clear),
                track_floor=min(c.margin,track_clear),rear_tracks=rear_tracks)
