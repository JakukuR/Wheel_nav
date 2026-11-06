"""Find the followed person in a raw depth cloud by connectivity, without a mask.

The tracker says roughly where the person stands; connectivity says which points
belong to them. The cloud must be the raw one: the obstacle filter keeps only the
lowest 0.5 m, where two legs are separate pieces that only the torso joins.
A component that is too wide or has no body height is refused, and the caller
keeps its tracked-position fallback, so a person touching a wall never takes the
wall with them.
"""
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

_NEIGHBOURS=np.ones((3,3,3),dtype=bool)


def _components(xyz,leaf):
    cells=np.floor(xyz/leaf).astype(np.int64)
    cells-=cells.min(axis=0)
    grid=np.zeros(cells.max(axis=0)+1,dtype=bool)
    grid[cells[:,0],cells[:,1],cells[:,2]]=True
    labels,count=ndimage.label(grid,structure=_NEIGHBOURS)
    return labels[cells[:,0],cells[:,1],cells[:,2]],count


def _span(xyz):
    low,high=np.percentile(xyz,[2,98],axis=0)
    return float(max(high[0]-low[0],high[1]-low[1])),float(high[2]-low[2])


def select_person(points,center,*,leaf=.08,fine_leaf=.05,search_radius=.9,seed_radius=.35,
                  min_z=.12,max_width=1.1,min_height=.5,min_points=8,foot_radius=.12,
                  in_box=None,clip_box=None,others=()):
    """Return (mask over points, info). An empty mask means the caller falls back.

    in_box (optional, same-frame detection box per point) only chooses between
    components; it never adds a point the component does not hold. clip_box (same
    shape, usually in_box grown 1.1x) drops component points outside the person's
    box: a seated person and their chair are one component, the box is not.
    others (tracked positions of other people) splits a component shared with a
    bystander: points nearer to them stay in the cloud as their obstacle.
    """
    points=np.asarray(points,dtype=float).reshape(-1,3)
    center=np.asarray(center,dtype=float)
    mask=np.zeros(len(points),dtype=bool)
    if not len(points) or center.shape!=(2,) or not np.isfinite(center).all():
        return mask,dict(state='no_input')
    distance=np.linalg.norm(points[:,:2]-center,axis=1)
    window=np.flatnonzero((distance<=search_radius)&(points[:,2]>=min_z))
    if len(window)<min_points:
        return mask,dict(state='too_few_points')
    near=distance[window]<=seed_radius
    if not near.any():
        return mask,dict(state='no_seed_points')
    for current in (leaf,fine_leaf):
        labels,_=_components(points[window],current)
        votes=near if in_box is None else near|np.asarray(in_box,dtype=bool)[window]
        best=np.bincount(labels[votes]).argmax() if votes.any() else 0
        body=window[labels==best]
        if len(body)<min_points:
            return mask,dict(state='too_few_points')
        width,height=_span(points[body])
        if width<=max_width:
            break
    info=dict(leaf_m=current,points=len(body),width_m=round(width,3),height_m=round(height,3))
    if width>max_width:
        return mask,dict(info,state='merged_too_wide')
    if height<min_height:
        return mask,dict(info,state='no_body_height')
    mask[body]=True
    # Shoes reach below the clustering band; the floor there must not bridge bodies.
    low=body[points[body,2]<.35]
    feet=np.flatnonzero((points[:,2]<min_z)&(points[:,2]>=.02)&(distance<=search_radius))
    if len(low) and len(feet):
        gap,_=cKDTree(points[low,:2]).query(points[feet,:2],k=1)
        mask[feet[gap<=foot_radius]]=True
    if clip_box is not None:
        mask&=np.asarray(clip_box,dtype=bool)
    others=np.asarray(others,dtype=float).reshape(-1,2)
    if len(others):
        indices=np.flatnonzero(mask)
        nearest=np.linalg.norm(points[indices,:2,None]-others.T[None],axis=1).min(axis=1)
        mask[indices[distance[indices]>nearest]]=False
    return mask,dict(info,state='selected')
