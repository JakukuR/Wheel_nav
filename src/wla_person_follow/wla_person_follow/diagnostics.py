"""Read-only telemetry helpers. Never supply values to a motion predicate."""
import math
import numpy as np


def finite_json(value):
    """Telemetry must not invalidate an otherwise valid control publication."""
    if isinstance(value, dict):return {str(k):finite_json(v) for k,v in value.items()}
    if isinstance(value, (list,tuple,np.ndarray)):return [finite_json(v) for v in value]
    if isinstance(value, (float,np.floating)):return float(value) if math.isfinite(value) else None
    if isinstance(value, (bool,np.bool_)):return bool(value)
    if isinstance(value, (int,np.integer)):return int(value)
    return value


def observed_target(objects, ident, stamp_s, now_s):
    row=objects.get(ident) if ident else None
    return finite_json(dict(id=ident or None,source='relative_objects',stamp_s=stamp_s,
        age_s=now_s-stamp_s,detected=bool(row and row.get('detected')),
        surface_distance_m=row.get('distance') if row else None,
        bearing_rad=row.get('yaw') if row else None))


def occupancy_source(mapping, point, now):
    """Nearest EDT source to the QUERY CELL, including padded boundaries.

    No new occupancy provenance is invented. A tied nearest cell is a
    representative source, not evidence that a specific person made that cell.
    This diagnostic-only helper never modifies the map or its distance field.
    """
    index=np.floor(np.asarray(point)/mapping.res).astype(int)-mapping.lower
    if np.any(index<0) or np.any(index>=mapping.n):
        return dict(kind='map_outside',query_cell_local=index.tolist())
    occupied=np.argwhere(mapping.occupied)
    # The distance transform pads a one-cell blocked border.
    edge=np.arange(-1,mapping.n+1)
    border=np.vstack((np.column_stack((np.full_like(edge,-1),edge)),
        np.column_stack((np.full_like(edge,mapping.n),edge)),
        np.column_stack((edge,np.full_like(edge,-1))),
        np.column_stack((edge,np.full_like(edge,mapping.n)))))
    cells=np.vstack((occupied,border));d2=np.sum((cells-index)**2,axis=1)
    nearest=int(np.argmin(d2));cell=cells[nearest];global_cell=cell+mapping.lower
    result=dict(kind='map_occupancy' if nearest<len(occupied) else 'padded_map_boundary',
        query_cell_local=index.tolist(),cell_local=cell.tolist(),cell_reference=global_cell.tolist(),
        center_reference_m=((global_cell+.5)*mapping.res).tolist(),
        edt_source_distance_m=float(np.sqrt(d2[nearest])*mapping.res),
        tied_sources=int(np.count_nonzero(d2==d2[nearest])),
        semantic_owner='unattributed_occupancy',source_is_representative=True)
    if nearest<len(occupied):
        slot=global_cell%mapping.n
        result.update(cell_age_s=None if now is None else float(now-mapping.stamp[tuple(slot)]),
                      cell_log_odds=float(mapping.log[tuple(slot)]))
    return finite_json(result)


def front_distances(samples, queried, tracks, config, speed):
    """Diagnostic scan of every supplied track; does not decide forward_blocked.

    Distances are along the tested path, not raw camera/front-axis ranges.
    People use the same segment minimum and endpoint threshold as the gate.
    """
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(samples,axis=0),axis=1))]
    blockers=[]
    hit=np.flatnonzero(queried<=config.radius+config.margin+.026)
    if len(hit):blockers.append(dict(kind='static_distance_field',path_distance_m=float(arc[hit[0]])))
    times=arc/np.clip(config.max_v if speed is None else speed,.15,config.max_v)
    for track in tracks:
        relative=samples-(np.asarray(track['p'])+times[:,None]*np.asarray(track['v']))
        a=relative[:-1];delta=np.diff(relative,axis=0)
        u=np.clip(-np.sum(a*delta,axis=1)/np.maximum(np.sum(delta**2,axis=1),1e-12),0.,1.)
        distance=np.linalg.norm(a+u[:,None]*delta,axis=1)
        hit=np.flatnonzero(distance<=track['r']+config.radius+config.margin+.025*times[1:])
        if len(hit):
            j=int(hit[0]);blockers.append(dict(kind='person_track',id=track.get('id'),
                path_distance_m=float(arc[j]+u[j]*(arc[j+1]-arc[j])),
                minimum_relative_distance_m=float(distance[j])))
    blockers.sort(key=lambda row:row['path_distance_m'])
    return dict(nearest_block_path_distance_m=blockers[0]['path_distance_m'] if blockers else None,
        nearest_block_source=blockers[0] if blockers else None,blocking_sources=blockers,
        diagnostic_scan_only=True)


def wheel_message(msg, received_s, accepted, generation):
    p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist
    return finite_json(dict(source='/wheel/odom',stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
        received_monotonic_s=received_s,accepted_by_local_motion=accepted,reference_generation=generation,
        frame_id=msg.header.frame_id,child_frame_id=msg.child_frame_id,
        twist=dict(vx_mps=v.linear.x,vy_mps=v.linear.y,omega_rps=v.angular.z),
        message_pose=dict(position_m=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w]),
        message_pose_used_for_control=False))
