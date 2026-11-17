"""Small raw-depth evidence sidecar for the installed mode 3 worker.

The sampled ray coverage is engineering evidence, not a calibrated volume proof.
Startup prior is carried separately and is never counted as measured free space.
"""

import hashlib
import json
import math
import numpy as np


SOURCE = 'mode3_raw_depth_v1'
CALIBRATION = 'floor_tf_sampling_candidate_v1'
BANDS = {'low': (.08, .25), 'middle': (.25, .45), 'high': (.45, .65)}


def packet_from_depth(depth, k, rotation, translation, pose, *, stamp_ns, pose_stamp_ns,
                      follow_session, reference_generation, transport_generation):
    """Project unmasked, unfiltered first returns in the measured wheel reference."""
    size, cell = 30, .05
    origin = np.floor((np.asarray(pose[:2], dtype=float)-size*cell/2)/cell)*cell
    data = np.asarray(depth, dtype=float)[::8, ::8]
    v, u = np.indices(data.shape)
    z = data.ravel()
    valid = np.isfinite(z) & (z >= .15) & (z <= 5.)
    z, u, v = z[valid], (u.ravel()*8)[valid], (v.ravel()*8)[valid]
    points = np.stack(((u-k[2])*z/k[0], (v-k[5])*z/k[4], z), axis=1)
    points = points @ np.asarray(rotation).T+np.asarray(translation)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    turn = np.array([[c, -s], [s, c]])
    points[:, :2] = points[:, :2] @ turn.T+np.asarray(pose[:2])
    camera = np.asarray(translation, dtype=float).copy()
    camera[:2] = camera[:2] @ turn.T+np.asarray(pose[:2])

    def cells(values):
        if len(values) == 0:
            return []
        indices = np.floor((values[:, :2]-origin)/cell).astype(int)
        indices = indices[((indices >= 0) & (indices < size)).all(1)]
        occupied = np.zeros(size*size, dtype=bool)
        occupied[indices[:, 0]*size+indices[:, 1]] = True
        codes = np.flatnonzero(occupied)
        return np.stack((codes//size, codes%size), axis=1).tolist()

    floor = cells(points[np.abs(points[:, 2]) <= .06])
    blocked = cells(points[(points[:, 2] > .06) & (points[:, 2] <= .65)])
    # Sample at half a cell along the portion of each ray inside the local ROI.
    # Endpoint reserve avoids declaring the return surface itself clear.
    lengths = np.linalg.norm(points-camera, axis=1)
    distances = np.arange(.025, 1.5, .025)
    ratios = distances[None, :]/np.maximum(lengths[:, None], .001)
    free = camera+(points[:, None, :]-camera)*ratios[:, :, None]
    free = free[(ratios < 1.-.05/np.maximum(lengths[:, None], .001))]
    measured = {name: cells(free[(free[:, 2] >= low) & (free[:, 2] < high)])
                for name, (low, high) in BANDS.items()}
    identity = hashlib.sha256(np.asarray([*k, *np.asarray(rotation).ravel(), *translation],
                                        dtype='<f8').tobytes()).hexdigest()
    return dict(schema='kalo-measured-clearance/v1', source=SOURCE, calibration=CALIBRATION,
        calibration_sha256=identity, evidence_level='sampled_raw_depth_rays',
        frame='wheel_reference', follow_session=follow_session, reference_generation=reference_generation,
        transport_generation=transport_generation, depth_stamp_ns=stamp_ns, pose_stamp_ns=pose_stamp_ns,
        observed_pose=list(pose), origin_xy=origin.tolist(), cell_m=cell, width=size,
        obstacle_returns_unmasked=True, floor_hits=floor, measured_clear=measured, blocked=blocked)


def with_controller_prior(packet, prior, *, grid=None, map_lower=None, resolution=None):
    """Preserve the controller's declared prior without promoting it to a measurement."""
    result = dict(packet)
    result['controller_prior'] = prior
    if grid is not None and resolution == packet['cell_m']:
        width = packet['width']
        offset = np.rint(np.asarray(packet['origin_xy'])/resolution).astype(int)-np.asarray(map_lower, dtype=int)
        indices = np.indices((width, width)).reshape(2, -1).T
        native = indices+offset
        valid = ((native >= 0) & (native < np.asarray(grid).shape)).all(1)
        indices, native = indices[valid], native[valid]
        values = np.asarray(grid)[native[:, 0], native[:, 1]]
        result['controller_map'] = dict(source='native_mode3_local_grid_mixed',
            origin_xy=packet['origin_xy'], cell_m=resolution, depth_stamp_ns=packet['depth_stamp_ns'],
            reference_generation=packet['reference_generation'],
            free=indices[values == 0].tolist(), blocked=indices[values == 100].tolist())
    if len(json.dumps(result, separators=(',', ':')).encode()) > 40000:
        return dict(status='unavailable', reason='clearance_packet_budget_exceeded',
                    depth_stamp_ns=packet['depth_stamp_ns'])
    return result
