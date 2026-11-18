"""Small raw-depth evidence sidecar for the installed mode 3 worker.

The sampled ray coverage is engineering evidence, not a calibrated volume proof.
Startup prior is carried separately and is never counted as measured free space.
"""

import hashlib
import base64
import json
import math
import numpy as np


SOURCE = 'mode3_raw_depth_v1'
CALIBRATION = 'floor_tf_sampling_candidate_v1'
BANDS = {'low': (.08, .25), 'middle': (.25, .45), 'high': (.45, .65)}


def current_packet(raw, mapped):
    """A successful old map must not hide a newer raw acquisition/coverage failure."""
    if mapped is None:
        return raw
    if raw is None or mapped.get('depth_stamp_ns', 0) >= raw.get('depth_stamp_ns', 0):
        return mapped
    if (mapped.get('controller_map') is not None
            and (mapped.get('follow_session'), mapped.get('reference_generation')) ==
                (raw.get('follow_session'), raw.get('reference_generation'))):
        # Keep both acquisition times: fresh raw obstacles plus the latest native
        # map, whose own timestamp/TTL remain unchanged.
        return dict(raw, controller_map=mapped['controller_map'])
    return raw


def coverage_decision(valid_fraction, front_fraction, *, valid_min, front_min,
                      map_exploration=False, active=False, mode=''):
    """Map updates use valid returns, not a whole-image fill ratio.

    Native mode3/4 still require a current feasible controller result before output.
    Other active tasks retain their existing aggregate-coverage policy.
    """
    mapping = map_exploration and valid_fraction > 0. and (not active or mode in {'3', '4'})
    native = valid_fraction >= valid_min and front_fraction >= front_min
    return native or mapping, 'map_exploration' if mapping else 'native_coverage'


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


def with_controller_prior(packet, prior, *, grid=None, map_lower=None, resolution=None, cloud=None):
    """Preserve the controller's declared prior without promoting it to a measurement."""
    result = dict(packet)
    result['controller_prior'] = prior
    if grid is not None and resolution == packet['cell_m']:
        values = np.asarray(grid)
        result['controller_map'] = dict(source='native_mode3_local_grid_mixed',
            origin_xy=(np.asarray(map_lower)*resolution).tolist(), cell_m=resolution,
            width=values.shape[0], height=values.shape[1], encoding='bitset-row-major/v1',
            depth_stamp_ns=packet['depth_stamp_ns'],
            reference_generation=packet['reference_generation'],
            free_bits=base64.b64encode(np.packbits((values == 0).ravel()).tobytes()).decode(),
            blocked_bits=base64.b64encode(np.packbits((values == 100).ravel()).tobytes()).decode())
        if cloud is not None and len(cloud['points']):
            from _wla_sim_geometry import ray_slots
            lower = np.asarray(cloud['points'])[:, 2] <= cloud['origin'][2]+.02
            endpoint = np.ascontiguousarray(np.asarray(cloud['points_odom'])[lower, :2], dtype=float)
            observed = np.zeros(values.shape, dtype=bool)
            if len(endpoint):
                a, b = ray_slots(endpoint, np.asarray(cloud['origin_odom'][:2], dtype=float),
                                 np.asarray(map_lower, dtype=float), resolution, values.shape[0])
                observed[a, b] = True
            result['controller_map']['sensor_bits'] = base64.b64encode(np.packbits(observed.ravel()).tobytes()).decode()
    return result
