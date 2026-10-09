"""Project aligned depth into optical and robot-base coordinates."""
import numpy as np


def rotation_from_quaternion(q):
    x, y, z, w = q
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        raise ValueError('invalid transform quaternion')
    s = 2.0 / n
    return np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)],
    ], dtype=np.float32)


def target_pose(depth, box, intrinsics, minimum=0.25, maximum=5.0):
    fx, fy, cx, cy = intrinsics
    x1, y1, x2, y2 = box
    h, w = depth.shape
    xa = max(0, min(w, int(x1 + .35*(x2-x1))))
    xb = max(0, min(w, int(x1 + .65*(x2-x1))))
    ya = max(0, min(h, int(y1 + .25*(y2-y1))))
    yb = max(0, min(h, int(y1 + .60*(y2-y1))))
    patch = depth[ya:yb, xa:xb]
    if patch.size < 30:
        return None
    good = patch[(patch >= minimum) & (patch <= maximum)]
    if good.size < max(20, int(patch.size * .20)):
        return None
    z = float(np.median(good))
    u = (xa + xb) * .5
    v = (ya + yb) * .5
    return np.array([(u-cx)*z/fx, (v-cy)*z/fy, z], dtype=np.float32)


def points_to_map(depth, intrinsics, rotation, translation, exclusion=None,
                  minimum=0.25, maximum=5.0, height_min=.10, height_max=1.50,
                  stride=8):
    fx, fy, cx, cy = intrinsics
    h, w = depth.shape
    vv, uu = np.mgrid[0:h:stride, 0:w:stride]
    zz = depth[::stride, ::stride]
    valid = (zz >= minimum) & (zz <= maximum)
    if exclusion is not None:
        x1, y1, x2, y2 = exclusion
        valid &= ~((uu >= x1) & (uu < x2) & (vv >= y1) & (vv < y2))
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float32)
    z = zz[valid]
    optical = np.stack(((uu[valid]-cx)*z/fx, (vv[valid]-cy)*z/fy, z), axis=1)
    points = optical @ rotation.T + translation
    return points[(points[:, 2] >= height_min) & (points[:, 2] <= height_max)]
