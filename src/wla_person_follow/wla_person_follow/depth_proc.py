"""Project aligned depth only into robot-base coordinates for local safety."""
import numpy as np


def rotation_from_quaternion(q):
    x, y, z, w = q
    n = x*x+y*y+z*z+w*w
    if not np.isfinite(q).all() or not .99 <= n <= 1.01:
        raise ValueError('invalid_transform_quaternion')
    s = 2./n
    return np.array([[1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
                     [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
                     [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)]], dtype=np.float32)


def points_to_base(depth, intrinsics, rotation, translation, minimum=.25,
                   maximum=5., height_min=.10, height_max=1.50, stride=8):
    fx, fy, cx, cy = intrinsics
    h, w = depth.shape
    vv, uu = np.mgrid[0:h:stride, 0:w:stride]
    zz = depth[::stride, ::stride]
    valid = np.isfinite(zz) & (zz >= minimum) & (zz <= maximum)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float32)
    z = zz[valid]
    optical = np.stack(((uu[valid]-cx)*z/fx, (vv[valid]-cy)*z/fy, z), axis=1)
    points = optical @ rotation.T + translation
    return points[(points[:, 2] >= height_min) & (points[:, 2] <= height_max)]
