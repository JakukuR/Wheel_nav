"""Pure, bounded geometry and frame pairing for the read-only RGB debugger."""
from collections import OrderedDict
import cv2
import numpy as np


class FrameCache:
    def __init__(self, max_count=24, max_bytes=64*1024*1024):
        self.frames = OrderedDict()
        self.bytes = 0
        self.max_count, self.max_bytes = max_count, max_bytes

    def put(self, key, value, size):
        if key in self.frames:
            self.bytes -= self.frames.pop(key)[1]
        if size > self.max_bytes:
            return
        self.frames[key] = (value, size)
        self.bytes += size
        while self.bytes > self.max_bytes or len(self.frames) > self.max_count:
            self.bytes -= self.frames.popitem(last=False)[1][1]

    def get(self, key):
        item = self.frames.get(key)
        return item[0] if item else None


def relative_lookup(raw, relative):
    """A track id alone is never an identity, and distances are frame-specific."""
    if not relative or any(raw.get(k) != relative.get(k) for k in ('session', 'epoch', 'stamp_ns')):
        return {}
    if not raw.get('session') or not relative.get('enabled'):
        return {}
    prefix = f"{raw['session']}:{raw['epoch']}:"
    return {str(o['id'])[len(prefix):]: o for o in relative.get('objects', [])
            if o.get('detected') and str(o.get('id', '')).startswith(prefix)}


def matrix(q, t):
    q = np.asarray(q, dtype=float)
    if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .01:
        raise ValueError('invalid_transform_quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = t
    if not np.isfinite(result).all():
        raise ValueError('nonfinite_transform')
    return result


def transform(points, mat):
    points = np.asarray(points, dtype=float)
    return points @ mat[:3, :3].T + mat[:3, 3]


class Camera:
    def __init__(self, width, height, k, d, model):
        self.width, self.height = int(width), int(height)
        self.k = np.array(k, dtype=float).reshape(3, 3)
        self.d = np.array(d, dtype=float)
        self.model = model
        if not np.isfinite(self.k).all() or not np.isfinite(self.d).all() or self.k[0, 0] <= 0 or self.k[1, 1] <= 0:
            raise ValueError('invalid_intrinsics')
        if model not in ('plumb_bob', 'rational_polynomial', 'equidistant'):
            raise ValueError('unsupported_distortion_model:'+model)
        if (model == 'equidistant' and len(self.d) != 4) or (model != 'equidistant' and len(self.d) not in (0, 4, 5, 8, 12, 14)):
            raise ValueError('invalid_distortion_coefficients')

    def project(self, points):
        pts = np.asarray(points, dtype=float).reshape(-1, 1, 3)
        if not len(pts):
            return np.empty((0, 2))
        if self.model == 'equidistant':
            uv, _ = cv2.fisheye.projectPoints(pts, np.zeros(3), np.zeros(3), self.k, self.d)
        else:
            uv, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), self.k, self.d)
        return uv.reshape(-1, 2)

    def points(self, points):
        pts = np.asarray(points).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1) & (pts[:, 2] >= .05)]
        uv = self.project(pts)
        valid = np.isfinite(uv).all(axis=1) & (uv[:, 0] >= 0) & (uv[:, 0] < self.width) & (uv[:, 1] >= 0) & (uv[:, 1] < self.height)
        return np.round(uv[valid], 1).tolist()

    def segments(self, segments):
        seg = np.asarray(segments, dtype=float).reshape(-1, 2, 3).copy()
        seg = seg[np.isfinite(seg).all(axis=(1, 2)) & (np.max(seg[:, :, 2], axis=1) >= .05)]
        if not len(seg):
            return []
        for i in (0, 1):
            bad = seg[:, i, 2] < .05
            a, b = seg[bad, i], seg[bad, 1-i]
            seg[bad, i] = a + ((.05-a[:, 2])/(b[:, 2]-a[:, 2]))[:, None]*(b-a)
        uv = self.project(seg.reshape(-1, 3)).reshape(-1, 2, 2)
        out = []
        for pair in uv:
            if not np.isfinite(pair).all():
                continue
            # Bound conversion before passing signed integer coordinates to OpenCV.
            pair = np.rint(np.clip(pair, -1e7, 1e7)).astype(int)
            ok, a, b = cv2.clipLine((0, 0, self.width, self.height), tuple(pair[0]), tuple(pair[1]))
            if ok:
                out.append([*a, *b])
        return out


def voxel_edges(points, size, limit=600):
    if not np.isfinite(size) or not .01 <= size <= 1:
        raise ValueError('invalid_voxel_size')
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    keys = np.unique(np.floor(points/size).astype(np.int64), axis=0)
    # Deterministic sampling preserves coverage, independent of unordered publisher order.
    keys = keys[::max(1, int(np.ceil(len(keys)/limit)))][:limit]
    offsets = np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)])
    corners = (keys[:, None, :]+offsets[None, :, :])*size
    edges = [(a, b) for a in range(8) for b in range(a+1, 8) if bin(a^b).count('1') == 1]
    return corners[:, edges, :].reshape(-1, 2, 3), len(keys)


def path_edges(points, limit=500):
    """Subdivide for distortion while never bridging across invalid vertices."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    result = []
    for a, b in zip(points[:-1], points[1:]):
        if not np.isfinite([a, b]).all():
            continue
        count = min(32, max(1, int(np.ceil(np.linalg.norm(b-a)/.1))))
        samples = np.linspace(a, b, count+1)
        result.extend(zip(samples[:-1], samples[1:]))
        if len(result) >= limit:
            break
    return np.asarray(result[:limit], dtype=float).reshape(-1, 2, 3)
