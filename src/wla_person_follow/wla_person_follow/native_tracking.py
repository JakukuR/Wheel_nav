"""Tracking backends: the Python reference or the C++ follow core (phase 2).

The facades keep the C++ state in _wla_follow_core and offer the attributes
and methods of the Python reference (sim_core/tracking.py VisualTrack and
PeopleTracks, sim_core/target_reference.py TargetHistory, bezier_reference and
bspline_corners), which stays the numerical reference. Results are
bit-identical (test/test_native_tracking.py, the replay comparison).
Selected by numeric_backend like the map (native_map.py); never falls back.
"""
import numpy as np
from .sim_core.target_reference import bezier_reference, bspline_corners
from .sim_core.tracking import PeopleTracks, VisualTrack
try:
    import _wla_follow_core as _core
except ImportError:  # Reported when the cpp backend is requested, never silently replaced.
    _core = None

BACKENDS = ('python', 'cpp')


def check_backend(backend):
    if backend not in BACKENDS:
        raise ValueError('unknown_numeric_backend: ' + str(backend))
    if backend == 'cpp' and (_core is None or not hasattr(_core, 'VisualTrack')):
        raise RuntimeError('cpp_numeric_backend_unavailable: _wla_follow_core has no tracking')
    return backend


def make_visual_track(backend='python', timeout=.8, **options):
    if check_backend(backend) == 'python':
        return VisualTrack(timeout, **options)
    return NativeVisualTrack(timeout, **options)


def make_people_tracks(backend='python'):
    return PeopleTracks() if check_backend(backend) == 'python' else NativePeopleTracks()


def reference_functions(backend='python'):
    """(bezier_reference, bspline_corners) of the backend."""
    if check_backend(backend) == 'python':
        return bezier_reference, bspline_corners
    return native_bezier_reference, native_bspline_corners


def native_bezier_reference(history):
    return _core.bezier_reference(np.asarray(history, dtype=float).reshape(-1, 2))


def native_bspline_corners(path, corner=.35, step=.05):
    return _core.bspline_corners(np.asarray(path, dtype=float).reshape(-1, 2), float(corner), float(step))


class NativeTargetHistory:
    """TargetHistory over a C++ history (owned here or by a C++ track)."""
    backend = 'cpp'

    def __init__(self, window_s=5., period_s=.4, max_gap_s=.6, *, _history=None):
        self._core = _core.TargetHistory(window_s, period_s, max_gap_s) if _history is None else _history
        self._lists = (None, None)

    window_s = property(lambda self: self._core.window_s)
    period_s = property(lambda self: self._core.period_s)
    max_gap_s = property(lambda self: self._core.max_gap_s)

    def update(self, point, stamp):
        self._core.update(np.asarray(point, dtype=float), float(stamp))

    def points(self):
        return list(self._core.points())

    def as_lists(self):
        """points() as plain lists; cached until the next accepted observation."""
        revision = self._core.revision
        if self._lists[0] != revision:
            self._lists = (revision, self._core.points().tolist())
        return [point[:] for point in self._lists[1]]

    def diagnostic(self):
        return dict(window_s=self.window_s, sample_period_s=self.period_s,
                    sample_count=self._core.sample_count,
                    oldest_stamp_s=self._core.oldest_stamp,
                    last_observation_stamp_s=self._core.last_observation_stamp,
                    expiry_clock='last_accepted_observation')


class NativeVisualTrack:
    backend = 'cpp'

    def __init__(self, timeout=.8, *, history_window_s=5., history_sample_period_s=.4,
                 history_max_gap_s=.6, meas_std=.025, accel_std=.7, gate=None, gate_max_skips=2,
                 coast_decay=None):
        self.timeout, self.meas_std, self.accel_std = float(timeout), float(meas_std), float(accel_std)
        self.gate, self.gate_max_skips = gate, int(gate_max_skips)
        self.coast_decay = None if coast_decay is None else tuple(float(v) for v in coast_decay)
        self._core = _core.VisualTrack(self.timeout, float(history_window_s), float(history_sample_period_s),
                                       float(history_max_gap_s), self.meas_std, self.accel_std,
                                       None if gate is None else float(gate), self.gate_max_skips,
                                       self.coast_decay)
        self.trail = NativeTargetHistory(_history=self._core.trail)

    # Filter state, read-only: x and P are new arrays per access.
    x = property(lambda self: self._core.x)
    P = property(lambda self: self._core.P)
    time = property(lambda self: self._core.time)
    seen = property(lambda self: self._core.seen)
    skips = property(lambda self: self._core.skips)
    rejected = property(lambda self: self._core.rejected)

    @property
    def history(self):
        return self.trail.points()

    def predicted_position(self, t):
        return self._core.predicted_position(float(t))

    def update(self, z, t):
        return self._core.update(None if z is None else np.asarray(z, dtype=float), float(t))


class NativePeopleTracks:
    """PeopleTracks over C++ tracks; detection keys map to int64 ids here."""
    backend = 'cpp'

    def __init__(self):
        self._core = _core.PeopleTracks()
        self._ids, self._keys, self._next = {}, {}, 0

    def _id(self, key):
        if key not in self._ids:
            self._ids[key] = self._next
            self._keys[self._next] = key
            self._next += 1
        return self._ids[key]

    def _rows(self, result):
        ids, p, v, r, age = result
        return [dict(p=p[k], v=v[k], r=float(r[k]), age=float(age[k]), id=self._keys[int(ids[k])])
                for k in range(len(ids))]

    def _forget_dropped(self):
        if len(self._ids) > 2*len(self._core.ids()) + 16:
            live = set(self._core.ids())
            self._keys = {i: key for i, key in self._keys.items() if i in live}
            self._ids = {key: i for i, key in self._keys.items()}

    def keys(self):
        return [self._keys[i] for i in self._core.ids()]

    def drop(self, key):
        if key in self._ids:
            self._core.drop(self._ids[key])

    def update(self, detections, pose, t):
        ids = np.fromiter((self._id(key) for key in detections), dtype=np.int64, count=len(detections))
        points = np.array([np.asarray(z, dtype=float)[:2] for z in detections.values()], dtype=float).reshape(-1, 2)
        rows = self._rows(self._core.update(ids, points, float(t)))
        self._forget_dropped()
        return rows

    def predict(self, t):
        return self._rows(self._core.predict(float(t)))
