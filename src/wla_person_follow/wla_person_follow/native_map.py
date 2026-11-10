"""Rolling depth map backends: the Python reference or the C++ follow core.

NativeRollingDepthMap keeps all map state in _wla_follow_core.RollingMap and
offers the attributes and methods of sim_core.rolling_depth_map.RollingDepthMap,
which stays the numerical reference. The two compare element-wise equal
(test/test_native_rolling_map.py and the replay comparison). Arrays read here
are read-only: ring state (log, stamp, ...) is a live view of the C++ storage;
window arrays and derived fields are new arrays per change, as the reference
replaces them.
"""
import numpy as np
from .sim_core.common import ROBOT_RADIUS_M
from .sim_core.rolling_depth_map import RollingDepthMap
try:
    from _wla_follow_core import RollingMap as _NativeRollingMap
except ImportError:  # Reported when the cpp backend is requested, never silently replaced.
    _NativeRollingMap = None

BACKENDS = ('python', 'cpp')


def map_backend(mapping):
    return getattr(mapping, 'backend', 'python')


def make_rolling_map(backend='python', **options):
    if backend == 'python':
        return RollingDepthMap(**options)
    if backend == 'cpp':
        if _NativeRollingMap is None:
            raise RuntimeError('cpp_numeric_backend_unavailable: _wla_follow_core not importable')
        return NativeRollingDepthMap(**options)
    raise ValueError('unknown_numeric_backend: ' + str(backend))


def _readonly(array):
    array.setflags(write=False)
    return array


class NativeRollingDepthMap:
    backend = 'cpp'

    def __init__(self, resolution=.1, size=120, ttl=12., robot_radius=ROBOT_RADIUS_M, target_clear_ttl=.5):
        self._core = _NativeRollingMap(resolution, size, ttl, robot_radius, target_clear_ttl)
        self.log, self.stamp, self.target_clear_stamp, self.occupied_since, self.tags = self._core.ring_views()
        self._lower = self._window = self._fields = None

    @classmethod
    def _wrap(cls, core):
        """View of a C++ map owned elsewhere (the FollowCore of native_engine)."""
        self = cls.__new__(cls)
        self._core = core
        self.log, self.stamp, self.target_clear_stamp, self.occupied_since, self.tags = core.ring_views()
        self._lower = self._window = self._fields = None
        return self

    # Scalars and settings.
    res = property(lambda self: self._core.res)
    n = property(lambda self: self._core.n)
    ttl = property(lambda self: self._core.ttl)
    robot_radius = property(lambda self: self._core.robot_radius)
    version = property(lambda self: self._core.version)

    @property
    def target_clear_ttl(self):
        return self._core.target_clear_ttl

    @target_clear_ttl.setter
    def target_clear_ttl(self, value):
        self._core.target_clear_ttl = float(value)

    @property
    def local_assumed_free_radius(self):
        return self._core.local_assumed_free_radius

    @local_assumed_free_radius.setter
    def local_assumed_free_radius(self, value):
        self._core.local_assumed_free_radius = float(value)

    @property
    def local_assumed_free_center(self):
        center = self._core.local_assumed_free_center
        if center is None:
            raise AttributeError('local_assumed_free_center')
        return center

    @local_assumed_free_center.setter
    def local_assumed_free_center(self, value):
        self._core.local_assumed_free_center = np.asarray(value, dtype=float)

    # Window arrays, cached per window position.
    @property
    def lower(self):
        wid = self._core.window_id
        if self._lower is None or self._lower[0] != wid:
            self._lower = (wid, _readonly(self._core.lower))
        return self._lower[1]

    def _window_arrays(self):
        wid = self._core.window_id
        if self._window is None or self._window[0] != wid:
            if not self._core.has_window:
                raise AttributeError('cells')
            self._window = (wid, *(_readonly(a) for a in self._core.window_arrays()))
        return self._window

    cells = property(lambda self: self._window_arrays()[1])
    ix = property(lambda self: self._window_arrays()[2])
    iy = property(lambda self: self._window_arrays()[3])

    # Derived fields, cached per rebuild.
    def _built_fields(self):
        bid = self._core.build_id
        if self._fields is None or self._fields[0] != bid:
            if not self._core.has_fields:
                raise AttributeError('occupied')
            self._fields = (bid, *(_readonly(a) for a in self._core.fields()))
        return self._fields

    sensor_known = property(lambda self: self._built_fields()[1])
    occupied = property(lambda self: self._built_fields()[2])
    known = property(lambda self: self._built_fields()[3])
    assumed_free = property(lambda self: self._built_fields()[4])
    dist = property(lambda self: self._built_fields()[5])
    free_dist = property(lambda self: self._built_fields()[6])

    # Operations (same signatures as the reference).
    def slide(self, position):
        self._core.slide(np.asarray(position, dtype=float))

    def _slots(self, xy):
        return self._core.slots(np.asarray(xy, dtype=float).reshape(-1, 2))

    def initial_clear_patch(self, position, t, radius=1., only_unknown=False, hold_until_motion=False):
        return self._core.initial_clear_patch(np.asarray(position, dtype=float), float(t), float(radius),
                                              bool(only_unknown), bool(hold_until_motion))

    def advance_startup_prior(self, t, release=False):
        self._core.advance_startup_prior(float(t), bool(release))

    def update(self, cloud, pose, t, rebuild=True):
        clouds = list(cloud) if isinstance(cloud, (list, tuple)) else [cloud]
        self._core.update(clouds, np.asarray(pose, dtype=float), float(t), bool(rebuild))

    def drop_established(self, points, trail, t, radius=.6):
        return self._core.drop_established(np.asarray(points, dtype=float).reshape(-1, 3),
                                           np.asarray(trail, dtype=float).reshape(-1, 3), float(t), float(radius))

    def retire_person_trail(self, trail, t, radius=.30, window=.5, max_gap=2.):
        return self._core.retire_person_trail(np.asarray(trail, dtype=float).reshape(-1, 3), float(t),
                                              float(radius), float(window), float(max_gap))

    def retire_disc(self, center, radius, t):
        return self._core.retire_disc(float(center[0]), float(center[1]), float(radius), float(t))

    def rebuild(self, t, reuse=False):
        self._core.rebuild(float(t), bool(reuse))

    # Read-only helpers shared with the reference implementation.
    query = RollingDepthMap.query
    snapshot = RollingDepthMap.snapshot
