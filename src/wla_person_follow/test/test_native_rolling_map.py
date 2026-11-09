"""The C++ rolling map equals the Python reference element-wise (phase 1, 2026-11-09).

Random scenarios drive both maps through the same calls; after every call all
ring state, window arrays, derived fields (EDTs included) and return values
must be identical, not merely close. The cpp backend must never fall back.
"""
import numpy as np
import pytest
from wla_person_follow import native_map
from wla_person_follow.native_map import NativeRollingDepthMap, make_rolling_map
from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap

RING = ('log', 'stamp', 'target_clear_stamp', 'occupied_since', 'tags')
WINDOW = ('cells', 'ix', 'iy')
FIELDS = ('sensor_known', 'occupied', 'known', 'assumed_free', 'dist', 'free_dist')


def assert_same(py, cpp):
    assert py.version == cpp.version
    assert py.target_clear_ttl == cpp.target_clear_ttl
    np.testing.assert_array_equal(py.lower, cpp.lower)
    for name in RING:
        a, b = getattr(py, name), getattr(cpp, name)
        assert a.dtype == b.dtype and a.shape == b.shape, name
        np.testing.assert_array_equal(a, b, err_msg=name)
    assert hasattr(py, 'cells') == hasattr(cpp, 'cells')
    if hasattr(py, 'cells'):
        for name in WINDOW:
            np.testing.assert_array_equal(getattr(py, name), getattr(cpp, name), err_msg=name)
    assert hasattr(py, 'occupied') == hasattr(cpp, 'occupied')
    if hasattr(py, 'occupied'):
        for name in FIELDS:
            a, b = getattr(py, name), getattr(cpp, name)
            assert a.dtype == b.dtype, name
            np.testing.assert_array_equal(a, b, err_msg=name)


def _cloud(rng, pose, heads=1):
    clouds = []
    for _ in range(heads):
        count = int(rng.integers(0, 400))
        ang = rng.uniform(-1.2, 1.2, count) + pose[2]
        rng_m = rng.uniform(.2, 5., count)
        z = np.where(rng.random(count) < .3, rng.uniform(-.05, .035, count), rng.uniform(.0, 1.9, count))
        local = np.c_[rng_m*np.cos(ang - pose[2]), rng_m*np.sin(ang - pose[2]), z]
        world = np.c_[pose[0] + rng_m*np.cos(ang), pose[1] + rng_m*np.sin(ang), z]
        cloud = dict(points=local, points_odom=world, dynamic=rng.random(count) < .1,
                     origin=np.array([0., 0., .55]), origin_odom=np.array([pose[0], pose[1], .55]))
        if rng.random() < .5:
            cloud['marking_points_odom'] = world[(z >= .08) & (z <= 1.6)][::2]
        if rng.random() < .4:
            k = int(rng.integers(0, 40))
            cloud['target_points_odom'] = np.c_[pose[0] + 1.5 + rng.normal(0, .15, k), pose[1] + rng.normal(0, .15, k),
                                                rng.uniform(.03, .6, k)]
        clouds.append(cloud)
    return clouds[0] if heads == 1 and rng.random() < .5 else clouds


@pytest.mark.parametrize('seed', range(8))
def test_random_sequences_match_reference(seed):
    rng = np.random.default_rng(seed)
    options = dict(resolution=.05, size=int(rng.choice([40, 64, 160])), ttl=float(rng.choice([2., 12.])),
                   robot_radius=.19, target_clear_ttl=.5)
    py, cpp = RollingDepthMap(**options), NativeRollingDepthMap(**options)
    pose = np.array([rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-3, 3)])
    t = 100.
    for m in (py, cpp):
        m.slide(pose[:2]); m.rebuild(t)
    assert_same(py, cpp)
    if rng.random() < .7:
        held = [m.initial_clear_patch(pose[:2], t, .4, only_unknown=bool(rng.random() < .5), hold_until_motion=True)
                for m in (py, cpp)]
        assert held[0] == held[1]
    trail = np.empty((0, 3))
    for step in range(60):
        t += float(rng.choice([.02, .1, .2, 1.5]))
        pose = pose + np.r_[rng.normal(0, .12, 2), rng.normal(0, .1)]
        if rng.random() < .05:
            pose[:2] += rng.uniform(-6, 6, 2)  # window jump
        if rng.random() < .6:
            trail = np.vstack((trail, [t, pose[0] + 1.5, pose[1]]))[-30:]
        clouds = _cloud(rng, pose, heads=int(rng.choice([1, 1, 2])))
        rebuild = bool(rng.random() < .5)
        for m in (py, cpp):
            m.update(clouds, pose, t, rebuild=rebuild)
        assert_same(py, cpp)
        if rng.random() < .5:
            points = np.c_[pose[0] + rng.uniform(-2, 3, 50), pose[1] + rng.uniform(-2, 2, 50), rng.uniform(0, 1, 50)]
            a, b = py.drop_established(points, trail, t), cpp.drop_established(points, trail, t)
            np.testing.assert_array_equal(np.asarray(a).reshape(-1, 3), b)
        if rng.random() < .5:
            radius = float(rng.choice([0., .3, .6]))
            assert py.retire_person_trail(trail, t, radius) == cpp.retire_person_trail(trail, t, radius)
            assert_same(py, cpp)
        if rng.random() < .3:
            release = bool(rng.random() < .3)
            for m in (py, cpp):
                m.advance_startup_prior(t, release=release)
            assert_same(py, cpp)
        center = pose[:2] + (0. if rng.random() < .5 else rng.normal(0, .05, 2))
        local_radius = float(rng.choice([0., .3]))
        clear_ttl = float(rng.choice([.3, .5])) if rng.random() < .1 else None
        for m in (py, cpp):
            m.slide(pose[:2])
            m.local_assumed_free_center = center.copy()
            m.local_assumed_free_radius = local_radius
            if clear_ttl is not None:
                m.target_clear_ttl = clear_ttl
        reuse = bool(rng.random() < .7)
        later = t + float(rng.choice([0., .05, 3.]))
        for m in (py, cpp):
            m.rebuild(later, reuse=reuse)
        assert_same(py, cpp)
        xy = pose[:2] + rng.uniform(-4, 4, (200, 2))
        for unknown in (False, True):
            np.testing.assert_array_equal(py.query(xy, unknown=unknown), cpp.query(xy, unknown=unknown))
        assert py.snapshot(t, pose) == cpp.snapshot(t, pose)
        for a, b in zip(py._slots(xy), cpp._slots(xy)):
            np.testing.assert_array_equal(a, b)


def test_arrays_are_read_only_and_fields_do_not_change_under_a_holder():
    cpp = NativeRollingDepthMap(resolution=.05, size=40)
    cpp.slide([0., 0.]); cpp.rebuild(0.)
    held = cpp.dist
    cloud = dict(points=np.array([[.5, 0., .3]]), points_odom=np.array([[.5, 0., .3]]), dynamic=np.zeros(1, bool),
                 origin=np.array([0., 0., .55]), origin_odom=np.array([0., 0., .55]))
    before = held.copy()
    cpp.update(cloud, np.zeros(3), 1.)
    np.testing.assert_array_equal(held, before)
    assert not np.array_equal(cpp.dist, before)
    for name in RING + FIELDS + WINDOW + ('lower',):
        with pytest.raises(ValueError):
            getattr(cpp, name)[...] = 0


def test_backend_selection_never_falls_back(monkeypatch):
    assert make_rolling_map('python').__class__ is RollingDepthMap
    assert make_rolling_map('cpp').backend == 'cpp'
    with pytest.raises(ValueError):
        make_rolling_map('gpu')
    monkeypatch.setattr(native_map, '_NativeRollingMap', None)
    with pytest.raises(RuntimeError):
        make_rolling_map('cpp')


def test_before_window_and_build_attributes_match_reference():
    py, cpp = RollingDepthMap(), NativeRollingDepthMap()
    for name in ('cells', 'occupied', 'dist', 'local_assumed_free_center'):
        assert hasattr(py, name) == hasattr(cpp, name) == False
    np.testing.assert_array_equal(py.lower, cpp.lower)
