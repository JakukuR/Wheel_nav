"""Observation path (J02 2026-10-30): cheap snapshots and exact map reuse.

Person observations reached the planner ~65 ms after publication: deep copies
held the GIL on the receive thread and every 20 Hz replan rebuilt two EDTs.
"""
from copy import deepcopy
import numpy as np
from wla_person_follow.motion_memory import LocalMotion
from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
from wla_person_follow.sim_core.tracking import VisualTrack


def test_motion_snapshot_is_independent_and_equal():
    m = LocalMotion()
    for i in range(140):
        m.update(.02 * i, (.3, 0., .1), .02 * i)
    s = m.snapshot()
    assert list(s.samples) == list(m.samples) and s.generation == m.generation
    assert s.samples.maxlen == m.samples.maxlen
    assert s.pose_at(2.7) == m.pose_at(2.7)
    m.update(2.8, (.3, 0., .1), 2.8)
    assert len(s.samples) == 128 and s.samples[-1][0] < 2.8


def test_prediction_copy_leaves_the_track_unchanged():
    t = VisualTrack(timeout=.8)
    for k in range(6):
        t.update(np.array([1. + .1 * k, .2]), .1 * k)
    before = (t.x.copy(), t.P.copy(), t.time, t.seen, list(t.history))
    expected = deepcopy(t).update(None, .9)[0]
    from copy import copy
    np.testing.assert_array_equal(copy(t).update(None, .9)[0], expected)
    np.testing.assert_array_equal(t.x, before[0]); np.testing.assert_array_equal(t.P, before[1])
    assert (t.time, t.seen) == before[2:4]
    np.testing.assert_array_equal(np.array(t.history), np.array(before[4]))


def _map(rng, radius):
    m = RollingDepthMap()
    m.local_assumed_free_radius = radius
    pts = np.c_[rng.uniform(.3, 3., (400, 2)) * [1, 1] - [0, 1.5], rng.uniform(.05, .4, 400)]
    cloud = dict(points=pts, points_odom=pts, origin=np.array([0, 0, .3]), origin_odom=np.array([0, 0, .3]),
                 dynamic=np.zeros(len(pts), bool))
    m.slide(np.zeros(2)); m.local_assumed_free_center = np.zeros(2)
    m.update(cloud, np.zeros(3), 1., rebuild=False); m.rebuild(1.)
    return m


def _same(a, b):
    for name in ('known', 'occupied', 'assumed_free', 'dist', 'free_dist'):
        np.testing.assert_array_equal(getattr(a, name), getattr(b, name), err_msg=name)


def test_reused_map_equals_a_full_rebuild():
    rng = np.random.default_rng(3)
    for radius in (0., .5):
        a = _map(rng, radius); b = deepcopy(a)
        steps = [(1.05, (0., 0.)), (1.10, (.02, 0.)), (1.15, (.04, .01)), (1.30, (.07, .02))]
        for t, c in steps:
            for m in (a, b):
                m.slide(np.array(c)); m.local_assumed_free_center = np.array(c)
            a.rebuild(t, reuse=True); b.rebuild(t)
            _same(a, b)


def test_reuse_notices_expiry_and_new_evidence():
    rng = np.random.default_rng(4)
    a = _map(rng, .5); b = deepcopy(a)
    # Make one occupied cell expire between two control ticks.
    i, j = np.argwhere(a.occupied)[0]; ix, iy = a.ix[i, j], a.iy[i, j]
    for m in (a, b): m.stamp[ix, iy] = 1.06 - m.ttl
    a.rebuild(1.05, reuse=True); b.rebuild(1.05); _same(a, b)
    a.rebuild(1.07, reuse=True); b.rebuild(1.07); _same(a, b)
    assert not a.occupied[i, j]
    pts = np.array([[1., .2, .3]])
    cloud = dict(points=pts, points_odom=pts, origin=np.array([0, 0, .3]), origin_odom=np.array([0, 0, .3]),
                 dynamic=np.zeros(1, bool))
    for m in (a, b): m.update(cloud, np.zeros(3), 1.08, rebuild=False)
    a.rebuild(1.09, reuse=True); b.rebuild(1.09); _same(a, b)


def test_slide_within_the_same_window_changes_nothing():
    m = _map(np.random.default_rng(5), 0.)
    state = (m.log.copy(), m.stamp.copy(), m.tags.copy(), m.version)
    m.slide(np.array([.03, .04]))
    assert m.version == state[3]
    for x, y in zip((m.log, m.stamp, m.tags), state[:3]): np.testing.assert_array_equal(x, y)
