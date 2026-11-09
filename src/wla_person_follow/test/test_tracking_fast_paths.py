"""The allocation-free tracking paths equal the copy-based ones they replace (2026-11-09).

Profiling a replay of the 2026-11-06 walk showed the tracker snapshot, its
prediction and the people prediction were recomputed about 19 times per
planning cycle through copy/deepcopy of every Kalman filter.
"""
from copy import copy, deepcopy
import numpy as np
from wla_person_follow import follow_session
from wla_person_follow.sim_core.tracking import PeopleTracks, VisualTrack


def _walk(track, rng, steps=60, start=0.):
    t, p, v = start, np.array([2., 0.]), np.array([.9, .3])
    for _ in range(steps):
        t += .1; p = p + v*.1
        track.update(p + rng.normal(0., .03, 2), t)
    return t


def test_predicted_position_matches_update_none():
    rng = np.random.default_rng(3)
    for decay in (None, follow_session.TARGET_COAST_DECAY):
        track = VisualTrack(timeout=2., gate=13.8, coast_decay=decay)
        assert track.predicted_position(0.) is None
        t = _walk(track, rng)
        for ahead in (0., .05, .3, .69, .71, 1.2, 1.99, 2.5, 6.):
            expected = copy(track).update(None, t+ahead)[0]
            np.testing.assert_allclose(track.predicted_position(t+ahead), expected, rtol=0, atol=1e-12)
        x = track.x.copy(); track.predicted_position(t+1.)
        np.testing.assert_array_equal(track.x, x)


def test_people_predict_matches_deepcopy_update_and_leaves_tracks():
    rng = np.random.default_rng(5)
    people = PeopleTracks()
    for k in range(30):
        t = .1*k
        people.update({'a': np.array([1.+.05*k, 0.]) + rng.normal(0, .02, 2),
                       **({'b': np.array([3., 1.-.03*k])} if k < 22 else {})}, None, t)
    keys = set(people.tracks); assert keys == {'a', 'b'}
    for now in (t, t+.4, t+1.2, t+2.):
        old = deepcopy(people).update({}, None, now)
        new = people.predict(now)
        assert [r['id'] for r in new] == [r['id'] for r in old]
        for a, b in zip(new, old):
            np.testing.assert_allclose(a['p'], b['p'], atol=1e-12)
            np.testing.assert_allclose(a['v'], b['v'], atol=1e-12)
            assert a['r'] == b['r'] and a['age'] == b['age']
    assert set(people.tracks) == keys


def test_history_lists_are_cached_but_refreshed_and_independent():
    track = VisualTrack(timeout=2.)
    rng = np.random.default_rng(1)
    t = _walk(track, rng, steps=20)
    first = track.trail.as_lists()
    assert first == [p.tolist() for p in track.history]
    first[0][0] = 99.
    assert track.trail.as_lists() == [p.tolist() for p in track.history]
    track.update(np.array([5., 5.]), t+.1)
    assert track.trail.as_lists() == [p.tolist() for p in track.history]
