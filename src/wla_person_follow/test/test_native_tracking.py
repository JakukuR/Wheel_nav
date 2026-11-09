"""The C++ tracking equals the Python reference bit for bit (phase 2, 2026-11-09).

Random measurement streams (jumps that trip the gate, gaps that lose the
track, repeated stamps, coasting) drive a Python and a C++ track through the
same calls; every return value and all filter state must be identical, not
merely close. The reference curves are compared on random histories and
polylines. The cpp backend must never fall back.
"""
import numpy as np
import pytest
from wla_person_follow import follow_session, native_tracking
from wla_person_follow.native_tracking import (NativePeopleTracks, NativeTargetHistory, NativeVisualTrack,
                                               make_people_tracks, make_visual_track, reference_functions)
from wla_person_follow.sim_core.target_reference import TargetHistory, bezier_reference, bspline_corners
from wla_person_follow.sim_core.tracking import PeopleTracks, VisualTrack


def same(a, b):
    """Exact equality of reference results (None, arrays, tuples, dicts, floats)."""
    if a is None or b is None:
        return a is None and b is None
    if isinstance(a, dict):
        return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)) and not isinstance(a, np.ndarray):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        a, b = np.asarray(a), np.asarray(b)
        return a.shape == b.shape and np.array_equal(a, b)
    return type(a) is type(b) and a == b if not isinstance(a, float) else float(a) == float(b)


def assert_same_track(py, cpp):
    for name in ('x', 'P', 'time', 'seen', 'skips', 'rejected'):
        assert same(getattr(py, name), getattr(cpp, name)), name
    assert same(py.history, cpp.history)
    assert py.trail.as_lists() == cpp.trail.as_lists()
    assert py.trail.diagnostic() == cpp.trail.diagnostic()


def _stream(rng, steps):
    """(z or None, t): a walk with jitter, range jumps, dropouts, gaps and repeats."""
    t, p = 100., np.array([rng.uniform(-3, 3), rng.uniform(-3, 3)])
    v = rng.normal(0, .8, 2)
    for _ in range(steps):
        t += float(rng.choice([0., .033, .067, .1, .3, .9, 2.5], p=[.03, .3, .3, .2, .08, .06, .03]))
        v = v + rng.normal(0, .3, 2)*.1
        p = p + v*.067
        roll = rng.random()
        if roll < .2:
            yield None, t
        elif roll < .27:
            yield p + rng.normal(0, .6, 2), t   # range jump: gate
        else:
            yield p + rng.normal(0, .03, 2), t


OPTIONS = [dict(), dict(gate=13.8), dict(gate=13.8, coast_decay=(.7, .4)),
           dict(meas_std=follow_session.TARGET_MEAS_STD_M, accel_std=follow_session.TARGET_ACCEL_STD_MPS2,
                gate=follow_session.TARGET_INNOVATION_GATE, coast_decay=follow_session.TARGET_COAST_DECAY,
                history_max_gap_s=.6),
           dict(gate=4., gate_max_skips=1, coast_decay=(0., .25), history_window_s=3., history_sample_period_s=.2)]


@pytest.mark.parametrize('seed', range(12))
def test_visual_track_matches_reference(seed):
    rng = np.random.default_rng(seed)
    options = OPTIONS[seed % len(OPTIONS)]
    timeout = float(rng.choice([.8, 1.5, 2.]))
    py, cpp = VisualTrack(timeout, **options), NativeVisualTrack(timeout, **options)
    assert same(py.predicted_position(0.), cpp.predicted_position(0.))
    assert same(py.update(None, 0.), cpp.update(None, 0.))
    for z, t in _stream(rng, 300):
        assert same(py.update(z, t), cpp.update(z, t))
        assert_same_track(py, cpp)
        for ahead in (0., .05, .4, .75, 1.6, 4.):
            assert same(py.predicted_position(t + ahead), cpp.predicted_position(t + ahead))
    assert py.rejected > 0 or 'gate' not in options


def test_target_history_matches_reference():
    rng = np.random.default_rng(7)
    py, cpp = TargetHistory(4., .3, .5), NativeTargetHistory(4., .3, .5)
    t = 0.
    for _ in range(400):
        t += float(rng.choice([-.1, 0., .05, .2, .45, .7]))
        point = rng.normal(0, 2, 2)
        py.update(point, t); cpp.update(point, t)
        assert same(py.points(), cpp.points())
        assert py.as_lists() == cpp.as_lists() and py.diagnostic() == cpp.diagnostic()
    for bad in ([np.nan, 0.], [0., 0., 0.]):
        for h in (py, cpp):
            with pytest.raises(ValueError, match='invalid_target_history_observation'):
                h.update(bad, t + 1.)
    for args in ((1., .4, .6), (5., .1, .6), (20., .2, .6), (5., .4, 3.), (np.nan, .4, .6)):
        for cls in (TargetHistory, NativeTargetHistory):
            with pytest.raises(ValueError, match='invalid_target_history_configuration'):
                cls(*args)


@pytest.mark.parametrize('seed', range(6))
def test_people_tracks_match_reference(seed):
    rng = np.random.default_rng(100 + seed)
    py, cpp = PeopleTracks(), NativePeopleTracks()
    people = {key: rng.uniform(-4, 4, 2) for key in ('a', 'b', 3, 'd', ('e', 1))}
    t = 0.
    for step in range(200):
        t += float(rng.choice([.033, .067, .1, .6, 1.8], p=[.4, .3, .2, .07, .03]))
        detections = {}
        for key in people:
            people[key] = people[key] + rng.normal(0, .05, 2)
            if rng.random() < .55:
                detections[key] = np.r_[people[key], rng.uniform(0, 1)]  # rows may carry extra values
        if rng.random() < .1:
            key = list(people)[int(rng.integers(len(people)))]
            py.drop(key); cpp.drop(key)
        a, b = py.update(detections, None, t), cpp.update(detections, None, t)
        assert same(a, b), step
        assert py.keys() == cpp.keys()
        for ahead in (0., .3, 1.2, 2.):
            assert same(py.predict(t + ahead), cpp.predict(t + ahead))
        assert py.keys() == cpp.keys()


def _history(rng):
    n = int(rng.choice([0, 1, 2, 3, 8, 14]))
    walk = np.cumsum(rng.normal(0, float(rng.choice([.005, .02, .15])), (n, 2)), axis=0)
    if n > 2 and rng.random() < .3:
        walk[int(rng.integers(1, n))] = walk[0]  # a reversal back to the start
    return [p for p in walk + rng.uniform(-3, 3, 2)]


def test_bezier_reference_matches_reference():
    rng = np.random.default_rng(11)
    for _ in range(3000):
        history = _history(rng)
        assert same(bezier_reference(history), native_tracking.native_bezier_reference(history))
    with pytest.raises(ValueError, match='nonfinite_target_history'):
        native_tracking.native_bezier_reference([[0., 0.], [np.inf, 1.]])


def test_bspline_corners_matches_reference():
    rng = np.random.default_rng(12)
    for _ in range(3000):
        n = int(rng.integers(1, 8))
        path = np.cumsum(rng.normal(0, float(rng.choice([.05, .4, 1.5])), (n, 2)), axis=0)
        if n > 2 and rng.random() < .3:
            path[2] = path[1]                       # duplicate vertex
        if n > 3 and rng.random() < .3:
            path[3] = 2*path[2] - path[1]           # collinear vertex
        corner = float(rng.choice([0., .1, .35, .35*.6, 1.]))
        step = float(rng.choice([.05, .02, .2]))
        assert same(bspline_corners(path, corner, step), native_tracking.native_bspline_corners(path, corner, step))
    for function in (bspline_corners, native_tracking.native_bspline_corners):
        with pytest.raises((IndexError, ValueError)):
            function(np.empty((0, 2)))


def test_backend_selection_never_falls_back(monkeypatch):
    assert make_visual_track('python').__class__ is VisualTrack
    assert make_visual_track('cpp', 2., gate=13.8).backend == 'cpp'
    assert make_people_tracks('cpp').backend == 'cpp'
    assert reference_functions('python') == (bezier_reference, bspline_corners)
    with pytest.raises(ValueError):
        make_people_tracks('gpu')
    monkeypatch.setattr(native_tracking, '_core', None)
    for call in (lambda: make_visual_track('cpp'), lambda: make_people_tracks('cpp'),
                 lambda: reference_functions('cpp')):
        with pytest.raises(RuntimeError):
            call()


def test_cpp_engine_and_tracker_use_the_core():
    from wla_person_follow.sim_runtime import SimulationEngine
    engine = SimulationEngine(np.zeros(3), 0., numeric_backend='cpp')
    assert engine.people.backend == 'cpp'
    assert engine.controller.bezier_reference is native_tracking.native_bezier_reference
    assert SimulationEngine(np.zeros(3), 0.).people.__class__ is PeopleTracks
    tracker = follow_session.FollowTracker('7', 0., numeric_backend='cpp')
    tracker._accept(np.array([2., 0.]), np.zeros(3), 0.)
    assert tracker.track.backend == 'cpp'
    with pytest.raises(ValueError):
        follow_session.FollowTracker('7', 0., numeric_backend='gpu')
