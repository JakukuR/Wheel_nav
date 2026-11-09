"""Followed-person Kalman without the 1 m/s clamp (2026-11-09 retune).

Replay of the 2026-11-06 A* walk: 24 % of the frames were faster than 1 m/s and
the clamp held them there; the range jumps of a person leaving the image edge
are now skipped by an innovation gate, and a long coast fades its velocity.
"""
from copy import copy
import numpy as np
from wla_person_follow import follow_session
from wla_person_follow.sim_core.tracking import VisualTrack


def _target():
    return VisualTrack(timeout=2., meas_std=follow_session.TARGET_MEAS_STD_M,
                       accel_std=follow_session.TARGET_ACCEL_STD_MPS2,
                       gate=follow_session.TARGET_INNOVATION_GATE,
                       coast_decay=follow_session.TARGET_COAST_DECAY)


def test_fast_walk_is_not_clamped():
    tr = _target()
    for k in range(60):
        _, v, _ = tr.update(np.array([1.6 * .067 * k, 0.]), .067 * k)
    assert abs(np.linalg.norm(v) - 1.6) < .05


def test_single_range_jump_is_skipped_but_a_real_one_lands():
    tr = _target()
    for k in range(40):
        tr.update(np.array([2. + .8 * .067 * k, 0.]), .067 * k)
    t = .067 * 40; x = 2. + .8 * t
    tr.update(np.array([x + .5, 0.]), t)
    assert tr.rejected == 1 and tr.seen < t and np.linalg.norm(tr.x[2:]) < 1.
    tr.update(np.array([x + .8 * .067, 0.]), t + .067)
    assert tr.rejected == 1 and tr.seen == t + .067
    for j in range(2, 6):  # the person really is 0.5 m further: lands after two skips
        tr.update(np.array([x + .5 + .8 * .067 * j, 0.]), t + .067 * j)
    assert tr.rejected == 3 and tr.x[0] > x + .5


def test_coast_keeps_full_velocity_then_fades():
    tr = _target()
    for k in range(40):
        tr.update(np.array([.8 * .067 * k, 0.]), .067 * k)
    seen, p, v = tr.seen, tr.x[:2].copy(), tr.x[2:].copy()
    delay, tau = follow_session.TARGET_COAST_DECAY
    early = copy(tr).update(None, seen + delay)
    np.testing.assert_allclose(early[0], p + v * delay)
    late = copy(tr).update(None, seen + delay + 1.)
    k = np.exp(-1. / tau)
    np.testing.assert_allclose(late[0], p + v * (delay + tau * (1 - k)))
    np.testing.assert_allclose(late[1], v * k)
    # The shallow copy must not touch the live track.
    np.testing.assert_array_equal(tr.x[:2], p); np.testing.assert_array_equal(tr.x[2:], v)
    assert tr.seen == seen


def test_coast_is_continuous_across_repeated_predictions():
    tr = _target()
    for k in range(40):
        tr.update(np.array([.8 * .067 * k, 0.]), .067 * k)
    once = copy(tr).update(None, tr.seen + 1.5)[0]
    step = copy(tr)
    for dt in (.3, .6, .9, 1.2, 1.5):
        stepped = step.update(None, tr.seen + dt)[0]
    np.testing.assert_allclose(stepped, once, atol=1e-9)
