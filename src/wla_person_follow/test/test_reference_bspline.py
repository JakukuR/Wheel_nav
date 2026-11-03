"""Reference polylines get B-spline corners only where clearance allows."""
from dataclasses import replace

import numpy as np

from wla_person_follow.sim_core.common import Tuning
from wla_person_follow.sim_core.hybrid_controller import HybridFollower
from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
from wla_person_follow.sim_core.target_reference import bspline_corners


def _map(hits=()):
    mapping = RollingDepthMap(resolution=.05, size=120, ttl=12.)
    hits = np.asarray(hits, dtype=float).reshape(-1, 3)
    empty = np.empty((0, 3))
    mapping.update(dict(points=empty, points_odom=empty, origin=np.array([0., 0., .5]),
                        origin_odom=np.array([0., 0., .5]), dynamic=np.empty(0, dtype=bool),
                        marking_points_odom=hits), np.zeros(3), 1.)
    return mapping


def _turn(path):
    heading = np.unwrap(np.arctan2(*np.diff(path, axis=0).T[::-1]))
    return np.abs(np.diff(heading)).max()


def test_corner_is_rounded_with_fixed_endpoints_and_straight_legs():
    polyline = np.array([[0., 0.], [1., 0.], [1., 1.]])
    curve = bspline_corners(polyline, .35)
    np.testing.assert_allclose(curve[0], polyline[0])
    np.testing.assert_allclose(curve[-1], polyline[-1])
    assert .05 < np.min(np.linalg.norm(curve - polyline[1], axis=1)) < .2
    assert np.abs(curve[curve[:, 0] < .6, 1]).max() < .01
    assert _turn(curve) < np.pi / 2 / 4  # No single 90 degree kink left.
    assert np.all(np.linalg.norm(np.diff(curve, axis=0), axis=1) <= .051)


def test_short_segments_limit_the_corner_and_straight_lines_are_unchanged():
    polyline = np.array([[0., 0.], [.2, 0.], [.2, .2]])
    curve = bspline_corners(polyline, .35)
    assert np.min(np.linalg.norm(curve - polyline[1], axis=1)) < .06
    line = np.array([[0., 0.], [.5, 0.], [1., 0.]])
    np.testing.assert_allclose(bspline_corners(line, .35)[:, 1], 0., atol=1e-9)


def test_controller_rounds_in_free_space():
    follower = HybridFollower(Tuning())
    path = np.array([[0., 0.], [1.5, 0.], [1.5, 1.5]])
    rounded = follower._round_corners(path, _map(), follower.c)
    assert len(rounded) > 3 and _turn(rounded) < np.pi / 8


def test_controller_keeps_polyline_when_the_curve_would_lose_clearance():
    follower = HybridFollower(Tuning())
    path = np.array([[0., 0.], [1.5, 0.], [1.5, 1.5]])
    # Obstacle just inside the corner: cutting it moves the path closer.
    blocker = [(1.5 - .32 + dx, .32 + dy, .5) for dx in (-.03, 0., .03) for dy in (-.03, 0., .03)]
    mapping = _map(blocker)
    rounded = follower._round_corners(path, mapping, follower.c)
    need = min(follower.c.radius + follower.c.margin, float(np.min(mapping.query(
        np.vstack([np.linspace(path[0], path[1], 40), np.linspace(path[1], path[2], 40)])))))
    assert float(np.min(mapping.query(rounded))) >= need - 1e-3
    disabled = replace(follower.c, reference_corner_m=0.)
    assert follower._round_corners(path, mapping, disabled) is path
