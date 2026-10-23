"""A confirmed target silhouette may retire its old hits, not nearby walls."""
import numpy as np
import sys
from types import SimpleNamespace

from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
from wla_person_follow.sim_runtime import target_points_from_removed_depth


def _cloud(*, hits=(), target=()):
    hits = np.asarray(hits, dtype=float).reshape(-1, 3)
    target = np.asarray(target, dtype=float).reshape(-1, 3)
    empty = np.empty((0, 3), dtype=float)
    return dict(points=empty, points_odom=empty, origin=np.array([0., 0., .5]),
                origin_odom=np.array([0., 0., .5]), dynamic=np.empty(0, dtype=bool),
                marking_points_odom=hits, target_points_odom=target)


def _cell(mapping, point):
    cell = np.floor(np.asarray(point)[:2] / mapping.res).astype(int) - mapping.lower
    return tuple(cell)


def test_confirmed_target_cells_remove_old_person_hit_but_keep_nearby_wall():
    mapping = RollingDepthMap(resolution=.1, size=40, ttl=12.)
    robot = np.zeros(3)
    person = (1.05, 0., .5)
    wall = (1.25, 0., .5)  # Within 0.35 m of the person, but not its mask cell.
    mapping.update(_cloud(hits=(person, wall)), robot, 1.)
    assert mapping.occupied[_cell(mapping, person)]
    assert mapping.occupied[_cell(mapping, wall)]

    mapping.update(_cloud(target=(person,)), robot, 1.1)
    assert not mapping.occupied[_cell(mapping, person)]
    assert mapping.known[_cell(mapping, person)]
    assert mapping.occupied[_cell(mapping, wall)]


def test_current_nontarget_hit_wins_in_a_shared_map_cell():
    mapping = RollingDepthMap(resolution=.1, size=40, ttl=12.)
    robot = np.zeros(3)
    shared = (1.05, 0., .5)
    mapping.update(_cloud(hits=(shared,)), robot, 1.)
    mapping.update(_cloud(hits=(shared,), target=(shared,)), robot, 1.1)
    assert mapping.occupied[_cell(mapping, shared)]


def test_target_only_clear_expires_to_unknown_without_fresh_free_ray():
    mapping = RollingDepthMap(resolution=.1, size=40, ttl=12.)
    robot = np.zeros(3)
    person = (1.05, 0., .5)
    mapping.update(_cloud(hits=(person,)), robot, 1.)
    mapping.update(_cloud(target=(person,)), robot, 1.1)
    mapping.rebuild(1.7)
    assert not mapping.occupied[_cell(mapping, person)]
    assert not mapping.known[_cell(mapping, person)]


def test_only_mask_removed_points_inside_the_target_radius_are_sent_to_map(monkeypatch):
    def fake_deproject(depth, intrinsics, rotation, translation, minimum, maximum, stride):
        assert np.isfinite(depth).any()
        return np.array([[1.05, 0., .5], [1.50, 0., .5]])

    monkeypatch.setitem(sys.modules, '_wla_sim_geometry', SimpleNamespace(deproject=fake_deproject))
    depth = np.array([[1., np.nan]])
    points = target_points_from_removed_depth(
        depth, (1., 1., 0., 0.), np.eye(3), np.zeros(3), np.zeros(3),
        np.array([1.05, 0.]), radius=.35)
    assert points.shape == (1, 3)
    np.testing.assert_allclose(points[0], [1.05, 0., .5])
