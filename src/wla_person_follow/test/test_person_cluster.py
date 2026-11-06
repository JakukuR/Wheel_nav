"""Connectivity finds the followed person; a wall or a bystander stays in the cloud."""
import numpy as np

from wla_person_follow.person_cluster import select_person
from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap


def _column(x, y, radius=.12, low=.1, high=1.6, step=.04):
    a, b, c = np.meshgrid(np.arange(x-radius, x+radius, step), np.arange(y-radius, y+radius, step),
                          np.arange(low, high, step), indexing='ij')
    return np.c_[a.ravel(), b.ravel(), c.ravel()]


def _wall(y, x=(0., 3.), high=1.6, step=.04):
    a, c = np.meshgrid(np.arange(x[0], x[1], step), np.arange(.1, high, step), indexing='ij')
    return np.c_[a.ravel(), np.full(a.size, y), c.ravel()]


def _rows(*parts):
    return np.vstack(parts), np.concatenate([np.full(len(p), i) for i, p in enumerate(parts)])


def test_two_legs_joined_only_by_the_torso_are_one_person():
    legs = np.vstack((_column(2., -.12, .06, .1, .9), _column(2.3, .12, .06, .1, .9)))
    torso = np.vstack((_column(2.15, 0., .2, .85, 1.5)))
    points, labels = _rows(legs, torso, _column(3.5, 1.5))
    mask, info = select_person(points, np.array([2.15, 0.]))
    assert info['state'] == 'selected'
    assert mask[labels < 2].all() and not mask[labels == 2].any()


def test_low_band_alone_would_split_the_legs():
    legs = np.vstack((_column(2., -.25, .06, .1, .5), _column(2.5, .25, .06, .1, .5)))
    mask, info = select_person(legs, np.array([2.25, 0.]), seed_radius=.45)
    assert info['state'] == 'no_body_height' and not mask.any()


def test_person_touching_a_wall_is_refused_and_the_wall_kept():
    points, labels = _rows(_column(1.5, .1), _wall(.24))
    mask, info = select_person(points, np.array([1.5, .1]))
    assert info['state'] in ('selected', 'merged_too_wide')
    assert not mask[labels == 1].sum() > .25*(labels == 1).sum()


def test_separate_wall_is_untouched():
    points, labels = _rows(_column(1.5, .1), _wall(.7))
    mask, info = select_person(points, np.array([1.5, .1]))
    assert info['state'] == 'selected' and mask[labels == 0].all() and not mask[labels == 1].any()


def test_bystander_in_the_same_component_stays():
    bridge = np.c_[np.full(10, 2.), np.linspace(0., .6, 10), np.full(10, 1.2)]
    points, labels = _rows(_column(2., 0.), _column(2., .6), bridge)
    mask, _ = select_person(points, np.array([2., 0.]), others=np.array([[2., .6]]))
    assert mask[labels == 0].all() and not mask[labels == 1].any()


def test_missing_seed_never_selects_anything():
    mask, info = select_person(_column(3.5, 1.5), np.array([2., 0.]))
    assert not mask.any() and info['state'] != 'selected'


def test_feet_below_the_clustering_band_go_with_the_person():
    feet = np.array([[2.05, 0., .06], [1.95, .05, .05]])
    floor = np.array([[2.6, 0., .01], [1.2, .0, .02]])
    points, labels = _rows(_column(2., 0.), feet, floor)
    mask, _ = select_person(points, np.array([2., 0.]))
    assert mask[labels == 1].all() and not mask[labels == 2].any()


def _map_cloud(hits):
    empty = np.empty((0, 3))
    return dict(points=empty, points_odom=empty, origin=np.array([0., 0., .5]),
                origin_odom=np.array([0., 0., .5]), dynamic=np.empty(0, dtype=bool),
                marking_points_odom=np.asarray(hits, dtype=float).reshape(-1, 3))


def test_cell_occupied_before_the_person_came_near_cannot_be_cleared():
    mapping = RollingDepthMap(resolution=.05, size=160, ttl=12.)
    robot = np.zeros(3)
    chair, leg = (2., .3, .3), (2., -.3, .3)
    mapping.update(_map_cloud([chair]), robot, 1.)
    mapping.update(_map_cloud([leg]), robot, 3.)
    trail = np.array([[.5, 3.5, 0.], [1., 3.5, 0.], [3., 2., -.3], [3.2, 2., .3]])
    kept = mapping.drop_established(np.array([chair, leg]), trail, 3.2)
    assert np.allclose(kept, [leg])


def test_no_trail_clears_nothing():
    mapping = RollingDepthMap(resolution=.05, size=160, ttl=12.)
    mapping.update(_map_cloud([(2., 0., .3)]), np.zeros(3), 1.)
    assert not len(mapping.drop_established(np.array([(2., 0., .3)]), np.empty((0, 3)), 1.))


def _speckle_scene():
    depth = np.full((120, 160), 3.)
    depth[:, :] += np.linspace(0., .5, 160)[None, :]
    return depth


def test_depth_speckles_drop_a_floating_patch_only():
    from wla_person_follow.sim_runtime import drop_depth_speckles
    depth = _speckle_scene()
    depth[40:46, 50:56] = 1.8
    original = depth.copy()
    result = drop_depth_speckles(depth, 150)
    assert np.isnan(result[40:46, 50:56]).all()
    assert np.isfinite(result[:30]).all()
    assert np.array_equal(depth, original)


def test_depth_speckles_keep_a_thin_tall_pole_and_invalid_pixels():
    from wla_person_follow.sim_runtime import drop_depth_speckles
    depth = _speckle_scene()
    depth[10:110, 80:84] = 1.5
    depth[0:5, 0:5] = np.nan
    result = drop_depth_speckles(depth, 150)
    assert np.allclose(result[10:110, 80:84], 1.5)
    assert np.isnan(result[0:5, 0:5]).all()
