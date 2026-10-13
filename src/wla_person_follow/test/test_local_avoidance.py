"""Local obstacle detours, geometry limits and fail-closed unknown space."""
import math
import numpy as np
from wla_person_follow.local_avoidance import LocalAvoidance, scene_from_points
from wla_person_follow.relative_control import RelativeFollower, Safety


def floor():
    x, y = np.meshgrid(np.arange(.1, 3.01, .05), np.arange(-3., 3.01, .05))
    return np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))


def box(x0=1.2, x1=1.45, y0=-.25, y1=.25, height=.2):
    x, y = np.meshgrid(np.arange(x0, x1+.01, .05), np.arange(y0, y1+.01, .05))
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, height)))


def test_height_band_one_centimeter_to_fifty_centimeters():
    values = [[1., y, z] for y, z in zip([-1., -.5, 0., .5, 1.], [.005, .01, .02, .50, .51])]
    scene = scene_from_points(values)
    assert [bool(scene.occupied[scene.cell(1., y)]) for y in [-1., -.5, 0., .5, 1.]] == [False, True, True, True, False]
    other = scene_from_points(values, height_min=.02)
    assert not other.occupied[other.cell(1., -.5)]
    assert other.occupied[other.cell(1., 0.)]


def test_clear_floor_keeps_direct_approach():
    scene = scene_from_points(floor())
    planner = LocalAvoidance()
    v, w = planner.command(scene, 0., 3., 1.4, .2, .45)
    assert v > 0 and abs(w) < 1e-6
    assert planner.last.reason == 'direct_local_path'


def test_center_obstacle_produces_collision_free_detour():
    scene = scene_from_points(np.vstack([floor(), box()]))
    assert not scene.segment_clear((0., 0.), (2., 0.))
    planner = LocalAvoidance()
    v, w = planner.command(scene, 0., 3.5, 1.4, .2, .45)
    assert planner.last.reason == 'local_detour'
    assert max(abs(y) for _, y in planner.last.path) > .55
    assert v > 0 or abs(w) > .01
    assert scene.command_clear(v, w)
    assert all(scene.segment_clear(a, b) for a, b in zip(planner.last.path, planner.last.path[1:]))
    side = planner.side
    planner.command(scene, .01, 3.5, 1.4, .2, .45)
    assert planner.side == side


def test_closed_wall_and_unknown_space_cannot_be_driven_through():
    for points in [np.empty((0, 3)), box(.70, .80, -3., 3.)]:
        scene = scene_from_points(points)
        planner = LocalAvoidance()
        assert planner.command(scene, 0., 3., 1.4, .2, .45) == (0., 0.)
        assert not planner.last.path


def test_unobserved_rear_remains_blocked_and_drop_is_occupied():
    scene = scene_from_points(np.vstack([floor(), [[1., .4, -.15]]]))
    assert not scene.clear(-1., 0.)
    assert scene.occupied[scene.cell(1., .4)]
    assert not scene.command_clear(-.1, 0.)


def test_side_can_change_when_previous_detour_becomes_blocked():
    planner = LocalAvoidance()
    first = scene_from_points(np.vstack([floor(), box()]))
    planner.plan(first, 0., 3.5, 1.4)
    old = planner.side
    assert old
    barrier = box(.8, 2.5, .25, 3.) if old > 0 else box(.8, 2.5, -3., -.25)
    second = scene_from_points(np.vstack([floor(), box(), barrier]))
    result = planner.plan(second, 0., 3.5, 1.4)
    assert result.path and planner.side == -old


def ready(core, now=10., distance=3.5):
    stamp = int(now*1e9)
    core.receive(dict(schema='wla-relative-detections/v1', base_frame='r680_mapping_floor',
        enabled=True, session='detector', epoch=0, stamp_ns=stamp,
        objects=[dict(detected=True,label='person',id='detector:0:1',yaw=0.,distance=distance)]), now, stamp)
    return core.command('start', 'request', 'follow', 'detector:0:1', 1., now)


def test_core_uses_detour_even_when_straight_safety_path_is_blocked():
    planner = LocalAvoidance(); core = RelativeFollower(local_planner=planner)
    assert ready(core)[0]
    scene = scene_from_points(np.vstack([floor(), box()]))
    v, w = core.tick(10.05, Safety(True, 1.2, False), scene)
    assert (v > 0 or abs(w) > 0) and core.reason == 'local_detour'
    assert scene.command_clear(v, w)
    assert core.tick(10.4, Safety(True, 1.2, False), scene) == (0., 0.)
    assert core.reason == 'observations_stale'


def test_no_scene_no_lease_or_emergency_never_uses_legacy_forward_path():
    planner = LocalAvoidance(); core = RelativeFollower(local_planner=planner)
    assert ready(core)[0]
    scene = scene_from_points(floor())
    assert core.tick(10.05, Safety(True, 10., True)) == (0., 0.)
    assert core.reason == 'local_scene_unavailable'
    assert core.tick(10.1, Safety(True, .4, True), scene) == (0., 0.)
    assert core.reason == 'emergency_obstacle'
    assert core.tick(12., Safety(True, 10., True), scene) == (0., 0.)
    assert core.state == 'expired'
