"""The C++ planner equals the Python reference exactly (phase 3, 2026-11-09).

Random scenes (walls across the straight route, known floor, unseen flanks,
people) are built into a Python and a C++ rolling map (equal, phase 1). The
Python HybridFollower / ApproachGoal and the C++ ones then run the same call
sequences; commands, every feasibility count, detour report, reference path
and approach goal must be identical, not merely close. The cpp backend must
never fall back.
"""
from dataclasses import replace
import numpy as np
import pytest
from wla_person_follow import native_planning
from wla_person_follow.native_map import NativeRollingDepthMap
from wla_person_follow.native_planning import (NativeApproachGoal, NativeHybridFollower, make_approach_goal,
                                               make_controller)
from wla_person_follow.sim_core.approach_goal import ApproachGoal, SWEEP_RADIUS
from wla_person_follow.sim_core.common import Tuning
from wla_person_follow.sim_core.hybrid_controller import HybridFollower, approach_tangent, chassis_yaw
from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap

STATUSES = ('visible', 'visible', 'visible', 'coast', 'search_heading', 'search_last_seen', 'lost', 'holding')


class Policy:
    """TrackingPolicy stand-in: the fields and pure predicates the MPC reads."""

    def __init__(self, rng):
        self.search_arrival_m = float(rng.choice([.35, .6]))
        self.search_heading = None if rng.random() < .3 else float(rng.uniform(-3, 3))
        self.search_started = None if rng.random() < .3 else 99. + float(rng.uniform(0, 3))
        self.search_side = float(rng.choice([1., -1.]))
        self._allowed = bool(rng.random() < .8)
        self._search = bool(rng.random() < .5)

    def translation_allowed(self, now):
        return self._allowed

    def active_search(self, now):
        return self._search


def _scene(rng, pose, target):
    """Clouds: floor ahead (known free), walls across the route, a few boxes."""
    p = pose[:2]
    heading = np.arctan2(*(target - p)[::-1])
    floor_r = rng.uniform(.3, 4.5, 3000)
    floor_a = pose[2] + rng.uniform(-.75, .75, 3000)
    floor = np.c_[p[0] + floor_r*np.cos(floor_a), p[1] + floor_r*np.sin(floor_a), rng.uniform(-.02, .02, 3000)]
    walls = []
    for _ in range(int(rng.choice([0, 1, 1, 2, 3]))):
        along = float(rng.uniform(.6, 2.5)); offset = float(rng.uniform(-1., 1.)); length = float(rng.uniform(.3, 2.2))
        centre = p + along*np.array([np.cos(heading), np.sin(heading)]) + offset*np.array([-np.sin(heading), np.cos(heading)])
        tilt = heading + np.pi/2 + rng.normal(0, .3)
        s = rng.uniform(-length/2, length/2, 400)
        walls.append(np.c_[centre[0] + s*np.cos(tilt), centre[1] + s*np.sin(tilt), rng.uniform(.1, 1.2, 400)])
    points = np.vstack([floor] + walls)
    keep = np.abs(np.arctan2(points[:, 1]-p[1], points[:, 0]-p[0]) - pose[2] + np.pi) % (2*np.pi) - np.pi
    points = points[np.abs(keep) < .78]
    c, s = np.cos(pose[2]), np.sin(pose[2])
    rel = points[:, :2] - p
    local = np.c_[rel @ np.array([[c, -s], [s, c]]), points[:, 2]]
    return dict(points=local, points_odom=points, dynamic=np.zeros(len(points), dtype=bool),
                origin=np.array([0., 0., .55]), origin_odom=np.array([p[0], p[1], .55]))


def _maps(rng, pose, target, t):
    maps = RollingDepthMap(resolution=.05, size=160, robot_radius=.19), \
        NativeRollingDepthMap(resolution=.05, size=160, robot_radius=.19)
    for m in maps:
        m.slide(pose[:2]); m.rebuild(t)
    free = float(rng.choice([0., .35, .5]))
    for m in maps:  # the engine's assumed-free disc round the robot (camera blind zone)
        m.local_assumed_free_radius = free
        m.local_assumed_free_center = pose[:2].copy()
    for k in range(int(rng.integers(1, 5))):
        view = pose + np.r_[rng.normal(0, .05, 2), rng.normal(0, .5)]
        cloud = _scene(rng, view, target)
        for m in maps:
            m.update(cloud, view, t + .1*k, rebuild=False)
    for m in maps:
        m.slide(pose[:2]); m.rebuild(t + .5)
    np.testing.assert_array_equal(maps[0].dist, maps[1].dist)
    np.testing.assert_array_equal(maps[0].free_dist, maps[1].free_dist)
    return maps


def _people(rng, pose):
    return [dict(id=k, p=pose[:2] + rng.uniform(-2.5, 2.5, 2), v=rng.normal(0, .4, 2), r=float(rng.choice([.2, .25, .3])))
            for k in range(int(rng.choice([0, 0, 1, 2, 4])))]


def _tuning(rng):
    c = Tuning(grid_route_planner=str(rng.choice(['dijkstra', 'astar'])))
    mode = int(rng.integers(0, 4))
    if mode == 1:
        c = replace(c, desired_gap=1.23, target_stop_gap=1.23, target_clearance_max=0., max_v=.30)
    elif mode == 2:
        c = replace(c, radius=SWEEP_RADIUS, desired_gap=0., target_stop_gap=.025, target_hard_gap=0.,
                    target_clearance_max=0., max_v=.20)
    elif mode == 3:
        c = replace(c, desired_gap=0., target_stop_gap=.05, target_hard_gap=0., target_clearance_max=0., max_v=.4)
    if rng.random() < .2:
        c = replace(c, chassis_tau_w=0.)
    return c


def assert_same_controller(py, cpp):
    assert py.last_infeasible == cpp.last_infeasible
    assert py.last_candidates == cpp.last_candidates
    np.testing.assert_array_equal(np.asarray(py.last_reference_path).reshape(-1, 2), cpp.last_reference_path)
    assert py.stuck_cycles == cpp.stuck_cycles
    assert py.backed == cpp.backed
    assert py.detour.last == cpp.detour.last
    assert len(py.trail) == len(cpp.trail)
    for a, b in zip(py.trail, cpp.trail):
        np.testing.assert_array_equal(a, b)


@pytest.mark.parametrize('seed', range(24))
def test_random_commands_match_reference(seed):
    rng = np.random.default_rng(1000 + seed)
    pose = np.array([rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-np.pi, np.pi)])
    ahead = pose[2] + rng.normal(0, .5)
    target = pose[:2] + rng.uniform(1., 4.)*np.array([np.cos(ahead), np.sin(ahead)])
    t = 100.
    maps = _maps(rng, pose, target, t)
    py, cpp = HybridFollower(Tuning()), NativeHybridFollower(Tuning())
    previous = np.array([rng.uniform(0, .6), rng.uniform(-.4, .4)])
    history = [target - k*.15*np.array([np.cos(ahead), np.sin(ahead)]) + rng.normal(0, .03, 2) for k in range(12)][::-1]
    c = _tuning(rng)
    for step in range(30):
        t += .1
        status = str(rng.choice(STATUSES))
        policy = None if rng.random() < .3 else Policy(rng)
        people = _people(rng, pose)
        reference = None
        face = None
        if rng.random() < .15:
            reference = np.vstack((pose[:2], (pose[:2] + target)/2 + rng.normal(0, .3, 2), target))
            face = target + rng.normal(0, .2, 2) if rng.random() < .5 else None
        velocity = rng.normal(0, .3, 2) if rng.random() < .6 else np.zeros(2)
        measured = previous + rng.normal(0, .05, 2) if rng.random() < .7 else None
        tgt = None if rng.random() < .05 else target
        recovery = bool(rng.random() < .05)
        if rng.random() < .1:
            c = _tuning(rng)
        py.c, cpp.c = c, c
        out = [f.command(pose.copy(), previous.copy(), tgt, velocity, list(history), m, people, status, policy=policy,
                         policy_time=t, reference_path=reference, face_point=face, measured=measured, recovery=recovery)
               for f, m in ((py, maps[0]), (cpp, maps[1]))]
        np.testing.assert_array_equal(np.asarray(out[0], dtype=float), out[1])
        assert_same_controller(py, cpp)
        # Drive on: the robot follows the command, the person walks on.
        v, w = out[0]
        pose = pose + np.array([v*.1*np.cos(pose[2]), v*.1*np.sin(pose[2]), w*.1])
        previous = np.asarray(out[0], dtype=float)
        target = target + velocity*.1
        history = history[1:] + [target.copy()]
        if rng.random() < .05:
            py.reset(); cpp.reset()


@pytest.mark.parametrize('seed', range(12))
def test_grid_route_and_helpers_match_reference(seed):
    rng = np.random.default_rng(2000 + seed)
    pose = np.array([rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-np.pi, np.pi)])
    target = pose[:2] + rng.uniform(1.5, 4.)*np.array([np.cos(pose[2]), np.sin(pose[2])])
    maps = _maps(rng, pose, target, 100.)
    py, cpp = HybridFollower(Tuning()), NativeHybridFollower(Tuning())
    for _ in range(6):
        c = _tuning(rng)
        goal = pose[:2] + rng.uniform(-3, 3, 2)
        a = py.detour._grid_route(pose[:2], goal, maps[0], c)
        b = cpp.detour._grid_route(pose[:2], goal, maps[1], c)
        assert (a is None) == (b is None)
        if a is not None:
            np.testing.assert_array_equal(a, b)
        for w in rng.uniform(-1.2, 1.2, 20):
            assert float(chassis_yaw(w, c)) == native_planning._core.chassis_yaw(float(w), c)
        people = _people(rng, pose)
        command = rng.uniform(-.3, .8, 2)
        expected = approach_tangent(pose, command, people, goal, c.radius)
        got = native_planning._core.approach_tangent(
            pose, command, np.asarray([o['p'] for o in people], dtype=float).reshape(-1, 2),
            np.asarray([o['v'] for o in people], dtype=float).reshape(-1, 2),
            np.asarray([o['r'] for o in people], dtype=float), goal, c.radius)
        assert expected == got
        path = np.vstack((pose[:2], rng.uniform(-2, 2, (int(rng.integers(1, 5)), 2))))
        np.testing.assert_array_equal(ApproachGoal._samples(path), native_planning._core.approach_samples(path))


def _same_approach(a, b):
    assert (a is None) == (b is None)
    if a is None:
        return
    assert list(a) == list(b)
    for key in a:
        if isinstance(a[key], np.ndarray):
            np.testing.assert_array_equal(a[key], b[key], err_msg=key)
        else:
            assert a[key] == b[key], key


@pytest.mark.parametrize('seed', range(10))
def test_approach_goal_matches_reference(seed):
    rng = np.random.default_rng(3000 + seed)
    pose = np.array([rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-np.pi, np.pi)])
    obj = pose[:2] + rng.uniform(.9, 2.5)*np.array([np.cos(pose[2]), np.sin(pose[2])])
    maps = _maps(rng, pose, obj, 100.)
    py, cpp = ApproachGoal(), NativeApproachGoal()
    t = 101.
    size = rng.uniform(.1, .4, 2)
    base = rng.uniform(-1, 1, (60, 2))*size
    for step in range(16):
        t += float(rng.choice([.1, .1, .2, .5]))
        keep = rng.random(len(base)) < (.05 if rng.random() < .1 else .8)
        surface = obj + base[keep] + rng.normal(0, .004, (int(keep.sum()), 2))
        if rng.random() < .15:
            obj = obj + rng.normal(0, .05, 2)
        _same_approach(py.select(pose, surface, maps[0], t), cpp.select(pose, surface, maps[1], t))
        assert py.generation == cpp.generation and py.stable_since == cpp.stable_since
        pose = pose + np.r_[rng.normal(0, .05, 2), rng.normal(0, .05)]


def test_backend_selection_never_falls_back(monkeypatch):
    assert isinstance(make_controller('python'), HybridFollower)
    assert isinstance(make_controller('cpp'), NativeHybridFollower)
    assert isinstance(make_approach_goal('cpp'), NativeApproachGoal)
    with pytest.raises(ValueError):
        make_controller('fortran')
    with pytest.raises(TypeError):  # the C++ planner reads the C++ map only
        NativeHybridFollower().command(np.zeros(3), np.zeros(2), np.ones(2), np.zeros(2), [], RollingDepthMap(), [],
                                       'visible')
    monkeypatch.setattr(native_planning, '_core', None)
    with pytest.raises(RuntimeError):
        make_controller('cpp')
    with pytest.raises(RuntimeError):
        make_approach_goal('cpp')


@pytest.mark.parametrize('seed', range(8))
def test_backing_out_matches_reference(seed):
    """Drive in (trail), then recovery / stuck plans back out along it, stall timeout included."""
    rng = np.random.default_rng(4000 + seed)
    pose = np.array([rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-np.pi, np.pi)])
    target = pose[:2] + 2.5*np.array([np.cos(pose[2]), np.sin(pose[2])])
    maps = _maps(rng, pose - np.r_[.8*np.cos(pose[2]), .8*np.sin(pose[2]), 0.], target, 100.)
    py, cpp = HybridFollower(Tuning()), NativeHybridFollower(Tuning())
    t, previous = 101., np.zeros(2)
    for step in range(40):
        t += float(rng.choice([.1, .1, .6]))
        if step < 8:
            pose = pose - np.r_[.1*np.cos(pose[2]), .1*np.sin(pose[2]), 0.]  # trail behind the robot
        elif rng.random() < .5:
            pose = pose + np.r_[previous[0]*.1*np.cos(pose[2]), previous[0]*.1*np.sin(pose[2]), previous[1]*.1]
        status = 'visible' if rng.random() < .8 else 'coast'
        recovery = step >= 8 and rng.random() < .6
        people = _people(rng, pose)
        out = [f.command(pose.copy(), previous.copy(), target, np.zeros(2), [target], m, people, status,
                         policy=None, policy_time=t, recovery=recovery)
               for f, m in ((py, maps[0]), (cpp, maps[1]))]
        np.testing.assert_array_equal(np.asarray(out[0], dtype=float), out[1])
        assert_same_controller(py, cpp)
        previous = np.asarray(out[0], dtype=float)


def _wall_maps(rng, pose, t):
    """A wall alongside the robot, 0.14-0.20 m from its centre (inside the 0.19 m
    body radius plus slack), sometimes with a second wall across, seen from behind."""
    heading = np.array([np.cos(pose[2]), np.sin(pose[2])]); normal = np.array([-heading[1], heading[0]])
    side = float(rng.choice([-1., 1.])); gap = float(rng.uniform(.14, .20))
    along = rng.uniform(-.6, 1.6, 600)
    wall = pose[:2] + along[:, None]*heading + side*(gap + .025)*normal
    if rng.random() < .5:
        across = rng.uniform(-.8, .8, 300)
        wall = np.vstack((wall, pose[:2] + float(rng.uniform(.35, .6))*heading + across[:, None]*normal))
    walls = np.c_[wall, rng.uniform(.1, 1.2, len(wall))]
    maps = RollingDepthMap(resolution=.05, size=160, robot_radius=.19), \
        NativeRollingDepthMap(resolution=.05, size=160, robot_radius=.19)
    for m in maps:
        m.slide(pose[:2]); m.rebuild(t)
    for k in range(3):
        view = pose - np.r_[(.8 + .1*k)*heading, 0.] + np.r_[rng.normal(0, .02, 2), rng.normal(0, .1)]
        r = rng.uniform(.3, 3., 2000); a = view[2] + rng.uniform(-.75, .75, 2000)
        floor = np.c_[view[0] + r*np.cos(a), view[1] + r*np.sin(a), rng.uniform(-.02, .02, 2000)]
        points = np.vstack((floor, walls))
        rel = (np.arctan2(points[:, 1]-view[1], points[:, 0]-view[0]) - view[2] + np.pi) % (2*np.pi) - np.pi
        points = points[np.abs(rel) < .78]
        c, s = np.cos(view[2]), np.sin(view[2]); d = points[:, :2] - view[:2]
        local = np.c_[d @ np.array([[c, -s], [s, c]]), points[:, 2]]
        cloud = dict(points=local, points_odom=points, dynamic=np.zeros(len(points), dtype=bool),
                     origin=np.array([0., 0., .55]), origin_odom=np.array([view[0], view[1], .55]))
        for m in maps:
            m.update(cloud, view, t + .1*k, rebuild=False)
    for m in maps:
        m.slide(pose[:2]); m.rebuild(t + .5)
    np.testing.assert_array_equal(maps[0].dist, maps[1].dist)
    return maps


@pytest.mark.parametrize('seed', range(10))
def test_escape_inside_obstacle_band_matches_reference(seed):
    """Start inside (or within the slack of) the obstacle band: turning in place and
    moving off the wall stay feasible, identically in both backends."""
    from wla_person_follow.sim_core.hybrid_controller import ESCAPE_SLACK_M
    rng = np.random.default_rng(6000 + seed)
    pose = np.array([rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-np.pi, np.pi)])
    t = 100.
    maps = _wall_maps(rng, pose, t)
    start = float(maps[0].query(pose[:2][None])[0]) - .19 - .001
    assert start < ESCAPE_SLACK_M
    ahead = pose[2] + rng.normal(0, .8)
    target = pose[:2] + rng.uniform(1.5, 3.)*np.array([np.cos(ahead), np.sin(ahead)])
    py, cpp = HybridFollower(Tuning()), NativeHybridFollower(Tuning())
    previous = np.zeros(2)
    if seed % 2 == 0:
        # Arrive along a trail first, so backing out is available as well.
        for j in range(6, 0, -1):
            back = pose - np.r_[.07*j*np.cos(pose[2]), .07*j*np.sin(pose[2]), 0.]
            out = [f.command(back, previous.copy(), target, np.zeros(2), [target], m, [], 'visible',
                             policy=None, policy_time=t, measured=previous.copy())
                   for f, m in ((py, maps[0]), (cpp, maps[1]))]
            np.testing.assert_array_equal(np.asarray(out[0], dtype=float), out[1])
            assert_same_controller(py, cpp)
            t += .1
    for step in range(16):
        t += .1
        out = [f.command(pose.copy(), previous.copy(), target, np.zeros(2), [target], m, [], 'visible',
                         policy=None, policy_time=t, measured=previous.copy())
               for f, m in ((py, maps[0]), (cpp, maps[1]))]
        np.testing.assert_array_equal(np.asarray(out[0], dtype=float), out[1])
        assert_same_controller(py, cpp)
        if step == 0:
            # Was: every rollout rejected at its first point (2026-11-06 17:31).
            assert not py.last_infeasible
        v, w = out[0]
        pose = pose + np.array([v*.1*np.cos(pose[2]), v*.1*np.sin(pose[2]), w*.1])
        previous = np.asarray(out[0], dtype=float)



@pytest.mark.parametrize('wall_x', [.14, .16, .18])
def test_backing_out_from_inside_the_band_matches_reference(wall_x):
    """Nose at a wall after driving up a trail: turning in place is all that is
    feasible, so after three plans both backends back out along the trail."""
    floor = np.array([[x, y, 0.] for x in np.arange(-2., 4., .05) for y in np.arange(-2., 2.01, .05)])
    wall = np.array([[wall_x, y, z] for y in np.arange(-.6, .61, .05) for z in (.2, .4)])
    points = np.r_[floor, wall]
    maps = RollingDepthMap(resolution=.05, size=160, robot_radius=.19), \
        NativeRollingDepthMap(resolution=.05, size=160, robot_radius=.19)
    for m in maps:
        m.slide(np.zeros(2))
        m.update(dict(points=points, points_odom=points, dynamic=np.zeros(len(points), bool),
                      origin=np.array([0, 0, .3]), origin_odom=np.array([0, 0, .3])), np.zeros(3), 0.)
    py, cpp = HybridFollower(Tuning()), NativeHybridFollower(Tuning())
    target, hist, t = np.array([2., 0.]), [np.array([1.5 + .1*i, 0.]) for i in range(5)], 1.
    poses = [np.array([x, 0., 0.]) for x in np.arange(-.6, .001, .05)] + [np.zeros(3)]*4
    for pose in poses:
        t += .1
        out = [f.command(pose.copy(), np.zeros(2), target, np.zeros(2), hist, m, [], 'visible', policy_time=t)
               for f, m in ((py, maps[0]), (cpp, maps[1]))]
        np.testing.assert_array_equal(np.asarray(out[0], dtype=float), out[1])
        assert_same_controller(py, cpp)
    assert py.last_candidates.get('backing') and out[0][0] < 0.
