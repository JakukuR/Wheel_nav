"""process_step with the C++ FollowCore equals the Python engine exactly (phase 4, 2026-11-09).

Closed-loop random runs (walls, boxes, people, a walking target, depth packets
with and without the target's clearing points, control-rate steps without
packets) go through sim_runtime.process_step twice: numeric_backend python (the
Python SimulationEngine with the Python map/tracking/MPC) and cpp (one
FollowCore call per step). Every step's result dict must be identical: keys and
their order, commands, reasons, candidate counts, paths, tuning, prior reports
and the occupancy grid. Runs cover task changes (map kept), wheel reference
changes (map dropped), idle steps, all five task modes, recovery, A* grid
routes, the startup and local free priors, trail clearing and a NeuPAN-style
external planner.
"""
import math
import numpy as np
import pytest
from wla_person_follow import sim_runtime as rt
from wla_person_follow.follow_session import TrackingPolicy
from wla_person_follow.sim_runtime import packet_from_filtered_points

ORIGIN = np.array([.18, 0., .5])
STATES = ('tracking', 'tracking', 'coasting', 'holding', 'search_last_seen', 'search_heading', 'lost')


def _same(a, b, where='result'):
    assert type(a) is type(b) or (isinstance(a, float) and isinstance(b, float)), (where, type(a), type(b), a, b)
    if isinstance(a, dict):
        assert list(a) == list(b), (where, list(a), list(b))
        for k in a:
            _same(a[k], b[k], f'{where}.{k}')
    elif isinstance(a, (list, tuple)):
        assert len(a) == len(b), (where, a, b)
        for k, (x, y) in enumerate(zip(a, b)):
            _same(x, y, f'{where}[{k}]')
    elif isinstance(a, np.ndarray):
        assert a.dtype == b.dtype and a.shape == b.shape, (where, a.dtype, b.dtype, a.shape, b.shape)
        assert np.array_equal(a, b, equal_nan=a.dtype.kind == 'f'), (where, a, b)
    elif isinstance(a, float):
        assert a == b or (math.isnan(a) and math.isnan(b)), (where, a, b)
    else:
        assert a is b or a == b, (where, a, b)


class Stub:
    """NeuPAN stand-in: deterministic in its inputs, records every call."""

    def __init__(self):
        self.calls, self.diagnostic, self.resets = [], None, 0

    def reset(self):
        self.resets += 1

    def plan(self, pose, target, obstacles, gap):
        obstacles = np.asarray(obstacles, dtype=float)
        self.calls.append((np.asarray(pose, dtype=float).copy(), np.asarray(target, dtype=float).copy(),
                           obstacles.copy(), float(gap)))
        delta = np.asarray(target, dtype=float) - pose[:2]
        self.diagnostic = dict(active='plan', points=len(obstacles))
        return np.array([min(.6, .3*np.linalg.norm(delta)), float(np.clip(np.arctan2(delta[1], delta[0]) - pose[2], -.8, .8))])


def _local(world, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    d = np.asarray(world, dtype=float)[:, :2] - pose[:2]
    return np.c_[c*d[:, 0] + s*d[:, 1], -s*d[:, 0] + c*d[:, 1], np.asarray(world)[:, 2]]


def _cloud(rng, pose, world_obstacles):
    """Obstacles inside a 0.75 rad half-angle cone; clearing rays over the floor."""
    rel = np.arctan2(world_obstacles[:, 1]-pose[1], world_obstacles[:, 0]-pose[0]) - pose[2]
    rel = (rel + np.pi) % (2*np.pi) - np.pi
    distance = np.linalg.norm(world_obstacles[:, :2]-pose[:2], axis=1)
    seen = world_obstacles[(np.abs(rel) < .75) & (distance < 4.5)]
    r = rng.uniform(.3, 4.)
    a = pose[2] + rng.uniform(-.75, .75, 800)
    rr = rng.uniform(.3, r, 800)
    floor = np.c_[pose[0] + rr*np.cos(a), pose[1] + rr*np.sin(a), rng.uniform(-.02, .02, 800)]
    return packet_from_filtered_points(_local(seen, pose), _local(floor, pose), ORIGIN, pose)


def _world(rng):
    walls = []
    for _ in range(int(rng.integers(1, 4))):
        centre = rng.uniform([.8, -2.], [4., 2.])
        tilt = rng.uniform(0, np.pi)
        s = rng.uniform(-rng.uniform(.3, 1.5), rng.uniform(.3, 1.5), 300)
        walls.append(np.c_[centre[0] + s*np.cos(tilt), centre[1] + s*np.sin(tilt), rng.uniform(.1, 1., 300)])
    return np.vstack(walls)


def _drive(seed, backend, steps=48, neupan=None, modes=None):
    rng = np.random.default_rng(seed)
    world = _world(rng)
    # The object of mode 2, also seen by the depth camera.
    box = np.array([float(rng.uniform(1.,1.8)), float(rng.uniform(-.5, .5))])
    side = np.linspace(-.2, .2, 9)
    world = np.vstack((world, np.c_[np.r_[box[0]-.2+0*side, box[0]+side], np.r_[box[1]+side, box[1]-.2+0*side],
                                    np.full(18, .3)]))
    pose = np.array([0., 0., float(rng.uniform(-.3, .3))])
    command = np.zeros(2)
    person = np.array([1.6, float(rng.uniform(-.4, .4))])
    walk = np.array([float(rng.uniform(.1, .5)), float(rng.uniform(-.3, .3))])
    others = {f'p{k}': np.array([float(rng.uniform(1., 4.)), float(rng.uniform(-2., 2.))]) for k in range(int(rng.integers(0, 3)))}
    other_walk = {k: rng.uniform(-.4, .4, 2) for k in others}
    astar = 'astar' if rng.random() < .5 else 'dijkstra'
    settings = dict(radius=float(rng.choice([0., .5])), local_radius=float(rng.choice([0., 0., .3])),
                    trail=float(rng.choice([0., .3])), ttl=float(rng.choice([.5, .8])))
    drawn = list(rng.choice(['0', '0', '1', '2', '3', '4'], 4))
    modes = drawn if modes is None else modes
    generation, request, t, history, last_seen = 0, 0, 10., [], None
    results, phase = [], 0
    for k in range(steps):
        if k and k % 12 == 0:
            request += 1
            phase += 1
            if rng.random() < .25:
                generation += 1
        mode = modes[phase % len(modes)]
        active = not (k % 12 >= 10 and phase % 2 == 1)
        camera = k % 2 == 0 or rng.random() < .3
        t = round(t + float(rng.choice([.05, .1])), 6)
        person = person + walk*.1
        for key in others:
            others[key] = others[key] + other_walk[key]*.1
        state = STATES[int(rng.integers(len(STATES)))] if rng.random() < .35 else 'tracking'
        packets, observation = [], None
        if camera:
            stamp = round(t - float(rng.choice([0., .02, .04])), 6)
            people_world = np.vstack([world] + [np.c_[np.full(30, p[0]) + rng.normal(0, .08, 30),
                                                      np.full(30, p[1]) + rng.normal(0, .08, 30),
                                                      rng.uniform(.1, 1.6, 30)] for p in others.values()])
            packet = _cloud(rng, pose, people_world)
            if rng.random() < .5:
                packet['target_points_odom'] = np.c_[person + rng.normal(0, .1, (40, 2)), rng.uniform(.05, .6, 40)]
                packet['target_points_from_cluster'] = True
            packets.append((stamp, packet))
            if rng.random() < .2:
                packets.append((round(stamp - .05, 6), _cloud(rng, pose, world)))
            seen = person + rng.normal(0, .03, 2)
            target = None if state in ('lost', 'search_heading') else seen
            people = {key: p + rng.normal(0, .03, 2) for key, p in others.items()}
            if target is not None:
                people['target'] = seen
                last_seen = stamp
                history.append(seen.copy())
                history = history[-12:]
            observation = (stamp, target, people, 'target' if target is not None else None)
        snapshot = None
        if rng.random() < .93:
            snapshot = dict(state=state, position=(person + rng.normal(0, .05, 2)).tolist(),
                            prediction=None if rng.random() < .1 else (person + walk*.3).tolist(),
                            velocity_mps=walk.tolist(), history=[h.tolist() for h in history],
                            last_seen=last_seen, history_sampling=dict(period_s=.4),
                            surface_offset_m=float(rng.choice([.28, .35])))
        policy = TrackingPolicy(state=state, last_seen=last_seen, prediction_s=2., target_id='target', revision=1,
                                search_heading=None if rng.random() < .3 else float(rng.uniform(-3, 3)),
                                search_started=None if rng.random() < .5 else t - 1., mode=mode,
                                search_side=float(rng.choice([1., -1.])))
        surface = None
        if mode == '2':
            s = np.linspace(-.2, .2, 9)
            surface = np.r_[np.c_[box[0] - .2 + 0*s, box[1] + s], np.c_[box[0] + s, box[1] - .2 + 0*s]]
        goal = dict(point=[2.5, float(rng.choice([-.8, .8]))], remaining=float(rng.uniform(-1.5, 1.5)))
        startup = dict(active=active, radius=settings['radius'], local_radius=settings['local_radius'],
                       motion_threshold=.03, numeric_backend=backend, task_mode=mode,
                       policy_snapshot=policy if active else None, tracker_snapshot=snapshot if active else None,
                       object_surface=surface, goal=goal, recovery=bool(rng.random() < .05),
                       grid_route_planner=astar, target_clear_ttl_s=settings['ttl'],
                       trail_clear_radius_m=settings['trail'], local_planner='neupan' if neupan else 'mpc')
        key = (f'r{request}', mode if active else '', 'target', 1, 0, generation)
        previous = command + rng.normal(0, .02, 2) if rng.random() < .8 else rng.uniform(-.3, .3, 2)
        result = rt.process_step((key, pose.copy(), previous, packets, observation, t, startup))
        results.append(result)
        command = np.asarray(result['command'], dtype=float)
        # Unicycle integration of the issued command (the same in both runs when equal).
        pose = pose + np.array([command[0]*math.cos(pose[2])*.1, command[0]*math.sin(pose[2])*.1, command[1]*.1])
    return results


def _reset(monkeypatch, neupan=None):
    for name, value in [('_PROCESS_ENGINE', None), ('_PROCESS_KEY', None), ('_PROCESS_STARTUP_USED', False),
                        ('_PROCESS_STARTUP_PRIOR', None), ('_PROCESS_NEUPAN', neupan)]:
        monkeypatch.setattr(rt, name, value)
    monkeypatch.setattr(rt, 'neupan_for', lambda planner: neupan if planner == 'neupan' else None)


@pytest.mark.parametrize('seed,modes', [(seed, None) for seed in range(10)] +
                         [(seed, ['2', '3', '2', '4']) for seed in (202, 211, 214, 221, 226)])
def test_follow_core_equals_python_engine(monkeypatch, seed, modes):
    _reset(monkeypatch)
    reference = _drive(seed, 'python', modes=modes)
    _reset(monkeypatch)
    native = _drive(seed, 'cpp', modes=modes)
    assert isinstance(rt._PROCESS_ENGINE, rt.NativeFollowEngine)
    for k, (a, b) in enumerate(zip(reference, native)):
        _same(a, b, f'seed {seed} step {k}')
    if modes is not None:
        assert {'object_clearance_goal', 'nearest_reachable_local_position'} & {r['reason'] for r in native}


@pytest.mark.parametrize('seed', range(3))
def test_external_planner_split_equals_python_engine(monkeypatch, seed):
    stub = Stub()
    _reset(monkeypatch, stub)
    reference = _drive(100 + seed, 'python', neupan=stub, modes=['0', '1', '0', '3'])
    reference_calls = stub.calls
    stub = Stub()
    _reset(monkeypatch, stub)
    native = _drive(100 + seed, 'cpp', neupan=stub, modes=['0', '1', '0', '3'])
    for k, (a, b) in enumerate(zip(reference, native)):
        _same(a, b, f'seed {seed} step {k}')
    assert len(stub.calls) == len(reference_calls) > 0
    for a, b in zip(reference_calls, stub.calls):
        _same(list(a), list(b), 'neupan call')


def test_backend_change_keeps_the_startup_prior_used(monkeypatch):
    _reset(monkeypatch)
    job = lambda t, backend, generation=0: (('r', '0', 't', 1, 0, generation), np.zeros(3), np.zeros(2), [], None, t,
                                           dict(active=True, radius=.5, numeric_backend=backend))
    first = rt.process_step(job(1., 'python'))
    assert first['startup_prior_consumed'] and first['startup_prior']['seeded_cells'] > 0
    native = rt.process_step(job(2., 'cpp'))
    # The new backend starts from an empty map; the prior is not seeded again.
    assert native['startup_prior_consumed'] and np.all(native['grid'] == -1)
    _same(native['startup_prior'], dict(first['startup_prior']), 'prior')
    back = rt.process_step(job(3., 'python', 1))
    assert back['startup_prior']['state'] == 'reference_changed'


def test_cpp_step_rejects_without_falling_back(monkeypatch):
    _reset(monkeypatch)
    monkeypatch.setattr(rt.NativeFollowEngine, '__init__', lambda *a, **k: (_ for _ in ()).throw(
        RuntimeError('cpp_numeric_backend_unavailable: _wla_follow_core has no FollowCore')))
    with pytest.raises(RuntimeError, match='cpp_numeric_backend_unavailable'):
        rt.process_step((('r', '0', 't', 1, 0, 0), np.zeros(3), np.zeros(2), [], None, 1.,
                         dict(active=False, radius=0., numeric_backend='cpp')))
    assert rt._PROCESS_ENGINE is None
