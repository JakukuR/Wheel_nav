"""Phase 3 comparison: replay a map record with planner calls (rec_planning.py)
into the Python reference and the C++ core. Maps are rebuilt call by call (both
backends); each planner call then runs on the Python follower with the Python
map and on the C++ follower with the C++ map. Commands, feasibility counts,
detour reports, reference paths and approach goals must be identical.
usage: compare_planning.py <record.pkl> <test-dir>
"""
import os, pickle, sys, time
import numpy as np
sys.path.insert(0, sys.argv[2])
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from recorded_policy import RecordedPolicy
from test_native_planning import _same_approach, assert_same_controller
from wla_person_follow.native_map import NativeRollingDepthMap
from wla_person_follow.native_planning import NativeApproachGoal, NativeHybridFollower
from wla_person_follow.sim_core.approach_goal import ApproachGoal
from wla_person_follow.sim_core.common import Tuning
from wla_person_follow.sim_core.hybrid_controller import HybridFollower
from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap


class Unpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if name == 'RecordedPolicy' and module in ('rec_planning', 'recorded_policy'):
            return RecordedPolicy  # without importing the recorder (it patches the planner)
        return super().find_class(module, name)


def records(path):
    with open(path, 'rb') as f:
        while True:
            try: yield Unpickler(f).load()
            except EOFError: return


maps, followers, approaches, counts = {}, {}, {}, {}
timing = {'py': {}, 'cpp': {}}
kinds = {}


def timed(tag, op, call):
    t0 = time.perf_counter(); result = call()
    timing[tag][op] = timing[tag].get(op, 0.) + time.perf_counter() - t0
    return result


def resolve(value, side):
    if isinstance(value, tuple) and len(value) == 2 and value[0] == 'map':
        return maps[value[1]][side]
    if isinstance(value, (list, tuple)):
        return type(value)(resolve(v, side) for v in value)
    return value


def fail(message):
    sys.exit(message)


for n, rec in enumerate(records(sys.argv[1])):
    if rec[0] != 'plan':
        rid, op, args, kwargs, _ = rec
        if op == '__init__':
            maps[rid] = (RollingDepthMap(*args, **kwargs), NativeRollingDepthMap(*args, **kwargs))
        elif op == 'set':
            for m in maps[rid]: setattr(m, args[0], args[1])
        else:
            for m in maps[rid]: getattr(m, op)(*args, **kwargs)
        counts['map.' + op] = counts.get('map.' + op, 0) + 1
        continue
    _, op, oid, args, kwargs, recorded, extra = rec
    counts[op] = counts.get(op, 0) + 1
    if op == 'follower':
        followers[oid] = (HybridFollower(Tuning()), NativeHybridFollower(Tuning()))
    elif op == 'reset':
        for f in followers[oid]: f.reset()
    elif op == 'grid_route':
        py, cpp = followers[oid]
        a = timed('py', op, lambda: py.detour._grid_route(*resolve(args, 0), **kwargs))
        b = timed('cpp', op, lambda: cpp.detour._grid_route(*resolve(args, 1), **kwargs))
        for x in (recorded, b):
            if (a is None) != (x is None) or (a is not None and not np.array_equal(a, x)):
                fail(f'{"reference is not deterministic" if x is recorded else "RESULT MISMATCH"} at record {n} ({op})')
    elif op == 'command':
        py, cpp = followers[oid]
        kw = dict(kwargs); tuning = kw.pop('tuning'); py.c = cpp.c = tuning
        pose, previous, target, velocity, history, _, obstacles, status, policy, policy_time = args
        outs = [timed(tag, op, lambda f=f, m=maps[args[5][1]][side]: f.command(
                    pose, previous, target, velocity, history, m, obstacles, status, policy=policy,
                    policy_time=policy_time, **kw))
                for tag, side, f in (('py', 0, py), ('cpp', 1, cpp))]
        if not (np.array_equal(np.asarray(outs[0], dtype=float), np.asarray(recorded, dtype=float))
                and py.last_candidates == extra['last_candidates']):
            fail(f'reference is not deterministic at record {n} ({op})')
        if not np.array_equal(np.asarray(outs[0], dtype=float), outs[1]):
            fail(f'COMMAND MISMATCH at record {n}: {outs[0]} vs {outs[1]}')
        try:
            assert_same_controller(py, cpp)
        except AssertionError as error:
            fail(f'DIAGNOSTIC MISMATCH at record {n}: {str(error)[:600]}')
        lc = py.last_candidates; d = py.detour.last or {}
        kind = ('backing' if lc.get('backing') else 'recovery' if lc.get('recovery') else 'looking' if lc.get('looking')
                else 'infeasible' if py.last_infeasible else 'sampled' if lc else 'idle')
        kinds['command:' + kind] = kinds.get('command:' + kind, 0) + 1
        if d.get('active'):
            dk = d.get('search', 'grid') if d.get('side') == 'grid' else 'holding' if d.get('holding') else \
                'look' if d.get('looking') else 'nothing' if d.get('reason') else 'edge'
            kinds['detour:' + dk] = kinds.get('detour:' + dk, 0) + 1
    elif op == 'approach':
        py, cpp = approaches.setdefault(oid, (ApproachGoal(), NativeApproachGoal()))
        a = timed('py', op, lambda: py.select(*resolve(args, 0)))
        b = timed('cpp', op, lambda: cpp.select(*resolve(args, 1)))
        try:
            _same_approach(a, recorded)
        except AssertionError:
            fail(f'reference is not deterministic at record {n} ({op})')
        try:
            _same_approach(a, b)
            assert py.generation == cpp.generation and py.stable_since == cpp.stable_since
        except AssertionError as error:
            fail(f'APPROACH MISMATCH at record {n}: {error}')

print('identical:', {k: v for k, v in counts.items() if not k.startswith('map.')})
print('branches:', kinds)
for op in sorted(timing['py']):
    k = counts[op]
    print(f'{op:12s} n={k:6d}  py {timing["py"][op]/k*1e3:8.3f} ms  cpp {timing["cpp"][op]/k*1e3:8.3f} ms')
