"""Phase 0 comparison: replay recorded map calls into the Python reference and
the C++ map; every result and all state must be identical after every call."""
import pickle, sys, time
import numpy as np
from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
from wla_person_follow.native_map import NativeRollingDepthMap
sys.path.insert(0, sys.argv[2])
from test_native_rolling_map import assert_same

def records(path):
    with open(path, 'rb') as f:
        while True:
            try: yield pickle.load(f)
            except EOFError: return

def same_result(a, b):
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        return np.array_equal(np.asarray(a).reshape(-1, 3) if np.size(a) else np.zeros((0, 3)),
                              np.asarray(b).reshape(-1, 3) if np.size(b) else np.zeros((0, 3)))
    return a == b

maps = {}; counts = {}; timing = {'py': {}, 'cpp': {}}
for rid, op, args, kwargs, recorded in records(sys.argv[1]):
    counts[op] = counts.get(op, 0) + 1
    if op == '__init__':
        maps[rid] = (RollingDepthMap(*args, **kwargs), NativeRollingDepthMap(*args, **kwargs)); continue
    py, cpp = maps[rid]
    if op == 'set':
        for m in (py, cpp): setattr(m, args[0], args[1])
        continue
    results = []
    for tag, m in (('py', py), ('cpp', cpp)):
        t0 = time.perf_counter(); results.append(getattr(m, op)(*args, **kwargs))
        timing[tag][op] = timing[tag].get(op, 0.) + time.perf_counter() - t0
    if not same_result(results[0], recorded):
        sys.exit(f'reference is not deterministic at call {sum(counts.values())} ({op})')
    if not same_result(results[0], results[1]):
        sys.exit(f'RESULT MISMATCH at call {sum(counts.values())} ({op})')
    try:
        assert_same(py, cpp)
    except AssertionError as error:
        sys.exit(f'STATE MISMATCH after call {sum(counts.values())} ({op}): {str(error)[:400]}')
print('identical after every call; calls:', counts)
for op in sorted(timing['py']):
    n = counts[op]
    print('  %-22s py %7.3f ms/call   cpp %7.3f ms/call   (%d calls)' % (op, timing['py'][op]/n*1e3, timing['cpp'][op]/n*1e3, n))
