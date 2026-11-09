"""Phase 4 comparison: replay recorded process_step jobs through each backend.

python3 compare_step.py record.pkl [phase3_sim_runtime.py]

Runs every job of a rec_step.py record, in order, through process_step with
numeric_backend python (the Python reference engine) and cpp (FollowCore), and
optionally through a phase-3 sim_runtime.py (Python engine over the C++ parts),
then compares each result with the reference exactly (test_follow_core._same)
and reports per-call times.
"""
import collections, importlib.util, os, pickle, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'test'))
from test_follow_core import _same  # noqa: E402
from wla_person_follow import sim_runtime  # noqa: E402


def records(path):
    with open(path, 'rb') as f:
        while True:
            try:
                yield pickle.load(f)
            except EOFError:
                return


def run(module, jobs, backend):
    for name, value in [('_PROCESS_ENGINE', None), ('_PROCESS_KEY', None), ('_PROCESS_STARTUP_USED', False),
                        ('_PROCESS_STARTUP_PRIOR', None)]:
        setattr(module, name, value)
    out, times = [], []
    for job in jobs:
        key, pose, previous, packets, observation, now, startup = job
        job = (key, pose.copy(), previous.copy(), packets, observation, now, dict(startup, numeric_backend=backend))
        started = time.perf_counter()
        try:
            result = module.process_step(job)
        except Exception as error:
            result = ('error', type(error).__name__, str(error))
        times.append(time.perf_counter()-started)
        out.append(result)
    return out, np.array(times)*1e3


def main():
    data = list(records(sys.argv[1]))
    jobs = [r[0] for r in data]
    print('jobs', len(jobs), 'active', sum(bool(j[6]['active']) for j in jobs),
          'modes', dict(collections.Counter(j[6].get('task_mode', '0') for j in jobs if j[6]['active'])))
    reference, t_py = run(sim_runtime, jobs, 'python')
    native, t_cpp = run(sim_runtime, jobs, 'cpp')
    runs = [('cpp', native, t_cpp)]
    if len(sys.argv) > 2:
        spec = importlib.util.spec_from_file_location('wla_person_follow.sim_runtime_phase3', sys.argv[2])
        phase3 = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(phase3)
        old, t_old = run(phase3, jobs, 'cpp')
        runs.append(('phase3', old, t_old))
    reasons = collections.Counter(r['reason'] if isinstance(r, dict) else r[0] for r in reference)
    print('reasons', dict(reasons))
    for name, results, _ in runs:
        bad = 0
        for k, (a, b) in enumerate(zip(reference, results)):
            try:
                _same(a, b, f'{name} job {k}')
            except AssertionError as error:
                bad += 1
                if bad <= 3:
                    print('DIFF', str(error)[:400])
        print(f'{name}: {len(results)-bad}/{len(results)} identical')
    recorded = sum(1 for (_, r, _), b in zip(data, native) if isinstance(r, dict) and _equal(r, b))
    print(f'recorded live results reproduced by cpp: {recorded}/{len(data)}')
    for name, t in [('python', t_py)] + [(n, t) for n, _, t in runs]:
        print(f'{name:7s} p50 {np.percentile(t, 50):7.2f} ms  p95 {np.percentile(t, 95):7.2f} ms  mean {t.mean():7.2f} ms')


def _equal(a, b):
    try:
        _same(a, b)
        return True
    except AssertionError:
        return False


if __name__ == '__main__':
    main()
