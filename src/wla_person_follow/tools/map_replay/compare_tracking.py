"""Phase 2 comparison: replay recorded tracking calls into the Python reference
and the C++ core; every result and all track state must be identical.

Besides the recorded people tracks, every person's measurement stream is also
run through a pair of tracks with the followed-target settings (coast decay,
target noise), so those paths see real data too.
usage: compare_tracking.py <record.pkl> <test-dir>
"""
import pickle, sys, time
import numpy as np
sys.path.insert(0, sys.argv[2])
from test_native_tracking import assert_same_track, same
from wla_person_follow import follow_session, native_tracking
from wla_person_follow.native_tracking import NativePeopleTracks, NativeVisualTrack
from wla_person_follow.sim_core.target_reference import bezier_reference, bspline_corners
from wla_person_follow.sim_core.tracking import PeopleTracks, VisualTrack

TARGET = dict(meas_std=follow_session.TARGET_MEAS_STD_M, accel_std=follow_session.TARGET_ACCEL_STD_MPS2,
              gate=follow_session.TARGET_INNOVATION_GATE, coast_decay=follow_session.TARGET_COAST_DECAY,
              history_max_gap_s=.6)


def records(path):
    with open(path, 'rb') as f:
        while True:
            try: yield pickle.load(f)
            except EOFError: return


people, targets, counts = {}, {}, {}
timing = {'py': {}, 'cpp': {}}


def timed(tag, op, call):
    t0 = time.perf_counter(); result = call()
    timing[tag][op] = timing[tag].get(op, 0.) + time.perf_counter() - t0
    return result


def check(op, n, recorded, a, b):
    if not same(a, recorded):
        sys.exit(f'reference is not deterministic at record {n} ({op})')
    if not same(a, b):
        sys.exit(f'RESULT MISMATCH at record {n} ({op})')


for n, rec in enumerate(records(sys.argv[1])):
    if rec[0] == 'people':
        _, rid, op, args, recorded = rec
        py, cpp = people.setdefault(rid, (PeopleTracks(), NativePeopleTracks()))
        a = timed('py', 'people.' + op, lambda: getattr(py, op)(*args))
        b = timed('cpp', 'people.' + op, lambda: getattr(cpp, op)(*args))
        check(op, n, recorded, a, b)
        if py.keys() != cpp.keys():
            sys.exit(f'TRACK SET MISMATCH at record {n}')
        if op == 'update':
            detections, _, t = args
            streams = targets.setdefault(rid, {})
            for key in list(detections) + [k for k in streams if k not in detections]:
                pair = streams.setdefault(key, (VisualTrack(2., **TARGET), NativeVisualTrack(2., **TARGET)))
                z = detections[key][:2] if key in detections else None
                if not same(pair[0].update(z, t), pair[1].update(z, t)):
                    sys.exit(f'TARGET-SETTINGS MISMATCH at record {n}')
                assert_same_track(*pair)
                if not same(pair[0].predicted_position(t + .2), pair[1].predicted_position(t + .2)):
                    sys.exit(f'TARGET PREDICTION MISMATCH at record {n}')
                counts['target_track.update'] = counts.get('target_track.update', 0) + 1
        op = 'people.' + op
    else:
        op, args, kwargs, recorded = rec
        py_f, cpp_f = {'bezier_reference': (bezier_reference, native_tracking.native_bezier_reference),
                       'bspline_corners': (bspline_corners, native_tracking.native_bspline_corners)}[op]
        a = timed('py', op, lambda: py_f(*args, **kwargs))
        b = timed('cpp', op, lambda: cpp_f(*args, **kwargs))
        check(op, n, recorded, a, b)
    counts[op] = counts.get(op, 0) + 1

print('identical:', counts)
for op in sorted(timing['py']):
    k = counts[op]
    print(f'{op:24s} n={k:6d}  py {timing["py"][op]/k*1e6:8.1f} us  cpp {timing["cpp"][op]/k*1e6:8.1f} us')
