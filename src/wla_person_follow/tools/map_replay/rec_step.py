"""Phase 4 recorder: run the follow worker and log every process_step job.

Used only for a domain-199 replay: `python3 rec_step.py -m module args...`.
Appends [job, result, seconds] records to $WLA_STEP_RECORD; compare_step.py
replays the jobs through both numeric backends. The TrackingPolicy in the job
is pickled as it is (a frozen dataclass of plain values).
"""
import os, pickle, runpy, sys, time
import numpy as np
from wla_person_follow import sim_runtime

OUT = open(os.environ['WLA_STEP_RECORD'], 'ab')


def snap(v):
    if isinstance(v, np.ndarray): return v.copy()
    if isinstance(v, dict): return {k: snap(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)): return type(v)(snap(x) for x in v)
    return v


_process_step = sim_runtime.process_step


def process_step(job):
    record = snap(job)
    started = time.perf_counter()
    try:
        result = _process_step(job)
    except Exception as error:  # recorded, then raised as before
        pickle.dump([record, ('error', type(error).__name__, str(error)), time.perf_counter()-started], OUT,
                    protocol=pickle.HIGHEST_PROTOCOL)
        OUT.flush()
        raise
    pickle.dump([record, snap(result), time.perf_counter()-started], OUT, protocol=pickle.HIGHEST_PROTOCOL)
    OUT.flush()
    return result


sim_runtime.process_step = process_step  # before the worker imports it by name

if __name__ == '__main__':
    assert sys.argv[1] == '-m'
    module = sys.argv[2]
    sys.argv = [module] + sys.argv[3:]
    runpy.run_module(module, run_name='__main__', alter_sys=True)
