"""Phase 0 recorder: run the follow worker and log every rolling map call.

Used only for a domain-199 replay: `python3 rec_main.py -m module args...`.
Appends [map_id, op, args, kwargs, result] records to $WLA_MAP_RECORD.
"""
import itertools, os, pickle, runpy, sys
import numpy as np
from wla_person_follow.sim_core import rolling_depth_map as R

OUT = open(os.environ['WLA_MAP_RECORD'], 'ab')
IDS = itertools.count()
SETTINGS = ('local_assumed_free_center', 'local_assumed_free_radius', 'target_clear_ttl')

def snap(v):
    if isinstance(v, np.ndarray): return v.copy()
    if isinstance(v, dict): return {k: snap(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)): return type(v)(snap(x) for x in v)
    return v

def dump(rec):
    pickle.dump(rec, OUT, protocol=pickle.HIGHEST_PROTOCOL); OUT.flush()

def wrap(name):
    original = getattr(R.RollingDepthMap, name)
    def call(self, *args, **kwargs):
        rec = [self._rec_id, name, snap(args), snap(kwargs)]
        live = self.__dict__.pop('_rec_live', False)  # internal calls are not recorded
        try:
            result = original(self, *args, **kwargs)
        finally:
            if live: object.__setattr__(self, '_rec_live', True)
        if live:
            rec.append(snap(result)); dump(rec)
        return result
    setattr(R.RollingDepthMap, name, call)

for name in ('slide', 'initial_clear_patch', 'advance_startup_prior', 'update', 'drop_established',
             'retire_person_trail', 'rebuild'):
    wrap(name)

_init = R.RollingDepthMap.__init__
def init(self, *args, **kwargs):
    object.__setattr__(self, '_rec_id', next(IDS))
    _init(self, *args, **kwargs)
    dump([self._rec_id, '__init__', snap(args), snap(kwargs), None])
    object.__setattr__(self, '_rec_live', True)
R.RollingDepthMap.__init__ = init

def setattr_(self, name, value):
    object.__setattr__(self, name, value)
    if name in SETTINGS and self.__dict__.get('_rec_live'):
        dump([self._rec_id, 'set', (name, snap(value)), {}, None])
R.RollingDepthMap.__setattr__ = setattr_

if __name__ == '__main__':
    assert sys.argv[1] == '-m'
    module = sys.argv[2]
    sys.argv = [module] + sys.argv[3:]
    runpy.run_module(module, run_name='__main__', alter_sys=True)
