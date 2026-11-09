"""Phase 2 recorder: log the people-track and reference-curve calls of the worker.

Imported by rec_main.py when $WLA_TRACK_RECORD is set (domain-199 replay only).
Appends ['people', id, op, args, result] and [function, args, kwargs, result].
"""
import itertools, os, pickle
import numpy as np
from wla_person_follow import native_tracking as N
from wla_person_follow.sim_core import hybrid_controller as H
from wla_person_follow.sim_core import tracking as T

OUT = open(os.environ['WLA_TRACK_RECORD'], 'ab')
IDS = itertools.count()


def snap(v):
    if isinstance(v, np.ndarray): return v.copy()
    if isinstance(v, dict): return {k: snap(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)): return type(v)(snap(x) for x in v)
    return v


def dump(rec):
    pickle.dump(rec, OUT, protocol=pickle.HIGHEST_PROTOCOL); OUT.flush()


def method(name):
    original = getattr(T.PeopleTracks, name)
    def call(self, *args):
        rid = self.__dict__.setdefault('_rec_id', next(IDS))
        result = original(self, *args)
        dump(['people', rid, name, snap(args), snap(result)])
        return result
    setattr(T.PeopleTracks, name, call)


def function(name):
    original = getattr(H, name)
    def call(*args, **kwargs):
        result = original(*args, **kwargs)
        dump([name, snap(args), snap(kwargs), snap(result)])
        return result
    setattr(H, name, call); setattr(N, name, call)


for name in ('update', 'predict', 'drop'):
    method(name)
for name in ('bezier_reference', 'bspline_corners'):
    function(name)
