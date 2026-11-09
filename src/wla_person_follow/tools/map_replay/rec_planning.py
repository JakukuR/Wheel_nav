"""Phase 3 recorder: log the planner calls of the worker into the map record.

Imported by rec_main.py when $WLA_PLAN_RECORD is set (domain-199 replay only).
Planner calls go into the $WLA_MAP_RECORD stream, interleaved with the map
calls, so compare_planning.py can rebuild the exact map each call read:
['plan', op, owner_id, args, kwargs, result, diagnostics]; maps are replaced
by ('map', id) and a TrackingPolicy by a RecordedPolicy.
"""
import itertools, os, pickle
import numpy as np
from wla_person_follow.sim_core import approach_goal as A
from wla_person_follow.sim_core import detour as D
from wla_person_follow.sim_core import hybrid_controller as H
from recorded_policy import RecordedPolicy

OUT = open(os.environ['WLA_MAP_RECORD'], 'ab')
IDS = itertools.count()
INSIDE = set()  # followers inside command(): their detour calls belong to it


def snap(v):
    if isinstance(v, np.ndarray): return v.copy()
    if isinstance(v, dict): return {k: snap(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)): return type(v)(snap(x) for x in v)
    if hasattr(v, '_rec_id') and hasattr(v, 'rebuild'): return ('map', v._rec_id)
    return v


def dump(rec):
    pickle.dump(rec, OUT, protocol=pickle.HIGHEST_PROTOCOL); OUT.flush()


def diagnostics(follower):
    return dict(last_infeasible=follower.last_infeasible, last_candidates=snap(follower.last_candidates),
                last_reference_path=np.array(follower.last_reference_path, dtype=float).reshape(-1, 2),
                stuck_cycles=follower.stuck_cycles, backed=follower.backed, detour=snap(follower.detour.last))


_init = H.HybridFollower.__init__
def init(self, *args, **kwargs):
    _init(self, *args, **kwargs)
    self._rec_id = next(IDS); self.detour._rec_owner = self._rec_id
    dump(['plan', 'follower', self._rec_id, (), {}, None, None])
H.HybridFollower.__init__ = init

_reset = H.HybridFollower.reset
def reset(self):
    _reset(self); self.detour._rec_owner = self._rec_id
    dump(['plan', 'reset', self._rec_id, (), {}, None, None])
H.HybridFollower.reset = reset

_command = H.HybridFollower.command
def command(self, pose, previous, target, velocity, history, local_map, obstacles, status, policy=None,
            policy_time=None, **kwargs):
    recorded = None if policy is None else RecordedPolicy(policy, policy_time)
    args = snap((pose, previous, target, velocity, history, local_map, obstacles, status, recorded, policy_time))
    kw = dict(snap(kwargs), tuning=self.c)
    INSIDE.add(self._rec_id)
    try:
        result = _command(self, pose, previous, target, velocity, history, local_map, obstacles, status,
                          policy=policy, policy_time=policy_time, **kwargs)
    finally:
        INSIDE.discard(self._rec_id)
    dump(['plan', 'command', self._rec_id, args, kw, snap(result), diagnostics(self)])
    return result
H.HybridFollower.command = command

_grid_route = D.DetourPlanner._grid_route
def grid_route(self, p, target, local_map, c, *args, **kwargs):
    owner = getattr(self, '_rec_owner', None)
    record = owner is not None and owner not in INSIDE
    snapped = snap((p, target, local_map, c) + args) if record else None
    result = _grid_route(self, p, target, local_map, c, *args, **kwargs)
    if record:
        dump(['plan', 'grid_route', owner, snapped, snap(kwargs), snap(result), None])
    return result
D.DetourPlanner._grid_route = grid_route

_select = A.ApproachGoal.select
def select(self, pose, surface, mapping, now):
    rid = self.__dict__.setdefault('_rec_id', next(IDS))
    args = snap((pose, surface, mapping, now))
    result = _select(self, pose, surface, mapping, now)
    dump(['plan', 'approach', rid, args, {}, snap(result),
          dict(generation=self.generation, stable_since=self.stable_since)])
    return result
A.ApproachGoal.select = select
