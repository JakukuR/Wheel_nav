"""Planning backends: the Python reference or the C++ follow core (phase 3).

NativeHybridFollower and NativeApproachGoal keep the planner state in
_wla_follow_core (HybridFollower with its DetourPlanner, ApproachGoal) and offer
the attributes and methods of the Python reference (sim_core/hybrid_controller.py,
sim_core/detour.py, sim_core/approach_goal.py), which stays the numerical
reference. Commands and every feasibility decision are bit-identical
(test/test_native_planning.py, the replay comparison). They read the C++
rolling map, so the cpp backend runs map, tracking and planning in C++.
Selected by numeric_backend like the map and tracking; never falls back.
"""
import numpy as np
from .sim_core.approach_goal import ApproachGoal, DESIRED_GAP
from .sim_core.common import Tuning
from .sim_core.hybrid_controller import HybridFollower
from .native_tracking import check_backend, reference_functions
try:
    import _wla_follow_core as _core
except ImportError:  # Reported when the cpp backend is requested, never silently replaced.
    _core = None


def check_planning_backend(backend):
    if check_backend(backend) == 'cpp' and not hasattr(_core, 'HybridFollower'):
        raise RuntimeError('cpp_numeric_backend_unavailable: _wla_follow_core has no planning')
    return backend


def make_controller(backend='python', config=Tuning()):
    if check_planning_backend(backend) == 'python':
        return HybridFollower(config, references=reference_functions(backend))
    return NativeHybridFollower(config)


def make_approach_goal(backend='python'):
    return ApproachGoal() if check_planning_backend(backend) == 'python' else NativeApproachGoal()


def _native_map(local_map):
    core = getattr(local_map, '_core', None)
    if core is None:
        raise TypeError('cpp_planner_needs_cpp_rolling_map')
    return core


def _points(values):
    return np.asarray(values, dtype=float).reshape(-1, 2)


def _optional(value):
    return None if value is None else np.asarray(value, dtype=float)


class NativeDetour:
    """DetourPlanner view of the C++ follower's detour planner."""

    def __init__(self, core):
        self._core = core
        self._trigger = 'line'

    @property
    def last(self):
        return self._core.detour_last

    @property
    def trigger(self):
        return self._trigger

    @trigger.setter
    def trigger(self, value):
        self._core.set_detour_trigger(value)
        self._trigger = value

    def _grid_route(self, p, target, local_map, c, window_m=3.5, clearance_weight=2.):
        return self._core.grid_route(_native_map(local_map), np.asarray(p, dtype=float),
                                     np.asarray(target, dtype=float), c, float(window_m), float(clearance_weight))


class NativeHybridFollower:
    backend = 'cpp'

    def __init__(self, config=Tuning()):
        self.c = config
        self._core = _core.HybridFollower()
        self.detour = NativeDetour(self._core)
        # Plain attributes like the reference: the engine overwrites them in mode 4.
        self.last_infeasible = False
        self.last_candidates = {}
        self.last_reference_path = np.empty((0, 2))

    stuck_cycles = property(lambda self: self._core.stuck_cycles)
    backed = property(lambda self: self._core.backed)
    trail = property(lambda self: list(self._core.trail))

    def reset(self):
        """Forget detour side/look state between tasks."""
        self._core.reset()
        self.detour._trigger = 'line'

    def command(self, pose, previous, target, velocity, history, local_map, obstacles,
                status, policy=None, policy_time=None, *, reference_path=None, face_point=None, measured=None,
                recovery=False):
        view = None
        if policy is not None:
            # TrackingPolicy predicates are pure; evaluate them once at policy_time.
            allowed = bool(policy.translation_allowed(policy_time))
            view = (float(getattr(policy, 'search_arrival_m', .35)), getattr(policy, 'search_heading', None),
                    getattr(policy, 'search_started', None), float(getattr(policy, 'search_side', 1.)),
                    allowed, bool(policy.active_search(policy_time)))
        out = self._core.command(
            _native_map(local_map), self.c, np.asarray(pose, dtype=float), np.asarray(previous, dtype=float),
            _optional(target), np.asarray(velocity, dtype=float), _points(history),
            _points([o['p'] for o in obstacles]), _points([o['v'] for o in obstacles]),
            np.asarray([o['r'] for o in obstacles], dtype=float), str(status), view,
            None if policy_time is None else float(policy_time),
            None if reference_path is None else _points(reference_path), _optional(face_point), _optional(measured),
            bool(recovery))
        self.last_infeasible = self._core.last_infeasible
        candidates = self._core.last_candidates
        if 'backed_m' in candidates:
            candidates['backed_m'] = round(candidates['backed_m'], 2)
        self.last_candidates = candidates
        self.last_reference_path = self._core.last_reference_path
        return out


class NativeApproachGoal:
    backend = 'cpp'
    _samples = staticmethod(ApproachGoal._samples)

    def __init__(self):
        self._core = _core.ApproachGoal()

    generation = property(lambda self: self._core.generation)
    stable_since = property(lambda self: self._core.stable_since)
    planned_at = property(lambda self: self._core.planned_at)
    center = property(lambda self: self._core.center)

    def select(self, pose, surface, mapping, now):
        report = self._core.select(np.asarray(pose, dtype=float), _points(surface), _native_map(mapping), float(now))
        if report is None:
            return None
        report.update(reason='nearest_reachable_local_position' if report['fallback'] else 'object_clearance_goal',
                      scope='current_rolling_map_connected_known_free',
                      footprint='0.48m square; conservative circular sweeps',
                      requested_clearance_m=DESIRED_GAP)
        return report
