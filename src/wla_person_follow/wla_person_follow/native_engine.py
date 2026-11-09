"""The C++ follow core as process_step's single numerical owner (phase 4).

NativeFollowEngine keeps the whole numerical state of a numeric_backend:=cpp
worker in _wla_follow_core.FollowCore: rolling map, people tracks, sampled MPC
with its detour planner, approach goal, target trail, point route, last issued
command and the startup prior. One step is one C++ call that runs without the
GIL; this module only converts inputs (policy predicates are evaluated here,
detection keys become int64 ids) and builds the result dict of
SimulationEngine.step / process_step, which stays the reference
(test/test_follow_core.py compares the two).

A local planner outside the core (NeuPAN, in its own neupan_cpp process) takes
a second call: the core stops before the policy limit, the bridge is asked
here, and FollowCore.commit applies the limit and records the command.
"""
from dataclasses import replace
import numpy as np
from .sim_core.approach_goal import DESIRED_GAP, SWEEP_RADIUS
from .sim_core.common import Tuning
from .native_map import NativeRollingDepthMap
from .neupan_follower import TRACKED_STATES as NEUPAN_STATES
try:
    import _wla_follow_core as _core
except ImportError:  # Reported when the cpp backend is requested, never silently replaced.
    _core = None

REFERENCE_METHODS = {'0': 'windowed_cubic_bezier', '2': 'connected_object_route', '3': 'clearance_grid_route',
                     '4': 'in_place_turn'}


def check_engine_backend():
    if not hasattr(_core, 'FollowCore'):
        raise RuntimeError('cpp_numeric_backend_unavailable: _wla_follow_core has no FollowCore')


def policy_view(policy, now):
    """What the core reads from follow_session.TrackingPolicy at the step time."""
    if policy is None:
        return None
    return (float(getattr(policy, 'search_arrival_m', .35)), getattr(policy, 'search_heading', None),
            getattr(policy, 'search_started', None), float(getattr(policy, 'search_side', 1.)),
            bool(policy.translation_allowed(now)), bool(policy.active_search(now)))


class NativeFollowEngine:
    backend = 'cpp'

    def __init__(self, pose, now, *, resolution, size, startup_used=False, startup_prior=None):
        check_engine_backend()
        base = Tuning()
        self._core = _core.FollowCore(base, float(resolution), int(size), float(base.radius), float(SWEEP_RADIUS),
                                      np.asarray(pose, dtype=float), float(now))
        self._generations, self._tokens = [], {}
        self._forget_people()
        self._map = None
        self.trail_clear_radius = 0.
        self.prior_base = None  # _PROCESS_STARTUP_PRIOR form of the last step's prior
        self.set_startup(startup_used, startup_prior)

    # State shared with the reference engine's attributes.
    @property
    def mapping(self):
        core = self._core.map
        if self._map is None or self._map._core is not core:
            self._map = NativeRollingDepthMap._wrap(core)
        return self._map

    @property
    def tuning(self):
        return replace(Tuning(), grid_route_planner=self._core.grid_route_planner)

    def _forget_people(self):
        self._ids, self._keys, self._next = {}, {}, 0

    def _id(self, key):
        if key not in self._ids:
            self._ids[key] = self._next
            self._keys[self._next] = key
            self._next += 1
        return self._ids[key]

    def _token(self, generation):
        """key[-1] of the job as an int64 for the core (compared by ==)."""
        for token, known in enumerate(self._generations):
            if known == generation:
                return token
        self._generations.append(generation)
        return len(self._generations)-1

    def set_startup(self, used, prior):
        if prior is not None:
            prior = dict(prior, reference_generation=self._token(prior['reference_generation']))
        self._core.set_startup(bool(used), prior)

    def reset_task(self):
        self._core.reset_task()
        self._forget_people()

    def reset_reference(self, pose, now):
        self._core.reset_reference(np.asarray(pose, dtype=float), float(now))
        self._forget_people()

    def step(self, key, pose, previous, packets, observation, now, startup, *, local_radius, radius,
             motion_threshold, grid_route, neupan=None):
        """process_step after its input checks; returns its result dict."""
        active = bool(startup['active'])
        task_mode = startup.get('task_mode', '0')
        policy = startup.get('policy_snapshot')
        snapshot = startup.get('tracker_snapshot')
        recovery = bool(startup.get('recovery', False))
        goal = startup.get('goal')
        self.trail_clear_radius = float(startup.get('trail_clear_radius_m', 0.))
        settings = dict(radius=radius, local_radius=local_radius, motion_threshold=motion_threshold,
                        grid_route_astar=grid_route == 'astar',
                        target_clear_ttl_s=float(startup.get('target_clear_ttl_s', .5)),
                        trail_clear_radius_m=self.trail_clear_radius, active=active, task_mode=str(task_mode),
                        recovery=recovery, policy=None, tracker=None, object_surface=None, goal_point=None,
                        goal_remaining=0., external_command=False)
        seen = None
        if active:
            settings['policy'] = policy_view(policy, now)
            if snapshot is not None:
                binding = snapshot['state']
                searching = binding.startswith('search_')
                settings['tracker'] = (
                    binding, snapshot['position'] if searching else None,
                    None if searching else snapshot['prediction'], np.asarray(snapshot['velocity_mps'], dtype=float),
                    np.asarray([np.asarray(p, dtype=float) for p in snapshot['history']], dtype=float).reshape(-1, 2),
                    float(snapshot.get('surface_offset_m', .28)))
            if task_mode == '2' and startup.get('object_surface') is not None:
                settings['object_surface'] = np.asarray(startup['object_surface'], dtype=float).reshape(-1, 2)
            elif task_mode == '3':
                settings['goal_point'] = np.asarray(goal['point'], dtype=float)
            elif task_mode == '4':
                settings['goal_remaining'] = float(goal['remaining'])
            settings['external_command'] = bool(
                neupan is not None and task_mode == '0' and not recovery and snapshot is not None
                and snapshot['state'] in NEUPAN_STATES and snapshot.get('prediction') is not None)
            if observation is not None:
                stamp, target, people, resolved = observation
                ids = np.fromiter((self._id(k) for k in people), dtype=np.int64, count=len(people))
                points = np.array([np.asarray(z, dtype=float)[:2] for z in people.values()],
                                  dtype=float).reshape(-1, 2)
                seen = (float(stamp), None if target is None else np.asarray(target, dtype=float)[:2], ids, points,
                        self._ids.get(resolved))
        pose = np.asarray(pose, dtype=float)
        r = self._core.step(pose, np.asarray(previous, dtype=float), list(packets), seen, float(now),
                            self._token(key[-1]), settings)
        if len(self._ids) > 16:
            live = self._core.people_ids()
            if len(self._ids) > 2*len(live)+16:
                live = set(live)
                self._keys = {i: k for i, k in self._keys.items() if i in live}
                self._ids = {k: i for i, k in self._keys.items()}
        result = self._result(r, pose, now, task_mode, policy, snapshot, neupan) if active else self._idle(r, now, task_mode)
        result.update(map_lower=r['map_lower'], resolution=r['resolution'],
                      startup_prior=self._prior_report(r['startup_prior'], key, now),
                      startup_prior_consumed=r['startup_consumed'],
                      person_trail_clear=dict(radius_m=self.trail_clear_radius, trail_points=r['trail_points'],
                                              retired_cells=r['trail_retired']),
                      local_free_prior=dict(source='operator_assumed_robot_free', radius_m=local_radius,
                                            center_odom=pose[:2].tolist(),
                                            state='continuous' if local_radius else 'disabled',
                                            assumed_cells=r['assumed_cells']),
                      grid=r['grid'])
        return result

    def _prior_report(self, prior, key, now):
        if prior is None:
            self.prior_base = None
            return None
        base = dict(source='operator_assumed_startup_free', radius_m=prior['radius_m'],
                    center_odom=list(prior['center_odom']), stamp_s=prior['stamp_s'], expires_s=prior['expires_s'],
                    motion_started_s=prior['motion_started_s'], motion_threshold_m=prior['motion_threshold_m'],
                    reference_generation=self._generations[prior['reference_generation']],
                    seeded_cells=prior['seeded_cells'])
        self.prior_base = base
        report = dict(base)
        if report['reference_generation'] != key[-1]:
            report.update(remaining_s=0., state='reference_changed')
        elif report['motion_started_s'] is None:
            report.update(remaining_s=None, state='waiting_for_translation')
        else:
            report['remaining_s'] = max(0., report['expires_s']-now)
            report['state'] = 'counting_down' if report['remaining_s'] > 0 else 'expired'
        return report

    @staticmethod
    def _idle(r, now, task_mode):
        return dict(policy_snapshot=None, command=np.zeros(2), target_state='idle',
                    recovery_state='idle', reason='task_inactive', tuning=r['tuning'],
                    path=np.empty((0, 2)), candidates={}, approach_goal=None, planner_feasible=False,
                    planner_activity='idle_map_only',
                    front_blockage=dict(evaluated=False, reason='task_inactive', control_stamp_s=now),
                    planner_target=dict(position_reference_m=None, velocity_reference_mps=[0., 0.],
                                        state='idle', task_mode=task_mode, reference_method='none',
                                        history_point_count=0, reference_point_count=0, history_sampling=None,
                                        last_measurement_stamp_s=None), obstacle_track_ids=[])

    def _result(self, r, pose, now, task_mode, policy, snapshot, neupan):
        candidates = r['candidates']
        if 'backed_m' in candidates:
            candidates['backed_m'] = round(candidates['backed_m'], 2)
        approach, diagnostic = r['approach'], None
        if approach is not None:
            approach.update(reason='nearest_reachable_local_position' if approach['fallback'] else 'object_clearance_goal',
                            scope='current_rolling_map_connected_known_free',
                            footprint='0.48m square; conservative circular sweeps',
                            requested_clearance_m=DESIRED_GAP)
            diagnostic = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in approach.items() if k != 'path'}
        command, path, neupan_status = r['command'], r['path'], None
        if r['command_pending']:
            cells = np.argwhere(r['grid'] == 100)
            points = (cells+r['map_lower']+.5)*r['resolution'] if len(cells) else np.zeros((0, 2))
            command = neupan.plan(pose, snapshot['prediction'], points, r['tuning']['desired_gap'])
            neupan_status = neupan.diagnostic
            path = np.vstack((pose[:2], snapshot['prediction']))
            command = self._core.commit(command, float(now), policy_view(policy, now))
        state, estimate = r['state'], r['estimate']
        return dict(policy_snapshot=policy, command=command, neupan=neupan_status, target_state=state,
                    recovery_state=state, reason=r['reason'], tuning=r['tuning'],
                    planner_activity=r['planner_activity'], path=path, candidates=candidates,
                    approach_goal=diagnostic, planner_feasible=bool(candidates.get('feasible', 0) > 0),
                    front_blockage=dict(evaluated=False, reason='mpc_owns_collision_decision', control_stamp_s=now),
                    planner_target=dict(position_reference_m=None if estimate is None else estimate.tolist(),
                                        velocity_reference_mps=r['velocity_reference'].tolist(), state=state,
                                        task_mode=task_mode,
                                        reference_method=REFERENCE_METHODS.get(task_mode, 'current_person'),
                                        history_point_count=r['history_count'], reference_point_count=len(path),
                                        history_sampling=snapshot.get('history_sampling') if snapshot else None,
                                        last_measurement_stamp_s=snapshot['last_seen'] if snapshot else None),
                    obstacle_track_ids=[self._keys[int(i)] for i in r['obstacle_ids']])
