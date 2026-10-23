"""Real observations -> frozen close-wide-flex core. No ROS or actuator I/O."""
from copy import deepcopy
from dataclasses import asdict, replace
import math
import numpy as np
from .sim_core.common import rotation
from .sim_core.wide_turn import WideTurnTuning
from .sim_core.hybrid_controller import HybridFollower
from .sim_core.rolling_depth_map import RollingDepthMap
from .sim_core.tracking import VisualTrack, PeopleTracks


def local_points(points, pose):
    result = np.asarray(points, dtype=float).copy()
    result[:, :2] = result[:, :2] @ rotation(pose[2]).T + pose[:2]
    return result


def packet_from_depth(depth, intrinsics, camera_rotation, translation, pose,
                      minimum=.15, maximum=5., stride=4):
    from _wla_sim_geometry import deproject
    points = deproject(depth, np.asarray(intrinsics, dtype=float),
                       np.asarray(camera_rotation, dtype=float),
                       np.asarray(translation, dtype=float), minimum, maximum, stride)
    from _wla_sim_geometry import filter_points
    obstacles, clearing = filter_points(points)
    return packet_from_filtered_points(obstacles, clearing, translation, pose)


def target_points_from_removed_depth(depth, intrinsics, camera_rotation, translation,
                                     pose, center_odom, *, radius=.35, stride=2,
                                     minimum=.15, maximum=5.):
    """Project only same-frame mask-removed target pixels into odom.

    The radius limits a verified silhouette; it never selects unrelated cloud
    points or clears an entire disk around a tracked coordinate.
    """
    empty=np.empty((0,3),dtype=float)
    if center_odom is None or not np.isfinite(depth).any():return empty
    center=np.asarray(center_odom,dtype=float)
    if center.shape!=(2,) or not np.isfinite(center).all() or not 0<radius<=.35:
        return empty
    from _wla_sim_geometry import deproject
    points=deproject(depth,np.asarray(intrinsics,dtype=float),
        np.asarray(camera_rotation,dtype=float),np.asarray(translation,dtype=float),
        minimum,maximum,stride)
    if not len(points):return empty
    world=local_points(points,pose)
    return world[np.linalg.norm(world[:,:2]-center,axis=1)<=radius]


def packet_from_filtered_points(obstacles, clearing, origin, pose):
    """Separate accepted obstacle endpoints from measured clearing rays.

    Native filtering owns height, local ground and isolated clusters. Rejected
    points are never re-marked by the legacy map's broader height band.
    """
    obstacles=np.asarray(obstacles,dtype=float).reshape(-1,3)
    clearing=np.asarray(clearing,dtype=float).reshape(-1,3)
    if not np.isfinite(obstacles).all() or not np.isfinite(clearing).all():
        raise ValueError('nonfinite_filtered_cloud')
    origin=np.asarray(origin,dtype=float)
    return dict(points=clearing,points_odom=local_points(clearing,pose),origin=origin,
                origin_odom=local_points(origin[None,:],pose)[0],
                dynamic=np.zeros(len(clearing),dtype=bool),obstacle_points=obstacles,
                marking_points_odom=local_points(obstacles,pose))


def measurements_from_relative(objects, pose, target_id, center_offset=.28, *, target_is_obstacle=False):
    """Existing detections are measured surfaces; use the demo's body model.

    Offset is explicit/configurable, not an observed anatomical center.
    Detector already performed image-time optical->base SE(3).
    """
    people, target = {}, None
    for key, row in objects.items():
        if not row['detected'] or row['label'] != 'person' or row['yaw'] is None:
            continue
        distance = row['distance'] + center_offset
        body = distance * np.array([math.cos(row['yaw']), math.sin(row['yaw'])])
        point = rotation(pose[2]) @ body + np.asarray(pose[:2])
        if key == target_id:
            target = point
            if target_is_obstacle:people[key]=point
        else:
            people[key] = point
    return target, people


class SimulationEngine:
    """Task controller owned by one worker; local map can survive task changes."""
    def __init__(self, pose, start_time, recovery_options=None, direct_tracking=False):
        self.tuning = WideTurnTuning()
        if direct_tracking:self.tuning=replace(self.tuning,wide_turn=False)
        self.controller = HybridFollower(self.tuning,recovery_options=recovery_options,direct_tracking=direct_tracking)
        self.mapping = RollingDepthMap(robot_radius=self.tuning.radius)  # same 12 m / .1 m / 12 s as demo
        self.mapping.slide(np.asarray(pose[:2]))
        self.mapping.rebuild(start_time)
        # Empty by default. Only the explicit, one-shot startup policy may seed it.
        self.target = VisualTrack(timeout=self.tuning.coast_timeout)
        self.people = PeopleTracks()
        self.last_measurement = -math.inf
        self.map_stamp = -math.inf
        self.start_time = start_time

    def step(self, pose, previous, packets, observation, now, policy=None, tracker_snapshot=None):
        for stamp, packet in sorted(packets, key=lambda item: item[0]):
            if stamp > self.map_stamp:
                self.mapping.update(packet, pose, stamp, rebuild=False)
                self.map_stamp = stamp
        self.mapping.slide(np.asarray(pose[:2]))
        self.mapping.local_assumed_free_center=np.asarray(pose[:2]).copy()
        self.mapping.rebuild(now)  # expiry never renews raw observation stamps
        visible = False
        if observation is not None:
            stamp, target, people = observation[:3]
            if len(observation)>3 and target is not None and not (len(observation)>4 and observation[4]):
                self.people.tracks.pop(observation[3],None)  # Rebound target is not a bystander.
            if self.last_measurement < stamp <= now:
                if tracker_snapshot is None:self.target.update(target, stamp)
                self.people.update(people, pose, stamp)
                self.last_measurement = stamp
                visible = target is not None
        # Predict copies to control time; delayed observations never update an
        # already future-propagated filter. Accepted history stays in the original.
        tracks = deepcopy(self.people).update({}, pose, now)
        if tracker_snapshot is None:
            estimate, velocity, state = deepcopy(self.target).update(None, now)
            history=self.target.history
            if visible and state != 'lost':state='visible'
            if policy is not None and not policy.translation_allowed(now):state='lost'
        else:
            binding_state=tracker_snapshot['state']
            value=tracker_snapshot['position'] if binding_state.startswith('search_') else tracker_snapshot['prediction']
            estimate=None if value is None else np.asarray(value,dtype=float)
            velocity=np.asarray(tracker_snapshot['velocity_mps'],dtype=float)
            history=[np.asarray(p,dtype=float) for p in tracker_snapshot['history']]
            state={'tracking':'visible','coasting':'coast'}.get(binding_state,binding_state)
            if state in ('binding','holding'):state='lost'
        # Recovery uses real elapsed time, not the number of surviving callbacks.
        self.controller.calls = max(0., (now-self.start_time)/self.tuning.step)
        self.controller.recovery.diagnostic_map_time=now
        command = self.controller.command(np.asarray(pose), np.asarray(previous),
            estimate, velocity, history, self.mapping, tracks, state, policy=policy, policy_time=now)
        if not np.isfinite(command).all():
            raise ValueError('nonfinite_core_command')
        recovery = self.controller.recovery
        reason = recovery.reason if recovery else ''
        candidates = self.controller.last_candidates
        if (abs(command[0]) < .01 and candidates.get('forward_total', 0) > 0
                and candidates.get('forward', 0) == 0):
            if candidates.get('forward_known') == 0:
                reason = 'forward_space_unobserved'
            elif candidates.get('forward_static') == 0:
                reason = 'forward_static_obstacle'
            elif candidates.get('forward_people') == 0:
                reason = 'forward_people_blocked'
            else:
                reason = 'forward_candidates_rejected'
        # The demo's least-bad braking fallback is diagnostic, not a validated
        # collision-free command. The hardware adapter never forwards it.
        if self.controller.last_infeasible:
            command = np.zeros(2)
            reason = 'no_feasible_candidate'
        if policy is not None:command=policy.limit_command(command,now)
        return dict(policy_snapshot=policy,command=np.asarray(command), target_state=state,
                    recovery_state=recovery.state, reason=reason,
                    wide_offset=self.controller.last_wide['offset'] if self.controller.last_wide else None,
                    rear_yield=self.controller.last_yield,
                    tuning=asdict(self.tuning), path=self.controller.last_reference_path.copy(),
                    candidates=self.controller.last_candidates,
                    front_blockage=dict(self.controller.recovery.front_diagnostics,control_stamp_s=now),
                    planner_target=dict(position_reference_m=None if estimate is None else np.asarray(estimate).tolist(),
                        velocity_reference_mps=np.asarray(velocity).tolist(),state=state,
                        reference_method='windowed_cubic_bezier' if self.controller.direct_tracking else 'legacy_reference',
                        history_point_count=len(history),reference_point_count=len(self.controller.last_reference_path),
                        history_sampling=tracker_snapshot.get('history_sampling') if tracker_snapshot is not None else self.target.trail.diagnostic(),
                        last_measurement_stamp_s=tracker_snapshot['last_seen'] if tracker_snapshot is not None else None if not math.isfinite(self.last_measurement) else self.last_measurement),
                    obstacle_track_ids=[o.get('id') for o in tracks])


_PROCESS_ENGINE = None
_PROCESS_KEY = None
_PROCESS_STARTUP_USED = False
_PROCESS_STARTUP_PRIOR = None


def worker_initializer():
    # ROS launch forwards SIGINT to the process group. Parent owns cleanup;
    # interrupting a worker blocked in the queue would corrupt shutdown.
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def warm_worker():
    """Import/initialize before accepting sensor jobs, outside control deadlines."""
    return asdict(WideTurnTuning())


def process_step(job):
    """Single spawned process owns the numerical state; no rclpy in this module."""
    global _PROCESS_ENGINE, _PROCESS_KEY, _PROCESS_STARTUP_USED, _PROCESS_STARTUP_PRIOR
    key, pose, previous, packets, observation, now, startup = job
    radius=float(startup['radius'])
    local_radius=float(startup.get('local_radius',0.))
    if not math.isfinite(local_radius) or not 0 <= local_radius <= .5:
        raise ValueError('invalid_local_assumed_free_radius')
    motion_threshold=float(startup.get('motion_threshold',.03))
    if not math.isfinite(motion_threshold) or not .001 <= motion_threshold <= .1:
        raise ValueError('invalid_startup_motion_threshold')
    if not math.isfinite(radius) or not 0 <= radius <= .5:
        raise ValueError('invalid_startup_clear_radius')
    if _PROCESS_KEY != key:
        old=_PROCESS_ENGINE
        same_reference=old is not None and _PROCESS_KEY[-1]==key[-1]
        recovery_options=dict(
            front_check_truncation=startup.get('front_check_truncation',False),
            front_check_max_m=startup.get('front_check_max_m',1.4),
            front_check_target_offset_m=startup.get('front_check_target_offset_m',.87),
            retreat_disabled=startup.get('retreat_disabled',False),
            reverse_policy_v2=startup.get('reverse_policy_v2',False),
            reverse_policy_allow_retreat=startup.get('reverse_policy_allow_retreat',False))
        _PROCESS_ENGINE = SimulationEngine(pose, now, recovery_options=recovery_options,direct_tracking=startup.get('direct_tracking',False))
        # Starting/stopping a task resets target/controller state, not recent
        # obstacle evidence. A wheel reference reset still discards the map.
        if same_reference:
            _PROCESS_ENGINE.mapping=old.mapping
            _PROCESS_ENGINE.map_stamp=old.map_stamp
        _PROCESS_KEY = key
    engine = _PROCESS_ENGINE
    engine.mapping.local_assumed_free_radius=local_radius
    engine.mapping.target_clear_ttl=float(startup.get('target_clear_ttl_s',.5))
    if startup['active'] and local_radius==0 and radius>0 and not _PROCESS_STARTUP_USED:
        # User-authorized assumption, once at the first follow task, not during
        # idle warm-up. Never erase observed obstacles or renew on task changes.
        engine.mapping.slide(np.asarray(pose[:2]))
        count=engine.mapping.initial_clear_patch(pose[:2],now,radius,only_unknown=True,hold_until_motion=True)
        _PROCESS_STARTUP_PRIOR=dict(source='operator_assumed_startup_free',
            radius_m=radius,center_odom=np.asarray(pose[:2]).tolist(),stamp_s=now,
            expires_s=None,motion_started_s=None,motion_threshold_m=motion_threshold,
            reference_generation=key[-1],seeded_cells=count)
        _PROCESS_STARTUP_USED=True
    prior=_PROCESS_STARTUP_PRIOR
    if (prior is not None and prior['reference_generation']==key[-1]
            and prior['motion_started_s'] is None):
        # Translation comes from wheel odometry, never from a commanded velocity.
        moved=np.linalg.norm(np.asarray(pose[:2])-prior['center_odom'])>=prior['motion_threshold_m']
        engine.mapping.advance_startup_prior(now,release=bool(moved))
        if moved:
            prior['motion_started_s']=now
            prior['expires_s']=now+engine.mapping.ttl
    result = engine.step(pose, previous, packets, observation, now, policy=startup.get('policy_snapshot'),tracker_snapshot=startup.get('tracker_snapshot'))
    prior=dict(_PROCESS_STARTUP_PRIOR) if _PROCESS_STARTUP_PRIOR else None
    if prior is not None:
        if prior['reference_generation']!=key[-1]:
            prior.update(remaining_s=0.,state='reference_changed')
        elif prior['motion_started_s'] is None:
            prior.update(remaining_s=None,state='waiting_for_translation')
        else:
            prior['remaining_s']=max(0.,prior['expires_s']-now)
            prior['state']='counting_down' if prior['remaining_s']>0 else 'expired'
    result.update(map_lower=engine.mapping.lower.copy(),resolution=engine.mapping.res,
        startup_prior=prior,startup_prior_consumed=_PROCESS_STARTUP_USED,
        local_free_prior=dict(source='operator_assumed_robot_free',radius_m=local_radius,
            center_odom=np.asarray(pose[:2]).tolist(),state='continuous' if local_radius else 'disabled',
            assumed_cells=int(np.count_nonzero(engine.mapping.assumed_free))),
        retreat_distance_m=engine.controller.recovery.retreat_distance,
        grid=np.where(engine.mapping.occupied,100,np.where(engine.mapping.known,0,-1)).astype(np.int8))
    return result
