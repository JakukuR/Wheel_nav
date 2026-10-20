"""Real observations -> frozen close-wide-flex core. No ROS or actuator I/O."""
from copy import deepcopy
from dataclasses import asdict
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


def measurements_from_relative(objects, pose, target_id, center_offset=.28):
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
        else:
            people[key] = point
    return target, people


class SimulationEngine:
    """Task controller owned by one worker; local map can survive task changes."""
    def __init__(self, pose, start_time):
        self.tuning = WideTurnTuning()
        self.controller = HybridFollower(self.tuning)
        self.mapping = RollingDepthMap(robot_radius=self.tuning.radius)  # same 12 m / .1 m / 12 s as demo
        self.mapping.slide(np.asarray(pose[:2]))
        self.mapping.rebuild(start_time)
        # Empty by default. Only the explicit, one-shot startup policy may seed it.
        self.target = VisualTrack(pose[:2], self.tuning.coast_timeout)
        self.people = PeopleTracks()
        self.last_measurement = -math.inf
        self.map_stamp = -math.inf
        self.start_time = start_time

    def step(self, pose, previous, packets, observation, now):
        for stamp, packet in sorted(packets, key=lambda item: item[0]):
            if stamp > self.map_stamp:
                self.mapping.update(packet, pose, stamp, rebuild=False)
                self.map_stamp = stamp
        self.mapping.rebuild(now)  # expiry never renews raw observation stamps
        visible = False
        if observation is not None:
            stamp, target, people = observation
            if self.last_measurement < stamp <= now:
                self.target.update(target, stamp)
                self.people.update(people, pose, stamp)
                self.last_measurement = stamp
                visible = target is not None
        # Predict copies to control time; delayed observations never update an
        # already future-propagated filter. Accepted history stays in the original.
        estimate, velocity, state = deepcopy(self.target).update(None, now)
        tracks = deepcopy(self.people).update({}, pose, now)
        if visible and state != 'lost':
            state = 'visible'
        # Recovery uses real elapsed time, not the number of surviving callbacks.
        self.controller.calls = max(0., (now-self.start_time)/self.tuning.step)
        command = self.controller.command(np.asarray(pose), np.asarray(previous),
            estimate, velocity, self.target.history, self.mapping, tracks, state)
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
        return dict(command=np.asarray(command), target_state=state,
                    recovery_state=recovery.state, reason=reason,
                    wide_offset=self.controller.last_wide['offset'] if self.controller.last_wide else None,
                    rear_yield=self.controller.last_yield,
                    tuning=asdict(self.tuning), path=np.asarray(self.target.history),
                    candidates=self.controller.last_candidates)


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
    if not math.isfinite(radius) or not 0 <= radius <= .5:
        raise ValueError('invalid_startup_clear_radius')
    if _PROCESS_KEY != key:
        old=_PROCESS_ENGINE
        same_reference=old is not None and _PROCESS_KEY[-1]==key[-1]
        _PROCESS_ENGINE = SimulationEngine(pose, now)
        # Starting/stopping a task resets target/controller state, not recent
        # obstacle evidence. A wheel reference reset still discards the map.
        if same_reference:
            _PROCESS_ENGINE.mapping=old.mapping
            _PROCESS_ENGINE.map_stamp=old.map_stamp
        _PROCESS_KEY = key
    engine = _PROCESS_ENGINE
    if startup['active'] and radius>0 and not _PROCESS_STARTUP_USED:
        # User-authorized assumption, once at the first follow task, not during
        # idle warm-up. Never erase observed obstacles or renew on task changes.
        engine.mapping.slide(np.asarray(pose[:2]))
        count=engine.mapping.initial_clear_patch(pose[:2],now,radius,only_unknown=True)
        _PROCESS_STARTUP_PRIOR=dict(source='operator_assumed_startup_free',
            radius_m=radius,center_odom=np.asarray(pose[:2]).tolist(),stamp_s=now,
            expires_s=now+engine.mapping.ttl,reference_generation=key[-1],seeded_cells=count)
        _PROCESS_STARTUP_USED=True
    result = engine.step(pose, previous, packets, observation, now)
    prior=dict(_PROCESS_STARTUP_PRIOR) if _PROCESS_STARTUP_PRIOR else None
    if prior is not None:
        prior['remaining_s']=max(0.,prior['expires_s']-now) if prior['reference_generation']==key[-1] else 0.
        prior['state']='active' if prior['remaining_s']>0 else 'expired_or_reference_changed'
    result.update(map_lower=engine.mapping.lower.copy(),resolution=engine.mapping.res,
        startup_prior=prior,startup_prior_consumed=_PROCESS_STARTUP_USED,
        grid=np.where(engine.mapping.occupied,100,np.where(engine.mapping.known,0,-1)).astype(np.int8))
    return result
