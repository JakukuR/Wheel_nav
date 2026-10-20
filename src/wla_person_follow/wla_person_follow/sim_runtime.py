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
                      minimum=.4, maximum=5., stride=4):
    from _wla_sim_geometry import deproject
    points = deproject(depth, np.asarray(intrinsics, dtype=float),
                       np.asarray(camera_rotation, dtype=float),
                       np.asarray(translation, dtype=float), minimum, maximum, stride)
    origin = np.asarray(translation, dtype=float)
    return dict(points=points, points_odom=local_points(points, pose),
                origin=origin, origin_odom=local_points(origin[None, :], pose)[0],
                # Real detector has boxes, not ideal instance segmentation.
                # Never erase a whole person box from the static depth evidence.
                dynamic=np.zeros(len(points), dtype=bool))


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
    """Persistent core, owned by one worker. Reset on request/reference change."""
    def __init__(self, pose, start_time):
        self.tuning = WideTurnTuning()
        self.controller = HybridFollower(self.tuning)
        self.mapping = RollingDepthMap(robot_radius=self.tuning.radius)  # same 12 m / .1 m / 12 s as demo
        self.mapping.slide(np.asarray(pose[:2]))
        self.mapping.rebuild(start_time)
        # No simulated initial_clear_patch: no sensor evidence exists for it.
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
    global _PROCESS_ENGINE, _PROCESS_KEY
    key, pose, previous, packets, observation, now = job
    if _PROCESS_KEY != key:
        _PROCESS_ENGINE = SimulationEngine(pose, now)
        _PROCESS_KEY = key
    engine = _PROCESS_ENGINE
    result = engine.step(pose, previous, packets, observation, now)
    result.update(map_lower=engine.mapping.lower.copy(),resolution=engine.mapping.res,
        grid=np.where(engine.mapping.occupied,100,np.where(engine.mapping.known,0,-1)).astype(np.int8))
    return result
