"""Shared tracker-to-MPC runtime for three target modes. No ROS or actuator I/O."""
from copy import deepcopy
from dataclasses import asdict, replace
import math
import numpy as np
from .sim_core.common import rotation
from .sim_core.common import Tuning
from .sim_core.approach_goal import ApproachGoal, SWEEP_RADIUS
from .sim_core.hybrid_controller import HybridFollower, seed_command, limit_step
from .sim_core.rolling_depth_map import RollingDepthMap
from .sim_core.tracking import PeopleTracks


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


def measurements_from_relative(objects, pose, target_id, center_offset=.28):
    """All other people remain dynamic obstacles, including during object approach."""
    people, target = {}, None
    for key, row in objects.items():
        if not row['detected'] or row['yaw'] is None or row['distance'] is None:
            continue
        if key != target_id and row['label'] != 'person':
            continue
        offset = center_offset if key == target_id else .28
        distance = row['distance'] + offset
        point = rotation(pose[2]) @ (distance*np.array([math.cos(row['yaw']),math.sin(row['yaw'])])) + np.asarray(pose[:2])
        if key == target_id:target=point
        else:people[key]=point
    return target, people


# Rolling map: 0.05 m cells over an 8 m window (+-4 m). Halves the cell
# quantisation of 0.1 m cells (passes 0.65-0.75 m corridors in simulation) while
# the smaller window keeps map work near 13 ms on the Orin; the MPC horizon
# (~2.9 m at 1.2 m/s) and object routes stay inside the window.
MAP_RESOLUTION_M=.05
MAP_SIZE_CELLS=160



class SimulationEngine:
    """Single tracker input, shared rolling map and shared MPC; no second FSM."""
    def __init__(self, pose, start_time):
        self.tuning=Tuning()
        self.controller=HybridFollower(self.tuning)
        self.mapping=RollingDepthMap(resolution=MAP_RESOLUTION_M,size=MAP_SIZE_CELLS,robot_radius=self.tuning.radius)
        self.mapping.slide(np.asarray(pose[:2]));self.mapping.rebuild(start_time)
        self.people=PeopleTracks()
        self.last_measurement=self.map_stamp=-math.inf
        self.approach=ApproachGoal()
        # Last issued planner command: rollout seed and per-interval step bound.
        self.last_command=None;self.last_command_at=None

    def update_map(self, pose, packets, now):
        """Maintain obstacle evidence and its expiry independently of task activity."""
        for stamp,packet in sorted(packets,key=lambda item:item[0]):
            if stamp>self.map_stamp:
                self.mapping.update(packet,pose,stamp,rebuild=False);self.map_stamp=stamp
        self.mapping.slide(np.asarray(pose[:2]))
        self.mapping.local_assumed_free_center=np.asarray(pose[:2]).copy()
        # Control-rate replans bring no packets: skip the EDTs whose inputs did
        # not change (exact; see RollingDepthMap.rebuild).
        self.mapping.rebuild(now,reuse=True)

    def step(self, pose, previous, packets, observation, now, policy=None,
             tracker_snapshot=None, task_mode='0', object_surface=None, *, active=True, recovery=False):
        self.update_map(pose,packets,now)
        if not active:
            self.controller.reset()
            # Do not predict people, generate approach goals or invoke the MPC.
            # Return fresh empty planning fields rather than the previous task's path.
            return dict(policy_snapshot=None,command=np.zeros(2),target_state='idle',
                recovery_state='idle',reason='task_inactive',tuning=asdict(self.tuning),
                path=np.empty((0,2)),candidates={},approach_goal=None,planner_feasible=False,
                planner_activity='idle_map_only',
                front_blockage=dict(evaluated=False,reason='task_inactive',control_stamp_s=now),
                planner_target=dict(position_reference_m=None,velocity_reference_mps=[0.,0.],
                    state='idle',task_mode=task_mode,reference_method='none',
                    history_point_count=0,reference_point_count=0,history_sampling=None,
                    last_measurement_stamp_s=None),obstacle_track_ids=[])
        return self._plan(pose,previous,observation,now,policy,tracker_snapshot,task_mode,object_surface,recovery)

    def _plan(self, pose, previous, observation, now, policy, tracker_snapshot,
              task_mode, object_surface, recovery=False):
        if observation is not None:
            stamp,target,people,resolved_id=observation
            if target is not None:self.people.tracks.pop(resolved_id,None)
            if self.last_measurement<stamp<=now:
                self.people.update(people,pose,stamp);self.last_measurement=stamp
        tracks=deepcopy(self.people).update({},pose,now)
        estimate=None;velocity=np.zeros(2);history=[];state='lost'
        if tracker_snapshot is not None:
            binding=tracker_snapshot['state']
            value=tracker_snapshot['position'] if binding.startswith('search_') else tracker_snapshot['prediction']
            estimate=None if value is None else np.asarray(value,dtype=float)
            velocity=np.asarray(tracker_snapshot['velocity_mps'],dtype=float)
            history=[np.asarray(point,dtype=float) for point in tracker_snapshot['history']]
            state={'tracking':'visible','coasting':'coast'}.get(binding,binding)
        tuning=self.tuning;route=None;face=None;approach=None
        if task_mode=='1':
            stop_gap=.95+(tracker_snapshot or {}).get('surface_offset_m',.28)
            tuning=replace(tuning,desired_gap=stop_gap,target_stop_gap=stop_gap,
                target_clearance_max=0.,max_v=.30)
            # Finite approach is to the current person, not a stale five-second trail.
            history=[];velocity=np.zeros(2)
        elif task_mode=='2':
            tuning=replace(tuning,radius=SWEEP_RADIUS,desired_gap=0.,target_stop_gap=.025,
                target_hard_gap=0.,target_clearance_max=0.,max_v=.20)
            velocity=np.zeros(2);history=[]
            if state=='visible' and object_surface is not None:
                surface=np.asarray(object_surface,dtype=float).reshape(-1,2)
                approach=self.approach.select(pose,surface,self.mapping,now)
                if approach is not None:
                    route=approach['path'];estimate=approach['point'];face=np.median(surface,axis=0)
            if approach is None:state='holding'
        self.controller.c=tuning
        seed=seed_command(previous,self.last_command,self.last_command_at,now)
        command=self.controller.command(np.asarray(pose),seed,estimate,velocity,
            history,self.mapping,tracks,state,policy=policy,policy_time=now,reference_path=route,face_point=face,
            measured=np.asarray(previous,dtype=float),recovery=recovery)
        if self.controller.last_candidates and not self.controller.last_infeasible:
            # Plans now run at control rate; the first change is bounded by the
            # time since the last issued command (zero/stop outputs stay exact).
            dt=tuning.step if self.last_command_at is None else now-self.last_command_at
            command=limit_step(command,seed,dt,tuning)
        reason=('object_goal_unavailable' if task_mode=='2' and approach is None else
            approach['reason'] if approach is not None else state if state.startswith('search_') else
            'tracker_prediction' if state=='coast' else 'tracker_follow')
        if self.controller.last_infeasible:
            command=np.zeros(2);reason='no_feasible_candidate'
        elif recovery:reason='depth_coverage_recovery'
        elif self.controller.last_candidates.get('backing'):reason='backing_out'
        if policy is not None:command=policy.limit_command(command,now)
        if not np.isfinite(command).all():raise ValueError('nonfinite_core_command')
        self.last_command=np.asarray(command,dtype=float).copy();self.last_command_at=now
        diagnostic=None if approach is None else {k:(v.tolist() if isinstance(v,np.ndarray) else v)
            for k,v in approach.items() if k!='path'}
        return dict(policy_snapshot=policy,command=np.asarray(command),target_state=state,
            recovery_state=state,reason=reason,tuning=asdict(tuning),
            planner_activity='candidate_evaluation' if self.controller.last_candidates else 'waiting_target',
            path=self.controller.last_reference_path.copy(),candidates=self.controller.last_candidates,
            approach_goal=diagnostic,planner_feasible=bool(self.controller.last_candidates.get('feasible',0)>0),
            front_blockage=dict(evaluated=False,reason='mpc_owns_collision_decision',control_stamp_s=now),
            planner_target=dict(position_reference_m=None if estimate is None else estimate.tolist(),
                velocity_reference_mps=velocity.tolist(),state=state,task_mode=task_mode,
                reference_method='windowed_cubic_bezier' if task_mode=='0' else 'connected_object_route' if task_mode=='2' else 'current_person',
                history_point_count=len(history),reference_point_count=len(self.controller.last_reference_path),
                history_sampling=tracker_snapshot.get('history_sampling') if tracker_snapshot else None,
                last_measurement_stamp_s=tracker_snapshot['last_seen'] if tracker_snapshot else None),
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
    return asdict(Tuning())


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
        _PROCESS_ENGINE = SimulationEngine(pose, now)
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
    result = engine.step(pose, previous, packets, observation, now, policy=startup.get('policy_snapshot'),tracker_snapshot=startup.get('tracker_snapshot'),task_mode=startup.get('task_mode','0'),object_surface=startup.get('object_surface'),active=startup['active'],recovery=bool(startup.get('recovery',False)))
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
        grid=np.where(engine.mapping.occupied,100,np.where(engine.mapping.known,0,-1)).astype(np.int8))
    return result
