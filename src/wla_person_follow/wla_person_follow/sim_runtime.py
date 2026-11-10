"""Shared tracker-to-MPC runtime for three target modes. No ROS or actuator I/O."""
from dataclasses import asdict, replace
import math
import numpy as np
from .sim_core.common import rotation
from .sim_core.common import Tuning
from .sim_core.approach_goal import ApproachGoal, SWEEP_RADIUS
from .sim_core.hybrid_controller import seed_command, limit_step
from .neupan_follower import NeuPANFollower, TRACKED_STATES as NEUPAN_STATES
from .native_map import BACKENDS as NUMERIC_BACKENDS, make_rolling_map, map_backend
from .native_tracking import make_people_tracks
from .native_planning import make_approach_goal, make_controller
from .native_engine import NativeFollowEngine


def local_points(points, pose):
    result = np.asarray(points, dtype=float).copy()
    result[:, :2] = result[:, :2] @ rotation(pose[2]).T + pose[:2]
    return result


def drop_depth_speckles(depth, max_pixels, max_diff=.075):
    """Blank small depth-connected patches: the stereo smear between a person's legs.

    Returns a new array in metres with NaN in the dropped pixels. A patch survives
    if it reaches max_pixels with neighbours within max_diff, so a tall thin pole
    stays while a few pixels floating off a leg edge go.
    """
    import cv2
    millimetres = np.clip(np.nan_to_num(np.asarray(depth, dtype=float), nan=0., posinf=0., neginf=0.) * 1000.,
                          0, 32767).astype(np.int16)
    cv2.filterSpeckles(millimetres, 0, int(max_pixels), int(round(max_diff * 1000.)))
    return np.where(millimetres == 0, np.nan, depth)


def packet_from_depth(depth, intrinsics, camera_rotation, translation, pose,
                      minimum=.15, maximum=5., stride=4):
    from _wla_sim_geometry import deproject
    points = deproject(depth, np.asarray(intrinsics, dtype=float),
                       np.asarray(camera_rotation, dtype=float),
                       np.asarray(translation, dtype=float), minimum, maximum, stride)
    from _wla_sim_geometry import filter_points
    obstacles, clearing = filter_points(points)
    return packet_from_filtered_points(obstacles, clearing, translation, pose)


def packet_without_person(depth, intrinsics, camera_rotation, translation, pose, center_odom,
                          others_odom=(), minimum=.15, maximum=5., stride=4):
    """Remove the person by connectivity in the raw cloud; (None, info) if refused.

    Must run before filter_points: its 0.5 m height limit leaves two separate legs.
    """
    from _wla_sim_geometry import deproject, filter_points
    from .person_cluster import select_person
    points = deproject(depth, np.asarray(intrinsics, dtype=float),
                       np.asarray(camera_rotation, dtype=float),
                       np.asarray(translation, dtype=float), minimum, maximum, stride)
    world = local_points(points, pose)
    mask, info = select_person(world, center_odom, others=others_odom)
    if not mask.any():
        return None, info
    obstacles, clearing = filter_points(points[~mask])
    packet = packet_from_filtered_points(obstacles, clearing, translation, pose)
    packet['target_points_odom'] = world[mask & (world[:, 2] >= .03) & (world[:, 2] <= .6)]
    packet['target_points_from_cluster'] = True
    return packet, info


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
# Person trail kept for ghost retirement: a little over the map's 12 s TTL.
TRAIL_RETENTION_S=13.
MAP_SIZE_CELLS=160
# Point goal (mode 3): route refresh period, arrival band and speed cap.
POINT_ROUTE_REFRESH_S=.5
POINT_GOAL_MAX_V=.80
# Orient (mode 4): in-place turn above the chassis yaw deadband (0.3 rad/s).
ORIENT_MIN_W=.35
ORIENT_MAX_W=.50
ORIENT_DONE_RAD=.05


class SimulationEngine:
    """Single tracker input, shared rolling map and shared MPC; no second FSM."""
    def __init__(self, pose, start_time, numeric_backend='python'):
        self.tuning=Tuning()
        self.controller=make_controller(numeric_backend,self.tuning)
        # numeric_backend 'cpp': map, people tracks, reference curves, MPC, detour and approach goal in the
        # C++ follow core (equal to the Python reference element-wise); never falls back silently.
        self.mapping=make_rolling_map(numeric_backend,resolution=MAP_RESOLUTION_M,size=MAP_SIZE_CELLS,robot_radius=self.tuning.radius)
        self.mapping.slide(np.asarray(pose[:2]));self.mapping.rebuild(start_time)
        self.people=make_people_tracks(numeric_backend)
        self.last_measurement=self.map_stamp=-math.inf
        self.approach=make_approach_goal(numeric_backend)
        self.point_route=None  # (goal, planned_at, path) for mode 3
        # Followed person's measured centres (stamp, x, y) in modes 0/1: the map
        # forgets hits that appeared where the person stood (retire_person_trail).
        self.target_trail=np.empty((0,3));self.trail_clear_radius=0.;self.trail_retired=0
        self.trail_checked=None
        # Last issued planner command: rollout seed and per-interval step bound.
        self.last_command=None;self.last_command_at=None
        self.neupan=None  # NeuPANFollower when local_planner is neupan

    def neupan_command(self, pose, tracker_snapshot, task_mode, recovery):
        """Person following with a target position only; search, recovery and other modes stay as they are."""
        if (self.neupan is None or task_mode!='0' or recovery or tracker_snapshot is None
                or tracker_snapshot['state'] not in NEUPAN_STATES or tracker_snapshot.get('prediction') is None):
            return None
        cells=np.argwhere(self.mapping.occupied)
        points=(cells+self.mapping.lower+.5)*self.mapping.res if len(cells) else np.zeros((0,2))
        return self.neupan.plan(pose,tracker_snapshot['prediction'],points,self.controller.c.desired_gap)

    def update_map(self, pose, packets, now):
        """Maintain obstacle evidence and its expiry independently of task activity."""
        for stamp,packet in sorted(packets,key=lambda item:item[0]):
            if stamp>self.map_stamp:
                if packet.get('target_points_from_cluster'):
                    packet=dict(packet,target_points_odom=self.mapping.drop_established(
                        packet['target_points_odom'],self.target_trail,stamp))
                self.mapping.update(packet,pose,stamp,rebuild=False);self.map_stamp=stamp
                disc=packet.get('retire_disc_odom')
                if disc is not None:
                    disc=np.asarray(disc,dtype=float).reshape(3)
                    self.mapping.retire_disc(disc[:2],float(disc[2]),stamp)
        self.mapping.slide(np.asarray(pose[:2]))
        self.mapping.local_assumed_free_center=np.asarray(pose[:2]).copy()
        if len(self.target_trail):
            self.target_trail=self.target_trail[self.target_trail[:,0]>=now-TRAIL_RETENTION_S]
            # Only new hits or a new trail point can create a match.
            state=(self.mapping.version,len(self.target_trail) and self.target_trail[-1,0])
            if state!=self.trail_checked:
                self.trail_retired=self.mapping.retire_person_trail(self.target_trail,now,self.trail_clear_radius)
                self.trail_checked=(self.mapping.version,state[1])
        # Control-rate replans bring no packets: skip the EDTs whose inputs did
        # not change (exact; see RollingDepthMap.rebuild).
        self.mapping.rebuild(now,reuse=True)

    def step(self, pose, previous, packets, observation, now, policy=None,
             tracker_snapshot=None, task_mode='0', object_surface=None, *, active=True, recovery=False, goal=None):
        if active and task_mode in ('0','1') and observation is not None and observation[1] is not None:
            stamp=float(observation[0])
            if not len(self.target_trail) or stamp>self.target_trail[-1,0]:
                self.target_trail=np.vstack((self.target_trail,[stamp,*np.asarray(observation[1],dtype=float)[:2]]))
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
        return self._plan(pose,previous,observation,now,policy,tracker_snapshot,task_mode,object_surface,recovery,goal)

    def _point_route(self, pose, goal, tuning, now):
        """Clearance-weighted grid route to a fixed odom point (DetourPlanner's Dijkstra).

        Reused for POINT_ROUTE_REFRESH_S while its remaining part stays clear;
        without a route the MPC gets the straight line and creeps on known space.
        """
        old=self.point_route
        if (old is not None and np.allclose(old[0],goal) and now-old[1]<POINT_ROUTE_REFRESH_S):
            path=old[2];near=int(np.argmin(np.linalg.norm(path-pose[:2],axis=1)))
            suffix=np.vstack((pose[:2],path[near+1:])) if near+1<len(path) else np.vstack((pose[:2],goal))
            if np.all(self.mapping.query(ApproachGoal._samples(suffix))>=tuning.radius):return suffix
        path=self.controller.detour._grid_route(np.asarray(pose[:2],dtype=float),goal,self.mapping,tuning)
        if path is None:path=np.vstack((pose[:2],goal))
        self.point_route=(goal.copy(),now,path)
        return path

    def _plan(self, pose, previous, observation, now, policy, tracker_snapshot,
              task_mode, object_surface, recovery=False, goal=None):
        if observation is not None:
            stamp,target,people,resolved_id=observation
            if target is not None:self.people.drop(resolved_id)
            if self.last_measurement<stamp<=now:
                self.people.update(people,pose,stamp);self.last_measurement=stamp
        tracks=self.people.predict(now)
        estimate=None;velocity=np.zeros(2);history=[];state='lost'
        if tracker_snapshot is not None:
            binding=tracker_snapshot['state']
            value=tracker_snapshot['position'] if binding.startswith('search_') else tracker_snapshot['prediction']
            estimate=None if value is None else np.asarray(value,dtype=float)
            velocity=np.asarray(tracker_snapshot['velocity_mps'],dtype=float)
            history=[np.asarray(point,dtype=float) for point in tracker_snapshot['history']]
            state={'tracking':'visible','coasting':'coast'}.get(binding,binding)
            if binding=='holding' and task_mode in ('0','1'):state='coast'
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
        elif task_mode=='3':
            tuning=replace(tuning,desired_gap=0.,target_stop_gap=.05,target_hard_gap=0.,
                target_clearance_max=0.,max_v=POINT_GOAL_MAX_V)
            estimate=np.asarray(goal['point'],dtype=float);velocity=np.zeros(2);history=[];state='visible'
            route=self._point_route(pose,estimate,tuning,now)
        elif task_mode=='4':
            velocity=np.zeros(2);history=[];state='visible'
        self.controller.c=tuning
        seed=seed_command(previous,self.last_command,self.last_command_at,now)
        if task_mode=='4':
            # Turning in place never changes the circular footprint's clearance.
            remaining=float(goal['remaining'])
            w=0. if abs(remaining)<=ORIENT_DONE_RAD else math.copysign(float(np.clip(1.2*abs(remaining),ORIENT_MIN_W,ORIENT_MAX_W)),remaining)
            self.controller.last_candidates={};self.controller.last_infeasible=False
            self.controller.last_reference_path=np.empty((0,2))
            dt=tuning.step if self.last_command_at is None else now-self.last_command_at
            command=limit_step(np.array([0.,w]),seed,dt,tuning)
        else:
            command=self.controller.command(np.asarray(pose),seed,estimate,velocity,
                history,self.mapping,tracks,state,policy=policy,policy_time=now,reference_path=route,face_point=face,
                measured=np.asarray(previous,dtype=float),recovery=recovery)
        if self.controller.last_candidates and not self.controller.last_infeasible:
            # Plans now run at control rate; the first change is bounded by the
            # time since the last issued command (zero/stop outputs stay exact).
            dt=tuning.step if self.last_command_at is None else now-self.last_command_at
            command=limit_step(command,seed,dt,tuning)
        reason=('point_goal' if task_mode=='3' else 'orient' if task_mode=='4' else
            'object_goal_unavailable' if task_mode=='2' and approach is None else
            approach['reason'] if approach is not None else state if state.startswith('search_') else
            'holding_last_trusted' if tracker_snapshot is not None and tracker_snapshot['state']=='holding' and state=='coast' else
            'tracker_prediction' if state=='coast' else 'tracker_follow')
        if self.controller.last_infeasible:
            command=np.zeros(2);reason='no_feasible_candidate'
        elif recovery:reason='depth_coverage_recovery'
        elif self.controller.last_candidates.get('backing'):reason='backing_out'
        neupan_command=self.neupan_command(np.asarray(pose),tracker_snapshot,task_mode,recovery)
        neupan_status=None
        if neupan_command is not None:
            command=neupan_command;neupan_status=self.neupan.diagnostic
            self.controller.last_reference_path=np.vstack((np.asarray(pose)[:2],tracker_snapshot['prediction']))
        if policy is not None:command=policy.limit_command(command,now)
        if not np.isfinite(command).all():raise ValueError('nonfinite_core_command')
        self.last_command=np.asarray(command,dtype=float).copy();self.last_command_at=now
        diagnostic=None if approach is None else {k:(v.tolist() if isinstance(v,np.ndarray) else v)
            for k,v in approach.items() if k!='path'}
        return dict(policy_snapshot=policy,command=np.asarray(command),neupan=neupan_status,target_state=state,
            recovery_state=state,reason=reason,tuning=asdict(tuning),
            planner_activity='candidate_evaluation' if self.controller.last_candidates else 'in_place_turn' if task_mode=='4' else 'waiting_target',
            path=self.controller.last_reference_path.copy(),candidates=self.controller.last_candidates,
            approach_goal=diagnostic,planner_feasible=bool(self.controller.last_candidates.get('feasible',0)>0),
            front_blockage=dict(evaluated=False,reason='mpc_owns_collision_decision',control_stamp_s=now),
            planner_target=dict(position_reference_m=None if estimate is None else estimate.tolist(),
                velocity_reference_mps=velocity.tolist(),state=state,task_mode=task_mode,
                reference_method={'0':'windowed_cubic_bezier','2':'connected_object_route','3':'clearance_grid_route','4':'in_place_turn'}.get(task_mode,'current_person'),
                history_point_count=len(history),reference_point_count=len(self.controller.last_reference_path),
                history_sampling=tracker_snapshot.get('history_sampling') if tracker_snapshot else None,
                last_measurement_stamp_s=tracker_snapshot['last_seen'] if tracker_snapshot else None),
            obstacle_track_ids=[o.get('id') for o in tracks])


_PROCESS_ENGINE = None
_PROCESS_KEY = None
_PROCESS_STARTUP_USED = False
_PROCESS_STARTUP_PRIOR = None
_PROCESS_NEUPAN = None


def neupan_for(local_planner):
    global _PROCESS_NEUPAN
    if local_planner=='mpc':return None
    if local_planner!='neupan':raise ValueError('unknown_local_planner: '+str(local_planner))
    if _PROCESS_NEUPAN is None:_PROCESS_NEUPAN=NeuPANFollower()
    return _PROCESS_NEUPAN


def worker_initializer():
    # ROS launch forwards SIGINT to the process group. Parent owns cleanup;
    # interrupting a worker blocked in the queue would corrupt shutdown.
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def warm_worker(local_planner='mpc'):
    """Import/initialize before accepting sensor jobs, outside control deadlines."""
    neupan_for(local_planner)  # a missing NeuPAN build fails at startup, not during a task
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
    backend=startup.get('numeric_backend','python')
    if backend not in NUMERIC_BACKENDS:
        raise ValueError('unknown_numeric_backend: '+str(backend))
    if backend=='cpp':
        return _process_step_cpp(key,pose,previous,packets,observation,now,startup,radius,local_radius,motion_threshold)
    # A backend change starts from an empty map: no state crosses backends.
    if _PROCESS_KEY != key or map_backend(_PROCESS_ENGINE.mapping)!=backend:
        old=_PROCESS_ENGINE
        same_reference=(old is not None and _PROCESS_KEY[-1]==key[-1]
                        and map_backend(old.mapping)==backend)
        _PROCESS_ENGINE = SimulationEngine(pose, now, backend)
        # Starting/stopping a task resets target/controller state, not recent
        # obstacle evidence. A wheel reference reset still discards the map.
        if same_reference:
            _PROCESS_ENGINE.mapping=old.mapping
            _PROCESS_ENGINE.map_stamp=old.map_stamp
            _PROCESS_ENGINE.target_trail=old.target_trail
        _PROCESS_KEY = key
        if _PROCESS_NEUPAN is not None:_PROCESS_NEUPAN.reset()
    engine = _PROCESS_ENGINE
    engine.neupan=neupan_for(startup.get('local_planner','mpc'))
    grid_route=startup.get('grid_route_planner','dijkstra')
    if grid_route not in ('dijkstra','astar'):
        raise ValueError('unknown_grid_route_planner: '+str(grid_route))
    if engine.tuning.grid_route_planner!=grid_route:
        engine.tuning=replace(engine.tuning,grid_route_planner=grid_route)
    engine.mapping.local_assumed_free_radius=local_radius
    engine.mapping.target_clear_ttl=float(startup.get('target_clear_ttl_s',.5))
    engine.trail_clear_radius=float(startup.get('trail_clear_radius_m',0.))
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
    result = engine.step(pose, previous, packets, observation, now, policy=startup.get('policy_snapshot'),tracker_snapshot=startup.get('tracker_snapshot'),task_mode=startup.get('task_mode','0'),object_surface=startup.get('object_surface'),active=startup['active'],recovery=bool(startup.get('recovery',False)),goal=startup.get('goal'))
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
        person_trail_clear=dict(radius_m=engine.trail_clear_radius,trail_points=len(engine.target_trail),
            retired_cells=engine.trail_retired),
        local_free_prior=dict(source='operator_assumed_robot_free',radius_m=local_radius,
            center_odom=np.asarray(pose[:2]).tolist(),state='continuous' if local_radius else 'disabled',
            assumed_cells=int(np.count_nonzero(engine.mapping.assumed_free))),
        grid=np.where(engine.mapping.occupied,100,np.where(engine.mapping.known,0,-1)).astype(np.int8))
    return result


def _process_step_cpp(key, pose, previous, packets, observation, now, startup, radius, local_radius, motion_threshold):
    """process_step with numeric_backend cpp: one FollowCore call (native_engine.py).

    Same task/reference rules as the Python engine: a new task keeps the map,
    a wheel reference change starts a new one, and the startup prior is used
    once per process (mirrored in the globals so a backend change keeps it).
    """
    global _PROCESS_ENGINE, _PROCESS_KEY, _PROCESS_STARTUP_USED, _PROCESS_STARTUP_PRIOR
    engine=_PROCESS_ENGINE if isinstance(_PROCESS_ENGINE,NativeFollowEngine) else None
    if engine is None or _PROCESS_KEY!=key:
        if engine is None:
            engine=NativeFollowEngine(pose,now,resolution=MAP_RESOLUTION_M,size=MAP_SIZE_CELLS,
                startup_used=_PROCESS_STARTUP_USED,startup_prior=_PROCESS_STARTUP_PRIOR)
        elif _PROCESS_KEY[-1]==key[-1]:
            engine.reset_task()
        else:
            engine.reset_reference(pose,now)
        _PROCESS_ENGINE=engine;_PROCESS_KEY=key
        if _PROCESS_NEUPAN is not None:_PROCESS_NEUPAN.reset()
    neupan=neupan_for(startup.get('local_planner','mpc'))
    grid_route=startup.get('grid_route_planner','dijkstra')
    if grid_route not in ('dijkstra','astar'):
        raise ValueError('unknown_grid_route_planner: '+str(grid_route))
    result=engine.step(key,pose,previous,packets,observation,now,startup,local_radius=local_radius,radius=radius,
        motion_threshold=motion_threshold,grid_route=grid_route,neupan=neupan)
    _PROCESS_STARTUP_USED=result['startup_prior_consumed'];_PROCESS_STARTUP_PRIOR=engine.prior_base
    return result
