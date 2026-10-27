"""close-wide-flex ROS adapter. Motion disabled by default; no simulated truth."""
from collections import deque
from concurrent.futures import ProcessPoolExecutor, TimeoutError as ComputeTimeout
from multiprocessing import get_context
from copy import deepcopy
import json
import math
import signal
import threading
import time
import uuid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String, Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from wla_behavior_interfaces.srv import PersonCommand
from .camera import depth_meters
from .diagnostics import finite_json, observed_target, wheel_message
from wla_r680_navigation_py.target_masks import MaskDepthPairer, filter_target_depth
from wla_r680_navigation_py.raw_image import as_frame, frame_message, frame_meta
from .depth_proc import rotation_from_quaternion
from .motion_memory import LocalMotion
from .task_session import TaskSession
from .follow_config import PENDING_CALIBRATION_PARAMETERS, validate_follow_config, validate_detection_status_topic
from .follow_session import FollowTracker, RecoveryIdentity, TrackingPolicy, position, frame_target_ready
from .sim_core.common import rotation
from .sim_core.rolling_depth_map import native_ray_slots
from .sim_runtime import packet_from_depth, target_points_from_removed_depth, measurements_from_relative, local_points, process_step, warm_worker, worker_initializer
from .task_modes import mode_config, MODE_PERSON_FOLLOW, MODE_PERSON_APPROACH, MODE_OBJECT_APPROACH


def stamp_ns(msg):
    return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec


class FollowEngine:
    """Task/tracking/planning logic; transport owns ROS and executor lifetime."""
    def __init__(self, transport):
        self.io = transport
        defaults = dict(enable_motion=False, output_mode='mux',
            direct_topic='/r680_base/cmd_vel', prefix='/r680/person_follow',
            relative_topic='/r680/perception/relative_objects',
            detection_status_topic='/r680_nav/perception/status',
            depth_topic='/r680/d455/aligned_depth_to_color/image_raw',
            camera_info_topic='/r680/d455/color/camera_info',
            wheel_topic='/wheel/odom', base_frame='r680_mapping_floor',
            max_sensor_age_s=.30, depth_stride=4, depth_min_m=.15, depth_max_m=5.,
            surface_to_center_offset_m=.28, camera_min_valid_fraction=.25,
            camera_front_min_valid_fraction=.35, result_ttl_s=.15, startup_clear_radius_m=0., startup_motion_threshold_m=.03,
            target_mask_enabled=True, reid_enabled=False, reid_model_id='', reid_similarity=.40,
            reid_margin=.08, reid_confirm_frames=3, local_assumed_free_radius_m=0., tof_enabled=False,
            tof_topics=['/r680/tof/front_left/points','/r680/tof/front_right/points',
                        '/r680/tof/rear_left/points','/r680/tof/rear_right/points'],
            follow_preset='tracker_follow', target_prediction_s=2., search_arrival_m=.35,
            target_history_window_s=5., target_history_sample_period_s=.4,
            binding_jump_radius_m=.45, binding_jump_growth_mps=.25, binding_jump_radius_max_m=1.5,
            neighbor_ambiguity_radius_m=1.5,
            target_ghost_clear_enabled=True,target_bbox_filter_enabled=True,
            target_ghost_clear_radius_m=.35,target_ghost_clear_ttl_s=.5)
        self.p = {k:self.io.declare_parameter(k,v).value for k,v in defaults.items()}
        if (self.p['output_mode'] not in ('direct','mux') or
            not 0 < self.p['max_sensor_age_s'] <= .30 or
            not .3 <= self.p['target_prediction_s'] <= 3. or
            not 2. <= self.p['target_history_window_s'] <= 20. or
            not .2 <= self.p['target_history_sample_period_s'] <= 1. or
            self.p['target_history_window_s']/self.p['target_history_sample_period_s'] > 64 or
            not .25 <= self.p['search_arrival_m'] <= .6 or
            not 0 < self.p['result_ttl_s'] <= .20 or
            not 0 <= self.p['startup_clear_radius_m'] <= .5 or
            not 0 <= self.p['local_assumed_free_radius_m'] <= .5 or
            not .4 <= self.p['reid_similarity'] <= .99 or
            not .03 <= self.p['reid_margin'] <= .3 or
            not 2 <= self.p['reid_confirm_frames'] <= 5 or
            (self.p['reid_enabled'] and not self.p['reid_model_id']) or
            not .001 <= self.p['startup_motion_threshold_m'] <= .1 or
            not 1 <= self.p['depth_stride'] <= 16 or
            not .1 <= self.p['depth_min_m'] < self.p['depth_max_m'] <= 7 or
            not 0 <= self.p['surface_to_center_offset_m'] <= .5 or
            not .25 <= self.p['camera_min_valid_fraction'] <= 1 or
            not .35 <= self.p['camera_front_min_valid_fraction'] <= 1 or
            not 0 < self.p['binding_jump_radius_m'] <= self.p['binding_jump_radius_max_m'] <= 2 or
            not 0 <= self.p['binding_jump_growth_mps'] <= 1 or
            not .5 <= self.p['neighbor_ambiguity_radius_m'] <= 3 or
            not 0 < self.p['target_ghost_clear_radius_m'] <= .35 or
            not 0 < self.p['target_ghost_clear_ttl_s'] <= 1.):
            raise ValueError('invalid_sim_adapter_configuration')
        validate_detection_status_topic(self.p['detection_status_topic'])
        self.follow_preset=validate_follow_config(self.p)
        self.tracker=None;self.recovery_identity=None;self.tracker_key=None
        self.interest_at=-math.inf
        self.guard = threading.RLock()
        self.core = TaskSession(base_frame=self.p['base_frame'],max_age=self.p['max_sensor_age_s'])
        self.motion = LocalMotion()
        self.wheel_received = -math.inf
        self.wheel_diagnostic = None
        self.computation_diagnostic = None
        self.status_sequence = 0
        self.infos = deque(maxlen=24)
        self.pending = None
        self.depth_pairer = MaskDepthPairer()
        self.last_depth_stamp = 0
        self.tof = {}
        self.event = threading.Event()
        self.done = False
        self.result = None
        self.map_result = None
        self.reid_frames=deque(maxlen=16)
        self.reason = 'waiting_for_sensor_data'
        self.follow_session = uuid.uuid4().hex
        self.estop = False
        self.conflict_latched = False
        self.control_state = {}
        self.control_until = -math.inf
        self.retired_control_sessions = set()
        self.last_permission = False
        self.arrival_count = 0
        self.arrival_stamp = 0
        self.arrival_started = None
        self.arrival_detail = None
        self.arrival_goal_generation = None
        self.started_at = time.monotonic()
        self.tf = self.io.transform_buffer()
        prefix=self.p['prefix']
        self.preview_pub=self.io.create_publisher(Twist,prefix+'/cmd_vel_candidate',1)
        self.status_pub=self.io.create_publisher(String,prefix+'/status',5)
        self.path_pub=self.io.create_publisher(Path,prefix+'/local_path',1)
        self.grid_pub=self.io.create_publisher(OccupancyGrid,prefix+'/local_obstacles',1)
        self.reid_request_pub=self.io.create_publisher(String,prefix+'/reid_request',1) if self.p['reid_enabled'] or self.p['target_mask_enabled'] else None
        self.obstacles_pub=self.io.create_publisher(PointCloud2,prefix+'/depth_obstacles',qos_profile_sensor_data)
        self.direct_pub=None
        self.leased_pub=None
        if self.p['output_mode']=='mux':
            self.leased_pub=self.io.create_publisher(String,prefix+'/command_candidate',1)
        elif self.p['enable_motion']:
            # A disabled preview never creates a publisher on the hardware topic.
            self.direct_pub=self.io.create_publisher(Twist,self.p['direct_topic'],1)
        self.io.create_subscription(String,self.p['relative_topic'],self.on_relative,1)
        self.io.create_subscription(String,self.p['detection_status_topic'],self.on_status,1)
        if self.p['reid_enabled']:
            self.io.create_subscription(String,prefix+'/reid_features',self.on_reid,1)
        if self.p['target_mask_enabled']:
            self.io.create_subscription(String,'/r680/perception/target_mask',self.on_target_mask,1)
        # Depth arrives serialized; only the ~10 Hz frames compute() takes are deserialized.
        self.io.create_subscription(Image,self.p['depth_topic'],self.on_depth,qos_profile_sensor_data,raw=True)
        self.io.create_subscription(CameraInfo,self.p['camera_info_topic'],self.on_info,qos_profile_sensor_data)
        self.io.create_subscription(Odometry,self.p['wheel_topic'],self.on_wheel,qos_profile_sensor_data)
        self.io.create_subscription(String,'/r680_base/control_state',self.on_control,5)
        if self.p['tof_enabled']:
            for topic in self.p['tof_topics']:
                self.io.create_subscription(PointCloud2,topic,lambda m,k=topic:self.on_tof(k,m),qos_profile_sensor_data)
        self.io.create_service(PersonCommand,prefix+'/command',self.on_command)
        self.io.create_service(SetBool,prefix+'/set_estop',self.on_estop)
        self.pool=self.io.compute_pool()
        self.model_tuning = self.pool.submit(warm_worker).result(timeout=20.)
        self.future=None
        # A completed solution wakes the ROS executor immediately. The timer
        # still rechecks expiry/stop/lease when no new solution arrives.
        self.result_ready=self.io.create_guard_condition(self.tick)
        self.last_geometry=None
        self.worker=threading.Thread(target=self.compute,name='sim-follow-compute',daemon=True)
        self.worker.start()
        self.control_timer=self.io.create_timer(.05,self.tick)
        self.io.get_logger().info('unified tracker/MPC backend; output='+self.p['output_mode']+
            '; enable_motion='+str(self.p['enable_motion'])+'; waiting for explicit target request')

    def now_ns(self):
        return self.io.get_clock().now().nanoseconds

    def key(self):
        c=self.core
        return (c.request_id,c.mode,c.target_id,c.session,c.epoch,
                self.tracker.revision if self.tracker is not None else 0,
                c.active,self.motion.generation)

    def on_info(self,msg):
        with self.guard:self.infos.append(msg)

    def on_relative(self,msg):
        with self.guard:
            accepted=self.core.receive(msg.data,time.monotonic(),self.now_ns())
            if not accepted or not self.core.active:self.result=None
            self._tracking_update_locked()

    def on_reid(self,msg):
        try:
            if len(msg.data)>262144:return
            data=json.loads(msg.data);ns=data['stamp_ns']
            if (data.get('schema')!='wla-reid-features/v1' or data.get('base_frame')!=self.p['base_frame']
                    or data.get('model_id')!=self.p['reid_model_id'] or type(ns) is not int
                    or type(data.get('target_revision')) is not int
                    or not isinstance(data.get('diagnostics',{}),dict)
                    or not 0<=(self.now_ns()-ns)/1e9<=.6 or not isinstance(data.get('objects'),list)
                    or len(data['objects'])>4):return
            with self.guard:
                if (data['session'],data['epoch'])!=(self.core.session,self.core.epoch):return
                if not self.core.active or data.get('request_id')!=self.core.request_id:return
                if self.reid_frames and ns<=self.reid_frames[-1]['stamp_ns']:return
                self.reid_frames.append(data)
        except (ValueError,KeyError,TypeError):return

    def on_status(self,msg):
        with self.guard:
            self.core.receive_status(msg.data,time.monotonic(),self.now_ns())
            if not self.core.active:self.result=None

    def on_wheel(self,msg):
        with self.guard:
            previous=self.motion.generation
            diagnostic_accepted=False
            if msg.child_frame_id!=self.p['base_frame']:
                self.motion.samples.clear();self.motion.generation+=1
            else:
                v=msg.twist.twist
                if self.motion.update(stamp_ns(msg)/1e9,(v.linear.x,v.linear.y,v.angular.z),self.now_ns()/1e9):
                    self.wheel_received=time.monotonic()
                    diagnostic_accepted=True
            # Built only when status is published, not for every wheel sample.
            self.wheel_diagnostic=(msg,time.monotonic(),diagnostic_accepted,self.motion.generation)
            if previous!=self.motion.generation:self.result=None

    def on_depth(self,data):
        frame=as_frame(data);ns,_,height,width=frame_meta(frame);age=(self.now_ns()-ns)/1e9
        if ns<=0 or not -.05<=age<=self.p['max_sensor_age_s'] or not 0<width*height<=2_000_000:return
        with self.guard:
            if ns<=self.last_depth_stamp:return
            self.last_depth_stamp=ns
            self.depth_pairer.put_depth(ns,(frame,time.monotonic()+self.p['max_sensor_age_s']-max(0.,age)))
            self.event.set()

    def on_target_mask(self,msg):
        try:
            if len(msg.data)>360000:return
            data=json.loads(msg.data);ns=data['stamp_ns']
            if (data.get('schema')!='wla-target-mask/v1' or type(ns) is not int
                    or not 0<=(self.now_ns()-ns)/1e9<=.30):return
            with self.guard:
                if not self.core.active or data.get('request_id')!=self.core.request_id:return
                if (data.get('session'),data.get('epoch'))!=(self.core.session,self.core.epoch):return
                self.depth_pairer.put_mask(data);self.event.set()
        except (ValueError,KeyError,TypeError):return

    def on_tof(self,key,msg):
        if msg.width*msg.height>256 or not 0 <= (self.now_ns()-stamp_ns(msg))/1e9 <= .15:return
        with self.guard:
            old=self.tof.get(key)
            if old is None or stamp_ns(msg)>stamp_ns(old):self.tof[key]=msg

    def on_control(self,msg):
        if self.p['output_mode']!='mux':return
        try:
            data=json.loads(msg.data);ns=data['stamp_ns'];session=data['control_session']
            age=(self.now_ns()-ns)/1e9
            if (type(ns) is not int or not -.05<=age<=.50 or not isinstance(session,str) or
                not session or session in self.retired_control_sessions or
                data.get('mode') not in ('AUTO','MANUAL') or
                data.get('automatic_source') not in ('NAV','PERSON','OBJECT') or
                any(type(data.get(k)) is not int or data[k]<0 for k in ('source_generation','person_revoke_generation'))):return
            with self.guard:
                old=self.control_state
                if old and old['control_session']==session:
                    if ns<=old['stamp_ns'] or any(data[k]<old[k] for k in ('source_generation','person_revoke_generation')):return
                elif old:
                    self.retired_control_sessions.add(old['control_session'])
                revoked=bool(old) and (old['control_session']!=session or
                    data['person_revoke_generation']!=old['person_revoke_generation'] or
                    (not data.get('velocity_passthrough',False) and old['automatic_source']=='PERSON' and
                     (data['automatic_source']!='PERSON' or (old['mode']=='AUTO' and data['mode']!='AUTO'))))
                self.control_state=data;self.control_until=time.monotonic()+.50-max(age,0.)
                if revoked and self.core.active:
                    self.core._terminate('cancelled','motion_authority_revoked');self.result=None
        except (ValueError,TypeError,KeyError):return

    def on_command(self,req,res):
        with self.guard:
            if req.command=='start' and (self.estop or self.conflict_latched):
                accepted,reason=False,'estop_or_direct_publisher_conflict'
            else:
                old_key=self.key()
                accepted,reason=self.core.command(req.command,req.request_id,req.mode,req.target_id,req.lease_duration_s,time.monotonic())
                if old_key!=self.key() or not self.core.active:self.result=None
            self._tracking_update_locked()
            state=self.core.snapshot(time.monotonic())
            res.accepted,res.reason=accepted,reason
            for name in ('request_id','state','mode','target_id','enabled'):setattr(res,name,state[name])
            terminal=(req.request_id==state['request_id'] and state['state'] in ('arrived','stopped','cancelled','expired'))
            if not accepted and req.command!='status' and not terminal:
                res.request_id=req.request_id;res.mode=req.mode;res.target_id=req.target_id
                res.state='rejected';res.enabled=False
            elif accepted and req.command=='stop' and req.request_id and req.request_id!=state['request_id']:
                res.request_id=req.request_id;res.state='stopped';res.enabled=False;res.mode='';res.target_id=''
            return res

    def on_estop(self,req,res):
        with self.guard:
            self.estop=bool(req.data);self.result=None
            self.core._terminate('stopped','local_estop' if self.estop else 'estop_reset_requires_new_request')
            res.success=True;res.message='latched' if self.estop else 'reset; new request required'
            return res

    def _feature_rows(self,batch,pose):
        rows=[];prefix=f"{batch['session']}:{batch['epoch']}:"
        for row in batch['objects']:
            try:
                if not row['id'].startswith(prefix) or not row['id'][len(prefix):].isdigit():continue
                p=position(dict(row,detected=True,label='person'),pose,self.p['surface_to_center_offset_m'])
                if p is not None:rows.append(dict(id=row['id'],position=p,feature=row['feature']))
            except (ValueError,TypeError,KeyError,AttributeError):continue
        return rows

    @staticmethod
    def _tracking_policy(snapshot):
        if snapshot is None:return None
        return TrackingPolicy(snapshot['state'],snapshot['last_seen'],snapshot['prediction_s'],
                              snapshot['resolved_id'],snapshot['revision'],snapshot['search_heading'],
                              snapshot['search_arrival_m'],snapshot['search_started'],snapshot.get('mode','0'))

    def _tracking_update_locked(self):
        now=self.now_ns()/1e9
        c=self.core
        c._expire(time.monotonic())
        task_key=(c.request_id,c.mode,c.target_id,c.session,c.epoch,self.motion.generation)
        if not c.active:
            self.tracker=None;self.recovery_identity=None;self.tracker_key=None
        elif task_key!=self.tracker_key:
            cfg=mode_config(c.mode)
            self.tracker=FollowTracker(c.target_id,now,prediction_s=self.p['target_prediction_s'],
                freshness_s=self.p['max_sensor_age_s'],offset=0. if c.mode == MODE_OBJECT_APPROACH else self.p['surface_to_center_offset_m'],
                jump_radius=self.p['binding_jump_radius_m'],jump_growth=self.p['binding_jump_growth_mps'],
                jump_max=self.p['binding_jump_radius_max_m'],confirmations=self.p['reid_confirm_frames'],
                search_arrival=self.p['search_arrival_m'],
                history_window_s=self.p['target_history_window_s'],
                history_sample_period_s=self.p['target_history_sample_period_s'],
                expected_label=c.target_label, mode=c.mode)
            self.recovery_identity=RecoveryIdentity(similarity=self.p['reid_similarity'],margin=self.p['reid_margin'],
                confirmations=self.p['reid_confirm_frames']) if self.p['reid_enabled'] and cfg['reid'] else None
            self.tracker_key=task_key;self.interest_at=-math.inf;self.result=None
            self.arrival_count=0;self.arrival_stamp=0;self.arrival_started=None
            self.arrival_detail=None;self.arrival_goal_generation=None
        allowed=[]
        if c.active:
            stamp=c.last_stamp/1e9;pose=self.motion.pose_at(stamp)
            before=self.tracker.revision
            self.tracker.observe(c.objects,pose,stamp,now)
            snap=self.tracker.snapshot(now,self.motion.pose_at(now))
            cfg=mode_config(c.mode)
            points={ident:position(row,pose,self.tracker.offset,c.target_label) for ident,row in c.objects.items()}
            target=points.get(self.tracker.target_id)
            neighbor=target is not None and any(p is not None and ident!=self.tracker.target_id and np.linalg.norm(p-target)<self.p['neighbor_ambiguity_radius_m'] for ident,p in points.items())
            if self.recovery_identity is not None:
                sidecar=self.recovery_identity
                for batch in self.reid_frames:
                    ts=batch['stamp_ns']/1e9
                    if ts<=sidecar.last_frame_stamp or not 0<=now-ts<=.6:continue
                    if (batch.get('request_id'),batch.get('session'),batch.get('epoch')) != (task_key[0],task_key[3],task_key[4]):continue
                    if batch.get('target_revision')!=self.tracker.revision:continue
                    bp=self.motion.pose_at(ts)
                    if bp is None:continue
                    sidecar.observe(self._feature_rows(batch,bp),ts,batch.get('diagnostics'),self.tracker,snap,neighbor=neighbor)
                if snap['recovery_needed'] and self.tracker.rebind(sidecar.proposal,c.objects,pose,stamp,now):
                    sidecar.reacquired+=1;sidecar.reset_confirmation();sidecar.state='ready';sidecar.reason='reid_reacquired'
                    snap=self.tracker.snapshot(now,self.motion.pose_at(now))
                if snap['state']=='tracking':
                    allowed=[self.tracker.target_id]
                elif snap['prediction'] is not None and 0<=now-stamp<=self.p['max_sensor_age_s']:
                    pred=np.asarray(snap['prediction'])
                    allowed=[ident for ident,p in points.items() if p is not None and np.linalg.norm(p-pred)<=self.tracker.radius(now)]
                    if len(allowed)>4:allowed=[]
            if self.tracker.revision!=before:
                self.result=None;self.interest_at=-math.inf
        if self.reid_request_pub is not None and time.monotonic()-self.interest_at>=.1:
            self.reid_request_pub.publish(String(data=json.dumps(dict(
                schema='wla-reid-request/v1',stamp_ns=self.now_ns(),enabled=c.active,
                request_id=c.request_id,session=c.session,epoch=c.epoch,allowed_ids=allowed,
                target_id=self.tracker.target_id if self.tracker is not None else '',
                target_label=c.target_label if c.active else 'person',
                target_revision=self.tracker.revision if self.tracker is not None else 0,
                urgent=c.active))))
            self.interest_at=time.monotonic()

    def compute(self):
        next_at=0.;last_tof={}
        while not self.done:
            self.event.wait(.02)
            if self.done:return
            remaining=next_at-time.monotonic()
            if remaining>0:
                time.sleep(min(remaining,.1));continue
            with self.guard:
                self.event.clear();paired=self.depth_pairer.take(self.now_ns(),self.core.active and self.p['target_mask_enabled'],
                    require_mask=self.core.active)
                item,mask_packet=paired if paired is not None else (None,None)
                infos=tuple(self.infos);motion=deepcopy(self.motion);key=self.key()
                transport_generation=getattr(self.io,"generation",0)
                active=self.core.active;tof_messages=tuple(self.tof.values())
            if item is None:continue
            next_at=time.monotonic()+.1
            frame,valid_until=item;msg=frame_message(frame);stamp=stamp_ns(msg)/1e9
            processing_started=time.monotonic()
            depth_start_age_ms=(self.now_ns()-stamp_ns(msg))/1e6
            identity_status=None
            try:
                if time.monotonic()>=valid_until:raise ValueError('depth_expired_before_processing')
                pose=motion.pose_at(stamp)
                if pose is None or not motion.healthy(self.now_ns()/1e9):raise ValueError('wheel_pose_unavailable')
                info=min(infos,key=lambda m:abs(stamp_ns(m)/1e9-stamp),default=None)
                if (info is None or abs(stamp_ns(info)/1e9-stamp)>.25 or
                    info.header.frame_id!=msg.header.frame_id or (info.width,info.height)!=(msg.width,msg.height)):
                    raise ValueError('depth_calibration_unavailable')
                tr=self.tf.lookup_transform(self.p['base_frame'],msg.header.frame_id,Time(nanoseconds=stamp_ns(msg))).transform
                q=tr.rotation;r=rotation_from_quaternion((q.x,q.y,q.z,q.w))
                trans=np.array([tr.translation.x,tr.translation.y,tr.translation.z])
                depth=depth_meters(msg)
                valid=np.isfinite(depth)&(depth>=self.p['depth_min_m'])&(depth<=self.p['depth_max_m'])
                h,w=valid.shape;front=valid[int(h*.35):max(int(h*.8),int(h*.35)+1),int(w*.3):max(int(w*.7),int(w*.3)+1)]
                if np.mean(valid)<self.p['camera_min_valid_fraction'] or not front.size or np.mean(front)<self.p['camera_front_min_valid_fraction']:
                    raise ValueError('insufficient_depth_coverage')
                # Transform measurements at acquisition time, but control from
                # the latest wheel pose after depth processing, not image pose.
                with self.guard:
                    if key!=self.key():raise ValueError('reference_changed_during_depth')
                    motion=deepcopy(self.motion)
                    objects=deepcopy(self.core.objects);obs_stamp=self.core.last_stamp/1e9
                control_stamp=self.now_ns()/1e9
                control_pose=motion.pose_at(control_stamp)
                if control_pose is None:raise ValueError('control_time_wheel_pose_unavailable')
                observation=None;op=motion.pose_at(obs_stamp)
                resolved_id=key[2];identity_status=dict(state='disabled')
                binding_status=None;supervisor_status=None
                with self.guard:
                    self._tracking_update_locked()
                    if key!=self.key():raise ValueError('target_revision_changed_during_depth')
                    binding_status=self.tracker.snapshot(control_stamp,control_pose) if active and self.tracker is not None else None
                    mask_geometry=self.tracker.observed_position(stamp) if active and self.tracker is not None else None
                    supervisor_status=self.recovery_identity.snapshot() if active and self.recovery_identity is not None else dict(state='disabled')
                identity_status=supervisor_status
                resolved_id=binding_status['resolved_id'] if binding_status else None
                resolved_source=dict(source='FollowTracker',
                    active=active,accepted_id=resolved_id if active else None,
                    observation_pose_available=op is not None,selection=deepcopy(binding_status))
                if active and op is not None and 0<=control_stamp-obs_stamp<=self.p['max_sensor_age_s']:
                    cfg=mode_config(key[1])
                    target,people=measurements_from_relative(objects,op,resolved_id or '',binding_status['surface_offset_m'])
                    if binding_status['state'] not in ('tracking','coasting'):
                        target=None
                    observation=(obs_stamp,target,people,resolved_id)
                mask_started=time.monotonic()
                original_depth=depth
                mode=key[1] if active else MODE_PERSON_FOLLOW
                filter_target=active and mask_geometry is not None
                depth,mask_status,confirmed_target_mask=filter_target_depth(depth,mask_packet,
                    target_id=resolved_id if filter_target else None,
                    request_id=key[0],session=key[3],epoch=key[4],
                    depth_stamp_ns=stamp_ns(msg),frame_id=msg.header.frame_id,now_ns=self.now_ns(),
                    mode='bbox' if mode != MODE_OBJECT_APPROACH and self.p['target_bbox_filter_enabled'] else 'segmentation',return_confirmed=True)
                mask_status['bbox_filter_enabled']=self.p['target_bbox_filter_enabled']
                mask_status['compute_ms']=(time.monotonic()-mask_started)*1000.
                mask_status['decision_basis']='follow_tracker'
                mask_status['geometry_synchronized']=mask_geometry is not None
                if active:
                    ready=frame_target_ready(mask_packet,target_id=resolved_id,revision=binding_status['revision'],
                        request_id=key[0],session=key[3],epoch=key[4],stamp_ns=stamp_ns(msg),
                        frame_id=msg.header.frame_id,shape=depth.shape,now_ns=self.now_ns(),mask_applied=mask_status['applied'])
                    mask_status['frame_ready']=ready
                    mask_status['target_present']=None if mask_packet is None else mask_packet.get('target_present')
                    if not ready:
                        with self.guard:
                            if key==self.key():
                                self.result=None;self.reason='waiting_target_pointcloud'
                                self.computation_diagnostic=dict(key=key,control_stamp_s=control_stamp,
                                    target_mask=deepcopy(mask_status),geometry_binding=deepcopy(binding_status),
                                    identity_supervision=deepcopy(identity_status),compute_error=None)
                        continue
                # Object segmentation extracts its observed surface only; every
                # original depth pixel remains in obstacle/clearing processing.
                object_surface=None
                if mode == MODE_OBJECT_APPROACH:
                    depth=original_depth
                    mask_status['surface_extracted']=mask_status['applied']
                    mask_status['applied']=False
                    mask_status['removed_pixels']=0
                    mask_status['reason']='object_cloud_preserved'
                frontend_begun=time.monotonic()
                packet=packet_from_depth(depth,(info.k[0],info.k[4],info.k[2],info.k[5]),r,trans,pose,
                    self.p['depth_min_m'],self.p['depth_max_m'],self.p['depth_stride'])
                # The full box is omitted from live cloud, but only the
                # independently confirmed silhouette may retire old map hits.
                target_points=np.empty((0,3),dtype=float)
                ghost_reason=('object_cloud_preserved' if mode == MODE_OBJECT_APPROACH else
                    'disabled' if not self.p['target_ghost_clear_enabled'] else
                    'no_mask_confirmed_target_pixels' if not np.any(confirmed_target_mask) else
                    'target_geometry_not_synchronized' if mask_geometry is None
                    else 'mask_confirmed')
                if ghost_reason=='mask_confirmed':
                    removed_depth=np.where(confirmed_target_mask,original_depth,np.nan)
                    target_points=target_points_from_removed_depth(removed_depth,
                        (info.k[0],info.k[4],info.k[2],info.k[5]),r,trans,pose,
                        mask_geometry,radius=self.p['target_ghost_clear_radius_m'],stride=self.p['depth_stride'],
                        minimum=self.p['depth_min_m'],maximum=self.p['depth_max_m'])
                if mode == MODE_OBJECT_APPROACH and np.any(confirmed_target_mask):
                    from _wla_sim_geometry import deproject
                    body_surface=deproject(np.where(confirmed_target_mask,original_depth,np.nan),
                        np.array([info.k[0],info.k[4],info.k[2],info.k[5]]),r,trans,
                        self.p['depth_min_m'],self.p['depth_max_m'],self.p['depth_stride'])
                    body_surface=body_surface[(body_surface[:,2]>.04)&(body_surface[:,2]<2.)]
                    surface=local_points(body_surface,pose)
                    if len(surface)>=3:
                        object_surface=surface[:,:2]
                        # Selected object points remain occupied even if cluster
                        # filtering would discard a small item in this frame.
                        packet['marking_points_odom']=np.vstack((packet['marking_points_odom'],surface))
                        packet['obstacle_points']=np.vstack((packet['obstacle_points'],body_surface))
                packet['target_points_odom']=target_points
                mask_status['target_ghost_points']=len(target_points)
                mask_status['target_ghost_reason']=ghost_reason
                mask_status['target_ghost_radius_m']=self.p['target_ghost_clear_radius_m']
                mask_status['target_ghost_clear_enabled']=self.p['target_ghost_clear_enabled']
                mask_status['target_ghost_clear_ttl_s']=self.p['target_ghost_clear_ttl_s']
                frontend_ms=(time.monotonic()-frontend_begun)*1000.
                packets=[(stamp,packet)]
                # Optional anonymous ToF points: no packets or inferred clear
                # rays exist while disabled. Only actual newer frames are used.
                if self.p['tof_enabled']:
                    from sensor_msgs_py.point_cloud2 import read_points_numpy
                    for tfmsg in tof_messages:
                        ts=stamp_ns(tfmsg)/1e9
                        if not last_tof.get(tfmsg.header.frame_id,-math.inf)<ts<=stamp or stamp-ts>.15:continue
                        tp=motion.pose_at(ts)
                        if tp is None:continue
                        tr2=self.tf.lookup_transform(self.p['base_frame'],tfmsg.header.frame_id,Time(nanoseconds=stamp_ns(tfmsg))).transform
                        tq=tr2.rotation;rr=rotation_from_quaternion((tq.x,tq.y,tq.z,tq.w))
                        tt=np.array([tr2.translation.x,tr2.translation.y,tr2.translation.z])
                        pts=read_points_numpy(tfmsg,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
                        pts=pts[np.isfinite(pts).all(axis=1)]@rr.T+tt
                        last_tof[tfmsg.header.frame_id]=ts
                        packets.append((ts,dict(points=pts,points_odom=local_points(pts,tp),origin=tt,
                            origin_odom=local_points(tt[None,:],tp)[0],dynamic=np.zeros(len(pts),dtype=bool))))
                v=motion.samples[-1][2]
                begun=time.monotonic()
                if self.future is not None and not self.future.done():
                    raise ValueError('compute_worker_busy_no_queue_growth')
                self.future=self.pool.submit(process_step,(key,np.asarray(control_pose),np.array([v[0],v[2]]),packets,observation,control_stamp,
                    dict(active=active,radius=self.p['startup_clear_radius_m'],
                         motion_threshold=self.p['startup_motion_threshold_m'],
                         local_radius=self.p['local_assumed_free_radius_m'],
                         target_clear_ttl_s=self.p['target_ghost_clear_ttl_s'],
                         tracker_snapshot=binding_status,
                         task_mode=mode,object_surface=object_surface,
                         policy_snapshot=self._tracking_policy(binding_status) if active else None)))
                try:
                    result=self.future.result(timeout=self.p['result_ttl_s'])
                except ComputeTimeout:
                    raise ValueError('compute_deadline_exceeded')
                result.update(target_observation=observed_target(objects,key[2],obs_stamp,control_stamp),
                    resolved_target_observation=observed_target(objects,resolved_id,obs_stamp,control_stamp),
                    resolved_target_source=resolved_source,
                    target_mask=mask_status,reid=identity_status,geometry_binding=binding_status,
                    identity_supervision=supervisor_status,resolved_target_id=resolved_id if active else None,
                    transport_generation=transport_generation,key=key,stamp_ns=stamp_ns(msg),valid_until=min(valid_until,begun+self.p['result_ttl_s']),
                    frontend_ms=frontend_ms,obstacle_points=len(packet['obstacle_points']),clearing_points=len(packet['points']),
                    observation_stamp_ns=int(obs_stamp*1e9),ready_at=time.monotonic(),
                    latency=dict(depth_start_age_ms=depth_start_age_ms,frontend_ms=frontend_ms,
                        depth_to_result_ms=(self.now_ns()-stamp_ns(msg))/1e6,
                        observation_to_result_ms=(self.now_ns()/1e9-obs_stamp)*1000.,
                        processing_ms=(time.monotonic()-processing_started)*1000.),
                    compute_ms=(time.monotonic()-begun)*1000.,pose=np.asarray(control_pose),control_stamp_ns=int(control_stamp*1e9))
                with self.guard:
                    self.computation_diagnostic=dict(key=key,control_stamp_s=control_stamp,
                        depth_stamp_ns=stamp_ns(msg),target_mask=deepcopy(mask_status),
                        target_observation=result['target_observation'],
                        resolved_target_observation=result['resolved_target_observation'],
                        resolved_target_source=resolved_source,reid=deepcopy(identity_status),
                        geometry_binding=deepcopy(binding_status),identity_supervision=deepcopy(supervisor_status),
                        front_blockage=result.get('front_blockage'),planner_target=result.get('planner_target'),
                        planner_activity=result.get('planner_activity'),
                        obstacle_track_ids=result.get('obstacle_track_ids'),compute_error=None)
                    if key==self.key() and time.monotonic()<result['valid_until']:
                        self.result=result;self.map_result=result;self.reason=result['reason'] or 'simulation_core'
                        self.result_ready.trigger()
                    else:
                        self.result=None
                        self.reason=('expired_or_superseded_computation:'+str(round(result['compute_ms'],1))+'ms;'+
                            'source_age='+str(round((self.now_ns()-result['stamp_ns'])/1e9,3))+';key_changed='+str(key!=self.key()))
                # Publish the same points handed to the map, once per accepted
                # frame. Original acquisition stamp is retained (never renewed).
                if time.monotonic()<result['valid_until'] and key==self.key():
                    header=Header(stamp=msg.header.stamp,frame_id=self.p['base_frame'])
                    self.obstacles_pub.publish(create_cloud_xyz32(header,packet['obstacle_points'].astype(np.float32)))
            except Exception as exc:
                with self.guard:
                    self.result=None;self.reason=str(exc)[:180]
                    self.computation_diagnostic=dict(key=key,control_stamp_s=self.now_ns()/1e9,
                        compute_error=str(exc)[:180],front_blockage=None,target_mask=None,
                        reid=deepcopy(identity_status) if 'identity_status' in locals() else None,
                        geometry_binding=deepcopy(binding_status) if 'binding_status' in locals() else None,
                        identity_supervision=deepcopy(supervisor_status) if 'supervisor_status' in locals() else None,
                        details_unavailable=True)

    def _target_freshness_for_output(self,result,state,ns):
        # Empty fresh frames permit recovery; a stalled detector does not.
        fresh=bool(state['observations_fresh'])
        age=None if self.tracker is None or self.tracker.last_seen is None else ns/1e9-self.tracker.last_seen
        return fresh, 'source_observations_fresh' if fresh else 'target_observations_stale', age

    def _terminal_arrival_locked(self, result, healthy, now, ns):
        """Finite tasks latch only after fresh, observed, stationary confirmations."""
        mode=self.core.mode
        if not healthy or mode not in ('1','2') or self.tracker is None:
            self.arrival_count=0;self.arrival_started=None
            return False
        row=self.core.objects.get(self.tracker.target_id)
        live=self.tracker.snapshot(ns/1e9,self.motion.pose_at(ns/1e9))
        if (live['state']!='tracking' or row is None or not row.get('detected') or
                row.get('distance') is None or row.get('yaw') is None):
            self.arrival_count=0;self.arrival_started=None
            return False
        stamp=result['stamp_ns']
        if stamp==self.arrival_stamp:return False
        self.arrival_stamp=stamp
        v=self.motion.samples[-1][2]
        stationary=abs(v[0])<.04 and abs(v[2])<.08
        goal=result.get('approach_goal')
        if mode=='1':
            close=.65<=row['distance']<=.99 and abs(row['yaw'])<=.16
            detail=dict(reason='person_surface_distance_reached',surface_distance_m=row['distance'],requested_distance_m=.95)
        else:
            generation=None if goal is None else goal['generation']
            if generation!=self.arrival_goal_generation:
                self.arrival_count=0;self.arrival_started=None
                self.arrival_goal_generation=generation
            close=(goal is not None and goal['stable_s']>=.6 and goal['endpoint_distance_m']<=.065 and
                goal['current_clearance_m']>=.15 and abs(row['yaw'])<=.22)
            detail=goal
        confirmed=close and stationary and result.get('planner_feasible',False)
        if not confirmed:
            self.arrival_count=0;self.arrival_started=None
            return False
        if self.arrival_started is None:self.arrival_started=now
        self.arrival_count+=1
        if self.arrival_count<3 or now-self.arrival_started<.2:return False
        self.arrival_detail=detail
        self.core._terminate('arrived',detail['reason'])
        self.result=None
        return True

    def _calibration_diagnostic(self,result):
        return dict(status='pending_calibration',
            reason='tracker_defaults_require_sensor_replay_and_physical_validation',
            parameters={name:self.p[name] for name in PENDING_CALIBRATION_PARAMETERS})

    def check_direct_owner(self):
        if self.direct_pub is None:return False
        endpoints=self.io.get_publishers_info_by_topic(self.p['direct_topic'])
        # Count all endpoints, including another node with the same name.
        if len(endpoints)>1:
            self.conflict_latched=True
            self.core._terminate('cancelled','direct_publisher_conflict')
            self.result=None
        return not self.conflict_latched and len(endpoints)==1 and time.monotonic()-self.started_at>=1.

    def tick(self):
        with self.guard:
            now=time.monotonic();ns=self.now_ns()
            self._tracking_update_locked()
            state=self.core.snapshot(now)
            result=self.result
            target_fresh,target_freshness_reason,target_observation_age=self._target_freshness_for_output(result,state,ns)
            healthy=(result is not None and result['key']==self.key() and now<result['valid_until'] and
                0<=(ns-result['stamp_ns'])/1e9<=self.p['max_sensor_age_s'] and
                self.motion.healthy(ns/1e9) and now-self.wheel_received<=.20 and
                target_fresh and self.core.gate_enabled and state['enabled'] and not self.estop)
            # Report the exact existing output predicates; do not change their
            # thresholds or renew a command merely because it took long to compute.
            blockers=[]
            if not state['enabled']:blockers.append('task_inactive')
            if self.estop:blockers.append('local_estop')
            if result is None:
                blockers.append('no_fresh_planner_result')
            else:
                if result['key']!=self.key():blockers.append('planner_reference_changed')
                if now>=result['valid_until']:blockers.append('planner_result_expired')
                if not 0<=(ns-result['stamp_ns'])/1e9<=self.p['max_sensor_age_s']:blockers.append('depth_stale')
            if not self.motion.healthy(ns/1e9):blockers.append('wheel_motion_invalid')
            if now-self.wheel_received>.20:blockers.append('wheel_odom_stale')
            if not target_fresh:blockers.append(target_freshness_reason)
            if not self.core.gate_enabled:blockers.append('detector_gate_closed')
            command=result['command'] if healthy else np.zeros(2)
            policy=result.get('policy_snapshot') if result is not None else None
            if policy is not None:
                command=policy.limit_command(command,ns/1e9)
                live=self.tracker.snapshot(ns/1e9,self.motion.pose_at(ns/1e9)) if self.tracker is not None else None
                same_phase=(live is not None and (live['state']==policy.state or
                    {live['state'],policy.state} <= {'tracking','coasting'}))
                if not same_phase:
                    command=np.zeros(2);healthy=False;blockers.append('tracking_phase_changed')
                elif live is not None:
                    command=self._tracking_policy(live).limit_command(command,ns/1e9)
                    if live['state'] in ('binding','holding'):
                        healthy=False;blockers.append(live['reason'])
            if healthy and 'first_output_ms' not in result.get('latency',{}):
                result.setdefault('latency',{}).update(
                    first_output_ms=(now-result.get('ready_at',now))*1000.,
                    depth_to_candidate_ms=(ns-result['stamp_ns'])/1e6,
                    observation_to_candidate_ms=(ns-result.get('observation_stamp_ns',ns))/1e6)
            previously_permitted=self.last_permission
            owner=self.check_direct_owner() if self.direct_pub is not None else False
            if self.conflict_latched and previously_permitted and self.direct_pub is not None:
                self.direct_pub.publish(Twist())  # one bounded stop on loss of exclusive ownership
            permitted=bool(self.p['enable_motion'] and not self.estop and
                (owner if self.p['output_mode']=='direct' else
                 now<self.control_until and -.05<=(ns-self.control_state.get('stamp_ns',0))/1e9<=.50 and
                 not self.control_state.get('estop_latched',False) and
                 (self.control_state.get('velocity_passthrough') is True or
                  (self.control_state.get('mode')=='AUTO' and self.control_state.get('automatic_source')=='PERSON'))))
            if self.last_permission and not permitted and self.core.active:
                self.core._terminate('cancelled','motion_authority_lost');command=np.zeros(2);healthy=False
            self.last_permission=permitted
            if self.conflict_latched:command=np.zeros(2);healthy=False
            if not permitted:blockers.append('motion_permission_unavailable')
            if self.conflict_latched:blockers.insert(0,'direct_publisher_conflict')
            if self._terminal_arrival_locked(result,healthy,now,ns):
                command=np.zeros(2);healthy=False;blockers.append('task_arrived')
            if self.core.active:
                self.core.state=('searching' if result.get('geometry_binding',{}).get('state','').startswith('search_') else 'following' if self.core.mode=='0' else 'approaching') if healthy else 'waiting'
                self.core.reason=self.reason if healthy else ('direct_publisher_conflict' if self.conflict_latched else 'waiting_for_fresh_inputs')
            state=self.core.snapshot(now)
            preview=Twist();preview.linear.x=float(command[0]);preview.angular.z=float(command[1])
            self.preview_pub.publish(preview)
            actual=command if permitted and healthy else np.zeros(2)
            if self.direct_pub is not None and not self.conflict_latched and owner:
                out=Twist();out.linear.x=float(actual[0]);out.angular.z=float(actual[1]);self.direct_pub.publish(out)
            if self.leased_pub is not None:
                self.leased_pub.publish(String(data=json.dumps(dict(schema='wla-person-candidate/v1',
                    follow_session=self.follow_session,request_id=state['request_id'],stamp_ns=ns,
                    linear=float(actual[0]),angular=float(actual[1])))))
            state.update(schema='wla-person-status/v1',stamp_ns=ns,follow_session=self.follow_session,
                planner='tracker_sampling_mpc',numeric_backend='python_mpc_cpp_map' if native_ray_slots is not None else 'python_reference',native_frontend='cpp_deproject_ground_cluster_filter',
                obstacle_topic=self.p['prefix']+'/depth_obstacles',
                base_frame=self.p['base_frame'],output_mode=self.p['output_mode'],
                candidate_only=not self.p['enable_motion'],motion_permitted=permitted and healthy,
                linear_mps=float(command[0]),angular_rps=float(command[1]),safety_fresh=healthy,
                safety_reason=blockers[0] if blockers else self.reason,
                output_blockers=blockers,planner_reason=self.reason,
                target_freshness_reason=target_freshness_reason,
                target_observation_age_s=target_observation_age,
                planner_candidates=result.get('candidates',{}) if result else {},
                planner_activity=result.get('planner_activity','waiting_result') if result else 'waiting_result',
                result_remaining_ms=round((result['valid_until']-now)*1000.,1) if result else None,
                reference_generation=self.motion.generation,
                robot_radius_m=(result.get('tuning',self.model_tuning)['radius'] if result else self.model_tuning['radius']),obstacle_margin_m=self.model_tuning['margin'],
                tof_enabled=self.p['tof_enabled'],direct_publisher_conflict=self.conflict_latched,
                startup_clear_radius_m=self.p['startup_clear_radius_m'],
                startup_motion_threshold_m=self.p['startup_motion_threshold_m'],
                local_assumed_free_radius_m=self.p['local_assumed_free_radius_m'],
                local_free_prior=self.map_result.get('local_free_prior') if self.map_result else None,
                startup_prior=self.map_result.get('startup_prior') if self.map_result else None,
                startup_prior_consumed=self.map_result.get('startup_prior_consumed',False) if self.map_result else False,
                fault=self.conflict_latched or self.estop,
                target_tracking=result['target_state'] if result else 'unavailable',
                target_mask=self.map_result.get('target_mask') if self.map_result else None,
                reid_enabled=self.p['reid_enabled'],
                reid=(self.recovery_identity.snapshot() if self.recovery_identity is not None
                      else dict(state='disabled')),
                geometry_binding=(self.tracker.snapshot(ns/1e9,self.motion.pose_at(ns/1e9)) if self.tracker is not None else None),
                identity_supervision=self.recovery_identity.snapshot() if self.recovery_identity is not None else None,
                resolved_target_id=self.tracker.target_id if self.tracker is not None else None,
                resolved_target_source=self.map_result.get('resolved_target_source') if self.map_result else None,
                calibration=self._calibration_diagnostic(result),
                follow_preset=self.follow_preset,
                follow_config={name:self.p[name] for name in (
                    'target_prediction_s','search_arrival_m','binding_jump_radius_m',
                    'target_history_window_s','target_history_sample_period_s',
                    'binding_jump_growth_mps','binding_jump_radius_max_m','reid_enabled',
                    'reid_similarity','reid_margin','reid_confirm_frames','neighbor_ambiguity_radius_m',
                    'target_mask_enabled','target_bbox_filter_enabled','target_ghost_clear_enabled')},
                approach_goal=result.get('approach_goal') if result else None,
                arrival=self.arrival_detail,
                task_mode=self.core.mode,
                task_mode_label=mode_config(self.core.mode)['label'] if self.core.mode in ('0','1','2') else None,
                terminal_arrival=state['state'] == 'arrived',
                recovery_state=result['recovery_state'] if result else 'waiting',
                frontend_ms=self.map_result.get('frontend_ms') if self.map_result else None,
                filtered_obstacle_points=self.map_result.get('obstacle_points') if self.map_result else None,
                clearing_points=self.map_result.get('clearing_points') if self.map_result else None,
                latency=self.map_result.get('latency',{}) if self.map_result else {},
                compute_ms=result['compute_ms'] if result else None)
            # Diagnostics are attached AFTER the unchanged command decision/publication.
            self.status_sequence+=1
            current_pose=self.motion.pose_at(ns/1e9)
            diagnostic=self.computation_diagnostic
            same_reference=diagnostic is not None and diagnostic.get('key')==self.key()
            detail=deepcopy(diagnostic) if same_reference else None
            if detail is not None:
                detail.pop('key',None)
                detail['age_s']=ns/1e9-detail['control_stamp_s']
                detail['planner_result_fresh']=bool(result is not None and result['key']==self.key() and now<result['valid_until'])
            state.update(diagnostic_sequence=self.status_sequence,
                diagnostic_schema='wla-follow-diagnostics/v1',
                target_observation=observed_target(self.core.objects,self.core.target_id,self.core.last_stamp/1e9,ns/1e9),
                measured_motion=dict(wheel=wheel_message(*self.wheel_diagnostic,source=self.p['wheel_topic']) if self.wheel_diagnostic else None,
                    integrated_pose_reference_m_rad=None if current_pose is None else list(current_pose),
                    pose_stamp_ns=ns,reference_generation=self.motion.generation,
                    pose_source='LocalMotion wheel-twist integration; not global localization'),
                actual_published_command=dict(linear_mps=float(actual[0]),angular_rps=float(actual[1])),
                diagnostics=finite_json(detail))
            self.status_pub.publish(String(data=json.dumps(state,allow_nan=False)))
            mapped=self.map_result
            if (mapped is not None and mapped['key']==self.key() and now<mapped['valid_until'] and
                    0<=(ns-mapped['stamp_ns'])/1e9<=self.p['max_sensor_age_s']):
                geometry_key=(mapped['stamp_ns'],healthy)
                if geometry_key!=self.last_geometry:
                    self.publish_geometry(mapped,ns,show_path=healthy)
                    self.last_geometry=geometry_key

    def publish_geometry(self,result,ns,show_path=True):
        pose=result['pose'];rot=rotation(pose[2])
        path=Path();path.header.frame_id=self.p['base_frame']
        path.header.stamp.sec,path.header.stamp.nanosec=divmod(result.get('control_stamp_ns',result['stamp_ns']),1_000_000_000)
        for point in ((result['path']-pose[:2])@rot if show_path else []):
            item=PoseStamped();item.header=path.header
            item.pose.position.x=float(point[0]);item.pose.position.y=float(point[1]);item.pose.orientation.w=1.
            path.poses.append(item)
        self.path_pub.publish(path)
        grid=OccupancyGrid();grid.header=path.header
        grid.info.resolution=result['resolution'];grid.info.width,grid.info.height=result['grid'].shape
        origin=(result['map_lower']*result['resolution']-pose[:2])@rot
        grid.info.origin.position.x=float(origin[0]);grid.info.origin.position.y=float(origin[1])
        grid.info.origin.orientation.z=float(np.sin(-pose[2]/2));grid.info.origin.orientation.w=float(np.cos(-pose[2]/2))
        grid.data=result['grid'].T.ravel().tolist();self.grid_pub.publish(grid)

    def close(self):
        with self.guard:
            self.done=True;self.core._terminate('stopped','node_shutdown');self.result=None
            if self.io.context.ok() and self.direct_pub is not None and not self.conflict_latched:
                if len(self.io.get_publishers_info_by_topic(self.p['direct_topic']))==1:self.direct_pub.publish(Twist())
        self.event.set();self.worker.join(timeout=3.)
        self.pool.shutdown(wait=True,cancel_futures=True)


class SimulationFollow(FollowEngine, Node):
    """Compatibility ROS adapter, selected explicitly with transport:=python."""
    def __init__(self, **kwargs):
        Node.__init__(self, 'person_follow', **kwargs)
        FollowEngine.__init__(self, self)

    def transform_buffer(self):
        buffer = Buffer()
        self.listener = TransformListener(buffer, self)
        return buffer

    def compute_pool(self):
        return ProcessPoolExecutor(max_workers=1, mp_context=get_context('spawn'), initializer=worker_initializer)


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=SimulationFollow();rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)  # repeated Ctrl-C must not interrupt cleanup
        if node is not None:node.close();node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
