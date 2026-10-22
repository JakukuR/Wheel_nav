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
from .depth_proc import rotation_from_quaternion
from .local_memory import LocalMotion
from .relative_control import RelativeFollower
from .reid_identity import TargetIdentity
from .follow_policy import PolicySnapshot
from .follow_config import PENDING_CALIBRATION_PARAMETERS, target_freshness_decision, validate_follow_preset
from .geometry_binding import GeometryBinding, GeometryBindingConfig
from .identity_supervisor import IdentitySupervisor, IdentitySupervisorConfig
from .sim_core.common import rotation
from .sim_core.rolling_depth_map import native_ray_slots
from .sim_runtime import packet_from_depth, measurements_from_relative, local_points, process_step, warm_worker, worker_initializer


def stamp_ns(msg):
    return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec


class SimulationFollow(Node):
    def __init__(self, **kwargs):
        super().__init__('person_follow', **kwargs)
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
            target_mask_enabled=True, reid_enabled=False, reid_model_id='', reid_max_lost_s=8., reid_similarity=.80,
            reid_margin=.08, reid_confirm_frames=3, local_assumed_free_radius_m=0., tof_enabled=False, tof_topics=['/r680/tof/front_left/points',
                '/r680/tof/front_right/points','/r680/tof/rear_left/points','/r680/tof/rear_right/points'],
            follow_preset='compatibility', geometry_binding_v2=False,
            front_check_truncation_enabled=False, front_check_max_m=1.4,
            front_check_target_offset_m=.87, binding_missing_grace_s=.20,
            binding_jump_radius_m=.45, binding_jump_growth_mps=.25, binding_jump_radius_max_m=1.5,
            enrollment_speed_cap_enabled=False, enrollment_speed_cap_mps=.30,
            enrollment_timeout_s=10., supervision_window=10, suspect_negative_frames=6,
            suspect_recovery_frames=6, suspect_timeout_s=12., neighbor_ambiguity_radius_m=1.5,
            neighbor_clear_hold_s=.50, active_search_s=8., target_release_s=30.,
            view_expansion_similarity=.75, view_expansion_neighbor_radius_m=1.5,
            view_expansion_min_interval_s=1.0, view_expansion_initial_slots=2,
            identity_supervisor_v2=False, lost_task_release=False, mask_binding_v2=False,
            reverse_policy_v2=False, reverse_policy_allow_retreat=False, retreat_disabled=False,
            diagnostic_v2=False)
        self.p = {k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        if (self.p['output_mode'] not in ('direct','mux') or
            not 0 < self.p['max_sensor_age_s'] <= .30 or
            not 0 < self.p['result_ttl_s'] <= .20 or
            not 0 <= self.p['startup_clear_radius_m'] <= .5 or
            not 0 <= self.p['local_assumed_free_radius_m'] <= .5 or
            not 1 <= self.p['reid_max_lost_s'] <= 15 or
            not .7 <= self.p['reid_similarity'] <= .99 or
            not .03 <= self.p['reid_margin'] <= .3 or
            not 2 <= self.p['reid_confirm_frames'] <= 5 or
            (self.p['reid_enabled'] and not self.p['reid_model_id']) or
            not .001 <= self.p['startup_motion_threshold_m'] <= .1 or
            not 1 <= self.p['depth_stride'] <= 16 or
            not .1 <= self.p['depth_min_m'] < self.p['depth_max_m'] <= 7 or
            not 0 <= self.p['surface_to_center_offset_m'] <= .5 or
            not .25 <= self.p['camera_min_valid_fraction'] <= 1 or
            not .35 <= self.p['camera_front_min_valid_fraction'] <= 1 or
            not 0 <= self.p['binding_missing_grace_s'] <= 1 or
            not 0 < self.p['binding_jump_radius_m'] <= self.p['binding_jump_radius_max_m'] <= 2 or
            not 0 <= self.p['binding_jump_growth_mps'] <= 1 or
            not .1 < self.p['front_check_max_m'] <= 1.4 or
            not 0 <= self.p['front_check_target_offset_m'] <= .95 or
            not .1 <= self.p['enrollment_speed_cap_mps'] <= .5 or
            not 1 <= self.p['enrollment_timeout_s'] <= 60 or
            not 3 <= self.p['supervision_window'] <= 30 or
            not 1 <= self.p['suspect_negative_frames'] <= self.p['supervision_window'] or
            not 1 <= self.p['suspect_recovery_frames'] <= self.p['supervision_window'] or
            not 1 <= self.p['suspect_timeout_s'] <= 60 or
            not .5 <= self.p['neighbor_ambiguity_radius_m'] <= 3 or
            not 0 <= self.p['neighbor_clear_hold_s'] <= 2 or
            not 1 <= self.p['active_search_s'] < self.p['target_release_s'] <= 60 or
            not .5 <= self.p['view_expansion_similarity'] <= self.p['reid_similarity'] or
            not .5 <= self.p['view_expansion_neighbor_radius_m'] <= 3 or
            not .1 <= self.p['view_expansion_min_interval_s'] <= 10 or
            not 1 <= self.p['view_expansion_initial_slots'] < 8):
            raise ValueError('invalid_sim_adapter_configuration')
        self.follow_preset=validate_follow_preset(self.p)
        self.guard = threading.RLock()
        # Only the tested request/identity/lease ledger is reused. Its old
        # proportional/local-planner tick is never called by this backend.
        self.core = RelativeFollower(base_frame=self.p['base_frame'],max_age=self.p['max_sensor_age_s'])
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
        self.started_at = time.monotonic()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        prefix=self.p['prefix']
        self.preview_pub=self.create_publisher(Twist,prefix+'/cmd_vel_candidate',1)
        self.status_pub=self.create_publisher(String,prefix+'/status',5)
        self.path_pub=self.create_publisher(Path,prefix+'/local_path',1)
        self.grid_pub=self.create_publisher(OccupancyGrid,prefix+'/local_obstacles',1)
        self.reid_request_pub=self.create_publisher(String,prefix+'/reid_request',1) if self.p['reid_enabled'] or self.p['target_mask_enabled'] else None
        self.obstacles_pub=self.create_publisher(PointCloud2,prefix+'/depth_obstacles',qos_profile_sensor_data)
        self.direct_pub=None
        self.leased_pub=None
        if self.p['output_mode']=='mux':
            self.leased_pub=self.create_publisher(String,prefix+'/command_candidate',1)
        elif self.p['enable_motion']:
            # A disabled preview never creates a publisher on the hardware topic.
            self.direct_pub=self.create_publisher(Twist,self.p['direct_topic'],1)
        self.create_subscription(String,self.p['relative_topic'],self.on_relative,1)
        self.create_subscription(String,self.p['detection_status_topic'],self.on_status,1)
        if self.p['reid_enabled']:
            self.create_subscription(String,prefix+'/reid_features',self.on_reid,1)
        if self.p['target_mask_enabled']:
            self.create_subscription(String,'/r680/perception/target_mask',self.on_target_mask,1)
        self.create_subscription(Image,self.p['depth_topic'],self.on_depth,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,self.p['camera_info_topic'],self.on_info,qos_profile_sensor_data)
        self.create_subscription(Odometry,self.p['wheel_topic'],self.on_wheel,qos_profile_sensor_data)
        self.create_subscription(String,'/r680_base/control_state',self.on_control,5)
        if self.p['tof_enabled']:
            for topic in self.p['tof_topics']:
                self.create_subscription(PointCloud2,topic,lambda m,k=topic:self.on_tof(k,m),qos_profile_sensor_data)
        self.create_service(PersonCommand,prefix+'/command',self.on_command)
        self.create_service(SetBool,prefix+'/set_estop',self.on_estop)
        self.pool=ProcessPoolExecutor(max_workers=1,mp_context=get_context('spawn'),initializer=worker_initializer)
        self.model_tuning = self.pool.submit(warm_worker).result(timeout=20.)
        self.future=None
        # A completed solution wakes the ROS executor immediately. The timer
        # still rechecks expiry/stop/lease when no new solution arrives.
        self.result_ready=self.create_guard_condition(self.tick)
        self.last_geometry=None
        self.worker=threading.Thread(target=self.compute,name='sim-follow-compute',daemon=True)
        self.worker.start()
        self.control_timer=self.create_timer(.05,self.tick)
        self.get_logger().info('close-wide-flex backend; output='+self.p['output_mode']+
            '; enable_motion='+str(self.p['enable_motion'])+'; waiting for explicit target request')

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def key(self):
        c=self.core
        return (c.request_id,c.target_id,c.session,c.epoch,self.motion.generation)

    def on_info(self,msg):
        with self.guard:self.infos.append(msg)

    def on_relative(self,msg):
        with self.guard:
            accepted=self.core.receive(msg.data,time.monotonic(),self.now_ns())
            if not accepted or not self.core.active:self.result=None

    def on_reid(self,msg):
        try:
            if len(msg.data)>262144:return
            data=json.loads(msg.data);ns=data['stamp_ns']
            if (data.get('schema')!='wla-reid-features/v1' or data.get('base_frame')!=self.p['base_frame']
                    or data.get('model_id')!=self.p['reid_model_id'] or type(ns) is not int
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
            self.wheel_diagnostic=wheel_message(msg,time.monotonic(),diagnostic_accepted,self.motion.generation)
            if previous!=self.motion.generation:self.result=None

    def on_depth(self,msg):
        ns=stamp_ns(msg); age=(self.now_ns()-ns)/1e9
        if ns<=0 or not -.05<=age<=self.p['max_sensor_age_s'] or not 0<msg.width*msg.height<=2_000_000:return
        with self.guard:
            if ns<=self.last_depth_stamp:return
            self.last_depth_stamp=ns
            self.depth_pairer.put_depth(ns,(msg,time.monotonic()+self.p['max_sensor_age_s']-max(0.,age)))
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
            if req.command=='start' and req.mode!='follow':
                accepted,reason=False,'simulation_backend_supports_follow_only_use_legacy_for_approach'
            elif req.command=='start' and (self.estop or self.conflict_latched):
                accepted,reason=False,'estop_or_direct_publisher_conflict'
            else:
                old_key=self.key()
                accepted,reason=self.core.command(req.command,req.request_id,req.mode,req.target_id,req.lease_duration_s,time.monotonic())
                if old_key!=self.key() or not self.core.active:self.result=None
            state=self.core.snapshot(time.monotonic())
            res.accepted,res.reason=accepted,reason
            for name in ('request_id','state','mode','target_id','enabled'):setattr(res,name,state[name])
            terminal=(req.request_id==state['request_id'] and state['state'] in ('arrived','stopped','cancelled','expired'))
            if not accepted and req.command!='status' and not terminal:
                res.request_id=req.request_id;res.mode=req.mode;res.target_id=req.target_id
                res.state='rejected';res.enabled=False
            elif accepted and req.command=='stop' and req.request_id and req.request_id!=state['request_id']:
                res.request_id=req.request_id;res.state='stopped';res.enabled=False;res.mode='';res.target_id=''
            if accepted and req.command=='start' and self.reid_request_pub is not None:
                self.reid_request_pub.publish(String(data=json.dumps(dict(
                    schema='wla-reid-request/v1',stamp_ns=self.now_ns(),enabled=True,urgent=True,
                    request_id=self.core.request_id,session=self.core.session,epoch=self.core.epoch,
                    allowed_ids=[self.core.target_id],target_id=self.core.target_id))))
            return res

    def on_estop(self,req,res):
        with self.guard:
            self.estop=bool(req.data);self.result=None
            self.core._terminate('stopped','local_estop' if self.estop else 'estop_reset_requires_new_request')
            res.success=True;res.message='latched' if self.estop else 'reset; new request required'
            return res

    def compute(self):
        next_at=0.;last_tof={};identity=None;identity_key=None;binding=None;supervisor=None
        while not self.done:
            self.event.wait(.02)
            if self.done:return
            remaining=next_at-time.monotonic()
            if remaining>0:
                time.sleep(min(remaining,.1));continue
            with self.guard:
                self.event.clear();paired=self.depth_pairer.take(self.now_ns(),self.core.active and self.p['target_mask_enabled'])
                item,mask_packet=paired if paired is not None else (None,None)
                infos=tuple(self.infos);motion=deepcopy(self.motion);key=self.key()
                active=self.core.active;tof_messages=tuple(self.tof.values())
            if item is None:continue
            next_at=time.monotonic()+.1
            msg,valid_until=item;stamp=stamp_ns(msg)/1e9
            processing_started=time.monotonic()
            depth_start_age_ms=(self.now_ns()-stamp_ns(msg))/1e6
            identity_status=None
            consumed_identity_frames=[]
            enrollment_speed_cap_status=None
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
                    reid_frames=tuple(self.reid_frames)
                control_stamp=self.now_ns()/1e9
                control_pose=motion.pose_at(control_stamp)
                if control_pose is None:raise ValueError('control_time_wheel_pose_unavailable')
                observation=None;op=motion.pose_at(obs_stamp)
                resolved_id=key[1];identity_status=dict(state='disabled')
                binding_status=None;supervisor_status=None
                if self.p['reid_enabled']:
                    if not active:
                        identity=None;identity_key=None;binding=None;supervisor=None
                        identity_status=dict(state='inactive')
                    else:
                        if identity_key!=key:
                            identity=TargetIdentity(key[1],max_lost=self.p['reid_max_lost_s'],
                                similarity=self.p['reid_similarity'],margin=self.p['reid_margin'],
                                confirmations=self.p['reid_confirm_frames'],model_id=self.p['reid_model_id'],start_time=control_stamp)
                            binding=GeometryBinding(key[1],control_stamp,GeometryBindingConfig(
                                freshness_s=self.p['max_sensor_age_s'],
                                missing_grace_s=self.p['binding_missing_grace_s'],
                                jump_radius_m=self.p['binding_jump_radius_m'],
                                jump_growth_mps=self.p['binding_jump_growth_mps'],
                                jump_radius_max_m=self.p['binding_jump_radius_max_m'],
                                active_search_s=self.p['active_search_s'],
                                release_s=self.p['target_release_s'],
                                surface_to_center_offset_m=self.p['surface_to_center_offset_m']))
                            supervisor=IdentitySupervisor(key[1],model_id=self.p['reid_model_id'],
                                start_time=control_stamp,config=IdentitySupervisorConfig(
                                    similarity=self.p['reid_similarity'],margin=self.p['reid_margin'],
                                    confirmations=self.p['reid_confirm_frames'],
                                    supervision_window=self.p['supervision_window'],
                                    suspect_negative_frames=self.p['suspect_negative_frames'],
                                    suspect_recovery_frames=self.p['suspect_recovery_frames'],
                                    suspect_timeout_s=self.p['suspect_timeout_s'],
                                    neighbor_ambiguity_radius_m=self.p['neighbor_ambiguity_radius_m'],
                                    neighbor_clear_hold_s=self.p['neighbor_clear_hold_s'],
                                    enrollment_timeout_s=self.p['enrollment_timeout_s'],
                                    active_search_s=self.p['active_search_s'],
                                    release_s=self.p['target_release_s'],
                                    view_expansion_similarity=self.p['view_expansion_similarity'],
                                    view_expansion_neighbor_radius_m=self.p['view_expansion_neighbor_radius_m'],
                                    view_expansion_min_interval_s=self.p['view_expansion_min_interval_s'],
                                    view_expansion_initial_slots=self.p['view_expansion_initial_slots']))
                            identity_key=key
                        if self.p['geometry_binding_v2']:
                            neighbor_ambiguity=bool(op is not None and supervisor is not None and
                                supervisor.neighbor_near(objects,op,binding.resolved_id or key[1],
                                    self.p['neighbor_ambiguity_radius_m'],self.p['surface_to_center_offset_m']))
                            suspect_reason='neighbor_ambiguity' if neighbor_ambiguity else None
                            resolved_id=binding.update(objects,op,obs_stamp,control_stamp,
                                enrollment_complete=supervisor.complete,suspect_reason=suspect_reason)
                            for batch in reid_frames:
                                ts=batch['stamp_ns']/1e9
                                if ts<=supervisor.last_frame_stamp or not 0<=control_stamp-ts<=.6:continue
                                if (batch['session'],batch['epoch'])!=(key[2],key[3]) or batch.get('request_id')!=key[0]:continue
                                bp=motion.pose_at(ts)
                                if bp is None:continue
                                rows=[];prefix=f'{key[2]}:{key[3]}:'
                                for row in batch['objects']:
                                    try:
                                        if (not row['id'].startswith(prefix) or not row['id'][len(prefix):].isdigit()
                                                or not math.isfinite(row['yaw']) or not abs(row['yaw'])<=math.pi
                                                or not math.isfinite(row['distance']) or not 0<row['distance']<=20):continue
                                        d=row['distance']+self.p['surface_to_center_offset_m'];a=bp[2]+row['yaw']
                                        rows.append(dict(id=row['id'],feature=row['feature'],
                                            position=np.asarray(bp[:2])+d*np.array([np.cos(a),np.sin(a)])))
                                    except (ValueError,KeyError,TypeError,AttributeError):continue
                                model_ok=batch.get('model_id')==self.p['reid_model_id']
                                supervisor.observe(rows if model_ok else [],ts,
                                    batch.get('diagnostics') if model_ok else {binding.resolved_id or key[1]:'feature_model_mismatch'},
                                    geometry_state=binding.state,neighbor_ambiguity=neighbor_ambiguity,
                                    binding_snapshot=binding.snapshot(control_stamp))
                                consumed_identity_frames.append(deepcopy(supervisor.snapshot(control_stamp)))
                            if binding.state=='lost' and supervisor.last_frame_stamp<control_stamp:
                                supervisor.observe([],control_stamp,{},geometry_state=binding.state,
                                    binding_snapshot=binding.snapshot(control_stamp))
                            proposal=supervisor.reacquire_proposal
                            if binding.state=='lost' and proposal is not None:
                                accepted=binding.rebind(proposal['id'],objects,op,obs_stamp,
                                    control_stamp,enrollment_complete=supervisor.complete,confirmation=proposal)
                                if accepted is not None:
                                    resolved_id=accepted
                                    supervisor.rebind(accepted,control_stamp)
                            if supervisor.state.startswith('suspect_'):
                                binding.suspect(supervisor.reason)
                            binding_status=binding.snapshot(control_stamp)
                            supervisor_status=supervisor.snapshot(control_stamp)
                            identity_status=supervisor_status
                            if active and supervisor.end_reason:
                                with self.guard:
                                    if key==self.key() and self.core.active:
                                        self.core._terminate('expired',supervisor.end_reason)
                                        self.result=None;self.reason=supervisor.end_reason
                                continue
                        else:
                            for batch in reid_frames:
                                ts=batch['stamp_ns']/1e9
                                if ts<=identity.frame_stamp or not 0<=control_stamp-ts<=.6:continue
                                if (batch['session'],batch['epoch'])!=(key[2],key[3]) or batch.get('request_id')!=key[0]:continue
                                bp=motion.pose_at(ts)
                                if bp is None:continue
                                rows=[];prefix=f'{key[2]}:{key[3]}:'
                                for row in batch['objects']:
                                    try:
                                        if (not row['id'].startswith(prefix) or not row['id'][len(prefix):].isdigit()
                                                or not math.isfinite(row['yaw']) or not abs(row['yaw'])<=math.pi
                                                or not math.isfinite(row['distance']) or not 0<row['distance']<=20):continue
                                        d=row['distance']+self.p['surface_to_center_offset_m'];a=bp[2]+row['yaw']
                                        rows.append(dict(id=row['id'],feature=row['feature'],
                                            position=np.asarray(bp[:2])+d*np.array([np.cos(a),np.sin(a)])))
                                    except (ValueError,KeyError,TypeError,AttributeError):continue
                                identity.consume(rows,ts,batch['model_id'],batch.get('diagnostics'))
                                consumed_identity_frames.append(deepcopy(identity.candidate_frame))
                            resolved_id=identity.select(objects,op,obs_stamp,control_stamp,self.p['surface_to_center_offset_m']) if op is not None else None
                            identity_status=identity.snapshot(control_stamp)

                if self.reid_request_pub is not None:
                    if self.p['geometry_binding_v2']:
                        allowed=(supervisor.request_ids(objects,op,binding_snapshot=binding.snapshot(control_stamp),
                            center_offset=self.p['surface_to_center_offset_m']) if active and supervisor is not None and op is not None else [])
                    else:
                        allowed=identity.candidate_ids(objects,op,obs_stamp,control_stamp,self.p['surface_to_center_offset_m']) if active and identity is not None else []
                    with self.guard:
                        if key==self.key() and active==self.core.active:
                            self.reid_request_pub.publish(String(data=json.dumps(dict(
                                schema='wla-reid-request/v1',stamp_ns=self.now_ns(),enabled=active,
                                request_id=key[0],session=key[2],epoch=key[3],allowed_ids=allowed,target_id=(resolved_id or '') if active else '',
                                urgent=active and (resolved_id is None or identity_status.get('state')!='tracking')))))
                if identity is not None and self.p['reid_enabled']:
                    identity_status['candidate_request']=(supervisor.request_ids(objects,op,
                        binding_snapshot=binding.snapshot(control_stamp),center_offset=self.p['surface_to_center_offset_m'])
                        if self.p['geometry_binding_v2'] and supervisor is not None and op is not None else deepcopy(identity.request_diagnostics))
                    identity_status['consumed_candidate_frames']=consumed_identity_frames
                if self.p['geometry_binding_v2'] and binding is not None:
                    resolved_source=dict(source=('GeometryBinding.select' if self.p['reid_enabled'] else 'task_target_reid_disabled'),
                        active=active,accepted_id=resolved_id if active else None,
                        observation_pose_available=op is not None,
                        selection=deepcopy(binding_status))
                else:
                    resolved_source=dict(source='TargetIdentity.select' if self.p['reid_enabled'] else 'task_target_reid_disabled',
                        active=active,accepted_id=resolved_id if active else None,
                        observation_pose_available=op is not None,
                        selection=(deepcopy(identity.selection_diagnostics) if op is not None else
                            dict(accepted=False,reason='select_not_called_observation_pose_unavailable')) if identity is not None else None)
                if active and op is not None and 0<=control_stamp-obs_stamp<=self.p['max_sensor_age_s']:
                    target,people=measurements_from_relative(objects,op,resolved_id or '',self.p['surface_to_center_offset_m'])
                    observation=(obs_stamp,target,people,resolved_id)
                # In v2 the mask follows the same-task geometry binding, not a
                # cosmetic ReID state. Legacy mode preserves its old criteria.
                mask_started=time.monotonic()
                depth,mask_status=filter_target_depth(depth,mask_packet,
                    target_id=resolved_id if active else None,request_id=key[0],session=key[2],epoch=key[3],
                    depth_stamp_ns=stamp_ns(msg),frame_id=msg.header.frame_id,now_ns=self.now_ns())
                mask_status['compute_ms']=(time.monotonic()-mask_started)*1000.
                if self.p['mask_binding_v2']:
                    mask_status['decision_basis']='geometry_binding'
                    mask_status['binding_state']=None if binding_status is None else binding_status.get('state')
                frontend_begun=time.monotonic()
                packet=packet_from_depth(depth,(info.k[0],info.k[4],info.k[2],info.k[5]),r,trans,pose,
                    self.p['depth_min_m'],self.p['depth_max_m'],self.p['depth_stride'])
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
                         front_check_truncation=self.p['front_check_truncation_enabled'],
                         front_check_max_m=self.p['front_check_max_m'],
                         front_check_target_offset_m=self.p['front_check_target_offset_m'],
                         retreat_disabled=self.p['retreat_disabled'],
                         reverse_policy_v2=self.p['reverse_policy_v2'],
                         reverse_policy_allow_retreat=self.p['reverse_policy_allow_retreat'],
                         policy_snapshot=PolicySnapshot.from_binding(binding,control_stamp) if self.p['geometry_binding_v2'] and active and binding is not None else None)))
                try:
                    result=self.future.result(timeout=self.p['result_ttl_s'])
                except ComputeTimeout:
                    raise ValueError('compute_deadline_exceeded')
                if (self.p['enrollment_speed_cap_enabled'] and identity_status
                        and identity_status.get('state')=='enrolling'
                        and result['command'][0]>self.p['enrollment_speed_cap_mps']):
                    result['command'][0]=float(self.p['enrollment_speed_cap_mps'])
                    enrollment_speed_cap_status=dict(applied=True,
                        reason='background_enrollment_speed_cap',
                        cap_mps=float(self.p['enrollment_speed_cap_mps']))
                result.update(target_observation=observed_target(objects,key[1],obs_stamp,control_stamp),
                    resolved_target_observation=observed_target(objects,resolved_id,obs_stamp,control_stamp),
                    resolved_target_source=resolved_source,
                    target_mask=mask_status,reid=identity_status,geometry_binding=binding_status,enrollment_speed_cap=enrollment_speed_cap_status,
                    identity_supervision=supervisor_status,resolved_target_id=resolved_id if active else None,
                    key=key,stamp_ns=stamp_ns(msg),valid_until=min(valid_until,begun+self.p['result_ttl_s']),
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
        """Apply source freshness, with v2's bounded Kalman coast grace."""
        binding=result.get('geometry_binding') or {} if result is not None else {}
        observation_ns=result.get('observation_stamp_ns') if result is not None else None
        observation_age=(None if type(observation_ns) is not int
                         else (ns-observation_ns)/1e9)
        allowed,reason=target_freshness_decision(
            source_fresh=state['observations_fresh'],
            geometry_binding_v2=self.p['geometry_binding_v2'],
            binding_state=binding.get('state'),observation_age_s=observation_age,
            max_sensor_age_s=self.p['max_sensor_age_s'],
            missing_grace_s=self.p['binding_missing_grace_s'])
        return allowed,reason,observation_age

    def _calibration_diagnostic(self,result):
        binding=(result or {}).get('geometry_binding') or {}
        supervision=(result or {}).get('identity_supervision') or {}
        front=(result or {}).get('front_blockage') or {}
        return dict(status='pending_calibration',
            reason='v2_defaults_require_real_sensor_and_rosbag_replay',
            parameters={name:self.p[name] for name in PENDING_CALIBRATION_PARAMETERS},
            active_triggers=dict(
                geometry_binding=dict(state=binding.get('state'),reason=binding.get('reason'),
                    last_position_error_m=binding.get('last_position_error_m'),
                    position_jump_limit_m=binding.get('position_jump_limit_m'),
                    lost_s=binding.get('lost_s'),recovery_phase=binding.get('recovery_phase')),
                identity_supervision=dict(state=supervision.get('state'),reason=supervision.get('reason'),
                    suspect_type=supervision.get('suspect_type'),suspect_trigger=supervision.get('suspect_trigger'),
                    suspect_age_s=supervision.get('suspect_age_s'),
                    latest_template_update=(supervision.get('template_update_log') or [None])[-1]),
                front_blockage=dict(reason=front.get('reason'),source=front.get('source'),
                    effective_check_length_m=front.get('effective_check_length_m'),
                    path_range_m=front.get('path_range_m'),
                    nearest_block_path_distance_m=front.get('nearest_block_path_distance_m'),
                    trigger_sample_reference_m=front.get('trigger_sample_reference_m'))))

    def check_direct_owner(self):
        if self.direct_pub is None:return False
        endpoints=self.get_publishers_info_by_topic(self.p['direct_topic'])
        # Count all endpoints, including another node with the same name.
        if len(endpoints)>1:
            self.conflict_latched=True
            self.core._terminate('cancelled','direct_publisher_conflict')
            self.result=None
        return not self.conflict_latched and len(endpoints)==1 and time.monotonic()-self.started_at>=1.

    def tick(self):
        with self.guard:
            now=time.monotonic();ns=self.now_ns();state=self.core.snapshot(now)
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
            if self.p['reid_enabled'] and state['enabled'] and result is not None:
                identity=result.get('reid',{})
                if self.p['geometry_binding_v2']:
                    binding=result.get('geometry_binding') or {}
                    if binding.get('state')=='suspect' or str(identity.get('state','')).startswith('suspect_'):
                        healthy=False
                        blockers.append('identity_suspect_stop')
                elif identity.get('state') in ('enrolling','expired'):
                    healthy=False
                    blockers.append(identity.get('reason','reid_identity_unconfirmed'))
            command=result['command'] if healthy else np.zeros(2)
            policy=result.get('policy_snapshot') if result is not None else None
            if self.p['geometry_binding_v2'] and policy is not None:
                command=policy.limit_command(command,ns/1e9)
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
            if self.core.active:
                self.core.state='following' if healthy else 'waiting'
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
                planner='close-wide-flex',numeric_backend='python_mpc_cpp_map' if native_ray_slots is not None else 'python_reference',native_frontend='cpp_deproject_ground_cluster_filter',
                obstacle_topic=self.p['prefix']+'/depth_obstacles',
                base_frame=self.p['base_frame'],output_mode=self.p['output_mode'],
                candidate_only=not self.p['enable_motion'],motion_permitted=permitted and healthy,
                linear_mps=float(command[0]),angular_rps=float(command[1]),safety_fresh=healthy,
                safety_reason=blockers[0] if blockers else self.reason,
                output_blockers=blockers,planner_reason=self.reason,
                target_freshness_reason=target_freshness_reason,
                target_observation_age_s=target_observation_age,
                planner_candidates=result.get('candidates',{}) if result else {},
                result_remaining_ms=round((result['valid_until']-now)*1000.,1) if result else None,
                reference_generation=self.motion.generation,
                robot_radius_m=self.model_tuning['radius'],obstacle_margin_m=self.model_tuning['margin'],
                tof_enabled=self.p['tof_enabled'],direct_publisher_conflict=self.conflict_latched,
                startup_clear_radius_m=self.p['startup_clear_radius_m'],
                startup_motion_threshold_m=self.p['startup_motion_threshold_m'],
                local_assumed_free_radius_m=self.p['local_assumed_free_radius_m'],
                local_free_prior=self.map_result.get('local_free_prior') if self.map_result else None,
                retreat_distance_m=self.map_result.get('retreat_distance_m') if self.map_result else None,
                startup_prior=self.map_result.get('startup_prior') if self.map_result else None,
                startup_prior_consumed=self.map_result.get('startup_prior_consumed',False) if self.map_result else False,
                fault=self.conflict_latched or self.estop,
                target_tracking=result['target_state'] if result else 'unavailable',
                target_mask=self.map_result.get('target_mask') if self.map_result else None,
                reid_enabled=self.p['reid_enabled'],
                reid=self.map_result.get('reid') if self.map_result else None,
                geometry_binding=self.map_result.get('geometry_binding') if self.map_result else None,
                identity_supervision=self.map_result.get('identity_supervision') if self.map_result else None,
                resolved_target_id=self.map_result.get('resolved_target_id') if self.map_result else None,
                resolved_target_source=self.map_result.get('resolved_target_source') if self.map_result else None,
                calibration=self._calibration_diagnostic(result),
                follow_preset=self.follow_preset,
                follow_config={name:self.p[name] for name in (
                    'geometry_binding_v2','front_check_truncation_enabled','enrollment_speed_cap_enabled',
                    'enrollment_speed_cap_mps','identity_supervisor_v2','mask_binding_v2',
                    'reverse_policy_v2','reverse_policy_allow_retreat','retreat_disabled',
                    'lost_task_release','diagnostic_v2','binding_missing_grace_s',
                    'front_check_max_m','front_check_target_offset_m','enrollment_timeout_s',
                    'supervision_window','suspect_negative_frames','suspect_recovery_frames',
                    'suspect_timeout_s','neighbor_ambiguity_radius_m','neighbor_clear_hold_s',
                    'active_search_s','target_release_s','view_expansion_similarity',
                    'view_expansion_neighbor_radius_m','view_expansion_min_interval_s',
                    'view_expansion_initial_slots')},
                recovery_state=result['recovery_state'] if result else 'waiting',
                wide_offset=result['wide_offset'] if result else None,
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
                measured_motion=dict(wheel=deepcopy(self.wheel_diagnostic),
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
            if self.context.ok() and self.direct_pub is not None and not self.conflict_latched:
                if len(self.get_publishers_info_by_topic(self.p['direct_topic']))==1:self.direct_pub.publish(Twist())
        self.event.set();self.worker.join(timeout=3.)
        self.pool.shutdown(wait=True,cancel_futures=True)


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
