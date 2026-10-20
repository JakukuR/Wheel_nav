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
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from wla_behavior_interfaces.srv import PersonCommand
from .camera import depth_meters
from .depth_proc import rotation_from_quaternion
from .local_memory import LocalMotion
from .relative_control import RelativeFollower
from .sim_core.common import rotation
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
            max_sensor_age_s=.30, depth_stride=4, depth_min_m=.4, depth_max_m=5.,
            surface_to_center_offset_m=.28, camera_min_valid_fraction=.25,
            camera_front_min_valid_fraction=.35, result_ttl_s=.15,
            tof_enabled=False, tof_topics=['/r680/tof/front_left/points',
                '/r680/tof/front_right/points','/r680/tof/rear_left/points','/r680/tof/rear_right/points'])
        self.p = {k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        if (self.p['output_mode'] not in ('direct','mux') or
            not 0 < self.p['max_sensor_age_s'] <= .30 or
            not 0 < self.p['result_ttl_s'] <= .20 or
            not 1 <= self.p['depth_stride'] <= 16 or
            not .1 <= self.p['depth_min_m'] < self.p['depth_max_m'] <= 7 or
            not 0 <= self.p['surface_to_center_offset_m'] <= .5 or
            not .25 <= self.p['camera_min_valid_fraction'] <= 1 or
            not .35 <= self.p['camera_front_min_valid_fraction'] <= 1):
            raise ValueError('invalid_sim_adapter_configuration')
        self.guard = threading.RLock()
        # Only the tested request/identity/lease ledger is reused. Its old
        # proportional/local-planner tick is never called by this backend.
        self.core = RelativeFollower(base_frame=self.p['base_frame'],max_age=self.p['max_sensor_age_s'])
        self.motion = LocalMotion()
        self.wheel_received = -math.inf
        self.infos = deque(maxlen=24)
        self.pending = None
        self.last_depth_stamp = 0
        self.tof = {}
        self.event = threading.Event()
        self.done = False
        self.result = None
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
        self.direct_pub=None
        self.leased_pub=None
        if self.p['output_mode']=='mux':
            self.leased_pub=self.create_publisher(String,prefix+'/command_candidate',1)
        elif self.p['enable_motion']:
            # A disabled preview never creates a publisher on the hardware topic.
            self.direct_pub=self.create_publisher(Twist,self.p['direct_topic'],1)
        self.create_subscription(String,self.p['relative_topic'],self.on_relative,1)
        self.create_subscription(String,self.p['detection_status_topic'],self.on_status,1)
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
        self.pool.submit(warm_worker).result(timeout=20.)
        self.future=None
        self.worker=threading.Thread(target=self.compute,name='sim-follow-compute',daemon=True)
        self.worker.start()
        self.create_timer(.05,self.tick)
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

    def on_status(self,msg):
        with self.guard:
            self.core.receive_status(msg.data,time.monotonic(),self.now_ns())
            if not self.core.active:self.result=None

    def on_wheel(self,msg):
        with self.guard:
            previous=self.motion.generation
            if msg.child_frame_id!=self.p['base_frame']:
                self.motion.samples.clear();self.motion.generation+=1
            else:
                v=msg.twist.twist
                if self.motion.update(stamp_ns(msg)/1e9,(v.linear.x,v.linear.y,v.angular.z),self.now_ns()/1e9):
                    self.wheel_received=time.monotonic()
            if previous!=self.motion.generation:self.result=None

    def on_depth(self,msg):
        ns=stamp_ns(msg); age=(self.now_ns()-ns)/1e9
        if ns<=0 or not -.05<=age<=self.p['max_sensor_age_s'] or not 0<msg.width*msg.height<=2_000_000:return
        with self.guard:
            if ns<=self.last_depth_stamp:return
            self.last_depth_stamp=ns
            self.pending=(msg,time.monotonic()+self.p['max_sensor_age_s']-max(0.,age))
            self.event.set()

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
            return res

    def on_estop(self,req,res):
        with self.guard:
            self.estop=bool(req.data);self.result=None
            self.core._terminate('stopped','local_estop' if self.estop else 'estop_reset_requires_new_request')
            res.success=True;res.message='latched' if self.estop else 'reset; new request required'
            return res

    def compute(self):
        next_at=0.;last_tof={}
        while not self.done:
            self.event.wait(.1)
            if self.done:return
            remaining=next_at-time.monotonic()
            if remaining>0:
                time.sleep(min(remaining,.1));continue
            with self.guard:
                self.event.clear();item=self.pending;self.pending=None
                infos=tuple(self.infos);motion=deepcopy(self.motion);key=self.key()
                objects=deepcopy(self.core.objects);obs_stamp=self.core.last_stamp/1e9
                active=self.core.active;tof_messages=tuple(self.tof.values())
            if item is None:continue
            next_at=time.monotonic()+.1
            msg,valid_until=item;stamp=stamp_ns(msg)/1e9
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
                packet=packet_from_depth(depth,(info.k[0],info.k[4],info.k[2],info.k[5]),r,trans,pose,
                    self.p['depth_min_m'],self.p['depth_max_m'],self.p['depth_stride'])
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
                # Transform measurements at acquisition time, but control from
                # the latest wheel pose after depth processing, not image pose.
                with self.guard:
                    if key!=self.key():raise ValueError('reference_changed_during_depth')
                    motion=deepcopy(self.motion)
                control_stamp=self.now_ns()/1e9
                control_pose=motion.pose_at(control_stamp)
                if control_pose is None:raise ValueError('control_time_wheel_pose_unavailable')
                observation=None;op=motion.pose_at(obs_stamp)
                if active and op is not None and 0<=control_stamp-obs_stamp<=self.p['max_sensor_age_s']:
                    target,people=measurements_from_relative(objects,op,key[1],self.p['surface_to_center_offset_m'])
                    observation=(obs_stamp,target,people)
                v=motion.samples[-1][2]
                begun=time.monotonic()
                if self.future is not None and not self.future.done():
                    raise ValueError('compute_worker_busy_no_queue_growth')
                self.future=self.pool.submit(process_step,(key,np.asarray(control_pose),np.array([v[0],v[2]]),packets,observation,control_stamp))
                try:
                    result=self.future.result(timeout=self.p['result_ttl_s'])
                except ComputeTimeout:
                    raise ValueError('compute_deadline_exceeded')
                result.update(key=key,stamp_ns=stamp_ns(msg),valid_until=min(valid_until,begun+self.p['result_ttl_s']),
                    compute_ms=(time.monotonic()-begun)*1000.,pose=np.asarray(control_pose),control_stamp_ns=int(control_stamp*1e9))
                with self.guard:
                    if key==self.key() and time.monotonic()<result['valid_until']:
                        self.result=result;self.reason=result['reason'] or 'simulation_core'
                    else:
                        self.result=None
                        self.reason=('expired_or_superseded_computation:'+str(round(result['compute_ms'],1))+'ms;'+
                            'source_age='+str(round((self.now_ns()-result['stamp_ns'])/1e9,3))+';key_changed='+str(key!=self.key()))
            except Exception as exc:
                with self.guard:self.result=None;self.reason=str(exc)[:180]

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
            healthy=(result is not None and result['key']==self.key() and now<result['valid_until'] and
                0<=(ns-result['stamp_ns'])/1e9<=self.p['max_sensor_age_s'] and
                self.motion.healthy(ns/1e9) and now-self.wheel_received<=.20 and
                state['observations_fresh'] and self.core.gate_enabled and state['enabled'] and not self.estop)
            command=result['command'] if healthy else np.zeros(2)
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
                planner='close-wide-flex',numeric_backend='python_reference',native_frontend='cpp_deproject',
                base_frame=self.p['base_frame'],output_mode=self.p['output_mode'],
                candidate_only=not self.p['enable_motion'],motion_permitted=permitted and healthy,
                linear_mps=float(command[0]),angular_rps=float(command[1]),safety_fresh=healthy,
                safety_reason=self.reason,reference_generation=self.motion.generation,
                tof_enabled=self.p['tof_enabled'],direct_publisher_conflict=self.conflict_latched,
                fault=self.conflict_latched or self.estop,
                target_tracking=result['target_state'] if result else 'unavailable',
                recovery_state=result['recovery_state'] if result else 'waiting',
                wide_offset=result['wide_offset'] if result else None,
                compute_ms=result['compute_ms'] if result else None)
            self.status_pub.publish(String(data=json.dumps(state,allow_nan=False)))
            if healthy:self.publish_geometry(result,ns)

    def publish_geometry(self,result,ns):
        pose=result['pose'];rot=rotation(pose[2])
        path=Path();path.header.frame_id=self.p['base_frame']
        path.header.stamp.sec,path.header.stamp.nanosec=divmod(result.get('control_stamp_ns',result['stamp_ns']),1_000_000_000)
        for point in (result['path']-pose[:2])@rot:
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
