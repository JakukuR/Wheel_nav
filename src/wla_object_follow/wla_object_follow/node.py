"""Object behavior with shared RGB-D/TF/lease plumbing, independent endpoints."""
from collections import deque
import math
import time
import numpy as np
import rclpy
from std_msgs.msg import String
from wla_behavior_interfaces.srv import ObjectCommand
from wla_detection_bridge.stream import DetectionInbox,decode
from wla_person_follow.node import PersonFollow,stamp_ns
from wla_person_follow.depth_proc import points_to_base
from .control import ObjectFollower

def reference_points(points,pose,inverse=False):
    points=np.asarray(points,dtype=np.float32).reshape(-1,3).copy()
    c,s=math.cos(pose[2]),math.sin(pose[2])
    if inverse:points[:,:2]=(points[:,:2]-np.asarray(pose[:2])) @ np.array([[c,-s],[s,c]])
    else:points[:,:2]=points[:,:2] @ np.array([[c,s],[-s,c]])+np.asarray(pose[:2])
    return points

class ObjectFollow(PersonFollow):
    NODE_NAME='object_follow';PREFIX='/r680/object_follow';SOURCE='OBJECT';COMMAND_TYPE=ObjectCommand
    EXTRA_DEFAULTS=dict(object_clearance_m=.15,footprint_front_m=.24,
                        footprint_rear_m=.24,footprint_half_width_m=.24,
                        close_max_linear_mps=.20,close_gain=.8,response_delay_s=.7,
                        braking_deceleration_mps2=.3)

    def __init__(self):
        self.raw=DetectionInbox();self.depth_records=deque(maxlen=12);self.surface=None
        super().__init__()
        self.create_subscription(String,'/r680/perception/detections',self.on_raw,1)
        self.create_service(ObjectCommand,'/wla_agent/object_command',self.on_command)

    def make_core(self):
        return ObjectFollower(self.p['base_frame'],self.p['max_sensor_age_s'],
            self.p['follow_distance_m'],self.p['approach_distance_m'],
            self.p['max_linear_mps'],self.p['max_angular_rps'],self.p['emergency_distance_m'],
            self.local_planner,require_estimate=True,clearance=self.p['object_clearance_m'],
            front=self.p['footprint_front_m'],rear=self.p['footprint_rear_m'],
            half_width=self.p['footprint_half_width_m'],
            close_max=self.p['close_max_linear_mps'],close_gain=self.p['close_gain'],
            reaction=self.p['response_delay_s'],deceleration=self.p['braking_deceleration_mps2'])

    def on_raw(self,message):
        with self.guard:
            try:self.raw.push(message.data,self.ros_ns(),time.monotonic())
            except (ValueError,TypeError,KeyError):
                self.surface=None;self.raw.batch=None;return
            self.refresh_surface()

    def on_projected_depth(self,message,info,depth,rotation,translation,points):
        with self.guard:
            stamp=stamp_ns(message);pose=self.local_motion.pose_at(stamp/1e9)
            if pose is None:return
            self.depth_records.append((message,info,depth,rotation,translation,points,
                                       pose,self.local_motion.generation))
            self.refresh_surface()

    def refresh_surface(self):
        self.surface=None
        if not self.core.active or self.raw.batch is None:return
        batch,_=self.raw.batch
        try:decode(batch,self.ros_ns(),self.p['max_sensor_age_s'])
        except ValueError:self.surface=None;return
        identity=(batch['session'],batch['epoch'])
        if identity!=(self.core.session,self.core.epoch):self.surface=None;return
        row=next((o for o in batch['objects'] if
            f"{batch['session']}:{batch['epoch']}:{o['track_id']}"==self.core.target_id and
            o['label']==self.core.target_label),None)
        if row is None:self.surface=None;return
        record=min(self.depth_records,key=lambda q:abs(stamp_ns(q[0])-batch['stamp_ns']),default=None)
        if record is None:return
        message,info,depth,rotation,translation,points,pose,generation=record
        stamp=stamp_ns(message)
        if (abs(stamp-batch['stamp_ns'])>35_000_000 or
                generation!=self.local_motion.generation or message.header.frame_id!=batch['frame_id'] or
                (message.width,message.height)!=(batch['width'],batch['height'])):return
        x0,y0,x1,y1=row['bbox_xyxy'];x0,y0=int(math.floor(x0)),int(math.floor(y0))
        x1,y1=int(math.ceil(x1)),int(math.ceil(y1))
        target=points_to_base(depth[y0:y1,x0:x1],
            (info.k[0],info.k[4],info.k[2]-x0,info.k[5]-y0),rotation,translation,
            minimum=self.p['depth_min_m'],maximum=self.p['depth_max_m'],
            height_min=.01,height_max=2.5,stride=2)
        self.surface=(self.core.target_id,generation,stamp,batch['stamp_ns'],
                      reference_points(points,pose),reference_points(target,pose))

    def update_target_estimate(self):
        estimate=super().update_target_estimate()
        self.refresh_surface()
        self.core.geometry=None;self.core.geometry_until=-math.inf
        if self.local_motion.samples:
            velocity=self.local_motion.samples[-1][2]
            self.core.measured_velocity=(velocity[0],velocity[2])
        now=self.ros_ns();pose=self.local_motion.pose_at(now/1e9)
        if self.surface is not None and pose is not None:
            target_id,generation,depth_stamp,batch_stamp,points,target=self.surface
            age=max((now-depth_stamp)/1e9,(now-batch_stamp)/1e9)
            if (target_id==self.core.target_id and generation==self.local_motion.generation and
                    -.05<=age<=self.p['max_sensor_age_s']):
                # A newer depth obstacle must stop us without waiting for another YOLO inference.
                latest=self.depth_records[-1] if self.depth_records else None
                if latest is not None and latest[-1]==generation and 0<=(now-stamp_ns(latest[0]))/1e9<=self.p['max_sensor_age_s']:
                    points=np.vstack((points,reference_points(latest[5],latest[6])))
                self.core.geometry=(reference_points(points,pose,True),reference_points(target,pose,True),batch_stamp)
                self.core.geometry_until=time.monotonic()+self.p['max_sensor_age_s']-max(0.,age)
        return estimate

    def publish(self,linear,angular):
        if time.monotonic()>=self.core.geometry_until:linear,angular=self.core._zero()
        super().publish(linear,angular)

def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=ObjectFollow();rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if node is not None:node.close();node.destroy_node()
        rclpy.try_shutdown()
