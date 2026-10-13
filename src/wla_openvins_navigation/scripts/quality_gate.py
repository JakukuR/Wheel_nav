#!/usr/bin/env python3
"""Separate inertial continuity, map insertion and ground-drive permission."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtabmap_msgs.msg import Info
from wla_openvins_navigation.msg import EstimatorStatus


class Gate(Node):
    def __init__(self):
        super().__init__('openvins_quality_gate')
        self.mode=self.declare_parameter('mode','mapping').value
        self.motion_enabled=self.declare_parameter('motion_output_enabled',True).value
        self.status=None;self.status_seen=0.;self.contact='UNKNOWN';self.contact_seen=0.
        self.identity=None;self.generation_changed=False;self.last_map_stamp=0.;self.last_forwarded=0.
        self.last_degraded_stamp=0.;self.last_odom_stamp=0.
        self.last_match_stamp=0.;self.map_confirmed=False
        self.mapping=self.create_publisher(Odometry,self.declare_parameter('mapping_odom_topic','/openvins_trial/mapping_odom').value,10)
        self.motion=self.create_publisher(Bool,'motion_ready',10)
        self.ready=self.create_publisher(Bool,'mapping_ready',10)
        self.reason=self.create_publisher(String,'gate_status',10)
        self.create_subscription(EstimatorStatus,'estimator_status',self.on_status,10)
        self.create_subscription(Odometry,self.declare_parameter('odom_topic','/openvins_trial/odom').value,self.on_odom,qos_profile_sensor_data)
        self.create_subscription(String,self.declare_parameter('contact_topic','/r680_nav/contact_state').value,self.on_contact,10)
        self.create_subscription(PoseWithCovarianceStamped,'/d455_slam/localization_pose',self.on_map,qos_profile_sensor_data)
        self.create_subscription(Info,'/d455_slam/info',self.on_info,qos_profile_sensor_data)
        self.create_timer(.05,self.tick)

    @staticmethod
    def stamp(t): return t.sec+t.nanosec*1e-9

    def on_contact(self,m):
        self.contact=m.data if m.data in ('GROUNDED','CARRIED','UNKNOWN') else 'UNKNOWN'
        self.contact_seen=time.monotonic()

    def on_status(self,s):
        identity=(s.instance_id,s.generation)
        if self.identity is None: self.identity=identity
        if identity!=self.identity or s.generation!=0: self.generation_changed=True
        self.status=s;self.status_seen=time.monotonic()
        if not self.visual_ready(): self.invalidate_map(self.stamp(s.header.stamp))

    def invalidate_map(self,stamp):
        self.last_degraded_stamp=max(self.last_degraded_stamp,stamp)
        self.last_match_stamp=0.;self.last_map_stamp=0.;self.map_confirmed=False

    def confirm_map(self):
        # Match and pose must belong to the same recent backend update, after
        # the last visual loss. Callback delivery order does not matter.
        if (self.visual_ready() and self.last_match_stamp>self.last_degraded_stamp and
                self.last_map_stamp>self.last_degraded_stamp and
                abs(self.last_map_stamp-self.last_match_stamp)<=.05):
            self.map_confirmed=True

    def on_info(self,m):
        stamp=self.stamp(m.header.stamp)
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if (self.visual_ready() and -.05<=age<=.5 and stamp>self.last_degraded_stamp and
                (m.loop_closure_id>0 or m.proximity_detection_id>0)):
            self.last_match_stamp=stamp;self.confirm_map()

    def on_map(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        age=self.get_clock().now().nanoseconds*1e-9-self.stamp(m.header.stamp)
        if (m.header.frame_id=='map' and -.05<=age<=.5 and
                all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,*m.pose.covariance)) and
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<.02 and
                all(0<=m.pose.covariance[i]<=1. for i in (0,7,14,21,28,35))):
            self.last_map_stamp=self.stamp(m.header.stamp)
            self.confirm_map()

    def recent_status(self):
        s=self.status
        return (s is not None and not self.generation_changed and time.monotonic()-self.status_seen<.2 and
                -.05<=self.get_clock().now().nanoseconds*1e-9-self.stamp(s.header.stamp)<.2 and
                s.state_valid)

    def local_ok(self):
        return (self.recent_status() and self.status.mode==self.status.VIO_TRACKING and
                self.status.uncertainty_acceptable)

    def visual_ready(self):
        # Permit global relocalization with a recovered visual update even when
        # accumulated local position uncertainty still prevents ground driving.
        s=self.status
        return (self.recent_status() and s.mode in (s.VIO_TRACKING,s.RECOVERING) and
                0<=s.visual_update_age_s<=.5 and s.recovery_updates>=3)

    def grounded(self): return self.contact=='GROUNDED' and time.monotonic()-self.contact_seen<.3

    def on_odom(self,m):
        t=self.stamp(m.header.stamp);now=self.get_clock().now().nanoseconds*1e-9
        s=self.status
        allow_input=self.local_ok() and self.grounded() if self.mode=='mapping' else self.visual_ready()
        if (not allow_input or
                not -.05<=now-t<=.2 or t<=self.last_odom_stamp or m.header.frame_id!=s.header.frame_id or
                abs(t-self.stamp(s.state_stamp))>.2): return
        values=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z,
                m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w,
                *m.pose.covariance,*m.twist.covariance]
        if not all(math.isfinite(v) for v in values): return
        self.mapping.publish(m);self.last_forwarded=time.monotonic();self.last_odom_stamp=t

    def tick(self):
        local=self.local_ok();ground=self.grounded()
        now=self.get_clock().now().nanoseconds*1e-9
        if not self.recent_status() or (self.map_confirmed and now-self.last_map_stamp>=3.):
            self.invalidate_map(now)
        map_ok=self.mode=='mapping' or (self.map_confirmed and
            -.05<=now-self.last_map_stamp<3.)
        output_fresh=time.monotonic()-self.last_forwarded<.2
        self.ready.publish(Bool(data=local and output_fresh))
        self.motion.publish(Bool(data=self.motion_enabled and local and ground and map_ok and output_fresh))
        self.reason.publish(String(data='generation_changed' if self.generation_changed else
            'local_not_tracking' if not local else 'contact_'+self.contact if not ground else
            'waiting_map_localization' if not map_ok else 'commissioning_no_motion' if not self.motion_enabled else 'ready'))


def main():
    rclpy.init();node=Gate()
    try: rclpy.spin(node)
    finally: node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
