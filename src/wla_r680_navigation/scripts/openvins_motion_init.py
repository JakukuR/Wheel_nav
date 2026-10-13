#!/usr/bin/env python3
"""One bounded commissioning arc through the existing chassis arbiter.

Requires explicit --execute. Does not grant Nav2 motion permission. The operator
provides the clear, level test area; this tool has no contact/obstacle detector.
"""
import argparse
import json
import math
import signal
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rcl_interfaces.srv import GetParameters
from wla_openvins_navigation.msg import EstimatorStatus


# Same future-stamp allowance as frontend.cpp::fresh() and its clock fault.
# Real D455 global-time samples can lead ROS time by fractions of a millisecond.
CLOCK_LEAD_TOLERANCE_S = .05


class Abort(RuntimeError):
    pass


class Arc:
    """Wall-time bound remains effective even if wheel odometry freezes."""
    def __init__(self, linear=.2, angular=.5, duration=1.0):
        if not (all(math.isfinite(v) for v in (linear, angular, duration))
                and 0 < linear <= .2 and 0 < angular <= .5 and 0 < duration <= 1.0):
            raise ValueError('arc limits: 0<v<=0.2 m/s, 0<w<=0.5 rad/s, 0<t<=1 s')
        self.linear, self.angular, self.duration = linear, angular, duration
        self.start = self.last = None
        self.travel = self.yaw = 0.0
        self.position = None

    def observe(self, x, y, yaw):
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise Abort('nonfinite_wheel_pose')
        if self.position:
            px, py, pa = self.position
            self.travel += math.hypot(x-px, y-py)
            self.yaw += abs(math.atan2(math.sin(yaw-pa), math.cos(yaw-pa)))
        self.position = x, y, yaw
        if self.travel >= .30 or self.yaw >= .65:
            raise Abort('wheel_motion_envelope_exceeded')

    def command(self, now):
        if not math.isfinite(now) or (self.last is not None and now < self.last):
            raise Abort('nonmonotonic_runtime_clock')
        self.last = now
        if self.start is None:
            self.start = now
        elapsed = now-self.start
        if elapsed >= self.duration:
            return 0.0, 0.0
        # A short ramp avoids a step while preserving the requested curvature.
        scale = min(1.0, elapsed/.20)
        return self.linear*scale, self.angular*scale


class MotionInit(Node):
    def __init__(self, result, execute=False):
        super().__init__('openvins_motion_init')
        self.execute = execute
        self.result = Path(result)
        self.control = self.status = self.wheel = None
        self.control_at = self.status_at = self.wheel_at = 0.0
        self.identity = self.control_identity = None
        self.manual_event = self.cancelled = False
        self.requested_auto = self.acquired = False
        self.arc = Arc()
        self.moving = False
        self.state = 'waiting_inputs'
        self.started = time.monotonic()
        self.reason = ''
        self.motion_requested = False
        self.command_count = 0
        self.peak_wheel_v = self.peak_wheel_w = 0.0
        self.release_confirmed = False
        self.guard_verified = False
        topic = '/r680_nav/chassis_cmd_vel' if execute else '/r680_nav/init_motion_preview'
        self.out = self.create_publisher(Twist, topic, 1)
        self.report = self.create_publisher(String, '/r680_nav/init_motion_state', 10)
        self.create_subscription(String, '/r680_base/control_state', self.on_control, 10)
        self.create_subscription(Twist, '/r680_manual/cmd_vel', self.on_manual, 1)
        self.create_subscription(Odometry, '/wheel/odom', self.on_wheel, qos_profile_sensor_data)
        self.create_subscription(EstimatorStatus, '/openvins/estimator_status', self.on_status, 10)
        self.mode_client = self.create_client(SetBool, '/r680_base/set_automatic')
        self.guard_client = self.create_client(GetParameters, '/r680_command_guard/get_parameters')
        self.last_wheel_stamp = None
        self.fault = ''

    def stamp_fresh(self, stamp, age=.20):
        delta = self.get_clock().now().nanoseconds*1e-9 - stamp.sec - stamp.nanosec*1e-9
        return -CLOCK_LEAD_TOLERANCE_S <= delta <= age

    def on_control(self, message):
        try:
            c = json.loads(message.data)
            age = (self.get_clock().now().nanoseconds - c['stamp_ns'])*1e-9
            if not -.05 <= age <= .5:
                return
            key = c['control_session'], c['source_generation']
            if self.control_identity is not None and key != self.control_identity:
                self.fault = 'chassis_authority_generation_changed'
            self.control, self.control_at = c, time.monotonic()
        except (ValueError, TypeError, KeyError):
            self.fault = 'invalid_chassis_control_report'

    def on_manual(self, message):
        values = message.linear.x, message.angular.z
        if not all(math.isfinite(x) for x in values) or any(abs(x) >= .01 for x in values):
            self.manual_event = True

    def on_status(self, message):
        if not self.stamp_fresh(message.header.stamp, .5):
            return
        key = message.instance_id, message.generation
        if self.identity is not None and key != self.identity:
            self.fault = 'frontend_generation_changed'
        self.status, self.status_at = message, time.monotonic()

    def on_wheel(self, message):
        if not self.stamp_fresh(message.header.stamp):
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        v, w = message.twist.twist.linear.x, message.twist.twist.angular.z
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if (not all(math.isfinite(x) for x in (p.x,p.y,p.z,norm,v,w)) or not .98 <= norm <= 1.02
                or (self.last_wheel_stamp is not None and stamp <= self.last_wheel_stamp)):
            self.fault = 'invalid_or_reordered_wheel_odom'
            return
        self.last_wheel_stamp = stamp
        self.wheel, self.wheel_at = message, time.monotonic()
        self.peak_wheel_v = max(self.peak_wheel_v, abs(v))
        self.peak_wheel_w = max(self.peak_wheel_w, abs(w))
        if self.moving:
            try:
                self.arc.observe(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
                if abs(v) > .30 or abs(w) > .75:
                    raise Abort('measured_chassis_overspeed')
            except Abort as error:
                self.fault = str(error)

    def blocker(self, mode='MANUAL'):
        if self.cancelled or not rclpy.ok():
            return 'cancelled'
        if self.fault:
            return self.fault
        if self.manual_event:
            return 'manual_takeover'
        now = time.monotonic()
        c, s = self.control, self.status
        if c is None or now-self.control_at > .5:
            return 'waiting_for_chassis_control'
        if c.get('estop_latched') is not False:
            return 'estop_latched'
        if c.get('automatic_source') != 'NAV':
            return 'chassis_source_not_nav'
        if mode and c.get('mode') != mode:
            return 'chassis_mode_' + str(c.get('mode'))
        if self.wheel is None or now-self.wheel_at > .2 or not self.stamp_fresh(self.wheel.header.stamp):
            return 'waiting_for_fresh_wheel_odom'
        if s is None or now-self.status_at > .5 or not self.stamp_fresh(s.header.stamp,.5):
            return 'waiting_for_frontend_report'
        if not s.instance_id or s.generation != 0 or s.mode == s.INVALID:
            return 'frontend_fault_or_reset'
        if not (-CLOCK_LEAD_TOLERANCE_S <= s.imu_age_s <= .10
                and -CLOCK_LEAD_TOLERANCE_S <= s.image_age_s <= .35):
            return 'waiting_for_fresh_camera_imu'
        if s.reason in ('waiting_for_matching_camera_info','waiting_for_imu_clock_stability'):
            return s.reason
        if s.initialized and not s.state_valid:
            return 'invalid_initialized_state'
        return ''

    def stopped(self):
        return self.wheel is not None and abs(self.wheel.twist.twist.linear.x) < .01 and abs(self.wheel.twist.twist.angular.z) < .01

    def publish(self, v=0.0, w=0.0):
        message = Twist();message.linear.x=v;message.angular.z=w
        self.out.publish(message)
        if v or w:
            self.motion_requested = True;self.command_count += 1

    def step(self):
        rclpy.spin_once(self, timeout_sec=.02)

    def state_change(self, state, reason=''):
        changed = (self.state,self.reason) != (state,reason)
        self.state,self.reason=state,reason
        self.report.publish(String(data=state+(':'+reason if reason else '')))
        if changed:
            self.get_logger().info('Initialization motion: '+state+' '+reason)
        self.save()

    def save(self):
        data=dict(state=self.state,reason=self.reason,execute=self.execute,
                  motion_requested=self.motion_requested,nonzero_command_count=self.command_count,
                  requested_linear_mps=self.arc.linear,requested_angular_radps=self.arc.angular,
                  maximum_command_duration_s=self.arc.duration,wheel_travel_m=self.arc.travel,
                  wheel_accumulated_yaw_rad=self.arc.yaw,peak_wheel_linear_mps=self.peak_wheel_v,
                  peak_wheel_angular_radps=self.peak_wheel_w,manual_release_confirmed=self.release_confirmed,
                  frontend_initialized=bool(self.status and self.status.initialized),
                  imu_age_s=self.status.imu_age_s if self.status else None,
                  image_age_s=self.status.image_age_s if self.status else None,
                  nav2_hardware_output_enabled=False,contact_and_obstacle_detection=False)
        self.result.parent.mkdir(parents=True,exist_ok=True)
        temp=self.result.with_suffix('.tmp');temp.write_text(json.dumps(data,indent=2));temp.replace(self.result)

    def await_response(self, future, timeout=2.0, neutral=False):
        deadline=time.monotonic()+timeout
        while not future.done() and time.monotonic()<deadline:
            if self.cancelled:
                raise Abort('cancelled')
            if neutral:
                self.publish()
            self.step()
        if not future.done():
            raise Abort('control_service_timeout')
        return future.result()

    def verify_guard(self):
        if not self.guard_client.wait_for_service(timeout_sec=2):
            raise Abort('navigation_guard_missing')
        request=GetParameters.Request();request.names=['hardware_output_enabled']
        result=self.await_response(self.guard_client.call_async(request))
        if len(result.values)!=1 or result.values[0].type!=1 or result.values[0].bool_value:
            raise Abort('normal_navigation_output_must_be_disabled')
        self.guard_verified=True
        self.verify_publishers()

    def verify_publishers(self):
        allowed={'r680_command_guard','openvins_motion_init'}
        publishers=self.get_publishers_info_by_topic('/r680_nav/chassis_cmd_vel')
        seen=[(p.node_namespace,p.node_name) for p in publishers]
        if (any(ns!='/' or name not in allowed for ns,name in seen)
                or len(seen)!=len(set(seen)) or ('/','r680_command_guard') not in seen):
            raise Abort('conflicting_chassis_command_publisher')

    def run(self):
        stable=None;last_report=0.0
        while time.monotonic()-self.started < 90:
            self.step();reason=self.blocker()
            if self.fault or self.cancelled or self.manual_event:
                raise Abort(reason)
            now=time.monotonic()
            if reason or not self.stopped():
                stable=None
                if now-last_report>1:
                    self.state_change('waiting_inputs',reason or 'wheel_not_stationary');last_report=now
                continue
            if self.status.initialized:
                self.state_change('already_initialized');return
            if stable is None:
                stable=now;self.state_change('stationary','hold_still_5_seconds_then_one_arc')
            if now-stable>=5:
                break
        else:
            raise Abort('startup_inputs_timeout')
        self.identity=self.status.instance_id,self.status.generation
        self.control_identity=self.control['control_session'],self.control['source_generation']
        self.verify_guard()
        if self.blocker() or not self.stopped():
            raise Abort(self.blocker() or 'wheel_not_stationary')
        if self.execute:
            if not self.mode_client.wait_for_service(timeout_sec=2):
                raise Abort('chassis_mode_service_missing')
            self.state_change('arming','Nav2_output_remains_disabled')
            deadline=time.monotonic()+.4
            while time.monotonic()<deadline:
                self.publish();self.step()
                if self.blocker() or not self.stopped():
                    raise Abort(self.blocker() or 'wheel_not_stationary')
            self.requested_auto=True
            response=self.await_response(self.mode_client.call_async(SetBool.Request(data=True)),neutral=True)
            if not response.success:
                raise Abort('auto_authority_rejected:'+response.message)
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:
                self.publish();self.step()
                if self.control and self.control.get('mode')=='AUTO':
                    self.acquired=True;break
            if not self.acquired:
                raise Abort('auto_authority_not_observed')
        self.state_change('moving','one_arc_maximum_1_second')
        self.moving=True
        p=self.wheel.pose.pose.position;q=self.wheel.pose.pose.orientation
        self.arc.observe(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        while True:
            reason=self.blocker('AUTO' if self.execute else 'MANUAL')
            if reason:
                raise Abort(reason)
            self.verify_publishers()
            command=self.arc.command(time.monotonic())
            self.publish(*command)
            if command==(0.,0.) and self.arc.last-self.arc.start>=self.arc.duration:
                break
            self.step()
        self.state_change('stopping')
        stopped_at=None;deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            self.publish();self.step()
            if self.fault or self.cancelled:
                raise Abort(self.fault or 'cancelled')
            if time.monotonic()-self.wheel_at<=.2 and self.stopped():
                if stopped_at is None:stopped_at=time.monotonic()
                if time.monotonic()-stopped_at>=.4:break
            else:stopped_at=None
        else:
            raise Abort('stop_not_confirmed_by_wheel')
        self.moving=False
        self.release()
        self.state_change('waiting_for_vio','motion_finished_no_retry')
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            self.step()
            if self.cancelled or self.fault:raise Abort(self.fault or 'cancelled')
            if (self.status and time.monotonic()-self.status_at<.5 and self.status.initialized
                    and self.status.state_valid and self.status.mode in (self.status.VIO_TRACKING,self.status.RECOVERING)):
                self.state_change('succeeded','VIO_initialized_navigation_accuracy_not_accepted');return
        raise Abort('motion_finished_but_vio_not_tracking_no_retry')

    def release(self):
        if not self.execute or not self.requested_auto:
            return
        # Only touch the authority that this attempt requested. A newly selected
        # PERSON/OBJECT source belongs to its own owner.
        current=self.control
        identity=(current.get('control_session'),current.get('source_generation')) if current else None
        if current and identity==self.control_identity and current.get('automatic_source')=='NAV':
            future=self.mode_client.call_async(SetBool.Request(data=False))
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:
                self.publish();self.step()
                if future.done():
                    self.release_confirmed=bool(future.result() and future.result().success)
                    if self.release_confirmed:break
        self.acquired=False

    def cleanup(self):
        # Runs for SIGINT, SIGTERM, exceptions, timeout, and successful completion.
        if self.execute and self.requested_auto:
            deadline=time.monotonic()+.3
            while rclpy.ok() and time.monotonic()<deadline:
                self.publish();self.step()
            self.release()
        self.save()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true',help='Authorize one real 0.2 m/s, 0.5 rad/s arc, at most 1 second')
    parser.add_argument('--result',default='/tmp/openvins_motion_init.json')
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node=MotionInit(args.result,args.execute)
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_:setattr(node,'cancelled',True))
    code=0
    try:
        node.run()
    except (Abort,KeyboardInterrupt) as error:
        node.state_change('failed',str(error) or 'cancelled');code=2
    except Exception as error:
        node.state_change('failed',type(error).__name__+':'+str(error));code=3
    finally:
        try:node.cleanup()
        finally:node.destroy_node();rclpy.shutdown()
    return code


if __name__=='__main__':
    raise SystemExit(main())
