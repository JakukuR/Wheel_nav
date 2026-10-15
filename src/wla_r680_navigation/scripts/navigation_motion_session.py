#!/usr/bin/env python3
"""One explicit navigation authorization after startup; never reclaim takeover.

The root entry runs this only with --enable-motion. --cuvslam-init authorizes
bounded initialization before localization; ordinary navigation waits for readiness.
This node publishes permission, not chassis velocity.
"""
import argparse
import json
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class Session(Node):
    def __init__(self, cuvslam_init=False):
        super().__init__('navigation_motion_session')
        self.cuvslam_init=cuvslam_init;self.init_authorized=False;self.init_state='';self.init_at=0.
        self.init_permission=None
        if cuvslam_init:
            self.init_permission=self.create_publisher(Bool,'/r680_nav/vio_init_base_authorized',1)
            self.create_subscription(String,'/r680_nav/vio_init_state',self.on_init,10)
        self.cancelled=False;self.control=None;self.control_at=0.;self.wheel=None;self.wheel_at=0.
        self.healthy=False;self.health_at=0.;self.identity=None;self.requested=False;self.guard_open=False
        self.goals={};self.authorized=False;self.owns_permission=False
        self.frontend='not_received';self.next_wait_report=0.
        self.permit=self.create_publisher(Bool,'/r680_nav/mission_motion_allowed',1)
        self.report=self.create_publisher(String,'/r680_nav/navigation_motion_state',10)
        self.auto=self.create_client(SetBool,'/r680_base/set_automatic')
        self.guard=self.create_client(SetParameters,'/r680_command_guard/set_parameters')
        self.create_subscription(String,'/r680_base/control_state',self.on_control,10)
        self.create_subscription(Odometry,'/wheel/odom',self.on_wheel,qos_profile_sensor_data)
        self.create_subscription(Bool,'/r680_nav/localization_ready',self.on_health,10)
        self.create_subscription(Odometry,'/openvins/odomimu',self.on_frontend,qos_profile_sensor_data)
        self.cancels={}
        for name in ('navigate_to_pose','navigate_through_poses'):
            self.cancels[name]=self.create_client(CancelGoal,f'/{name}/_action/cancel_goal')
            qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(GoalStatusArray,f'/{name}/_action/status',self.on_goals,qos)

    def on_init(self,m):self.init_state=m.data;self.init_at=time.monotonic()

    def on_control(self,m):
        try:
            c=json.loads(m.data);age=(self.get_clock().now().nanoseconds-c['stamp_ns'])*1e-9
            if -.05<=age<=.5:self.control=c;self.control_at=time.monotonic()
        except (ValueError,KeyError,TypeError):self.control=None

    def on_wheel(self,m):
        age=(self.get_clock().now()-rclpy.time.Time.from_msg(m.header.stamp)).nanoseconds*1e-9
        if -.05<=age<=.2:self.wheel=m;self.wheel_at=time.monotonic()

    def on_health(self,m):self.healthy=m.data;self.health_at=time.monotonic()
    def on_frontend(self,m):self.frontend='official_odom_received'
    def health_ready(self):return self.healthy and time.monotonic()-self.health_at<.5
    def on_goals(self,m):
        for s in m.status_list:self.goals[bytes(s.goal_info.goal_id.uuid)]=s.status
    def emit(self,state):
        self.report.publish(String(data=state));self.get_logger().info(state)
    def spin(self):
        if self.init_permission is not None:
            self.init_permission.publish(Bool(data=self.init_authorized))
        rclpy.spin_once(self,timeout_sec=.02)
    def permission(self,value=False):
        if self.owns_permission:self.permit.publish(Bool(data=value))
    def key(self):
        c=self.control or {};return c.get('control_session'),c.get('source_generation')
    def check_control(self,mode):
        c=self.control
        if c is None or time.monotonic()-self.control_at>.5:raise RuntimeError('chassis_report_stale')
        if self.identity is not None and self.key()!=self.identity:raise RuntimeError('authority_generation_changed')
        if c.get('automatic_source')!='NAV':raise RuntimeError('another_source_owns_chassis')
        if c.get('estop_latched') is not False:raise RuntimeError('estop_latched')
        if c.get('mode')!=mode:raise RuntimeError('manual_takeover' if mode=='AUTO' else 'chassis_not_manual')

    def wait(self,future,limit=3.):
        end=time.monotonic()+limit
        while not future.done() and time.monotonic()<end:
            if self.cancelled:raise RuntimeError('cancelled')
            self.permission();self.spin()
        if not future.done():raise RuntimeError('service_timeout')
        return future.result()

    def set_output(self,enabled):
        if not self.guard.wait_for_service(timeout_sec=2.):raise RuntimeError('command_guard_missing')
        req=SetParameters.Request();req.parameters=[Parameter(name='hardware_output_enabled',value=ParameterValue(type=1,bool_value=enabled))]
        # Record before calling so an uncertain service outcome is rolled back.
        if enabled:self.guard_open=True
        result=self.wait(self.guard.call_async(req))
        if not result or not result.results or not all(v.successful for v in result.results):raise RuntimeError('guard_parameter_rejected')
        self.guard_open=enabled

    def cancel_old_goals(self):
        pending=set()
        if not self.cancels['navigate_to_pose'].wait_for_service(timeout_sec=3.):raise RuntimeError('navigation_action_missing')
        for client in self.cancels.values():
            if not client.service_is_ready():continue
            result=self.wait(client.call_async(CancelGoal.Request()))
            if result.return_code!=CancelGoal.Response.ERROR_NONE:raise RuntimeError('old_goal_cancel_rejected')
            pending.update(bytes(g.goal_id.uuid) for g in result.goals_canceling)
        end=time.monotonic()+3.
        while pending and time.monotonic()<end:
            self.permission();self.spin()
            pending={g for g in pending if self.goals.get(g) not in (4,5,6)}
        if pending:raise RuntimeError('old_goal_still_active')

    def wait_for_health_and_stop(self, mode="MANUAL", require_health=True):
        # Readiness may drop between the outer startup probe and this process.
        # Keep permission false and wait for recovery; never repeat motion init.
        stable_since=None
        while not self.cancelled and rclpy.ok():
            self.permission();self.spin();self.check_control(mode)
            now=time.monotonic()
            if (self.wheel is None or now-self.wheel_at>.2 or
                    not abs(self.wheel.twist.twist.linear.x)<.01 or not abs(self.wheel.twist.twist.angular.z)<.01):
                raise RuntimeError('chassis_not_stopped')
            if not require_health or self.health_ready():
                if stable_since is None:stable_since=now
                if now-stable_since>=.5:return
            else:stable_since=None
            if now>=self.next_wait_report:
                reason=('localization_not_ready' if not self.healthy else
                        'localization_report_stale' if not self.health_ready() else 'zero_speed_handshake')
                self.emit(f'waiting_for_navigation_handoff: {reason}; frontend_last={self.frontend}; motion_permission=false')
                self.next_wait_report=now+5.
        raise RuntimeError('cancelled')

    def run(self):
        self.emit('waiting_for_navigation_handoff')
        until=time.monotonic()+5.
        while time.monotonic()<until:
            if self.cancelled:raise RuntimeError('cancelled')
            self.permission();self.spin()
            if self.control and self.wheel:break
        self.check_control('MANUAL');self.identity=self.key()
        owners=self.get_publishers_info_by_topic('/r680_nav/mission_motion_allowed')
        if any(p.node_name!=self.get_name() or p.node_namespace!='/' for p in owners) or len(owners)!=1:
            raise RuntimeError('another_mission_permission_owner')
        self.owns_permission=True
        if self.cuvslam_init:
            # This session starts alongside bringup, before guard discovery.
            deadline=time.monotonic()+30.
            while not self.guard.service_is_ready():
                if self.cancelled:raise RuntimeError('cancelled')
                self.permission();self.spin();self.check_control('MANUAL')
                if time.monotonic()>=deadline:raise RuntimeError('command_guard_missing')
        self.set_output(True) # Permission is still false: guard emits zero only.
        if not self.auto.wait_for_service(timeout_sec=2.):raise RuntimeError('chassis_mode_service_missing')
        if self.cuvslam_init:
            self.run_initialization()
            return
        while True:
            self.wait_for_health_and_stop()
            # Cancel again if recovery allowed more goals to arrive while waiting.
            self.cancel_old_goals()
            self.check_control('MANUAL')
            if self.health_ready():break
        self.requested=True
        result=self.wait(self.auto.call_async(SetBool.Request(data=True)))
        if not result.success:raise RuntimeError('auto_rejected:'+result.message)
        until=time.monotonic()+1.
        while time.monotonic()<until:
            self.permission();self.spin()
            if self.control and self.control.get('mode')=='AUTO':break
        self.check_control('AUTO');self.authorized=True
        self.maintain_navigation()

    def run_initialization(self):
        # Explicit authorization covers bounded initialization before map readiness.
        # The guard emits zero until the initializer sees our live authorization.
        self.wait_for_health_and_stop(require_health=False)
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            if self.cancelled:raise RuntimeError('cancelled')
            self.permission();self.spin();self.check_control('MANUAL')
            if self.init_state=='waiting_inputs' and time.monotonic()-self.init_at<.3:break
        else:raise RuntimeError('initializer_not_waiting_for_base_authorization')
        self.requested=True
        result=self.wait(self.auto.call_async(SetBool.Request(data=True)))
        if not result.success:raise RuntimeError('auto_rejected:'+result.message)
        deadline=time.monotonic()+1.
        while time.monotonic()<deadline:
            self.permission();self.spin()
            if self.control and self.control.get('mode')=='AUTO':break
        self.check_control('AUTO');self.init_authorized=True
        self.emit('initialization_enabled; AUTO/NAV; bounded initialization only')
        deadline=time.monotonic()+120.
        while not self.cancelled and time.monotonic()<deadline:
            self.permission();self.spin();self.check_control('AUTO')
            if time.monotonic()-self.init_at>.3:raise RuntimeError('initializer_report_stale')
            if self.init_state=='failed':raise RuntimeError('initialization_failed')
            if self.init_state=='succeeded':break
        else:raise RuntimeError('initialization_cancelled_or_timed_out')
        self.init_authorized=False
        self.cancel_old_goals()
        self.wait_for_health_and_stop(mode='AUTO')
        self.check_control('AUTO');self.authorized=True
        self.maintain_navigation()

    def maintain_navigation(self):
        self.emit('navigation_enabled; send a new goal; manual takeover will not be reclaimed')
        next_heartbeat=0.
        while not self.cancelled and rclpy.ok():
            self.spin();self.check_control('AUTO')
            if time.monotonic()>=next_heartbeat:
                self.permission(True);next_heartbeat=time.monotonic()+.05

    def cleanup(self):
        self.authorized=False;self.init_authorized=False
        # Cleanup must run even after SIGINT/SIGTERM.
        self.cancelled=False
        for _ in range(15):self.permission();self.spin()
        if self.guard_open:
            try:self.set_output(False)
            except Exception as error:self.get_logger().error('guard disable: '+str(error))
        if self.requested and self.key()==self.identity and (self.control or {}).get('automatic_source')=='NAV':
            try:self.wait(self.auto.call_async(SetBool.Request(data=False)),limit=1.)
            except Exception as error:self.get_logger().error('release AUTO: '+str(error))
        self.emit('navigation_permission_released')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--cuvslam-init',action='store_true')
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node=Session(cuvslam_init=args.cuvslam_init)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:setattr(node,'cancelled',True))
    code=0
    try:node.run()
    except Exception as error:node.emit('navigation_stopped: '+str(error));code=2
    finally:
        try:node.cleanup()
        finally:node.destroy_node();rclpy.shutdown()
    return code


if __name__=='__main__':raise SystemExit(main())
