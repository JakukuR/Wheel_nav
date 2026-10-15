#!/usr/bin/env python3
import json
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from wla_behavior_interfaces.srv import ObjectControl
from action_msgs.srv import CancelGoal
from control_policy import Policy


class Mux(Node):
    def __init__(self):
        super().__init__('r680_command_mux')
        self.p = Policy(*[self.declare_parameter(k, v).value for k, v in
                         [('max_forward_mps', 1.0), ('max_reverse_mps', .3), ('max_turn_rps', 1.5)]],
                        nav_manual_coexist=self.declare_parameter('nav_manual_coexist', True).value)
        self.control_session = uuid.uuid4().hex
        self.out = self.create_publisher(Twist, '/r680_base/cmd_vel', 1)
        self.report = self.create_publisher(String, '/r680_base/control_state', 10)
        self.create_subscription(Twist, '/r680_manual/cmd_vel', self.manual, 1)
        self.create_subscription(Twist, '/r680_nav/chassis_cmd_vel', self.auto, 1)
        self.create_subscription(String, '/r680/person_follow/command_candidate', self.person, 1)
        self.create_subscription(String, '/r680/person_follow/status', self.person_status, 1)
        self.create_subscription(String, '/r680/object_follow/command_candidate', self.object, 1)
        self.create_subscription(String, '/r680/object_follow/status', self.object_status, 1)
        self.create_subscription(String, '/gamepad_status', self.status, 10)
        self.create_subscription(Odometry, '/wheel/odom', self.wheel, qos_profile_sensor_data)
        self.create_service(SetBool, '/r680_base/set_automatic', self.mode)
        self.create_service(SetBool, '/r680_base/set_person_source', self.source)
        self.create_service(SetBool, '/r680_base/set_object_source', self.object_source)
        self.create_service(ObjectControl, '/r680_base/object_control', self.object_control)
        self.create_service(SetBool, '/r680_base/set_estop', self.estop)
        self.cancel = [self.create_client(CancelGoal, '/' + action + '/_action/cancel_goal')
                       for action in ('navigate_to_pose', 'navigate_through_poses')]
        self.create_timer(.02, self.tick)
        self.last_report = 0.

    def cancel_navigation(self):
        for client in self.cancel:
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())

    def manual(self, msg):
        previous = self.p.mode
        takeover = self.p.manual_input((msg.linear.x, msg.angular.z), time.monotonic())
        if previous == 'AUTO' and self.p.mode != 'AUTO':
            self.cancel_navigation(); self.last_report = 0.
        if takeover:
            self.get_logger().warn('Manual takeover latched; autonomous task revocation requested')

    def auto(self, msg):
        self.p.automatic_input((msg.linear.x, msg.angular.z), time.monotonic())

    @staticmethod
    def decode(msg):
        try:
            return json.loads(msg.data)
        except (ValueError, TypeError):
            return None

    def person(self, msg):
        self.p.person_input(self.decode(msg), time.monotonic(), self.get_clock().now().nanoseconds)

    def person_status(self, msg):
        self.p.person_status_input(self.decode(msg), time.monotonic(), self.get_clock().now().nanoseconds)

    def object(self,msg):
        self.p.object_input(self.decode(msg),time.monotonic(),self.get_clock().now().nanoseconds)

    def object_status(self,msg):
        self.p.object_status_input(self.decode(msg),time.monotonic(),self.get_clock().now().nanoseconds)

    def object_control(self,req,res):
        now_ns=self.get_clock().now().nanoseconds
        if req.control_session!=self.control_session or not 0<=now_ns-req.stamp_ns<=200_000_000:
            res.success,res.reason=False,'object_control_session_or_stamp_invalid'
        else:
            res.success,res.reason=self.p.object_control(req.command,req.source_generation,
                req.object_revoke_generation,req.request_id,req.follow_session,time.monotonic(),now_ns)
            if res.success:
                if req.command=='select':self.cancel_navigation()
                self.last_report=0.
        res.control_session=self.control_session
        res.source_generation=self.p.source_generation
        res.object_revoke_generation=self.p.object_channel.person_revoke_generation
        return res

    def object_source(self,req,res):
        previous=self.p.automatic_source
        res.success,res.message=self.p.set_object_source(req.data,time.monotonic(),self.get_clock().now().nanoseconds)
        if res.success and previous!=self.p.automatic_source:
            self.cancel_navigation();self.last_report=0.
        return res

    def status(self, msg):
        previous = self.p.mode
        previous_status = self.p.status
        status = msg.data.split(']', 1)[0].lstrip('[')
        self.p.gamepad_status_input(status, time.monotonic())
        if previous == 'AUTO' and self.p.mode != 'AUTO':
            self.cancel_navigation()
        if previous_status != status or previous != self.p.mode:
            self.last_report = 0.

    def estop(self, req, res):
        res.success, res.message = self.p.set_estop(req.data, time.monotonic())
        if res.success and req.data:
            self.cancel_navigation()
        self.last_report = 0.
        return res

    def wheel(self, msg):
        self.p.wheel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        age = self.get_clock().now().nanoseconds * 1e-9 - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if -.05 <= age <= .2:
            self.p.wheel_at = time.monotonic()

    def mode(self, req, res):
        if req.data:
            res.success, res.message = self.p.set_auto(time.monotonic(), self.get_clock().now().nanoseconds)
        else:
            self.p.set_manual(); self.cancel_navigation()
            res.success = True; res.message = 'MANUAL latched'
        self.last_report = 0.
        return res

    def source(self, req, res):
        previous = self.p.automatic_source
        res.success, res.message = self.p.set_person_source(req.data, time.monotonic(), self.get_clock().now().nanoseconds)
        if res.success and previous != self.p.automatic_source:
            self.cancel_navigation(); self.last_report = 0.
        return res

    def tick(self):
        now = time.monotonic(); previous = self.p.mode
        v, w = self.p.output(now)
        if previous == 'AUTO' and self.p.mode == 'MANUAL':
            self.cancel_navigation(); self.last_report = 0.
        msg = Twist(); msg.linear.x = v; msg.angular.z = w; self.out.publish(msg)
        if now - self.last_report > .2:
            self.last_report = now
            self.report.publish(String(data=json.dumps({
                'control_session': self.control_session, 'stamp_ns': self.get_clock().now().nanoseconds,
                'mode': self.p.mode, 'armed': self.p.armed,
                'nav_manual_coexist': self.p.nav_manual_coexist,
                'output_source': self.p.output_source,
                'manual_ready': self.p.manual_ready(now),
                'automatic_authorized': self.p.mode == 'AUTO' and not self.p.estop_latched,
                'estop_latched': self.p.estop_latched, 'estop_reason': self.p.estop_reason,
                'automatic_source': self.p.automatic_source,
                'source_generation': self.p.source_generation,
                'person_revoke_generation': self.p.person_revoke_generation,
                'object_revoke_generation': self.p.object_channel.person_revoke_generation,
                'object_request_id': self.p.object_channel.person_request_id,
                'object_follow_session': self.p.object_channel.person_session,
                'object_fresh': self.p.object_channel.person_ready(now) and now <= self.p.object_channel.person_until,
                'person_request_id': self.p.person_request_id,
                'follow_session': self.p.person_session,
                'gamepad': self.p.status, 'linear': v, 'angular': w,
                'navigation_fresh': now - self.p.auto_at < .15,
                'person_fresh': self.p.person_ready(now) and now <= self.p.person_until,
            })))


def main():
    # Keep the context alive until the final zero publication and node cleanup.
    # Default rclpy SIGINT teardown can invalidate it between ok() and publish().
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); node = Mux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(5):
                node.out.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
