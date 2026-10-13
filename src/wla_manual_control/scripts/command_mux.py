#!/usr/bin/env python3
import json
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from action_msgs.srv import CancelGoal
from control_policy import Policy


class Mux(Node):
    def __init__(self):
        super().__init__('r680_command_mux')
        self.p = Policy(*[self.declare_parameter(k, v).value for k, v in
                         [('max_forward_mps', 1.0), ('max_reverse_mps', .3), ('max_turn_rps', 1.5)]])
        self.control_session = uuid.uuid4().hex
        self.out = self.create_publisher(Twist, '/r680_base/cmd_vel', 1)
        self.report = self.create_publisher(String, '/r680_base/control_state', 10)
        self.create_subscription(Twist, '/r680_manual/cmd_vel', self.manual, 1)
        self.create_subscription(Twist, '/r680_nav/chassis_cmd_vel', self.auto, 1)
        self.create_subscription(String, '/r680/person_follow/command_candidate', self.person, 1)
        self.create_subscription(String, '/r680/person_follow/status', self.person_status, 1)
        self.create_subscription(String, '/gamepad_status', self.status, 10)
        self.create_subscription(Odometry, '/wheel/odom', self.wheel, qos_profile_sensor_data)
        self.create_service(SetBool, '/r680_base/set_automatic', self.mode)
        self.create_service(SetBool, '/r680_base/set_person_source', self.source)
        self.cancel = [self.create_client(CancelGoal, '/' + action + '/_action/cancel_goal')
                       for action in ('navigate_to_pose', 'navigate_through_poses')]
        self.create_timer(.02, self.tick)
        self.last_report = 0.

    def cancel_navigation(self):
        for client in self.cancel:
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())

    def manual(self, msg):
        if self.p.manual_input((msg.linear.x, msg.angular.z), time.monotonic()):
            self.cancel_navigation(); self.last_report = 0.
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

    def status(self, msg):
        previous = self.p.mode
        self.p.status = msg.data.split(']', 1)[0].lstrip('[')
        self.p.status_at = time.monotonic()
        if self.p.status not in ('ACTIVE', 'SPIN', 'STANDBY'):
            self.p.disarm()
            if previous == 'AUTO':
                self.cancel_navigation(); self.last_report = 0.

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
                'automatic_source': self.p.automatic_source,
                'source_generation': self.p.source_generation,
                'person_revoke_generation': self.p.person_revoke_generation,
                'person_request_id': self.p.person_request_id,
                'follow_session': self.p.person_session,
                'gamepad': self.p.status, 'linear': v, 'angular': w,
                'navigation_fresh': now - self.p.auto_at < .15,
                'person_fresh': self.p.person_ready(now) and now <= self.p.person_until,
            })))


def main():
    rclpy.init(); node = Mux()
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
