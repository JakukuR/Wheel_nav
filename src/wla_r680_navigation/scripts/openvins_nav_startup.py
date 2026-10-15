#!/usr/bin/env python3
"""Start Nav2 once a fresh OpenVINS localization stream and TF exist.

This only configures/activates lifecycle nodes. It grants no motion permission,
does not publish commands and never retries or resets a failed Nav2 startup.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ManageLifecycleNodes
from tf2_ros import Buffer, TransformListener


class Startup(Node):
    def __init__(self):
        super().__init__('openvins_nav_startup')
        self.odom = None
        self.odom_seen = 0.0
        self.future = None
        self.done = False
        self.failed = False
        self.begin = time.monotonic()
        self.timeout = float(self.declare_parameter('initialization_wait_s', 300.0).value)
        if not 1 <= self.timeout <= 600:
            raise ValueError('initialization_wait_s must be 1..600 seconds')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = self.create_client(ManageLifecycleNodes,
                                        '/lifecycle_manager_navigation/manage_nodes')
        self.create_subscription(Odometry, '/d455_slam/odom', self.on_odom, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)
        self.get_logger().info('Waiting for initialized OpenVINS, mapping odom and map TF before Nav2 startup')

    def fail(self, reason):
        self.failed = True
        self.get_logger().error('OpenVINS Nav2 startup failed: ' + reason)

    def on_odom(self, msg):
        self.odom, self.odom_seen = msg, time.monotonic()

    def fresh(self, stamp, limit):
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(stamp)).nanoseconds * 1e-9
        return -0.05 <= age <= limit

    def tick(self):
        if self.done or self.failed:
            return
        if self.future is not None:
            if self.future.done():
                try:
                    result = self.future.result()
                    if result and result.success:
                        self.done = True
                        self.get_logger().info('OpenVINS Nav2 startup completed; motion gates remain independent')
                    else:
                        self.fail('lifecycle manager rejected STARTUP')
                except Exception as error:
                    self.fail(str(error))
            elif time.monotonic() - self.requested_at > 120.0:
                self.fail('lifecycle manager response timeout; no automatic retry')
            return
        if time.monotonic() - self.begin > self.timeout:
            self.fail('initialization wait timeout')
            return
        # Official odomimu is published only after upstream initialization.
        o = self.odom
        if o is None:
            return
        if not (time.monotonic()-self.odom_seen <= 0.2 and self.fresh(o.header.stamp, 0.2)):
            return
        if o.header.frame_id != 'd455_floor_odom' or o.child_frame_id != 'r680_mapping_floor':
            return
        if not self.buffer.can_transform('map', 'r680_mapping_floor', rclpy.time.Time()):
            return
        if not self.client.service_is_ready():
            return
        request = ManageLifecycleNodes.Request()
        request.command = request.STARTUP
        self.requested_at = time.monotonic()
        self.future = self.client.call_async(request)
        self.get_logger().info('Fresh initialized OpenVINS stream available; requesting Nav2 STARTUP once')


def main():
    rclpy.init()
    node = Startup()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
