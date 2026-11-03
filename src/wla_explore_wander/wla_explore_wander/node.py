"""ROS adapter for the wander/curiosity loop. No velocity publisher: legs go to
the person-follow backend's PersonCommand service and through its mux gate."""
from dataclasses import fields
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
from wla_behavior_interfaces.srv import PersonCommand
from .explorer import Explorer, Params
from .policy import grid_ray_measure

GRID_PERIOD_S = .3


class ExploreNode(Node):
    def __init__(self):
        super().__init__('explore_wander')
        topics = dict(follow_prefix='/r680/person_follow', prefix='/r680/explore',
                      relative_topic='/r680/perception/relative_objects', wheel_topic='/wheel/odom',
                      control_topic='/r680_base/control_state', seed=-1)
        self.t = {k: self.declare_parameter(k, v).value for k, v in topics.items()}
        params = Params()
        for f in fields(Params):
            if f.name != 'weights':
                setattr(params, f.name, type(getattr(params, f.name))(self.declare_parameter(f.name, getattr(params, f.name)).value))
        for f in fields(params.weights):
            name = 'weights.'+f.name
            setattr(params.weights, f.name, float(self.declare_parameter(name, getattr(params.weights, f.name)).value))
        seed = self.t['seed'] if self.t['seed'] >= 0 else None
        self.explorer = Explorer(self, params, seed)
        prefix, follow = self.t['prefix'], self.t['follow_prefix']
        self.client = self.create_client(PersonCommand, follow+'/command')
        self.status_pub = self.create_publisher(String, prefix+'/status', 5)
        self.create_subscription(String, self.t['relative_topic'], self.on_relative, 1)
        self.create_subscription(Odometry, self.t['wheel_topic'], self.on_wheel, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, follow+'/local_obstacles', self.on_grid, 1)
        self.create_subscription(String, self.t['control_topic'], self.on_control, 5)
        self.create_subscription(String, follow+'/status', self.on_follow_status, 5)
        for name in ('start', 'pause', 'resume', 'stop'):
            self.create_service(Trigger, prefix+'/'+name, lambda req, res, n=name: self.on_trigger(n, res))
        self.grid_at = -math.inf
        self.create_timer(.1, lambda: self.explorer.tick(time.monotonic()))
        self.create_timer(.5, self.publish_status)
        self.get_logger().info('explore_wander ready; legs via '+follow+'/command; waiting for '+prefix+'/start')

    # Leg I/O used by Explorer.
    def _send(self, command, request_id, mode='', target_id='', lease=1.):
        if not self.client.service_is_ready():
            self.explorer.on_leg_reply(request_id, command, False, 'follow_backend_unavailable', 'rejected', time.monotonic())
            return
        req = PersonCommand.Request(command=command, request_id=request_id, mode=mode,
                                    target_id=target_id, lease_duration_s=float(lease))
        future = self.client.call_async(req)

        def done(f):
            try:
                res = f.result()
            except Exception as exc:  # noqa: BLE001 - reported as a failed leg
                self.explorer.on_leg_reply(request_id, command, False, 'service_error:'+str(exc)[:60], 'rejected', time.monotonic())
                return
            if command != 'stop':
                self.explorer.on_leg_reply(request_id, command, res.accepted, res.reason, res.state, time.monotonic())
        future.add_done_callback(done)

    def start_leg(self, request_id, mode, target_id, lease):
        self.get_logger().info(f'leg {request_id}: mode {mode} target {target_id}')
        self._send('start', request_id, mode, target_id, lease)

    def heartbeat(self, request_id, lease):
        self._send('heartbeat', request_id, lease=lease)

    def stop_leg(self, request_id):
        self._send('stop', request_id)

    # Inputs.
    def on_relative(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('schema') != 'wla-relative-detections/v1' or data.get('enabled') is not True:
                return
            if not -.05 <= (self.get_clock().now().nanoseconds-int(data['stamp_ns']))/1e9 <= .5:
                return
            rows = [row for row in data.get('objects', []) if isinstance(row, dict) and isinstance(row.get('label'), str)]
        except (ValueError, KeyError, TypeError):
            return
        self.explorer.on_detections(rows, time.monotonic())

    def on_wheel(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.explorer.on_wheel(stamp, msg.twist.twist.linear.x, msg.twist.twist.angular.z, time.monotonic())

    def on_grid(self, msg):
        now = time.monotonic()
        if now-self.grid_at < GRID_PERIOD_S or not msg.info.width or not msg.info.height:
            return
        self.grid_at = now
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        q = msg.info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.explorer.on_grid(grid_ray_measure(grid, msg.info.resolution, origin, yaw), now)

    def on_control(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(data, dict):
            self.explorer.on_control(data, time.monotonic())

    def on_follow_status(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(data, dict):
            self.explorer.on_follow_status(data, time.monotonic())

    def on_trigger(self, name, res):
        now = time.monotonic()
        res.success, res.message = getattr(self.explorer, name)(now)
        self.get_logger().info(f'{name}: {res.success} {res.message}')
        return res

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.explorer.status(time.monotonic()))))

    def close(self):
        if self.explorer.leg is not None:
            self.explorer.finish(time.monotonic(), 'node_shutdown')


def main(args=None):
    rclpy.init(args=args); node = None
    try:
        node = ExploreNode(); rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            # Give the stop request a moment to leave before the context goes away.
            end = time.monotonic()+.3
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
