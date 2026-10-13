"""Relative approach/follow service with local depth safety and leased candidates."""
from collections import deque
import json
import math
import threading
import time
import uuid

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from wla_behavior_interfaces.srv import PersonCommand

from .camera import depth_meters
from .costmap import LocalCostmap
from .depth_proc import points_to_base, rotation_from_quaternion
from .relative_control import RelativeFollower, Safety


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class PersonFollow(Node):
    def __init__(self):
        super().__init__('person_follow')
        defaults = dict(
            relative_topic='/r680/perception/relative_objects',
            detection_status_topic='/r680_nav/perception/status',
            depth_topic='/r680/d455/aligned_depth_to_color/image_raw',
            camera_info_topic='/r680/d455/color/camera_info',
            base_frame='r680_mapping_floor', max_sensor_age_s=.30,
            follow_distance_m=1.4, approach_distance_m=.95,
            max_linear_mps=.20, max_angular_rps=.45, emergency_distance_m=.60,
            obstacle_inflation_m=.45, obstacle_height_min_m=.10,
            obstacle_height_max_m=1.50, depth_min_m=.25, depth_max_m=5.,
            camera_min_valid_fraction=.25, camera_front_min_valid_fraction=.35,
            enable_motion=False)
        self.p = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.core = RelativeFollower(self.p['base_frame'], self.p['max_sensor_age_s'],
            self.p['follow_distance_m'], self.p['approach_distance_m'],
            self.p['max_linear_mps'], self.p['max_angular_rps'], self.p['emergency_distance_m'])
        if (not 0 < self.p['depth_min_m'] < self.p['depth_max_m'] <= 10 or
                not 0 <= self.p['obstacle_height_min_m'] < self.p['obstacle_height_max_m'] or
                not .30 <= self.p['obstacle_inflation_m'] <= .75 or
                not .25 <= self.p['camera_min_valid_fraction'] <= 1 or
                not .35 <= self.p['camera_front_min_valid_fraction'] <= 1):
            raise ValueError('invalid_depth_safety_configuration')
        self.guard = threading.RLock()
        self.event = threading.Event()
        self.done, self.pending = False, None
        self.infos = deque(maxlen=24)
        self.last_depth_stamp, self.safety_stamp, self.safety_until = 0, 0, -math.inf
        self.safety = Safety()
        self.safety_reason = 'waiting_for_depth'
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.control_state, self.control_seen = {}, -math.inf
        self.control_until = -math.inf
        self.retired_control_sessions = set()
        self.follow_session = uuid.uuid4().hex
        self.last_motion_permission = False
        self.cmd_pub = self.create_publisher(Twist, '/r680/person_follow/cmd_vel_candidate', 1)
        self.leased_pub = self.create_publisher(String, '/r680/person_follow/command_candidate', 1)
        self.status_pub = self.create_publisher(String, '/r680/person_follow/status', 5)
        self.create_subscription(String, self.p['relative_topic'], self.on_relative, 1)
        self.create_subscription(String, self.p['detection_status_topic'], self.on_detection_status, 1)
        self.create_subscription(Image, self.p['depth_topic'], self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.on_info, qos_profile_sensor_data)
        self.create_subscription(String, '/r680_base/control_state', self.on_control_state, 5)
        self.create_service(PersonCommand, '/r680/person_follow/command', self.on_command)
        self.worker = threading.Thread(target=self.depth_worker, name='relative-depth-safety', daemon=True)
        self.worker.start()
        self.create_timer(.05, self.tick)
        self.get_logger().info('Relative person service ready; waiting for explicit Agent command and detection. '
            + ('Motion candidates enabled; final mux authorization remains required.' if self.p['enable_motion']
               else 'Preview mode: leased motion candidates remain zero.'))

    def ros_ns(self):
        return self.get_clock().now().nanoseconds

    def on_relative(self, message):
        with self.guard:
            if not self.core.receive(message.data, time.monotonic(), self.ros_ns()):
                self.publish(0., 0.)
            elif not self.core.active or not self.core.objects.get(self.core.target_id):
                self.publish(0., 0.)

    def on_detection_status(self, message):
        with self.guard:
            self.core.receive_status(message.data, time.monotonic(), self.ros_ns())
            if not self.core.active:
                self.publish(0., 0.)

    def on_command(self, request, response):
        with self.guard:
            accepted, reason = self.core.command(request.command, request.request_id, request.mode,
                request.target_id, request.lease_duration_s, time.monotonic())
            state = self.core.snapshot(time.monotonic())
            acknowledgement = dict(state)
            terminal_for_caller = (request.request_id == state['request_id'] and
                state['state'] in ('arrived', 'stopped', 'cancelled', 'expired'))
            if not accepted and request.command != 'status' and not terminal_for_caller:
                acknowledgement.update(request_id=request.request_id, state='rejected', enabled=False,
                                       mode=request.mode, target_id=request.target_id)
            elif (accepted and request.command == 'stop' and request.request_id and
                  request.request_id != state['request_id']):
                # The service acknowledges the requested cancellation. Current
                # task state continues on the status stream without any change.
                acknowledgement.update(request_id=request.request_id, state='stopped', enabled=False,
                                       mode='', target_id='')
            response.accepted, response.reason = accepted, reason
            for field in ('request_id', 'state', 'mode', 'target_id', 'enabled'):
                setattr(response, field, acknowledgement[field])
            if accepted and (request.command == 'start' or request.command == 'stop' and
                             (not request.request_id or request.request_id == state['request_id'])):
                self.publish(0., 0.)
        return response

    def on_control_state(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                return
            session = data.get('control_session')
            if (data.get('automatic_source') not in ('NAV', 'PERSON') or
                    type(data.get('source_generation')) is not int or
                    type(data.get('person_revoke_generation')) is not int or
                    data['source_generation'] < 0 or data['person_revoke_generation'] < 0 or
                    not isinstance(session, str) or not 1 <= len(session) <= 128 or
                    data.get('mode') not in ('AUTO', 'MANUAL')):
                return
            source_stamp = data.get('stamp_ns')
            source_age = (self.ros_ns()-source_stamp)/1e9 if type(source_stamp) is int else math.inf
            if (type(source_stamp) is not int or source_stamp <= 0 or
                    not -.05 <= source_age <= .50):
                return
            with self.guard:
                previous = self.control_state
                if session in self.retired_control_sessions:
                    return
                same_session = session == previous.get('control_session')
                if previous and same_session and (source_stamp <= previous['stamp_ns'] or
                    data['source_generation'] < previous['source_generation'] or
                    data['person_revoke_generation'] < previous['person_revoke_generation']):
                    return
                if previous and not same_session:
                    if len(self.retired_control_sessions) >= 1024:
                        return
                    self.retired_control_sessions.add(previous['control_session'])
                revoked = bool(previous) and (
                    not same_session and previous['automatic_source'] == 'PERSON' or
                    same_session and data['person_revoke_generation'] != previous['person_revoke_generation'] or
                    previous['automatic_source'] == 'PERSON' and data['automatic_source'] != 'PERSON' or
                    previous['automatic_source'] == 'PERSON' and previous['mode'] == 'AUTO' and data['mode'] != 'AUTO')
                self.control_state, self.control_seen = data, time.monotonic()
                self.control_until = self.control_seen + max(0., .50-max(0., source_age))
                if revoked and self.core.active:
                    self.core._terminate('cancelled', 'motion_authority_revoked')
                    self.publish(0., 0.)
        except (ValueError, TypeError, KeyError):
            return

    def on_info(self, message):
        with self.guard:
            self.infos.append(message)

    def on_depth(self, message):
        stamp = stamp_ns(message)
        now = self.ros_ns()
        if (stamp <= 0 or not -.05 <= (now-stamp)/1e9 <= self.p['max_sensor_age_s'] or
                not 0 < message.width*message.height <= 2_000_000):
            return
        with self.guard:
            if stamp <= self.last_depth_stamp:
                return
            self.last_depth_stamp = stamp
            self.pending = (message, now, time.monotonic())
            self.event.set()

    def depth_worker(self):
        while True:
            self.event.wait(.1)
            with self.guard:
                if self.done:
                    return
                pending, self.pending = self.pending, None
                self.event.clear()
                infos = tuple(self.infos)
            if pending is None:
                continue
            message, ros_received, received = pending
            stamp = stamp_ns(message)
            valid_until = received + self.p['max_sensor_age_s'] - max(0., (ros_received-stamp)/1e9)
            if time.monotonic() >= valid_until:
                continue
            try:
                info = min(infos, key=lambda m: abs(stamp_ns(m)-stamp), default=None)
                if (info is None or abs(stamp_ns(info)-stamp) > 250_000_000 or
                        info.header.frame_id != message.header.frame_id or
                        (info.width, info.height) != (message.width, message.height) or
                        not np.isfinite(info.k).all() or info.k[0] <= 0 or info.k[4] <= 0):
                    raise ValueError('depth_calibration_unavailable')
                transform = self.tf.lookup_transform(self.p['base_frame'], message.header.frame_id,
                                                      Time(nanoseconds=stamp)).transform
                q, t = transform.rotation, transform.translation
                rotation = rotation_from_quaternion((q.x, q.y, q.z, q.w))
                translation = np.array([t.x, t.y, t.z], dtype=np.float32)
                if not np.isfinite(translation).all():
                    raise ValueError('invalid_mount_transform')
                depth = depth_meters(message)
                valid = np.isfinite(depth) & (depth >= self.p['depth_min_m']) & (depth <= self.p['depth_max_m'])
                h, w = valid.shape
                front = valid[int(h*.35):max(int(h*.80),int(h*.35)+1), int(w*.30):max(int(w*.70),int(w*.30)+1)]
                if (float(np.mean(valid)) < self.p['camera_min_valid_fraction'] or not front.size or
                        float(np.mean(front)) < self.p['camera_front_min_valid_fraction']):
                    raise ValueError('insufficient_depth_coverage')
                points = points_to_base(depth, (info.k[0],info.k[4],info.k[2],info.k[5]),
                    rotation, translation, minimum=self.p['depth_min_m'], maximum=self.p['depth_max_m'],
                    height_min=self.p['obstacle_height_min_m'], height_max=self.p['obstacle_height_max_m'])
                grid = LocalCostmap(inflation=self.p['obstacle_inflation_m'])
                grid.update(points)
                # Radius checks also stop rotation near visible side obstacles. The
                # forward sweep extends to .45 m plus footprint inflation; a 1 m
                # interaction target must not be treated as a wall 1.3 m away.
                nearest = float(np.min(np.hypot(points[:, 0], points[:, 1]))) if len(points) else math.inf
                safety = Safety(True, min(grid.front_distance(), nearest),
                                not any(grid.collision_at(x, 0.) for x in (.25,.35,.45)))
                with self.guard:
                    if not self.done and time.monotonic() < valid_until:
                        self.safety, self.safety_stamp, self.safety_until = safety, stamp, valid_until
                        self.safety_reason = 'valid_depth'
            except Exception as exc:
                with self.guard:
                    self.safety, self.safety_until = Safety(), -math.inf
                    self.safety_reason = str(exc)[:160]

    def motion_permitted(self, now):
        return bool(self.p['enable_motion'] and now < self.control_until and
                    -.05 <= (self.ros_ns()-self.control_state.get('stamp_ns',0))/1e9 <= .50 and
                    self.control_state.get('automatic_source') == 'PERSON' and
                    self.control_state.get('mode') == 'AUTO')

    def tick(self):
        now = time.monotonic()
        with self.guard:
            permitted = self.motion_permitted(now)
            if self.last_motion_permission and not permitted and self.core.active:
                self.core._terminate('cancelled', 'motion_authority_lost')
            self.last_motion_permission = permitted
            safety = self.safety if (now < self.safety_until and
                -.05 <= (self.ros_ns()-self.safety_stamp)/1e9 <= self.p['max_sensor_age_s']) else Safety()
            linear, angular = self.core.tick(now, safety)
            self.publish(linear, angular)

    def publish(self, linear, angular):
        now, ros_now = time.monotonic(), self.ros_ns()
        state = self.core.snapshot(now)
        permitted = self.motion_permitted(now)
        state.update(schema='wla-person-status/v1', stamp_ns=ros_now,
            follow_session=self.follow_session,
            base_frame=self.p['base_frame'], candidate_only=not self.p['enable_motion'],
            motion_permitted=permitted and state['enabled'],
            linear_mps=linear, angular_rps=angular, safety_reason=self.safety_reason,
            safety_fresh=now < self.safety_until)
        state['fault'] = bool(state['request_id'] and state['state'] == 'cancelled' and
            state['reason'] in ('detector_generation_changed', 'detection_disabled',
                               'motion_authority_revoked', 'motion_authority_lost'))
        self.status_pub.publish(String(data=json.dumps(state, allow_nan=False)))
        preview = Twist()
        preview.linear.x, preview.angular.z = float(linear), float(angular)
        self.cmd_pub.publish(preview)
        # The final mux independently checks source, mode, task, source age and limits.
        actual = (linear, angular) if permitted and state['enabled'] else (0., 0.)
        self.leased_pub.publish(String(data=json.dumps(dict(schema='wla-person-candidate/v1',
            follow_session=self.follow_session,
            request_id=state['request_id'], stamp_ns=ros_now,
            linear=float(actual[0]), angular=float(actual[1])), allow_nan=False)))

    def close(self):
        with self.guard:
            self.done = True
            self.core._terminate('stopped', 'node_shutdown')
            if self.context.ok():
                self.publish(0., 0.)
        self.event.set()
        self.worker.join(timeout=2.)
        if self.context.ok():
            self.publish(0., 0.)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PersonFollow()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
