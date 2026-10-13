"""Relative approach/follow service with local depth safety and leased candidates."""
from collections import deque
import json
import math
import threading
import time
import uuid

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
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
from .local_avoidance import LocalAvoidance, scene_from_points
from .local_memory import LocalMemory, LocalMotion, camera_in_reference
from .target_tracking import TargetTracker


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class PersonFollow(Node):
    NODE_NAME = 'person_follow'
    PREFIX = '/r680/person_follow'
    SOURCE = 'PERSON'
    COMMAND_TYPE = PersonCommand
    EXTRA_DEFAULTS = {}

    def __init__(self):
        super().__init__(self.NODE_NAME)
        defaults = dict(
            relative_topic='/r680/perception/relative_objects',
            detection_status_topic='/r680_nav/perception/status',
            depth_topic='/r680/d455/aligned_depth_to_color/image_raw',
            camera_info_topic='/r680/d455/color/camera_info',
            base_frame='r680_mapping_floor', max_sensor_age_s=.30,
            follow_distance_m=1.4, approach_distance_m=.95,
            max_linear_mps=.20, max_angular_rps=.45, emergency_distance_m=.60,
            obstacle_inflation_m=.45, obstacle_height_min_m=.01,
            obstacle_height_max_m=.50, depth_min_m=.25, depth_max_m=5.,
            camera_min_valid_fraction=.25, camera_front_min_valid_fraction=.35,
            target_prediction_horizon_s=2., target_prediction_max_std_m=.75,
            lost_target_stop_distance_m=.35,
            enable_motion=False)
        defaults.update(self.EXTRA_DEFAULTS)
        self.p = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.local_planner = LocalAvoidance()
        self.target_tracker = TargetTracker(self.p['target_prediction_horizon_s'], self.p['target_prediction_max_std_m'])
        self.target_estimate = None
        self.local_motion = LocalMotion()
        self.local_memory = LocalMemory()
        self.scene_generation = -1
        self.camera_reference_position = None
        self.camera_reference_rotation = None
        self.scene = None
        self.core = self.make_core()
        if (not 0 < self.p['depth_min_m'] < self.p['depth_max_m'] <= 10 or
                not 0 < self.p['obstacle_height_min_m'] < self.p['obstacle_height_max_m'] <= 1.5 or
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
        self.cmd_pub = self.create_publisher(Twist, self.PREFIX + '/cmd_vel_candidate', 1)
        self.leased_pub = self.create_publisher(String, self.PREFIX + '/command_candidate', 1)
        self.status_pub = self.create_publisher(String, self.PREFIX + '/status', 5)
        self.path_pub = self.create_publisher(Path, self.PREFIX + '/local_path', 1)
        self.grid_pub = self.create_publisher(OccupancyGrid, self.PREFIX + '/local_obstacles', 1)
        self.create_subscription(String, self.p['relative_topic'], self.on_relative, 1)
        self.create_subscription(String, self.p['detection_status_topic'], self.on_detection_status, 1)
        self.create_subscription(Image, self.p['depth_topic'], self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.on_info, qos_profile_sensor_data)
        self.create_subscription(String, '/r680_base/control_state', self.on_control_state, 5)
        self.create_subscription(Odometry, '/wheel/odom', self.on_wheel_motion, qos_profile_sensor_data)
        self.create_service(self.COMMAND_TYPE, self.PREFIX + '/command', self.on_command)
        self.worker = threading.Thread(target=self.depth_worker, name='relative-depth-safety', daemon=True)
        self.worker.start()
        self.create_timer(.05, self.tick)
        self.get_logger().info('Relative person service ready; waiting for explicit Agent command and detection. '
            + ('Motion candidates enabled; final mux authorization remains required.' if self.p['enable_motion']
               else 'Preview mode: leased motion candidates remain zero.'))

    def make_core(self):
        return RelativeFollower(self.p['base_frame'], self.p['max_sensor_age_s'],
            self.p['follow_distance_m'], self.p['approach_distance_m'],
            self.p['max_linear_mps'], self.p['max_angular_rps'], self.p['emergency_distance_m'], self.local_planner, require_estimate=True,
            lost_target_stop_distance=self.p['lost_target_stop_distance_m'])

    def on_projected_depth(self, message, info, depth, rotation, translation, points):
        pass

    def ros_ns(self):
        return self.get_clock().now().nanoseconds

    def on_relative(self, message):
        with self.guard:
            if not self.core.receive(message.data, time.monotonic(), self.ros_ns()):
                self.publish(0., 0.)
            elif not self.core.active:
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
            if (data.get('automatic_source') not in ('NAV', 'PERSON', 'OBJECT') or
                    type(data.get('source_generation')) is not int or
                    type(data.get(self.SOURCE.lower() + '_revoke_generation')) is not int or
                    data['source_generation'] < 0 or data[self.SOURCE.lower() + '_revoke_generation'] < 0 or
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
                    data[self.SOURCE.lower() + '_revoke_generation'] < previous[self.SOURCE.lower() + '_revoke_generation']):
                    return
                if previous and not same_session:
                    if len(self.retired_control_sessions) >= 1024:
                        return
                    self.retired_control_sessions.add(previous['control_session'])
                revoked = bool(previous) and (
                    not same_session and previous['automatic_source'] == self.SOURCE or
                    same_session and data[self.SOURCE.lower() + '_revoke_generation'] != previous[self.SOURCE.lower() + '_revoke_generation'] or
                    previous['automatic_source'] == self.SOURCE and data['automatic_source'] != self.SOURCE or
                    previous['automatic_source'] == self.SOURCE and previous['mode'] == 'AUTO' and data['mode'] != 'AUTO')
                self.control_state, self.control_seen = data, time.monotonic()
                self.control_until = self.control_seen + max(0., .50-max(0., source_age))
                if revoked and self.core.active:
                    self.core._terminate('cancelled', 'motion_authority_revoked')
                    self.publish(0., 0.)
        except (ValueError, TypeError, KeyError):
            return

    def on_wheel_motion(self, message):
        with self.guard:
            velocity = message.twist.twist
            if message.child_frame_id != self.p['base_frame']:
                self.local_motion.samples.clear()
                self.local_motion.generation += 1
            else:
                self.local_motion.update(stamp_ns(message)/1e9,
                    (velocity.linear.x, velocity.linear.y, velocity.angular.z), self.ros_ns()/1e9)
            if self.scene_generation != self.local_motion.generation:
                self.local_memory.reset()
                self.camera_reference_position = self.camera_reference_rotation = None
                self.scene = None
                self.safety_until = -math.inf

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
                    height_min=-3., height_max=3., stride=2)
                self.on_projected_depth(message,info,depth,rotation,translation,points)
                scene = scene_from_points(points, translation[:2],
                    radius=max(self.p['obstacle_inflation_m'], self.p['emergency_distance_m']+.10),
                    height_min=self.p['obstacle_height_min_m'], height_max=self.p['obstacle_height_max_m'])
                with self.guard:
                    pose = self.local_motion.pose_at(stamp/1e9)
                    if pose is None or not self.local_motion.healthy(self.ros_ns()/1e9):
                        raise ValueError('waiting_for_fresh_wheel_motion')
                    generation = self.local_motion.generation
                    scene = self.local_memory.apply(scene, pose, stamp/1e9, generation)
                    camera_rotation, camera_position = camera_in_reference(pose,rotation,translation)
                points = points[((points[:, 2] >= self.p['obstacle_height_min_m']) &
                                 (points[:, 2] <= self.p['obstacle_height_max_m'])) | (points[:, 2] < -.10)]
                grid = LocalCostmap(inflation=self.p['obstacle_inflation_m'])
                grid.update(points)
                # Radius checks also stop rotation near visible side obstacles. The
                # forward sweep extends to .45 m plus footprint inflation; a 1 m
                # interaction target must not be treated as a wall 1.3 m away.
                nearest = float(np.min(np.hypot(points[:, 0], points[:, 1]))) if len(points) else math.inf
                safety = Safety(True, min(grid.front_distance(), nearest),
                                not any(grid.collision_at(x, 0.) for x in (.25,.35,.45)))
                with self.guard:
                    if (not self.done and time.monotonic() < valid_until and
                            generation == self.local_motion.generation):
                        self.scene = scene
                        self.scene_generation = generation
                        self.camera_reference_position = camera_position.tolist()
                        self.camera_reference_rotation = camera_rotation.ravel().tolist()
                        self.safety, self.safety_stamp, self.safety_until = safety, stamp, valid_until
                        self.safety_reason = 'valid_depth'
            except Exception as exc:
                with self.guard:
                    self.scene = None
                    self.safety, self.safety_until = Safety(), -math.inf
                    self.safety_reason = str(exc)[:160]

    def update_target_estimate(self):
        now_ros=self.ros_ns()/1e9
        core=self.core
        key=(core.request_id,core.target_id,core.session,core.epoch,self.local_motion.generation)
        tracker=self.target_tracker
        if not core.active or not self.local_motion.healthy(now_ros):
            tracker.reset();self.target_estimate=None
            return None
        if key!=tracker.key:tracker.reset(key)
        row=core.objects.get(core.target_id)
        if core.last_stamp!=tracker.frame_stamp:
            if row is not None and row['detected'] and row['label']==core.target_label and row['yaw'] is not None:
                pose=self.local_motion.pose_at(core.last_stamp/1e9)
                if pose is not None:
                    tracker.update(core.last_stamp/1e9,row['yaw'],row['distance'],pose)
                    tracker.frame_stamp=core.last_stamp
            else:
                tracker.frame_stamp=core.last_stamp
        pose=self.local_motion.pose_at(now_ros)
        self.target_estimate=(tracker.estimate(now_ros,pose,core.target_id,core.last_stamp/1e9)
                              if pose is not None else None)
        return self.target_estimate

    def motion_permitted(self, now):
        return bool(self.p['enable_motion'] and now < self.control_until and
                    -.05 <= (self.ros_ns()-self.control_state.get('stamp_ns',0))/1e9 <= .50 and
                    self.control_state.get('automatic_source') == self.SOURCE and
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
            motion_ready = (self.local_motion.healthy(self.ros_ns()/1e9) and
                            self.scene_generation == self.local_motion.generation)
            control_ros=self.ros_ns()/1e9
            control_pose=self.local_motion.pose_at(control_ros) if motion_ready else None
            scene=(self.local_memory.project(control_pose,control_ros,self.local_motion.generation)
                   if safety.valid and motion_ready and control_pose is not None else None)
            self.scene=scene
            if not motion_ready:
                self.safety_reason = 'waiting_for_fresh_wheel_motion'
            estimate = self.update_target_estimate()
            linear, angular = self.core.tick(now, safety, scene, estimate)
            # Planning must not extend a nearly expired frame or task lease.
            finished = time.monotonic()
            if (not self.local_motion.healthy(self.ros_ns()/1e9) or
                    (estimate is not None and estimate.age+finished-now > self.p['target_prediction_horizon_s']) or
                    finished >= self.safety_until or finished >= self.core.expires_at or
                    finished >= self.core.lease_until or
                    not -.05 <= (self.ros_ns()-self.safety_stamp)/1e9 <= self.p['max_sensor_age_s']):
                linear, angular = self.core._zero()
            self.publish(linear, angular)

    def publish(self, linear, angular):
        now, ros_now = time.monotonic(), self.ros_ns()
        state = self.core.snapshot(now)
        permitted = self.motion_permitted(now)
        state.update(tracking_reference='wheel_integrated_local',
            reference_generation=self.local_motion.generation,
            camera_reference_position=self.camera_reference_position,
            camera_reference_rotation=self.camera_reference_rotation,
            camera_reference_stamp_ns=self.safety_stamp if self.camera_reference_position is not None else None)
        state.update(target_tracking=('measured' if self.target_estimate.observed else 'predicted') if self.target_estimate else 'unavailable',
            target_prediction_age_s=self.target_estimate.age if self.target_estimate else None,
            target_position_std_m=self.target_estimate.std if self.target_estimate else None,
            target_filter_reason=self.target_tracker.filter.reason,
            last_seen_yaw=self.target_estimate.last_seen_yaw if self.target_estimate else None,
            last_seen_distance=self.target_estimate.last_seen_distance if self.target_estimate else None)
        state.update(schema='wla-' + self.SOURCE.lower() + '-status/v1', stamp_ns=ros_now,
            follow_session=self.follow_session,
            base_frame=self.p['base_frame'], candidate_only=not self.p['enable_motion'],
            motion_permitted=permitted and state['enabled'],
            linear_mps=linear, angular_rps=angular, safety_reason=self.safety_reason,
            safety_fresh=now < self.safety_until,
            planner='robot_frame_depth_local',
            local_motion_fresh=self.local_motion.healthy(ros_now/1e9),
            obstacle_memory_s=self.local_memory.ttl,
            planning_clearance_m=max(self.p['obstacle_inflation_m'], self.p['emergency_distance_m']+.10),
            local_plan_reason=self.local_planner.last.reason,
            detour_side=self.local_planner.side,
            local_path_points=len(self.local_planner.last.path),
            obstacle_height_min_m=self.p['obstacle_height_min_m'],
            obstacle_height_max_m=self.p['obstacle_height_max_m'])
        state['fault'] = bool(state['request_id'] and state['state'] == 'cancelled' and
            state['reason'] in ('detector_generation_changed', 'detection_disabled',
                               'motion_authority_revoked', 'motion_authority_lost'))
        self.status_pub.publish(String(data=json.dumps(state, allow_nan=False)))
        self.publish_local_geometry(state, ros_now)
        preview = Twist()
        preview.linear.x, preview.angular.z = float(linear), float(angular)
        self.cmd_pub.publish(preview)
        # The final mux independently checks source, mode, task, source age and limits.
        actual = (linear, angular) if permitted and state['enabled'] else (0., 0.)
        self.leased_pub.publish(String(data=json.dumps(dict(schema='wla-' + self.SOURCE.lower() + '-candidate/v1',
            follow_session=self.follow_session,
            request_id=state['request_id'], stamp_ns=ros_now,
            linear=float(actual[0]), angular=float(actual[1])), allow_nan=False)))

    def publish_local_geometry(self, state, ros_now):
        path = Path()
        path.header.frame_id = self.p['base_frame']
        stamp = ros_now  # Local path/grid are reprojected at control time.
        path.header.stamp.sec, path.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        if state['enabled'] and state['safety_fresh'] and state['observations_fresh'] and state['state'] in ('following', 'approaching') and state['reason'] != 'target_prediction_align':
            for x, y in self.local_planner.last.path:
                pose = PoseStamped(); pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.w = 1.
                path.poses.append(pose)
        self.path_pub.publish(path)
        grid = OccupancyGrid(); grid.header = path.header
        if self.scene is not None and state['safety_fresh']:
            scene = self.scene
            grid.info.resolution = scene.resolution
            grid.info.width = grid.info.height = scene.free.shape[0]
            grid.info.origin.position.x = grid.info.origin.position.y = -(scene.center+.5)*scene.resolution
            grid.info.origin.orientation.w = 1.
            values = np.full(scene.free.shape, -1, dtype=np.int8)
            values[scene.observed] = 0
            values[scene.occupied] = 100
            grid.data = values.ravel().tolist()
        self.grid_pub.publish(grid)

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
