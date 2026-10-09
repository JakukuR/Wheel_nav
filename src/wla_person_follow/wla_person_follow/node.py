"""R680 D455 person follow node. Only publishes an isolated candidate Twist."""
import json
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool, Empty, Int32, String
from tf2_ros import Buffer, TransformListener

from .camera import DepthFilter, color_array, depth_meters
from .controller import FollowController
from .costmap import LocalCostmap
from .depth_proc import points_to_map, rotation_from_quaternion, target_pose
from .detector import PersonDetector
from .target_lock import Observation, TargetLock


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def base_to_world(xy, odom):
    x, y, yaw = odom
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x + c*xy[0] - s*xy[1], y + s*xy[0] + c*xy[1]])


def world_to_base(xy, odom):
    x, y, yaw = odom
    dx, dy = xy[0]-x, xy[1]-y
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c*dx + s*dy, -s*dx + c*dy])


class PersonFollow(Node):
    def __init__(self):
        super().__init__('person_follow')
        defaults = {
            'rgb_topic': '/r680/d455/color/image_raw',
            'depth_topic': '/r680/d455/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/r680/d455/color/camera_info',
            'odom_topic': '/wheel/odom',
            'base_frame': 'r680_mapping_floor',
            'output_topic': '/r680/person_follow/cmd_vel_candidate',
            'model_path': '/home/orin/Wheel_Legged_Agent/models/yolo11n.pt',
            'confidence': .35, 'image_size': 640,
            'camera_sync_tolerance_s': .035, 'max_sensor_age_s': .30,
            'max_odom_age_s': .30, 'target_distance_m': 1.4,
            'max_linear_mps': .20, 'max_angular_rps': .45,
            'prediction_horizon_s': 1.2, 'prediction_linear_mps': .08,
            'obstacle_height_min_m': .10, 'obstacle_height_max_m': 1.50,
            'obstacle_inflation_m': .45, 'emergency_distance_m': .60,
            'camera_min_valid_fraction': .25, 'camera_front_min_valid_fraction': .35,
            'depth_min_m': .25,
            'depth_max_m': 5., 'enable_motion': False,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.p['output_topic'] != '/r680/person_follow/cmd_vel_candidate':
            raise ValueError('output must remain the isolated person-follow candidate topic')
        if not 0 < self.p['max_linear_mps'] <= .30 or not 0 < self.p['max_angular_rps'] <= .50:
            raise ValueError('configured speed exceeds R680 commissioning limits')
        if not 0 < self.p['prediction_horizon_s'] <= 1.5:
            raise ValueError('prediction horizon must be <= 1.5 s')
        self.detector = PersonDetector(self.p['model_path'], self.p['confidence'],
                                       self.p['image_size'])
        self.depth_filter = DepthFilter()
        self.locked = TargetLock(self.p['prediction_horizon_s'])
        self.controller = FollowController(
            self.p['target_distance_m'], self.p['max_linear_mps'],
            self.p['max_angular_rps'], self.p['prediction_linear_mps'],
            self.p['emergency_distance_m'])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.guard = threading.RLock()
        self.event = threading.Event()
        self.done = False
        self.rgb = None
        self.depth = None
        self.pending = None
        self.last_pair_stamp = -1.
        self.intrinsics = None
        self.odom = None
        self.odom_received = 0.
        self.odom_stamp = 0.
        self.grid = None
        self.grid_received = 0.
        self.grid_stamp = 0.
        self.valid_fraction = 0.
        self.front_valid_fraction = 0.
        self.observations = []
        self.observations_received = 0.
        self.inference_ms = 0.
        self.last_error_log = 0.
        self.enabled = bool(self.p['enable_motion'])
        self.last_tick = time.monotonic()

        self.cmd_pub = self.create_publisher(Twist, self.p['output_topic'], 1)
        self.status_pub = self.create_publisher(String, '/r680/person_follow/status', 5)
        self.box_pub = self.create_publisher(String, '/r680/person_follow/detections', 5)
        self.debug_pub = self.create_publisher(CompressedImage, '/r680/person_follow/debug/compressed', 1)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/r680/person_follow/obstacles', 1)
        self.target_pub = self.create_publisher(PointStamped, '/r680/person_follow/target_base', 1)
        self.create_subscription(Image, self.p['rgb_topic'], self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, self.p['depth_topic'], self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['odom_topic'], self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(Int32, '/r680/person_follow/select_track_id',
                                 self._select_id, 5)
        self.create_subscription(Point, '/r680/person_follow/select_pixel',
                                 self._select_pixel, 5)
        self.create_subscription(Empty, '/r680/person_follow/clear', self._clear, 5)
        self.create_subscription(Bool, '/r680/person_follow/enable', self._enable, 5)
        self.create_timer(.05, self._control)
        self.worker = threading.Thread(target=self._process_loop, name='person_follow_perception',
                                       daemon=True)
        self.worker.start()
        self.get_logger().warn(
            f"Person follow starts {'ENABLED' if self.enabled else 'DISABLED'}; "
            f"publishes candidate velocity only to {self.p['output_topic']}")

    def _stamp_fresh(self, stamp, maximum):
        if stamp <= 0:
            return False
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        return -.10 <= age <= maximum

    def _on_rgb(self, msg):
        with self.guard:
            self.rgb = (msg, time.monotonic())
            self._pair()

    def _on_depth(self, msg):
        with self.guard:
            self.depth = (msg, time.monotonic())
            self._pair()

    def _pair(self):
        if self.rgb is None or self.depth is None:
            return
        rgb, rgb_time = self.rgb
        depth, depth_time = self.depth
        r_stamp = stamp_seconds(rgb.header.stamp)
        d_stamp = stamp_seconds(depth.header.stamp)
        if (abs(r_stamp - d_stamp) <= self.p['camera_sync_tolerance_s']
                and r_stamp > self.last_pair_stamp
                and rgb.width == depth.width and rgb.height == depth.height):
            self.pending = (rgb, depth, min(rgb_time, depth_time), r_stamp)
            self.last_pair_stamp = r_stamp
            self.event.set()

    def _on_info(self, msg):
        if msg.k[0] <= 0 or msg.k[4] <= 0:
            return
        with self.guard:
            self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _on_odom(self, msg):
        if msg.child_frame_id.lstrip('/') != self.p['base_frame']:
            return
        pose = msg.pose.pose
        values = (pose.position.x, pose.position.y, quaternion_yaw(pose.orientation))
        if not all(math.isfinite(v) for v in values):
            return
        with self.guard:
            self.odom = values
            self.odom_received = time.monotonic()
            self.odom_stamp = stamp_seconds(msg.header.stamp)

    def _odom_fresh(self, now):
        return (self.odom is not None
                and now - self.odom_received <= self.p['max_odom_age_s']
                and self._stamp_fresh(self.odom_stamp, self.p['max_odom_age_s']))

    def _select(self, candidates):
        with self.guard:
            if time.monotonic() - self.observations_received > .40 or not self._odom_fresh(time.monotonic()):
                self.get_logger().warn('selection rejected: observations or odometry stale')
                return
            if len(candidates) != 1 or not np.all(np.isfinite(candidates[0].world_xy)):
                self.get_logger().warn('selection rejected: expected exactly one visible valid-depth person')
                return
            self.locked.select(candidates[0], time.monotonic())
            self.controller.reset()
            self.get_logger().info(f'selected person track ID {candidates[0].track_id}')

    def _select_id(self, msg):
        with self.guard:
            candidates = [o for o in self.observations if o.track_id == msg.data]
        self._select(candidates)

    def _select_pixel(self, msg):
        with self.guard:
            candidates = [o for o in self.observations
                          if o.box[0] <= msg.x <= o.box[2] and o.box[1] <= msg.y <= o.box[3]]
        # Overlapping person boxes are ambiguous; require explicit ID or a new click.
        self._select(candidates)

    def _clear(self, _msg):
        with self.guard:
            self.locked.clear()
            self.enabled = False
            self.controller.reset()

    def _enable(self, msg):
        with self.guard:
            self.enabled = bool(msg.data)
            self.controller.reset()

    def _process_loop(self):
        while not self.done:
            self.event.wait(.1)
            if self.done:
                break
            with self.guard:
                item = self.pending
                self.pending = None
                self.event.clear()
                intrinsics = self.intrinsics
                odom = self.odom if self._odom_fresh(time.monotonic()) else None
            if item is None or intrinsics is None:
                continue
            rgb_msg, depth_msg, received, source_stamp = item
            if (time.monotonic() - received > self.p['max_sensor_age_s']
                    or not self._stamp_fresh(source_stamp, self.p['max_sensor_age_s'])):
                continue
            try:
                frame = color_array(rgb_msg)
                depth = self.depth_filter.apply(depth_meters(depth_msg))
                valid = (depth >= self.p['depth_min_m']) & (depth <= self.p['depth_max_m'])
                valid_fraction = float(np.mean(valid))
                h, w = valid.shape
                front_valid_fraction = float(np.mean(valid[int(h*.35):int(h*.80),
                                                          int(w*.30):int(w*.70)]))
                started = time.monotonic()
                tracks = self.detector.track(frame)
                inference_ms = (time.monotonic() - started) * 1000.
                self._publish_boxes(tracks, rgb_msg.header)
                self._publish_debug(frame, tracks, rgb_msg.header)
                transform = self.tf_buffer.lookup_transform(
                    self.p['base_frame'], rgb_msg.header.frame_id,
                    rclpy.time.Time(), timeout=Duration(seconds=.05))
                q = transform.transform.rotation
                rotation = rotation_from_quaternion((q.x, q.y, q.z, q.w))
                t = transform.transform.translation
                translation = np.array([t.x, t.y, t.z], dtype=np.float32)
                observations = []
                for track in tracks:
                    optical = target_pose(depth, track.box, intrinsics,
                                          self.p['depth_min_m'], self.p['depth_max_m'])
                    if optical is None:
                        continue
                    base = rotation @ optical + translation
                    if base[0] <= .15 or not np.all(np.isfinite(base)):
                        continue
                    world = (base_to_world(base[:2], odom) if odom is not None
                             else np.array([float('nan'), float('nan')]))
                    observations.append(Observation(track.track_id, track.box, track.appearance,
                                                    world, base[:2], track.confidence))
                with self.guard:
                    if odom is not None:
                        _, mode = self.locked.update(observations, time.monotonic())
                    else:
                        mode = self.locked.mode
                    exclusion = self.locked.last_box if odom is not None and mode == 'tracking' else None
                points = points_to_map(
                    depth, intrinsics, rotation, translation, exclusion=exclusion,
                    minimum=self.p['depth_min_m'], maximum=self.p['depth_max_m'],
                    height_min=self.p['obstacle_height_min_m'],
                    height_max=self.p['obstacle_height_max_m'])
                grid = LocalCostmap(inflation=self.p['obstacle_inflation_m'])
                grid.update(points)
                with self.guard:
                    self.grid = grid
                    self.grid_received = time.monotonic()
                    self.grid_stamp = source_stamp
                    self.valid_fraction = valid_fraction
                    self.front_valid_fraction = front_valid_fraction
                    self.observations = observations
                    self.observations_received = time.monotonic()
                    self.inference_ms = inference_ms
                self._publish_grid(grid, rgb_msg.header.stamp)
            except Exception as exc:
                if time.monotonic() - self.last_error_log > 5.:
                    self.get_logger().error(f'perception frame rejected: {exc}')
                    self.last_error_log = time.monotonic()
                with self.guard:
                    self.grid = None
                    self.valid_fraction = 0.
                    self.front_valid_fraction = 0.

    def _publish_boxes(self, observations, header):
        data = [{'id': o.track_id, 'bbox_xyxy': [round(v, 1) for v in o.box],
                 'confidence': round(o.confidence, 3),
                 'base_xy_m': None} for o in observations]
        msg = String()
        msg.data = json.dumps({'stamp': stamp_seconds(header.stamp), 'people': data})
        self.box_pub.publish(msg)

    def _publish_debug(self, frame, observations, header):
        if self.debug_pub.get_subscription_count() == 0:
            return
        with self.guard:
            selected = self.locked.track_id
            mode = self.locked.mode
        annotated = frame.copy()
        for obs in observations:
            x1, y1, x2, y2 = (int(v) for v in obs.box)
            color = (0, 255, 0) if obs.track_id == selected else (255, 255, 0)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, f'ID {obs.track_id} {obs.confidence:.2f}',
                        (x1, max(20, y1-8)), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
        cv2.putText(annotated, f'Target {selected} | {mode}', (12, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 2)
        ok, encoded = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if ok:
            msg = CompressedImage()
            msg.header = header
            msg.format = 'jpeg'
            msg.data = encoded.tobytes()
            self.debug_pub.publish(msg)

    def _publish_grid(self, grid, stamp):
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.p['base_frame']
        msg.info.resolution = grid.resolution
        msg.info.width = msg.info.height = grid.cells
        msg.info.origin.position.x = msg.info.origin.position.y = -grid.size/2
        msg.info.origin.orientation.w = 1.
        # Unknown cells stay unknown: a single forward camera cannot observe all 360 degrees.
        values = np.full(grid.occupied.shape, -1, dtype=np.int8)
        values[grid.occupied > 0] = 100
        msg.data = values.ravel().tolist()
        self.grid_pub.publish(msg)

    def _control(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        with self.guard:
            enabled = self.enabled
            sensor_ok = (self.grid is not None
                         and now - self.grid_received <= self.p['max_sensor_age_s']
                         and self._stamp_fresh(self.grid_stamp, self.p['max_sensor_age_s'])
                         and self.valid_fraction >= self.p['camera_min_valid_fraction']
                         and self.front_valid_fraction >= self.p['camera_front_min_valid_fraction'])
            odom_ok = self._odom_fresh(now)
            mode = self.locked.mode
            target_world = self.locked.position(now) if odom_ok else None
            target = world_to_base(target_world, self.odom) if target_world is not None else None
            if enabled and sensor_ok and odom_ok and mode in ('tracking', 'predicting'):
                linear, angular, reason = self.controller.command(
                    target, self.grid, dt, predicting=(mode == 'predicting'))
            else:
                self.controller.reset()
                linear, angular = 0., 0.
                reason = ('disabled' if not enabled else 'sensor_stale' if not sensor_ok
                          else 'odom_stale' if not odom_ok else mode)
            selected_id = self.locked.track_id
            missing = None if self.locked.last_seen is None else round(now-self.locked.last_seen, 2)
            valid_fraction = self.valid_fraction
            front_valid_fraction = self.front_valid_fraction
            inference_ms = self.inference_ms
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)
        if target is not None:
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = self.p['base_frame']
            point.point.x, point.point.y = float(target[0]), float(target[1])
            self.target_pub.publish(point)
        status = String()
        status.data = json.dumps({
            'mode': mode, 'selected_track_id': selected_id, 'enabled': enabled,
            'reason': reason, 'missing_s': missing,
            'target_base_xy_m': None if target is None else [round(float(v), 2) for v in target],
            'linear_mps': round(linear, 3), 'angular_rps': round(angular, 3),
            'camera_valid_fraction': round(valid_fraction, 3),
            'front_valid_fraction': round(front_valid_fraction, 3),
            'inference_ms': round(inference_ms, 1),
            'sensor_fresh': sensor_ok, 'odom_fresh': odom_ok,
        })
        self.status_pub.publish(status)

    def close(self):
        self.done = True
        self.event.set()
        self.worker.join(timeout=2.)
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PersonFollow()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
