#!/usr/bin/env python3
"""Sample D455 furniture detections into a map-bound GeoJSON sidecar.

This node never publishes navigation goals or motor commands. Object points are
depth surface candidates, not safe approach poses.
"""
import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO

FURNITURE = ('chair', 'dining table', 'couch', 'bed')


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def transform_point(point, tf):
    q = tf.rotation
    vector = np.array([q.x, q.y, q.z], dtype=float)
    p = np.array(point, dtype=float)
    rotated = p + 2 * (q.w * np.cross(vector, p) + np.cross(vector, np.cross(vector, p)))
    return (rotated + np.array([tf.translation.x, tf.translation.y,
                                tf.translation.z])).tolist()


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.' + path.name + '-', suffix='.tmp', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def color_array(msg):
    channels = {'rgb8': 3, 'bgr8': 3}.get(msg.encoding)
    if channels is None or msg.step < msg.width * channels:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < msg.step * msg.height:
        return None
    frame = raw[:msg.step * msg.height].reshape(msg.height, msg.step)[:, :msg.width * channels]
    frame = frame.reshape(msg.height, msg.width, channels)
    return np.ascontiguousarray(frame[:, :, ::-1] if msg.encoding == 'rgb8' else frame)


def depth_array(msg):
    if (msg.encoding != '16UC1' or msg.step < msg.width * 2
            or len(msg.data) < msg.step * msg.height):
        return None
    dtype = '>u2' if msg.is_bigendian else '<u2'
    return np.ndarray((msg.height, msg.width), dtype=dtype,
                      buffer=msg.data, strides=(msg.step, 2))


class FurnitureSemantic(Node):
    def __init__(self, args):
        super().__init__('r680_furniture_semantic')
        self.args = args
        self.path = Path(args.output)
        if self.path.exists():
            self.map = json.loads(self.path.read_text())
            if self.map.get('map_id') != args.map_id or self.map.get('schema') != 'wla-semantic-map-v1':
                raise ValueError('semantic map identity mismatch')
        else:
            self.map = {'type': 'FeatureCollection', 'schema': 'wla-semantic-map-v1',
                        'map_id': args.map_id, 'revision': 0, 'features': []}
        self.model = YOLO(args.model, task='detect')
        self.class_ids = [key for key, name in self.model.names.items() if name in FURNITURE]
        if len(self.class_ids) != len(FURNITURE):
            raise ValueError('furniture class missing from model')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.depths = deque(maxlen=5)
        self.infos = deque(maxlen=5)
        self.healthy = False
        self.healthy_count = 0
        self.last_frame = None
        self.last_inference = 0.0
        self.last_save = 0.0
        self.status_pub = self.create_publisher(String, '/r680_nav/semantic_status', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/r680_nav/semantic_markers',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Image, '/r680/d455/color/image_raw', self.on_color,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/r680/d455/aligned_depth_to_color/image_raw',
                                 lambda msg: self.depths.append(msg), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/r680/d455/color/camera_info',
                                 lambda msg: self.infos.append(msg), qos_profile_sensor_data)
        self.create_subscription(Bool, '/r680_nav/localization_ready', self.on_health, 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'furniture map={args.map_id}, output={self.path}')

    def on_health(self, msg):
        self.healthy = msg.data
        self.healthy_count = self.healthy_count + 1 if msg.data else 0

    def on_color(self, msg):
        self.last_frame = msg

    def transform(self, source_frame, image_stamp):
        requested = Time.from_msg(image_stamp)
        if not self.tf.can_transform('map', source_frame, requested):
            return None
        return self.tf.lookup_transform('map', source_frame, requested).transform

    def write(self):
        self.map['revision'] += 1
        atomic_json(self.path, self.map)
        self.last_save = time.monotonic()

    def record_home(self):
        if self.args.mark_home != 'true' or self.healthy_count < 10:
            return
        if any(f.get('properties', {}).get('role') == 'home' for f in self.map['features']):
            return
        if not self.tf.can_transform('map', 'r680_mapping_floor', Time()):
            return
        tf = self.tf.lookup_transform('map', 'r680_mapping_floor', Time()).transform
        self.map['features'].append({
            'type': 'Feature', 'id': 'home',
            'geometry': {'type': 'Point', 'coordinates':
                         [tf.translation.x, tf.translation.y, tf.translation.z]},
            'properties': {'role': 'home', 'label': '出生点', 'yaw_rad': yaw(tf.rotation),
                           'frame_id': 'map', 'source': 'mapping_start_tf',
                           'recorded_at': datetime.now(timezone.utc).isoformat()},
        })
        self.write()
        self.get_logger().info('recorded mapping start pose as home')

    def publish_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, feature in enumerate(self.map['features']):
            props = feature.get('properties', {})
            point = feature.get('geometry', {}).get('coordinates')
            if not isinstance(point, list) or len(point) != 3:
                continue
            home = props.get('role') == 'home'
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'home' if home else 'furniture'
            marker.id = index + 1
            marker.action = Marker.ADD
            marker.type = Marker.SPHERE
            marker.pose.position.x, marker.pose.position.y = point[:2]
            marker.pose.position.z = 0.12 if home else max(0.12, point[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 0.24 if home else 0.18
            marker.scale.z = 0.12
            marker.color.a = 0.95 if home else 0.75
            if home:
                marker.color.g = 0.85
            elif props.get('status') == 'confirmed':
                marker.color.b = 0.95
            else:
                marker.color.r = 1.0
                marker.color.g = 0.65
            markers.markers.append(marker)
            label = Marker()
            label.header = marker.header
            label.ns = 'semantic_labels'
            label.id = index + 1
            label.action = Marker.ADD
            label.type = Marker.TEXT_VIEW_FACING
            label.pose = marker.pose
            label.pose.position.z += 0.22
            label.scale.z = 0.17
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = '出生点' if home else props.get('label', '?') + (
                '' if props.get('status') == 'confirmed' else ' ?')
            markers.markers.append(label)
        self.marker_pub.publish(markers)

    def tick(self):
        status = {'map_id': self.args.map_id, 'healthy': self.healthy,
                  'home': any(f.get('properties', {}).get('role') == 'home'
                              for f in self.map['features']),
                  'furniture': sum(f.get('properties', {}).get('role') == 'furniture'
                                   for f in self.map['features'])}
        self.status_pub.publish(String(data=json.dumps(status)))
        if not hasattr(self, 'last_marker_pub') or time.monotonic() - self.last_marker_pub > 1.0:
            self.publish_markers()
            self.last_marker_pub = time.monotonic()
        if not self.healthy:
            return
        self.record_home()
        if self.healthy_count < 10 or self.last_frame is None or not self.depths or not self.infos:
            return
        if time.monotonic() - self.last_inference < 1.0 / self.args.hz:
            return
        image = self.last_frame
        self.last_frame = None
        depth = min(self.depths, key=lambda x: abs(stamp(x) - stamp(image)))
        info = min(self.infos, key=lambda x: abs(stamp(x) - stamp(image)))
        if abs(stamp(depth) - stamp(image)) > 0.08 or abs(stamp(info) - stamp(image)) > 0.25:
            return
        frame = color_array(image)
        depth_data = depth_array(depth)
        if frame is None or depth_data is None or frame.shape[:2] != depth_data.shape:
            return
        source_frame = info.header.frame_id or 'd455_color_optical_frame'
        tf = self.transform(source_frame, image.header.stamp)
        if tf is None or info.k[0] <= 0 or info.k[4] <= 0:
            return
        self.last_inference = time.monotonic()
        try:
            result = self.model.predict(frame, imgsz=416, conf=0.35,
                                        classes=self.class_ids, max_det=20,
                                        verbose=False)[0]
        except Exception as exc:
            self.get_logger().warning(f'furniture inference failed: {exc}')
            return
        changed = False
        view = [tf.translation.x, tf.translation.y]
        for box in result.boxes:
            label = self.model.names[int(box.cls.item())]
            x1, y1, x2, y2 = [float(x) for x in box.xyxy[0].tolist()]
            width, height = x2 - x1, y2 - y1
            if width < 12 or height < 12:
                continue
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            radius = max(3, int(min(width, height) * 0.08))
            u, v = int(cx), int(cy)
            patch = depth_data[max(0, v-radius):min(depth.height, v+radius+1),
                               max(0, u-radius):min(depth.width, u+radius+1)]
            valid = patch[(patch >= 250) & (patch <= 4000)]
            if valid.size < 10:
                continue
            distance_m = float(np.median(valid)) * .001
            xyz_camera = [(cx - info.k[2]) * distance_m / info.k[0],
                          (cy - info.k[5]) * distance_m / info.k[4], distance_m]
            xyz_map = transform_point(xyz_camera, tf)
            matched = None
            for feature in self.map['features']:
                props = feature.get('properties', {})
                if props.get('role') == 'furniture' and props.get('label') == label:
                    old = feature['geometry']['coordinates']
                    if math.dist(old[:2], xyz_map[:2]) <= 0.6:
                        matched = feature
                        break
            confidence = float(box.conf.item())
            if matched is None:
                matched = {'type': 'Feature', 'id': f'furniture-{len(self.map["features"])+1:04d}',
                           'geometry': {'type': 'Point', 'coordinates': xyz_map},
                           'properties': {'role': 'furniture', 'label': label,
                                          'confidence': confidence, 'observations': 0,
                                          'viewpoints_xy': [], 'status': 'tentative',
                                          'position_method': 'rgbd_bbox_surface_sample'}}
                self.map['features'].append(matched)
            props = matched['properties']
            count = props['observations']
            matched['geometry']['coordinates'] = [
                (count * old + fresh) / (count + 1)
                for old, fresh in zip(matched['geometry']['coordinates'], xyz_map)]
            props['observations'] = count + 1
            props['confidence'] = max(confidence, props['confidence'])
            props['last_seen'] = datetime.now(timezone.utc).isoformat()
            views = props['viewpoints_xy']
            if not any(math.dist(view, previous) < 0.25 for previous in views):
                views.append(view)
            if props['observations'] >= 3 and len(views) >= 2:
                props['status'] = 'confirmed'
            changed = True
        if changed:
            self.write()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--hz', type=float, default=1.0)
    parser.add_argument('--mark-home', choices=('true', 'false'), default='false')
    args = parser.parse_args()
    if not 0.1 <= args.hz <= 5:
        parser.error('--hz must be in [0.1, 5]')
    rclpy.init()
    node = FurnitureSemantic(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
