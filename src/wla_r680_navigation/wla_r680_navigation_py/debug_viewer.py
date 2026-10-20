"""Read-only ROS subscriptions -> two independent, timestamped RGB debug pages."""
import argparse
import base64
import gzip
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path as FilePath
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory

from .debug_projection import Camera, FrameCache, matrix, transform, voxel_edges, path_edges, relative_lookup


def stamp(msg):
    return msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec


def tf_matrix(t):
    q, p = t.rotation, t.translation
    return matrix([q.x, q.y, q.z, q.w], [p.x, p.y, p.z])


class DebugViewer(Node):
    def __init__(self, args):
        super().__init__('r680_perception_debug', enable_rosout=False, start_parameter_services=False)
        self.args = args
        self.frames = FrameCache()
        self.infos = deque(maxlen=30)
        self.raw = deque(maxlen=12)
        self.relative = deque(maxlen=12)
        self.sources, self.statuses = {}, {}
        self.histories = {}
        self.image_error = ''
        self.last_clients = {name: 0. for name in ('detections', 'planning')}
        self.wire_snapshots = {}
        self.jpeg_cache = None
        self.tf = Buffer(cache_time=Duration(seconds=15))
        self.listener = TransformListener(self.tf, self)
        self.snapshot = {name: self.encode({'view': name, 'reason': 'waiting_for_rgb', 'image': None})
                         for name in ('detections', 'planning')}
        self.snapshot_time = {name: time.monotonic() for name in self.snapshot}
        self.create_subscription(Image, args.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(String, '/r680/perception/detections', lambda m: self.on_json(m, self.raw), qos_profile_sensor_data)
        self.create_subscription(String, '/r680/perception/relative_objects', lambda m: self.on_json(m, self.relative), qos_profile_sensor_data)
        for key, topic in [('detector', '/r680_nav/perception/status'),
                           ('person', '/r680/person_follow/status'), ('object', '/r680/object_follow/status')]:
            self.create_subscription(String, topic, lambda m, k=key: self.on_status(k, m), qos_profile_sensor_data)
        self.specs = {
            'voxels': (PointCloud2, args.voxel_topic, 1., '#54e0ce', '当前帧局部体素' if args.mode == 'local' else '3D 确认体素'),
            'points': (PointCloud2, args.points_topic, .8, '#b5d5ff', '实时障碍点'),
            'person_path': (Path, '/r680/person_follow/local_path', 1., '#54e0ce', '人物路径'),
            'object_path': (Path, '/r680/object_follow/local_path', 1., '#ffbe69', '物品路径'),
            'nav_path': (Path, args.nav_path_topic, 5., '#b6a1ff', 'Nav2 全局路径'),
            'local_path': (Path, args.local_path_topic, 1., '#ff8db5', 'Nav2 局部轨迹'),
            'person_grid': (OccupancyGrid, '/r680/person_follow/local_obstacles', 1., '#71a7de', '人物二维障碍栅格'),
            'object_grid': (OccupancyGrid, '/r680/object_follow/local_obstacles', 1., '#c18a61', '物品二维障碍栅格'),
        }
        if args.mode == 'local':
            self.specs.pop('nav_path')
            self.specs.pop('local_path')
        for key, (typ, topic, _, _, _) in self.specs.items():
            self.create_subscription(typ, topic, lambda m, k=key: self.on_geometry(k, m), qos_profile_sensor_data)
        self.create_timer(.2, self.render)

    @staticmethod
    def encode(value):
        return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf8')

    def on_image(self, msg):
        if not 0 < msg.width*msg.height <= 2_000_000 or len(msg.data) > 10_000_000 or stamp(msg) <= 0:
            self.image_error = 'invalid_or_oversized_rgb'
            return
        self.image_error = ''
        self.frames.put((stamp(msg), msg.header.frame_id), (msg, time.monotonic()), len(msg.data))

    def on_info(self, msg):
        if len(msg.d) <= 14:
            self.infos.append((msg, time.monotonic()))

    def on_json(self, msg, target):
        try:
            if len(msg.data) > 256_000:
                return
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get('stamp_ns'), int):
                return
            if not isinstance(data.get('objects'), list) or len(data['objects']) > 1000:
                return
            target.append((data, time.monotonic()))
        except (ValueError, TypeError):
            pass

    def on_status(self, key, msg):
        if len(msg.data) < 32000:
            try:
                data = json.loads(msg.data)
                if isinstance(data, dict):
                    self.statuses[key] = (data, time.monotonic())
            except ValueError:
                pass

    def on_geometry(self, key, msg):
        oversized = ((isinstance(msg, PointCloud2) and (len(msg.data) > 4_000_000 or msg.width*msg.height > 100_000)) or
                     (isinstance(msg, Path) and len(msg.poses) > 10000) or
                     (isinstance(msg, OccupancyGrid) and len(msg.data) > 250000))
        self.sources[key] = (None if oversized else msg, time.monotonic())
        self.histories.setdefault(key, deque(maxlen=8)).append(self.sources[key])

    def fresh(self, source_ns, received, max_age):
        age = (self.get_clock().now().nanoseconds-source_ns)*1e-9
        return source_ns > 0 and -.05 <= age <= max_age and time.monotonic()-received <= max_age

    def image_data(self, item):
        msg, received = item
        if not self.fresh(stamp(msg), received, .8):
            raise ValueError('rgb_stale')
        cache_key=(stamp(msg),msg.header.frame_id,id(msg))
        if self.jpeg_cache and self.jpeg_cache[0]==cache_key:
            cached=self.jpeg_cache[1].copy()
            cached['age_ms']=round((self.get_clock().now().nanoseconds-stamp(msg))*1e-6)
            return cached
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(msg.encoding)
        if not channels or msg.step < msg.width*channels or len(msg.data) != msg.step*msg.height:
            raise ValueError('unsupported_rgb_encoding_or_stride')
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = rows[:, :msg.width*channels].reshape(msg.height, msg.width, channels)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'rgba8':
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        elif msg.encoding == 'bgra8':
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if not ok:
            raise ValueError('jpeg_encode_failed')
        result = {'image': base64.b64encode(jpg).decode('ascii'), 'width': msg.width, 'height': msg.height,
                'stamp_ns': str(stamp(msg)), 'frame_id': msg.header.frame_id,
                'age_ms': round((self.get_clock().now().nanoseconds-stamp(msg))*1e-6)}
        self.jpeg_cache=(cache_key,result)
        return result

    def camera(self, msg):
        candidates = [(i, t) for i, t in self.infos if i.header.frame_id == msg.header.frame_id and
                      (i.width, i.height) == (msg.width, msg.height)]
        if not candidates:
            raise ValueError('waiting_for_matching_camera_info')
        info, received = min(candidates, key=lambda it: abs(stamp(it[0])-stamp(msg)))
        if abs(stamp(info)-stamp(msg)) > 300_000_000 or time.monotonic()-received > 1.:
            raise ValueError('camera_info_stale_or_time_mismatch')
        if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset or info.roi.do_rectify:
            raise ValueError('camera_roi_or_binning_unsupported')
        if info.roi.width not in (0, msg.width) or info.roi.height not in (0, msg.height):
            raise ValueError('camera_roi_size_mismatch')
        return Camera(msg.width, msg.height, info.k, info.d, info.distortion_model)

    def source_transform(self, msg, img):
        if not msg.header.frame_id:
            raise ValueError('empty_source_frame')
        if self.args.mode == 'local':
            # Local geometry is robot-relative, not a persistent world snapshot.
            # No fake odom/map transform: only rigid camera/base extrinsics and a
            # bounded near-synchronous match. Motion within this interval is NOT compensated.
            if msg.header.frame_id not in (self.args.base_frame, img.header.frame_id):
                raise ValueError('local_mode_requires_robot_or_camera_frame')
            if abs(stamp(msg)-stamp(img)) > 30_000_000:
                raise ValueError('local_rgb_geometry_skew_over_30ms')
            t = self.tf.lookup_transform(img.header.frame_id, msg.header.frame_id, Time(nanoseconds=stamp(img)))
            return tf_matrix(t.transform)
        # No Time(0)/latest fallback. Cross-time geometry follows the fixed odom frame.
        if stamp(msg) == stamp(img):
            t = self.tf.lookup_transform(img.header.frame_id, msg.header.frame_id, Time(nanoseconds=stamp(img)))
        else:
            t = self.tf.lookup_transform_full(img.header.frame_id, Time(nanoseconds=stamp(img)),
                msg.header.frame_id, Time(nanoseconds=stamp(msg)), self.args.fixed_frame)
        return tf_matrix(t.transform)

    def detections(self):
        result = {'view': 'detections', 'image': None, 'objects': [], 'reason': 'waiting_for_rgb'}
        selected = None
        # Once the producer changes session/epoch, do not display a previous identity generation.
        identity = tuple(self.raw[-1][0].get(k) for k in ('session', 'epoch')) if self.raw else None
        for raw, received in reversed(self.raw):
            if tuple(raw.get(k) for k in ('session', 'epoch')) != identity:
                continue
            item = self.frames.get((raw.get('stamp_ns'), raw.get('frame_id')))
            if item and self.fresh(raw['stamp_ns'], received, .8):
                if (raw.get('width'), raw.get('height')) == (item[0].width, item[0].height):
                    selected = raw, item
                    break
        if selected:
            raw, item = selected
            result.update(self.image_data(item))
            distances = {}
            for rel, received in reversed(self.relative):
                if self.fresh(rel['stamp_ns'], received, .8):
                    distances = relative_lookup(raw, rel)
                    if distances:
                        break
            for obj in raw['objects']:
                box = obj.get('bbox_xyxy', [])
                if len(box) != 4 or not np.isfinite(box).all() or box[2] <= box[0] or box[3] <= box[1]:
                    continue
                conf = obj.get('confidence')
                if not isinstance(conf, (int, float)) or not np.isfinite(conf):
                    continue
                rel = distances.get(str(obj.get('track_id')), {})
                def number(key):
                    value = rel.get(key)
                    return value if isinstance(value, (int, float)) and np.isfinite(value) else None
                result['objects'].append({'bbox': box, 'label': str(obj.get('label', '?'))[:80],
                    'confidence': conf, 'track_id': obj.get('track_id'), 'distance': number('distance'), 'yaw': number('yaw')})
            result.update(reason='ok', session=raw.get('session'), epoch=raw.get('epoch'))
        elif self.frames.frames:
            result.update(self.image_data(next(reversed(self.frames.frames.values()))[0]))
            result['reason'] = 'waiting_for_exact_detection_frame'
        return result

    def geometry(self, key, msg, img, cam):
        mat = self.source_transform(msg, img)
        if isinstance(msg, PointCloud2):
            pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
            pts = pts[np.isfinite(pts).all(axis=1)]
            if key == 'voxels':
                # The producer stores representative samples, not voxel centers.
                edges, count = voxel_edges(pts, self.args.voxel_size, 600)
                return {'segments': cam.segments(transform(edges, mat)), 'count': count, 'total': len(pts)}
            pts = pts[::max(1, int(np.ceil(len(pts)/4000)))][:4000]
            return {'points': cam.points(transform(pts, mat)), 'count': len(pts)}
        if isinstance(msg, Path):
            if any(p.header.frame_id not in ('', msg.header.frame_id) for p in msg.poses):
                raise ValueError('mixed_path_frames')
            pts = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
            edges = path_edges(pts)
            return {'segments': cam.segments(transform(edges, mat)), 'count': len(msg.poses)}
        if msg.info.width*msg.info.height != len(msg.data) or not 0 < msg.info.resolution <= 1:
            if not msg.data:
                return {'segments': [], 'count': 0}
            raise ValueError('invalid_occupancy_grid')
        q, p = msg.info.origin.orientation, msg.info.origin.position
        origin = matrix([q.x, q.y, q.z, q.w], [p.x, p.y, p.z])
        rows, cols = np.where(np.array(msg.data).reshape(msg.info.height, msg.info.width) >= 50)
        stride = max(1, int(np.ceil(len(rows)/500)))
        cells = np.stack([cols[::stride], rows[::stride], np.zeros(len(rows[::stride]))], axis=1)
        offsets = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]])
        corners = (cells[:, None, :]+offsets)*msg.info.resolution
        edges = corners[:, [(0, 1), (1, 2), (2, 3), (3, 0)], :].reshape(-1, 2, 3)
        return {'segments': cam.segments(transform(edges, mat@origin)), 'count': len(cells)}

    def topdown(self):
        """Bounded, read-only geometry in the robot frame, independent of RGB.

        Local layers are acquisition-time robot-relative snapshots. They are not
        a globally registered map or motion-compensated to a common timestamp.
        """
        result = dict(frame_id=self.args.base_frame, layers=[], objects=[],
                      robot_radius=.19, obstacle_margin=.10, reason='waiting_for_topic',
                      registration='acquisition_time_robot_relative')
        def to_base(frame, ns):
            if frame == self.args.base_frame:
                return np.eye(4)
            if not frame:
                raise ValueError('empty_source_frame')
            return tf_matrix(self.tf.lookup_transform(self.args.base_frame, frame,
                             Time(nanoseconds=ns)).transform)

        for key, (typ, topic, ttl, color, label) in self.specs.items():
            if key == 'voxels':  # Same cloud as points in local mode; avoid duplicate payload.
                if self.args.mode == 'local':
                    continue
            layer = dict(id=key, topic=topic, color=color, label=label, reason='waiting_for_topic')
            result['layers'].append(layer)
            entry = self.sources.get(key)
            if not entry:
                continue
            msg, received = entry
            try:
                if msg is None:
                    raise ValueError('oversized_message')
                if not self.fresh(stamp(msg), received, min(ttl, .5)):
                    raise ValueError('source_stale_or_zero_stamp')
                mat = to_base(msg.header.frame_id, stamp(msg))
                layer.update(age_ms=round((self.get_clock().now().nanoseconds-stamp(msg))*1e-6),
                             stamp_ns=str(stamp(msg)))
                if isinstance(msg, PointCloud2):
                    pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
                    pts = transform(pts[np.isfinite(pts).all(axis=1)], mat)
                    pts = pts[(np.abs(pts[:, :2]) <= 8).all(axis=1)]
                    layer['total'] = len(pts)
                    pts = pts[::max(1, int(np.ceil(len(pts)/3000)))][:3000]
                    layer.update(kind='points', points=np.round(pts[:, :2], 3).tolist(), count=len(pts))
                elif isinstance(msg, Path):
                    if any(p.header.frame_id not in ('', msg.header.frame_id) for p in msg.poses):
                        raise ValueError('mixed_path_frames')
                    pts = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                                    for p in msg.poses], dtype=float).reshape(-1, 3)
                    if not np.isfinite(pts).all():
                        raise ValueError('nonfinite_path')
                    if len(pts) > 2000:
                        raise ValueError('topdown_path_too_large')
                    layer.update(kind='path', points=np.round(transform(pts, mat)[:, :2], 3).tolist(), count=len(pts))
                else:
                    if not msg.data:
                        layer.update(kind='grid', cells=[], count=0, width=0, height=0)
                    else:
                        if (msg.info.width*msg.info.height != len(msg.data) or len(msg.data) > 40000
                                or not 0 < msg.info.resolution <= 1):
                            raise ValueError('invalid_or_oversized_topdown_grid')
                        q, p = msg.info.origin.orientation, msg.info.origin.position
                        origin = mat @ matrix([q.x, q.y, q.z, q.w], [p.x, p.y, p.z])
                        if not np.isfinite(origin).all():
                            raise ValueError('invalid_grid_origin')
                        # Preserve the full origin rotation (including robot yaw).
                        basis = origin[:2, :2]*msg.info.resolution
                        cells = list(msg.data)
                        layer.update(kind='grid', cells=cells, width=msg.info.width, height=msg.info.height,
                                     origin=origin[:2, 3].tolist(), basis=basis.tolist(),
                                     count=sum(v >= 50 for v in cells), free_count=cells.count(0))
                layer['reason'] = 'ok'
            except Exception as exc:
                layer['reason'] = str(exc)[:180]

        status, received = self.statuses.get('person', ({}, 0.))
        status_fresh = time.monotonic()-received <= 1.
        radius = status.get('robot_radius_m')
        margin = status.get('obstacle_margin_m')
        if status_fresh and isinstance(radius, (int, float)) and np.isfinite(radius) and 0 < radius <= 1.:
            result['robot_radius'] = radius
        if status_fresh and isinstance(margin, (int, float)) and np.isfinite(margin) and 0 <= margin <= 1.:
            result['obstacle_margin'] = margin
        selected = status.get('target_id', '') if status_fresh and status.get('enabled') else ''
        result['follow'] = {k: status.get(k) for k in
                           ('state', 'safety_reason', 'recovery_state', 'linear_mps', 'angular_rps', 'compute_ms')} if status_fresh else {}
        result['objects_reason'] = 'waiting_for_topic'
        if self.relative:
            rel, received = self.relative[-1]
            try:
                if not self.fresh(rel['stamp_ns'], received, min(float(rel.get('ttl_s', .3)), .5)):
                    raise ValueError('source_stale_or_zero_stamp')
                if not rel.get('enabled'):
                    raise ValueError('detector_disabled')
                if self.raw and any(rel.get(k) != self.raw[-1][0].get(k) for k in ('session', 'epoch')):
                    raise ValueError('detector_generation_mismatch')
                mat = to_base(rel.get('base_frame', ''), rel['stamp_ns'])
                for obj in rel['objects']:
                    d, yaw = obj.get('distance'), obj.get('yaw')
                    if not obj.get('detected') or not isinstance(d, (int, float)) or not isinstance(yaw, (int, float)):
                        continue
                    if not np.isfinite([d, yaw]).all() or not 0 < d <= 15:
                        continue
                    xy = transform(np.array([[d*np.cos(yaw), d*np.sin(yaw), 0.]]), mat)[0, :2]
                    result['objects'].append(dict(id=str(obj.get('id', ''))[:160], label=str(obj.get('label', '?'))[:80],
                        xy=np.round(xy, 3).tolist(), selected=bool(selected and obj.get('id') == selected)))
                result['objects_reason'] = 'ok'
                result['objects_age_ms'] = round((self.get_clock().now().nanoseconds-rel['stamp_ns'])*1e-6)
            except Exception as exc:
                result['objects_reason'] = str(exc)[:180]
        if any(l['reason'] == 'ok' for l in result['layers']) or result['objects_reason'] == 'ok':
            result['reason'] = 'ok'
        return result

    def planning(self):
        result = {'view': 'planning', 'image': None, 'reason': 'waiting_for_rgb', 'layers': []}
        if not self.frames.frames:
            return result
        values = [v[0] for v in self.frames.frames.values()]
        # A small delay allows dynamic TF to catch up; never substitute the latest TF.
        now = self.get_clock().now().nanoseconds
        item = next((v for v in reversed(values) if now-stamp(v[0]) >= 100_000_000), values[-1])
        if self.args.mode == 'local':
            entry = self.sources.get('points')
            if entry and entry[0] is not None and self.fresh(stamp(entry[0]), entry[1], .8):
                item = min(values, key=lambda v: abs(stamp(v[0])-stamp(entry[0])))
            else:
                item = values[-1]
        result.update(self.image_data(item))
        img = item[0]
        try:
            cam = self.camera(img)
        except ValueError as e:
            result['reason'] = str(e)
            return result
        result['reason'] = 'ok'
        result['fixed_frame'] = self.args.fixed_frame if self.args.mode == 'odom' else None
        result['mode'] = self.args.mode
        result['voxel_size'] = self.args.voxel_size
        for key, (_, topic, ttl, color, label) in self.specs.items():
            layer = {'id': key, 'topic': topic, 'color': color, 'label': label, 'reason': 'waiting_for_topic'}
            entry = self.sources.get(key)
            if self.args.mode == 'local' and entry and entry[0] is not None and self.fresh(stamp(entry[0]), entry[1], ttl):
                current = entry[0]
                # An explicit empty path/grid cancels old geometry immediately.
                empty = (isinstance(current, Path) and not current.poses) or (isinstance(current, OccupancyGrid) and not current.data)
                if not empty:
                    candidates = [it for it in self.histories.get(key, []) if it[0] is not None and self.fresh(stamp(it[0]), it[1], ttl)]
                    entry = min(candidates, key=lambda it: abs(stamp(it[0])-stamp(img)), default=entry)
            if entry:
                msg, received = entry
                try:
                    if msg is None:
                        raise ValueError('oversized_message')
                    if not self.fresh(stamp(msg), received, ttl):
                        raise ValueError('source_stale_or_zero_stamp')
                    if abs(stamp(msg)-stamp(img))*1e-9 > ttl:
                        raise ValueError('image_source_time_mismatch')
                    layer.update(self.geometry(key, msg, img, cam))
                    layer.update(reason='ok', frame_id=msg.header.frame_id,
                                 skew_ms=round((stamp(msg)-stamp(img))*1e-6))
                except Exception as e:
                    layer['reason'] = str(e)[:220]
            result['layers'].append(layer)
        return result

    def render(self):
        for name, function in [('detections', self.detections), ('planning', self.planning)]:
            # Only render pages with recent clients. Neither HTTP nor rendering
            # waits on or calls the follower; all inputs are read-only subscriptions.
            if time.monotonic()-self.last_clients[name] > 2:
                continue
            start = time.monotonic()
            try:
                result = function()
                result['statuses'] = {k: {'age_s': round(time.monotonic()-v[1], 1),
                    'state': str(v[0].get('state', '')), 'reason': str(v[0].get('reason', ''))[:240]}
                    for k, v in self.statuses.items()}
                result['image_error'] = self.image_error
                result['domain'] = os.environ.get('ROS_DOMAIN_ID', '0')
                result['mode'] = self.args.mode
            except Exception as e:
                result = {'view': name, 'image': None, 'reason': str(e)[:250]}
            if name == 'planning':
                try:
                    overhead_started = time.monotonic()
                    result['topdown'] = self.topdown()
                    result['topdown_ms'] = round((time.monotonic()-overhead_started)*1000, 1)
                except Exception as exc:
                    result['topdown'] = dict(reason=str(exc)[:180], layers=[], objects=[])
            result['render_ms'] = round((time.monotonic()-start)*1000, 1)
            raw = self.encode(result)
            # Compress once per snapshot, not once per client. Publish immutable
            # bytes/ETag/time atomically so HTTP threads cannot mix frame versions.
            compressed = gzip.compress(raw, compresslevel=1, mtime=0)
            made = time.monotonic()
            etag = 'W/"%s-%s"' % (name, time.monotonic_ns())
            self.wire_snapshots[name] = (raw, compressed, etag, made)
            self.snapshot[name] = raw
            self.snapshot_time[name] = made


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--port', type=int, default=18081)
    p.add_argument('--mode', choices=['local', 'odom'], default='local', help='local: near-synchronous robot-frame geometry, no odometry/Nav2; odom: cross-time world geometry')
    p.add_argument('--base-frame', default='r680_mapping_floor')
    p.add_argument('--fixed-frame', default='d455_floor_odom')
    p.add_argument('--voxel-size', type=float, default=.10, help='Must match dynamic_obstacle_memory.voxel_size')
    p.add_argument('--image-topic', default='/r680/d455/color/image_raw')
    p.add_argument('--info-topic', default='/r680/d455/color/camera_info')
    p.add_argument('--voxel-topic', default=None)
    p.add_argument('--points-topic', default=None)
    p.add_argument('--nav-path-topic', default='/plan')
    p.add_argument('--local-path-topic', default='/optimal_trajectory')
    args, ros_args = p.parse_known_args()
    args.voxel_topic = args.voxel_topic or ('/r680/person_follow/depth_obstacles' if args.mode == 'local' else '/r680_nav/d455/points_confirmed')
    args.points_topic = args.points_topic or ('/r680/person_follow/depth_obstacles' if args.mode == 'local' else '/r680_nav/d455/points')
    if not .01 <= args.voxel_size <= 1 or not 0 < args.port < 65536:
        p.error('Invalid voxel size or port')
    # Debug rendering yields CPU scheduling priority to perception/control.
    # Lowering this process priority needs no privilege and changes no follower.
    try:
        os.nice(5)
    except OSError:
        pass
    rclpy.init(args=ros_args)
    node = DebugViewer(args)
    assets = FilePath(__file__).resolve().parents[1]/'debug_web'
    if not assets.is_dir():
        assets = FilePath(get_package_share_directory('wla_r680_navigation'))/'debug_web'

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        def setup(self):
            super().setup()
            self.connection.settimeout(3)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            compressed = False
            etag = None
            if path == '/api/config':
                data = node.encode({'schema': 'wla-perception-debug/v1', 'mode': args.mode,
                    'domain': os.environ.get('ROS_DOMAIN_ID', '0'), 'base_frame': args.base_frame,
                    'points_topic': args.points_topic, 'voxel_topic': args.voxel_topic})
                mime = 'application/json; charset=utf-8'
            elif path in ('/api/detections', '/api/planning'):
                name = path.rsplit('/', 1)[1]
                node.last_clients[name] = time.monotonic()
                snapshot = node.wire_snapshots.get(name)
                if snapshot is not None and time.monotonic()-snapshot[3] < 1:
                    raw, zipped, etag, _ = snapshot
                    if self.headers.get('If-None-Match') == etag:
                        self.send_response(304)
                        self.send_header('ETag', etag)
                        self.send_header('Cache-Control', 'no-cache')
                        self.send_header('Vary', 'Accept-Encoding')
                        self.end_headers()
                        return
                    # Only accept the actual coding token, not gzip;q=0.
                    codings = [v.strip().split(';') for v in self.headers.get('Accept-Encoding', '').split(',')]
                    compressed = any(v[0]=='gzip' and not any(p.strip() in ('q=0','q=0.0') for p in v[1:]) for v in codings)
                    data = zipped if compressed else raw
                else:
                    # Never 304 a frozen/expired snapshot: the client must learn
                    # that observations are unavailable even if the tag matches.
                    data = node.encode({'view': name, 'image': None, 'reason': 'waiting_for_rgb'})
                mime = 'application/json; charset=utf-8'
            elif path in ('/', '/detections', '/planning', '/app.js', '/topdown.js', '/style.css'):
                name = {'/app.js': 'app.js', '/topdown.js': 'topdown.js', '/style.css': 'style.css'}.get(path, 'index.html')
                data = (assets/name).read_bytes()
                mime = {'app.js': 'text/javascript', 'topdown.js': 'text/javascript', 'style.css': 'text/css', 'index.html': 'text/html'}[name]+'; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-cache' if etag else 'no-store')
            if etag:self.send_header('ETag', etag)
            if path.startswith('/api/'):
                self.send_header('Vary', 'Accept-Encoding')
            if compressed:self.send_header('Content-Encoding', 'gzip')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f'Read-only debug: http://<robot-ip>:{args.port}/detections and /planning', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
