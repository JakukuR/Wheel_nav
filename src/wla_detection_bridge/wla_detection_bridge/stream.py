"""Shared Agent detector wire contract. No model, ROS runtime or motion code."""
from collections import deque
import json
import math

SCHEMA = 'wla-detections/v1'
TOPIC = '/r680/perception/detections'


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def decode(value, now_ns, max_age_s=.3):
    if isinstance(value, str):
        if len(value) > 262144:
            raise ValueError('detection_batch_too_large')
        value = json.loads(value)
    if not isinstance(value, dict) or value.get('schema') != SCHEMA:
        raise ValueError('invalid_detection_schema')
    if (not isinstance(value.get('session'), str) or not 1 <= len(value['session']) <= 128
            or not isinstance(value.get('frame_id'), str) or not value['frame_id']
            or type(value.get('epoch')) is not int or value['epoch'] < 0
            or type(value.get('stamp_ns')) is not int or value['stamp_ns'] <= 0
            or type(value.get('width')) is not int or type(value.get('height')) is not int
            or value['width'] <= 0 or value['height'] <= 0
            or value['width'] * value['height'] > 2_000_000):
        raise ValueError('invalid_detection_header')
    age = (now_ns-value['stamp_ns']) / 1e9
    if not -.05 <= age <= max_age_s:
        raise ValueError('expired_detection_batch')
    objects = value.get('objects')
    if not isinstance(objects, list) or len(objects) > 256:
        raise ValueError('invalid_detection_objects')
    ids = set()
    for obj in objects:
        if (not isinstance(obj, dict) or type(obj.get('track_id')) is not int
                or not 0 <= obj['track_id'] < 2**31 or obj['track_id'] in ids
                or not isinstance(obj.get('label'), str) or not obj['label']
                or len(obj['label']) > 128):
            raise ValueError('invalid_detection_identity')
        ids.add(obj['track_id'])
        box = obj.get('bbox_xyxy')
        conf = obj.get('confidence')
        if (not isinstance(box, (list, tuple)) or len(box) != 4
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in box)
                or type(conf) not in (int, float) or not math.isfinite(conf)
                or not 0 <= conf <= 1
                or not 0 <= box[0] < box[2] <= value['width']
                or not 0 <= box[1] < box[3] <= value['height']):
            raise ValueError('invalid_detection_box')
    return value


def encode_batch(message, session, epoch, objects):
    value = dict(schema=SCHEMA, session=session, epoch=epoch,
                 stamp_ns=stamp_ns(message), frame_id=message.header.frame_id,
                 width=int(message.width), height=int(message.height), objects=objects)
    decode(value, value['stamp_ns'])
    return json.dumps(value, allow_nan=False, separators=(',', ':'))


def select(value, labels, confidence=0.):
    labels = set(labels)
    return [obj for obj in value['objects']
            if obj['label'] in labels and obj['confidence'] >= confidence]


class DetectionInbox:
    """Bounded source-time join; IDs are only meaningful within a session/epoch.

    Callers serialize access. Only fresh, exactly matching RGB and calibrated
    aligned depth are consumed; a failed join never pairs boxes with a new image.
    """
    def __init__(self, max_age_s=.3, sync_s=.035):
        self.max_age_s, self.sync_ns = max_age_s, int(sync_s*1e9)
        self.rgb, self.depth, self.info = (deque(maxlen=24) for _ in range(3))
        self.batch = None
        self.identity = None
        self.retired = deque(maxlen=128)
        self.last_stamp = 0

    def push(self, value, now_ns, received):
        batch = decode(value, now_ns, self.max_age_s)
        identity = (batch['session'], batch['epoch'])
        changed = identity != self.identity
        if changed:
            if (identity in self.retired or self.identity is not None
                    and identity[0] == self.identity[0] and identity[1] < self.identity[1]):
                raise ValueError('retired_detection_session')
            if self.identity is not None:
                self.retired.append(self.identity)
            self.identity, self.last_stamp = identity, 0
        if batch['stamp_ns'] <= self.last_stamp:
            raise ValueError('duplicate_or_reordered_detection')
        self.last_stamp = batch['stamp_ns']
        self.batch = (batch, received)
        return changed

    def take(self, now_ns, now_monotonic, require_rgb=True):
        if self.batch is None:
            return None
        batch, received = self.batch
        try:
            decode(batch, now_ns, self.max_age_s)
        except ValueError:
            self.batch = None
            return None
        if not 0 <= now_monotonic-received <= self.max_age_s:
            self.batch = None
            return None
        stamp = batch['stamp_ns']
        rgb = next((m for m in reversed(self.rgb) if stamp_ns(m) == stamp
                    and m.header.frame_id == batch['frame_id']), None)
        depth = min(self.depth, key=lambda m: abs(stamp_ns(m)-stamp), default=None)
        info = min(self.info, key=lambda m: abs(stamp_ns(m)-stamp), default=None)
        if ((require_rgb and rgb is None) or depth is None or info is None
                or abs(stamp_ns(depth)-stamp) > self.sync_ns
                or abs(stamp_ns(info)-stamp) > 250_000_000
                or info.header.frame_id != batch['frame_id']
                or depth.header.frame_id != batch['frame_id']
                or any((m.width, m.height) != (batch['width'], batch['height'])
                       for m in (depth, info) + ((rgb,) if rgb is not None else ()))
                or not all(math.isfinite(v) for v in info.k)
                or info.k[0] <= 0 or info.k[4] <= 0):
            return None
        self.batch = None
        return batch, rgb, depth, info, received
