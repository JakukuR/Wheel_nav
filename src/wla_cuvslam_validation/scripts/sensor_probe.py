#!/usr/bin/env python3
"""Bounded, read-only sensor timing probe. Does not publish commands."""
import argparse
import json
import math
import statistics
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, CameraInfo


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int((len(ordered) - 1) * fraction))]


class Probe(Node):
    def __init__(self):
        super().__init__('wla_vio_sensor_probe')
        self.samples = {}
        self.info = {}
        self.subscriptions_owned = []
        topics = {
            '/r680/d455/infra1/image_rect_raw': Image,
            '/r680/d455/infra2/image_rect_raw': Image,
            '/r680/d455/color/image_raw': Image,
            '/r680_vio_test/chassis/imu_raw': Imu,
        }
        for topic, kind in topics.items():
            self.samples[topic] = []
            self.subscriptions_owned.append(self.create_subscription(
                kind, topic, lambda msg, key=topic: self.on_sample(key, msg),
                qos_profile_sensor_data))
        for index in (1, 2):
            topic = f'/r680/d455/infra{index}/camera_info'
            self.subscriptions_owned.append(self.create_subscription(
                CameraInfo, topic, lambda msg, key=topic: self.on_info(key, msg),
                qos_profile_sensor_data))

    def on_sample(self, key, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        record = {'stamp': stamp, 'receipt': time.monotonic(),
                  'age_ms': (self.get_clock().now().nanoseconds * 1e-9 - stamp) * 1000,
                  'frame': msg.header.frame_id}
        if isinstance(msg, Imu):
            record['gyro'] = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            record['accel'] = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        else:
            record.update(width=msg.width, height=msg.height, encoding=msg.encoding, step=msg.step)
        self.samples[key].append(record)

    def on_info(self, key, msg):
        self.info[key] = {'width': msg.width, 'height': msg.height, 'k': list(msg.k),
                          'p': list(msg.p), 'd': list(msg.d), 'frame': msg.header.frame_id,
                          'distortion_model': msg.distortion_model}

    def report(self):
        result = {'topics': {}, 'camera_info': self.info,
                  'timestamp_note': 'chassis IMU stamps are host publication time; hardware sampling offset is unmeasured'}
        for key, data in self.samples.items():
            gaps = [b['stamp'] - a['stamp'] for a, b in zip(data, data[1:])]
            span = data[-1]['receipt'] - data[0]['receipt'] if len(data) > 1 else 0
            ages = [x['age_ms'] for x in data]
            item = {'count': len(data), 'receipt_hz': (len(data) - 1) / span if span > 0 else 0,
                    'stamp_gap_ms_p50': percentile(gaps, .5) * 1000 if gaps else None,
                    'stamp_gap_ms_p95': percentile(gaps, .95) * 1000 if gaps else None,
                    'stamp_gap_ms_max': max(gaps) * 1000 if gaps else None,
                    'nonincreasing_stamps': sum(x <= 0 for x in gaps),
                    'age_ms_p50': percentile(ages, .5), 'age_ms_p95': percentile(ages, .95),
                    'last': data[-1] if data else None}
            if data and 'gyro' in data[0]:
                for field in ['gyro', 'accel']:
                    values = [[x[field][axis] for x in data] for axis in range(3)]
                    item[field + '_mean'] = [statistics.mean(x) for x in values]
                    item[field + '_std'] = [statistics.pstdev(x) for x in values]
                item['accel_norm_mean'] = statistics.mean(
                    math.sqrt(sum(v * v for v in x['accel'])) for x in data)
            result['topics'][key] = item
        left = self.samples['/r680/d455/infra1/image_rect_raw']
        right = self.samples['/r680/d455/infra2/image_rect_raw']
        if left and right:
            rs = sorted(x['stamp'] for x in right)
            import bisect
            deltas = []
            for x in left:
                i = bisect.bisect_left(rs, x['stamp'])
                candidates = rs[max(0, i - 1):i + 1]
                deltas.append(min(abs(v - x['stamp']) for v in candidates) * 1000)
            result['stereo_nearest_stamp_ms_p95'] = percentile(deltas, .95)
            result['stereo_nearest_stamp_ms_max'] = max(deltas)
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(node.report(), indent=2) + '\n')
        print(path)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
