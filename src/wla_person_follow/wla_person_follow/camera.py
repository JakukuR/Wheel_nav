"""Decode D455 ROS images and filter already aligned depth frames."""
import cv2
import numpy as np


def color_array(msg):
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported color encoding: {msg.encoding}')
    arr = np.ndarray((msg.height, msg.step), dtype=np.uint8, buffer=msg.data)
    arr = arr[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR).copy() if msg.encoding == 'rgb8' else arr.copy()


def depth_meters(msg):
    if msg.encoding == '16UC1':
        raw = np.ndarray((msg.height, msg.step // 2), dtype='<u2', buffer=msg.data)
        if msg.is_bigendian:
            raw = raw.byteswap()
        depth = raw[:, :msg.width].astype(np.float32) * 0.001
    elif msg.encoding == '32FC1':
        raw = np.ndarray((msg.height, msg.step // 4), dtype='>f4' if msg.is_bigendian else '<f4',
                         buffer=msg.data)
        depth = raw[:, :msg.width].astype(np.float32)
    else:
        raise ValueError(f'unsupported depth encoding: {msg.encoding}')
    depth[~np.isfinite(depth)] = 0
    depth[depth < 0] = 0
    return depth


class DepthFilter:
    def __init__(self):
        self.previous = None

    def reset(self):
        self.previous = None

    def apply(self, depth):
        # Median is spatial; temporal blending occurs only for matching valid ranges.
        spatial = cv2.medianBlur(depth, 3)
        valid = depth > 0
        spatial[~valid] = 0
        if self.previous is not None and self.previous.shape == spatial.shape:
            stable = valid & (self.previous > 0) & (np.abs(spatial - self.previous) < 0.20)
            spatial[stable] = 0.65 * spatial[stable] + 0.35 * self.previous[stable]
        self.previous = spatial.copy()
        return spatial
