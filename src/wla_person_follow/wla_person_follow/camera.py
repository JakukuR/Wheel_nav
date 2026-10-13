"""Decode aligned D455 depth, preserving invalid pixels as unknown."""
import numpy as np


def depth_meters(msg):
    if msg.encoding not in ('16UC1', '32FC1'):
        raise ValueError('unsupported_depth_encoding')
    size = 2 if msg.encoding == '16UC1' else 4
    if (msg.width <= 0 or msg.height <= 0 or msg.step < msg.width*size or
            msg.step % size or len(msg.data) < msg.step*msg.height):
        raise ValueError('invalid_depth_buffer')
    dtype = ('>' if msg.is_bigendian else '<') + ('u2' if size == 2 else 'f4')
    depth = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.step, size)).astype(np.float32)
    if size == 2:
        depth *= .001
    depth[~np.isfinite(depth) | (depth < 0)] = 0
    return depth
