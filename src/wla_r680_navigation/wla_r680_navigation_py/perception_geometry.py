"""Shared D455 geometry extracted from the furniture mapping collector."""
import numpy as np

def transform_point(point, tf):
    q = tf.rotation
    vector = np.array([q.x, q.y, q.z], dtype=float)
    p = np.array(point, dtype=float)
    rotated = p + 2 * (q.w * np.cross(vector, p) + np.cross(vector, np.cross(vector, p)))
    return (rotated + np.array([tf.translation.x, tf.translation.y,
                                tf.translation.z])).tolist()

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

def bbox_point(box, depth, k):
    """Central depth surface candidate, not an object centre or approach pose."""
    x1, y1, x2, y2 = box
    width, height = x2-x1, y2-y1
    if (width < 12 or height < 12 or len(k) != 9
            or not np.isfinite(k).all() or k[0] <= 0 or k[4] <= 0):
        return None
    cx, cy = (x1+x2)/2, (y1+y2)/2
    radius = max(3, int(min(width, height)*.08))
    u, v = int(cx), int(cy)
    patch = depth[max(0,v-radius):min(depth.shape[0],v+radius+1),
                  max(0,u-radius):min(depth.shape[1],u+radius+1)]
    valid = patch[(patch >= 250) & (patch <= 4000)]
    if valid.size < 10:
        return None
    z = float(np.median(valid))*.001
    return [(cx-k[2])*z/k[0], (cy-k[5])*z/k[4], z]
