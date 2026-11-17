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

def touches_side(box, width, margin=.015):
    """Box reaches the left or right image border (within 1.5 % of the width)."""
    edge = max(4., margin*width)
    return bool(box[0] <= edge or box[2] >= width-edge)

def person_point(box, depth, k, quantile=.2):
    """Nearest body surface in the torso band of a person box.

    The box-centre median read the wall behind a person who was partly out of the
    image or turning a corner: range jumped 0.5-2.4 m while the bearing stayed
    within 0.08 rad (J02 2026-10-29/30), and the follow tracker froze on it. The
    body is in front of its background, so a low quantile of a wider torso band
    (middle 40 % of the width, 20-60 % of the height) stays on the person.
    """
    x1, y1, x2, y2 = box
    width, height = x2-x1, y2-y1
    if (width < 12 or height < 12 or len(k) != 9
            or not np.isfinite(k).all() or k[0] <= 0 or k[4] <= 0):
        return None
    cx = (x1+x2)/2
    u1, u2 = int(max(0, cx-.2*width)), int(min(depth.shape[1], cx+.2*width+1))
    v1, v2 = int(max(0, y1+.2*height)), int(min(depth.shape[0], y1+.6*height+1))
    patch = depth[v1:v2, u1:u2]
    valid = patch[(patch >= 250) & (patch <= 4000)]
    if valid.size < 10:
        return None
    z = float(np.quantile(valid, quantile))*.001
    cy = y1+.4*height
    return [(cx-k[2])*z/k[0], (cy-k[5])*z/k[4], z]

def bbox_point(box, depth, k):
    return bbox_point_details(box, depth, k)[0]


def bbox_point_details(box, depth, k):
    """A supported central surface; ambiguity is not a current object range."""
    quality = dict(method='central_surface_quality_v2', reason='invalid_input')
    if (depth is None or np.ndim(depth) != 2 or len(box) != 4
            or not np.isfinite(box).all() or len(k) != 9
            or not np.isfinite(k).all() or k[0] <= 0 or k[4] <= 0):
        return None, quality
    x1, y1, x2, y2 = box
    width, height = x2-x1, y2-y1
    if width < 12 or height < 12:
        return None, quality
    cx, cy = (x1+x2)/2, (y1+y2)/2
    if not (0 <= cx < depth.shape[1] and 0 <= cy < depth.shape[0]):
        return None, quality
    radius = max(3, int(min(width, height)*.08))
    u, v = int(cx), int(cy)
    patch = depth[max(0,v-radius):min(depth.shape[0],v+radius+1),
                  max(0,u-radius):min(depth.shape[1],u+radius+1)]
    valid = patch[(patch >= 250) & (patch <= 4000)]
    fraction = float(valid.size/max(1, patch.size))
    quality.update(valid_count=int(valid.size), sample_count=int(patch.size), valid_fraction=fraction,
                   roi=[max(0,u-radius), max(0,v-radius), min(depth.shape[1],u+radius+1), min(depth.shape[0],v+radius+1)])
    if valid.size < 25 or fraction < .2:
        quality['reason'] = 'insufficient_surface_support'
        return None, quality
    q10, z, q90 = np.quantile(valid, [.1, .5, .9])*.001
    quality.update(q10_m=float(q10), median_m=float(z), q90_m=float(q90))
    # Engineering initial tolerance, exposed in the diagnostic; no temporal carry.
    limit = max(.15, .15*z)
    quality['spread_limit_m'] = float(limit)
    if q90-q10 > limit:
        quality['reason'] = 'ambiguous_depth_surfaces'
        return None, quality
    quality['reason'] = 'supported_surface'
    return [(cx-k[2])*z/k[0], (cy-k[5])*z/k[4], float(z)], quality
