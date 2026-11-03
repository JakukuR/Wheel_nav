"""Bounded observation history and piecewise cubic Bezier MPC references."""
from collections import deque
import math
import numpy as np


class TargetHistory:
    """Resample accepted observations on a fixed time grid, never predictions.

    The window ends at the last accepted observation. With no new observations
    it freezes, retaining the route to the last seen position during search.
    Long observation gaps start a new sequence rather than inventing samples.
    """
    def __init__(self, window_s=5., period_s=.4, max_gap_s=.6):
        if (not 2. <= window_s <= 20. or not .2 <= period_s <= 1.
                or window_s / period_s > 64 or not .1 <= max_gap_s <= 2.):
            raise ValueError('invalid_target_history_configuration')
        self.window_s, self.period_s, self.max_gap_s = window_s, period_s, max_gap_s
        self.samples = deque(maxlen=int(math.ceil(window_s / period_s)) + 1)
        self.latest = None
        self.origin = None
        self.next_index = 0

    def update(self, point, stamp):
        point = np.asarray(point, dtype=float)
        if point.shape != (2,) or not np.isfinite(point).all() or not math.isfinite(stamp):
            raise ValueError('invalid_target_history_observation')
        if self.latest is not None and stamp <= self.latest[0]:
            return
        if self.latest is None or stamp - self.latest[0] > self.max_gap_s:
            self.samples.clear()
            self.origin, self.next_index = float(stamp), 1
            self.samples.append((float(stamp), point.copy()))
        else:
            previous_stamp, previous_point = self.latest
            sample_stamp = self.origin + self.next_index * self.period_s
            while sample_stamp <= stamp:
                alpha = (sample_stamp - previous_stamp) / (stamp - previous_stamp)
                sampled = previous_point + alpha * (point - previous_point)
                self.samples.append((sample_stamp, sampled))
                self.next_index += 1
                sample_stamp = self.origin + self.next_index * self.period_s
        self.latest = (float(stamp), point.copy())
        while self.samples and self.samples[0][0] < stamp - self.window_s:
            self.samples.popleft()

    def points(self):
        result = [point.copy() for _, point in self.samples]
        # The live endpoint is separate from the evenly spaced history samples.
        if self.latest is not None and (not self.samples or self.latest[0] > self.samples[-1][0] + 1e-6):
            result.append(self.latest[1].copy())
        return result

    def diagnostic(self):
        return dict(window_s=self.window_s, sample_period_s=self.period_s,
                    sample_count=len(self.samples),
                    oldest_stamp_s=self.samples[0][0] if self.samples else None,
                    last_observation_stamp_s=self.latest[0] if self.latest else None,
                    expiry_clock='last_accepted_observation')


def bspline_corners(path, corner=.35, step=.05):
    """Round a polyline's corners with a clamped uniform cubic B-spline.

    Control points: the start, then for every interior vertex the points `d`
    before and after it on its two segments plus the vertex itself, then the
    end (d = corner, at most 0.45 of either adjacent segment). Clamped knots
    keep both endpoints exact; the curve stays near straight segments and cuts
    each corner inside that +-d window. Sampled every `step` m of arc length.
    Corners under ~3 degrees are left out. Callers check clearance.
    """
    raw = np.asarray(path, dtype=float).reshape(-1, 2)
    keep = np.r_[True, np.linalg.norm(np.diff(raw, axis=0), axis=1) > 1e-6]
    raw = raw[keep]
    if len(raw) < 3 or corner <= 0:
        return raw.copy()
    controls = [raw[0]]
    for i in range(1, len(raw)-1):
        before, after = raw[i]-raw[i-1], raw[i+1]-raw[i]
        lb, la = np.linalg.norm(before), np.linalg.norm(after)
        ub, ua = before/lb, after/la
        if float(np.dot(ub, ua)) > math.cos(math.radians(3.)):
            controls.append(raw[i]); continue
        d = min(corner, .45*lb, .45*la)
        controls += [raw[i]-d*ub, raw[i], raw[i]+d*ua]
    controls.append(raw[-1])
    controls = np.asarray(controls)
    n, k = len(controls), 3
    if n < 4:
        return raw.copy()
    knots = np.r_[np.zeros(k), np.linspace(0., 1., n-k+1), np.ones(k)]
    u = np.linspace(0., 1., max(32, 12*n))
    # De Boor recursion over all samples at once (n is small).
    span = np.clip(np.searchsorted(knots, u, side='right')-1, k, n-1)
    d = np.stack([controls[span-k+j] for j in range(k+1)], axis=1)
    for r in range(1, k+1):
        for j in range(k, r-1, -1):
            i = span-k+j
            left, right = knots[i], knots[i+k+1-r]
            a = ((u-left)/np.where(right > left, right-left, 1.))[:, None]
            d[:, j] = (1.-a)*d[:, j-1]+a*d[:, j]
    curve = d[:, k]
    curve[-1] = raw[-1]
    arcs = np.r_[0., np.cumsum(np.linalg.norm(np.diff(curve, axis=0), axis=1))]
    count = max(2, int(math.ceil(arcs[-1]/step))+1)
    s = np.linspace(0., arcs[-1], count)
    return np.column_stack([np.interp(s, arcs, curve[:, j]) for j in range(2)])


def bezier_reference(history):
    """Sparse knots -> bounded smoothing -> C1 piecewise cubic Bezier curve.

    Endpoints stay fixed. Interior knots move at most 5 cm; shared tangent
    handles are at most 5 cm and a third of either adjacent chord. This limits
    overshoot, including short/reversing segments. Eight samples per cubic
    keep work bounded (normally at most 105 output points for a 5 s window).
    This is a geometric reference; only MPC validates actual swept motion.
    """
    raw = np.asarray(history, dtype=float).reshape(-1, 2)
    if not np.isfinite(raw).all():
        raise ValueError('nonfinite_target_history')
    if len(raw) < 2:
        return raw.copy()
    # Suppress stationary jitter without dropping the actual endpoint.
    knots = [raw[0]]
    for point in raw[1:-1]:
        if np.linalg.norm(point - knots[-1]) >= .03:
            knots.append(point)
    if np.linalg.norm(raw[-1] - knots[-1]) > 1e-6:
        knots.append(raw[-1])
    else:
        knots[-1] = raw[-1]
    knots = np.asarray(knots)
    if len(knots) < 2:
        return knots.copy()
    anchor = knots.copy()
    shift = .25 * (knots[:-2] + knots[2:] - 2. * knots[1:-1])
    anchor[1:-1] += shift * np.minimum(1., .05 / np.maximum(np.linalg.norm(shift, axis=1), 1e-12))[:, None]
    chord = np.diff(anchor, axis=0)
    length = np.linalg.norm(chord, axis=1)
    direction = chord / np.maximum(length[:, None], 1e-12)
    tangent = np.vstack((direction[0], direction[:-1] + direction[1:], direction[-1]))
    handle_length = np.minimum(.05, np.minimum(np.r_[length[0], length], np.r_[length, length[-1]]) / 3.)
    handle = tangent / np.maximum(np.linalg.norm(tangent, axis=1)[:, None], 1e-12) * handle_length[:, None]
    # All four control points of each cubic; shared handles give C1 joins.
    controls = np.stack((anchor[:-1], anchor[:-1] + handle[:-1],
                         anchor[1:] - handle[1:], anchor[1:]), axis=1)
    u = np.arange(8, dtype=float) / 8.
    basis = np.column_stack(((1-u)**3, 3*(1-u)**2*u, 3*(1-u)*u**2, u**3))
    curve = np.einsum('uk,skd->sud', basis, controls).reshape(-1, 2)
    curve = np.vstack((curve, anchor[-1]))
    return curve[np.r_[True, np.linalg.norm(np.diff(curve, axis=0), axis=1) > 1e-9]]
