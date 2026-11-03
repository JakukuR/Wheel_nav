"""Session spatial memory in the wheel-odometry frame. No ROS, no global map.

Odometry drift over a few minutes is accepted: every position here is only
used to rank where to look or walk next, never to drive blind.
"""
import math
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class WheelPose:
    """Midpoint integration of the wheel twist (same model as the follow backend)."""
    def __init__(self):
        self.x = self.y = self.yaw = 0.
        self.stamp = None

    def update(self, stamp, v, w):
        if not all(math.isfinite(value) for value in (stamp, v, w)):
            return False
        if self.stamp is not None:
            dt = stamp-self.stamp
            if dt <= 0:
                return False
            if dt <= .5:
                mid = self.yaw+.5*w*dt
                self.x += v*dt*math.cos(mid); self.y += v*dt*math.sin(mid)
                self.yaw = wrap(self.yaw+w*dt)
        self.stamp = stamp
        return True

    def xy(self):
        return np.array([self.x, self.y])

    def to_odom(self, distance, bearing):
        heading = self.yaw+bearing
        return self.xy()+distance*np.array([math.cos(heading), math.sin(heading)])

    def to_robot(self, point):
        delta = np.asarray(point, dtype=float)-self.xy()
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([c*delta[0]+s*delta[1], -s*delta[0]+c*delta[1]])


class VisitHeatmap:
    """Coarse "been here" map: seconds spent per cell, fading with time constant tau."""
    def __init__(self, cell=.5, tau=240.):
        self.cell, self.tau = cell, tau
        self.cells = {}

    def _key(self, xy):
        return (int(math.floor(xy[0]/self.cell)), int(math.floor(xy[1]/self.cell)))

    def _decayed(self, key, now):
        value, stamp = self.cells.get(key, (0., now))
        return value*math.exp(-max(0., now-stamp)/self.tau)

    def add(self, xy, now, seconds):
        key = self._key(xy)
        self.cells[key] = (self._decayed(key, now)+seconds, now)

    def level(self, xy, now, radius=1):
        """0 = never visited lately, -> 1 = visited for a long time (3x3 cells, centre weighted)."""
        i, j = self._key(xy); total = 0.
        for di in range(-radius, radius+1):
            for dj in range(-radius, radius+1):
                weight = 1. if di == dj == 0 else .35
                total += weight*self._decayed((i+di, j+dj), now)
        return 1.-math.exp(-total/20.)

    def prune(self, now, floor=.05):
        self.cells = {k: v for k, v in self.cells.items() if self._decayed(k, now) >= floor}


class ObjectInstance:
    __slots__ = ('ident', 'label', 'xy', 'first_seen', 'last_seen', 'sightings', 'last_sighting',
                 'visits', 'cooldown_until', 'moved_at', 'touched_at', 'failures')

    def __init__(self, ident, label, xy, now):
        self.ident, self.label, self.xy = ident, label, np.asarray(xy, dtype=float).copy()
        self.first_seen = self.last_seen = self.last_sighting = now
        self.sightings, self.visits, self.failures = 1, 0, 0
        self.cooldown_until = -math.inf
        self.moved_at = self.touched_at = -math.inf

    def summary(self, now):
        return dict(id=self.ident, label=self.label, xy=[round(float(v), 2) for v in self.xy],
                    sightings=self.sightings, visits=self.visits, age_s=round(now-self.first_seen, 1),
                    last_seen_s=round(now-self.last_seen, 1),
                    cooldown_s=round(max(0., self.cooldown_until-now), 1))


class ObjectMemory:
    """Detections clustered into instances by label + odom position (no tracker ids)."""
    def __init__(self, merge_radius=.6, moved_threshold=.35, sighting_period=1., touch_radius=.6,
                 max_instances=200):
        self.merge_radius, self.moved_threshold = merge_radius, moved_threshold
        self.sighting_period, self.touch_radius = sighting_period, touch_radius
        self.max_instances = max_instances
        self.instances = []
        self.next_id = 1

    def nearest(self, label, xy, radius):
        best, best_d = None, radius
        for inst in self.instances:
            if inst.label != label:
                continue
            d = float(np.linalg.norm(inst.xy-xy))
            if d <= best_d:
                best, best_d = inst, d
        return best

    def observe(self, label, xy, now):
        xy = np.asarray(xy, dtype=float)
        inst = self.nearest(label, xy, self.merge_radius)
        if inst is None:
            inst = ObjectInstance(self.next_id, label, xy, now); self.next_id += 1
            self.instances.append(inst)
            if len(self.instances) > self.max_instances:
                self.instances.remove(min(self.instances, key=lambda o: o.last_seen))
            return inst
        if np.linalg.norm(inst.xy-xy) > self.moved_threshold and now-inst.last_seen > 2.:
            inst.moved_at = now
        inst.xy = .7*inst.xy+.3*xy
        inst.last_seen = now
        if now-inst.last_sighting >= self.sighting_period:
            inst.sightings += 1; inst.last_sighting = now
        return inst

    def touch(self, person_xy, now):
        for inst in self.instances:
            if np.linalg.norm(inst.xy-person_xy) <= self.touch_radius:
                inst.touched_at = now

    def label_count(self, label):
        return sum(1 for inst in self.instances if inst.label == label)

    def get(self, ident):
        return next((inst for inst in self.instances if inst.ident == ident), None)


class SectorMemory:
    """Free reach per absolute heading sector, measured while the sector was in view."""
    def __init__(self, count=12, fov_half=math.radians(35)):
        self.count, self.fov_half = count, fov_half
        self.width = 2*math.pi/count
        self.reach = np.zeros(count); self.wide = np.zeros(count); self.stamp = np.full(count, -math.inf)

    def heading(self, index):
        return wrap(index*self.width)

    def update(self, pose, now, measure):
        """measure(bearing_in_robot_frame) -> (reach_m, wide_fraction) or None."""
        for index in range(self.count):
            bearing = wrap(self.heading(index)-pose.yaw)
            if abs(bearing) > self.fov_half:
                continue
            value = measure(bearing)
            if value is None:
                continue
            self.reach[index], self.wide[index] = value; self.stamp[index] = now

    def fresh(self, now, max_age):
        return [i for i in range(self.count) if now-self.stamp[i] <= max_age]
