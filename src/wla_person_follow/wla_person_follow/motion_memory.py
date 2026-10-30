"""Short wheel-twist integration used by the unified tracker/MPC adapter."""
from collections import deque
import math
def advance(pose, twist, dt):
    x, y, angle = pose; vx, vy, w = twist
    mid = angle+w*dt*.5
    return (x+(vx*math.cos(mid)-vy*math.sin(mid))*dt,
            y+(vx*math.sin(mid)+vy*math.cos(mid))*dt, angle+w*dt)


class LocalMotion:
    def __init__(self):
        self.samples = deque(maxlen=128)
        self.generation = 0

    def update(self, stamp, twist, now):
        if not all(math.isfinite(v) for v in (stamp, now, *twist)):
            self.samples.clear(); self.generation += 1; return False
        if not 0 <= now-stamp <= .20 or abs(twist[0]) > 1.5 or abs(twist[1]) > .3 or abs(twist[2]) > 2.:
            self.samples.clear(); self.generation += 1; return False
        pose = (0., 0., 0.)
        if self.samples:
            previous, prior, velocity = self.samples[-1]
            dt = stamp-previous
            if dt <= 0: return False
            if dt > .20:
                self.samples.clear(); self.generation += 1
            else:
                pose = advance(prior, tuple((a+b)*.5 for a,b in zip(velocity,twist)), dt)
        self.samples.append((stamp, pose, tuple(twist))); return True

    def snapshot(self):
        """Independent copy for a compute job: samples are immutable tuples, so
        copying the deque is enough (deepcopy cost ~0.9 ms per call on the car)."""
        copy = LocalMotion.__new__(LocalMotion)
        copy.samples = deque(self.samples, maxlen=self.samples.maxlen)
        copy.generation = self.generation
        return copy

    def healthy(self, now):
        return bool(self.samples and 0 <= now-self.samples[-1][0] <= .20)

    def pose_at(self, stamp):
        if not self.samples: return None
        values=list(self.samples)
        if stamp < values[0][0] or stamp-values[-1][0] > .05: return None
        for t, pose, twist in reversed(values):
            if t <= stamp: return advance(pose, twist, stamp-t)
        return None
