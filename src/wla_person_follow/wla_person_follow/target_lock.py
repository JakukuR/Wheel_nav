"""Selected-person identity gate. No automatic target acquisition."""
from dataclasses import dataclass
import numpy as np
from .kalman import PositionKalman


@dataclass
class Observation:
    track_id: int
    box: tuple
    appearance: np.ndarray
    world_xy: np.ndarray
    base_xy: np.ndarray
    confidence: float


class TargetLock:
    def __init__(self, horizon=1.2):
        self.horizon = float(horizon)
        self.clear()

    def clear(self):
        self.track_id = None
        self.appearance = None
        self.filter = None
        self.last_seen = None
        self.pending_id = None
        self.pending_count = 0
        self.mode = 'idle'
        self.last_box = None

    def select(self, observation, timestamp):
        self.clear()
        self.track_id = observation.track_id
        self.appearance = observation.appearance.copy()
        self.filter = PositionKalman(observation.world_xy, timestamp)
        self.last_seen = float(timestamp)
        self.last_box = observation.box
        self.mode = 'tracking'

    def update(self, observations, timestamp):
        if self.filter is None:
            return None, 'idle'
        predicted = self.filter.predict(timestamp)
        age = float(timestamp) - self.last_seen
        if age > self.horizon:
            self.mode = 'lost'
            return None, self.mode
        plausible = []
        for obs in observations:
            distance = float(np.linalg.norm(obs.world_xy - predicted))
            similarity = float(np.dot(obs.appearance, self.appearance))
            same_id = obs.track_id == self.track_id
            # ID alone is insufficient: reject an implausible ByteTrack handoff.
            limit = .50 + .35 * age
            minimum = .83 if same_id else .92
            if distance <= limit and similarity >= minimum:
                score = distance + 1.8 * (1. - similarity)
                plausible.append((score, obs, same_id))
        plausible.sort(key=lambda item: item[0])
        if len(plausible) > 1 and plausible[1][0] - plausible[0][0] < .35:
            self.mode = 'ambiguous'
            return None, self.mode
        if not plausible:
            self.pending_id = None
            self.pending_count = 0
            self.mode = 'predicting'
            return predicted, self.mode
        _, best, same_id = plausible[0]
        if not same_id:
            if best.track_id == self.pending_id:
                self.pending_count += 1
            else:
                self.pending_id = best.track_id
                self.pending_count = 1
            if self.pending_count < 3:
                self.mode = 'recovering'
                return None, self.mode
            self.track_id = best.track_id
        self.pending_id = None
        self.pending_count = 0
        updated = self.filter.update(best.world_xy, timestamp)
        self.appearance = self.appearance * .9 + best.appearance * .1
        self.appearance /= np.linalg.norm(self.appearance)
        self.last_seen = float(timestamp)
        self.last_box = best.box
        self.mode = 'tracking'
        return updated, self.mode

    def position(self, timestamp):
        if self.filter is None or self.mode in ('idle', 'lost', 'ambiguous', 'recovering'):
            return None
        if timestamp - self.last_seen > self.horizon:
            self.mode = 'lost'
            return None
        return self.filter.predict(timestamp)
