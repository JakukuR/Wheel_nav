"""Geometry-owned target binding, independent of ReID enrollment.

The selected detector track owns normal following.  Appearance may supervise or
reacquire a broken track, but it cannot revoke a continuous same-ID binding.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class GeometryBindingConfig:
    # New values are explicitly provisional until replayed with real data.
    freshness_s: float = .30
    missing_grace_s: float = .20
    jump_radius_m: float = .45
    jump_growth_mps: float = .25
    jump_radius_max_m: float = 1.50
    active_search_s: float = 8.
    release_s: float = 30.
    surface_to_center_offset_m: float = .28


class GeometryBinding:
    """Own one selected detector ID until a hard geometry break."""

    def __init__(self, target_id, start_time, config=GeometryBindingConfig()):
        if not isinstance(target_id, str) or not target_id:
            raise ValueError('invalid_geometry_target_id')
        if not self._valid_config(config):
            raise ValueError('invalid_geometry_binding_config')
        self.locked_id = target_id
        self.resolved_id = target_id
        self.c = config
        self.start_time = float(start_time)
        self.position = None
        self.velocity = np.zeros(2)
        self.last_seen = None
        self.last_source_stamp = -math.inf
        self.lost_since = None
        self.state = 'binding'
        self.reason = 'waiting_for_initial_geometry'
        self.source = 'user_selected_geometry_pending'
        self.last_error = None
        self.last_prediction = None
        self.breaks = 0

    @staticmethod
    def _valid_config(c):
        values=(c.freshness_s,c.missing_grace_s,c.jump_radius_m,c.jump_growth_mps,
                c.jump_radius_max_m,c.active_search_s,c.release_s,c.surface_to_center_offset_m)
        return (all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in values)
                and 0 < c.freshness_s <= .30 and 0 <= c.missing_grace_s <= 1.
                and 0 < c.jump_radius_m <= c.jump_radius_max_m <= 2.
                and 0 <= c.jump_growth_mps <= 1. and 0 < c.active_search_s < c.release_s <= 60.
                and 0 <= c.surface_to_center_offset_m <= .5)

    def prediction(self, t):
        if self.position is None:
            return None
        anchor = self.start_time if self.last_seen is None else self.last_seen
        return self.position + self.velocity * min(max(0., t-anchor), 1.5)

    def radius(self, age):
        return min(self.c.jump_radius_max_m,
                   self.c.jump_radius_m + self.c.jump_growth_mps * max(0., age))

    def _row_position(self, row, pose):
        yaw = row.get('yaw')
        distance = row.get('distance')
        pose = np.asarray(pose, dtype=float)
        if (row.get('detected') is not True or row.get('label') != 'person'
                or not isinstance(yaw,(int,float)) or isinstance(yaw,bool) or not math.isfinite(yaw)
                or not isinstance(distance,(int,float)) or isinstance(distance,bool)
                or not math.isfinite(distance) or distance <= 0 or distance > 20
                or pose.shape[0] < 3 or not np.isfinite(pose[:3]).all()):
            return None
        bearing = float(pose[2] + yaw)
        center_distance = float(distance + self.c.surface_to_center_offset_m)
        return pose[:2] + center_distance*np.array([math.cos(bearing), math.sin(bearing)])

    def _begin_loss(self, now, reason):
        if self.lost_since is None:
            self.lost_since = float(now)
        self.state = 'lost'
        self.reason = reason
        self.resolved_id = ''
        self.source = 'geometry_lost'
        self.breaks += 1

    def update(self, objects, pose, stamp, now, *, enrollment_complete=False,
               suspect_reason=None):
        """Accept current geometry and return the bound ID or ``None``.

        Missing rows are owned for ``freshness_s + missing_grace_s``.  A known
        row with a position jump breaks immediately and receives no grace.
        """
        stamp=float(stamp);now=float(now)
        if not math.isfinite(stamp) or not math.isfinite(now) or now < stamp-.05:
            self._begin_loss(now,'invalid_geometry_time')
            return None
        self.last_source_stamp=max(self.last_source_stamp,stamp)
        row=objects.get(self.resolved_id or self.locked_id) if isinstance(objects,dict) else None
        current_position=self._row_position(row,pose) if row is not None else None
        fresh=0. <= now-stamp <= self.c.freshness_s
        if current_position is not None and fresh:
            prior=self.prediction(stamp)
            error=None if prior is None else float(np.linalg.norm(current_position-prior))
            limit=None if self.last_seen is None else self.radius(stamp-self.last_seen)
            self.last_error=error
            if error is not None and error > limit:
                self._begin_loss(now,'current_position_jump')
                return None
            if self.last_seen is not None and .05 < stamp-self.last_seen < .8:
                raw=(current_position-self.position)/(stamp-self.last_seen)
                speed=float(np.linalg.norm(raw))
                raw*=min(1.,1.5/max(speed,1e-9))
                self.velocity=.6*self.velocity+.4*raw
            elif self.last_seen is not None and stamp-self.last_seen >= .8:
                self.velocity[:]=0.
            self.position=current_position.copy()
            self.last_seen=stamp
            self.lost_since=None
            self.resolved_id=self.locked_id
            self.source='geometry_same_id'
            self.last_prediction=current_position.tolist()
            if suspect_reason:
                self.state='suspect';self.reason=suspect_reason
            else:
                self.state='bound_tracking' if enrollment_complete else 'bound_untracked'
                self.reason='identity_enrolled' if enrollment_complete else 'enrollment_background'
            return self.resolved_id

        # A missing target row is not a position jump.  Preserve ownership for
        # the existing source freshness plus the configured post-freshness grace.
        anchor=self.last_seen if self.last_seen is not None else self.start_time
        age=max(0.,now-anchor)
        keep_for=self.c.freshness_s+self.c.missing_grace_s
        if age <= keep_for:
            self.state='bound_coasting'
            self.reason='target_measurement_missing_grace'
            self.source='geometry_missing_grace'
            self.last_prediction=(None if self.prediction(now) is None else self.prediction(now).tolist())
            return self.resolved_id
        self._begin_loss(now,'target_geometry_lost_timeout')
        return None

    def suspect(self, reason):
        if self.state in ('lost','ended'):
            return
        self.state='suspect'
        self.reason=reason

    def exit_suspect(self, *, enrollment_complete=False):
        if self.state=='suspect':
            self.state='bound_tracking' if enrollment_complete else 'bound_untracked'
            self.reason='identity_enrolled' if enrollment_complete else 'enrollment_background'

    def rebind(self, target_id, objects, pose, stamp, now, *, enrollment_complete=True):
        if not isinstance(target_id,str) or not target_id:
            raise ValueError('invalid_rebound_target_id')
        position=self._row_position((objects or {}).get(target_id),pose)
        self.locked_id=target_id;self.resolved_id=target_id
        self.position=None;self.velocity[:]=0.;self.last_seen=None;self.lost_since=None
        self.last_source_stamp=float(stamp);self.last_error=None;self.last_prediction=None
        self.state='bound_tracking' if enrollment_complete else 'bound_untracked'
        self.reason='reid_reacquired';self.source='reid_reacquired'
        if position is not None:
            self.position=position.copy();self.last_seen=float(stamp)
        return target_id

    def recovery_phase(self, now):
        if self.state!='lost' or self.lost_since is None:
            return 'none'
        age=max(0.,float(now)-self.lost_since)
        if age < self.c.active_search_s:
            return 'active_search'
        if age < self.c.release_s:
            return 'passive_reacquire'
        return 'release_due'

    def snapshot(self, now):
        age=None if self.last_seen is None else max(0.,float(now)-self.last_seen)
        pred=self.prediction(now)
        return dict(state=self.state,reason=self.reason,locked_id=self.locked_id,
            resolved_id=self.resolved_id or None,source=self.source,last_seen_age_s=age,
            position=None if self.position is None else self.position.tolist(),
            velocity_mps=self.velocity.tolist(),
            prediction=None if pred is None else pred.tolist(),last_position_error_m=self.last_error,
            position_jump_limit_m=None if age is None else self.radius(age),
            recovery_phase=self.recovery_phase(now),
            lost_s=None if self.lost_since is None else max(0.,float(now)-self.lost_since),
            breaks=self.breaks)
