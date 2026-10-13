"""Relative person control: explicit identity, bounded leases, no world frame."""
from dataclasses import dataclass
import json
import math


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def source_stamp(data):
    stamp = data.get('stamp_ns')
    if stamp is None:
        header = data['stamp']
        sec, ns = header['sec'], header['nanosec']
        if type(sec) is not int or type(ns) is not int or not 0 <= ns < 1_000_000_000:
            raise ValueError('invalid_stamp')
        stamp = sec * 1_000_000_000 + ns
    if type(stamp) is not int or stamp <= 0:
        raise ValueError('invalid_stamp')
    if 'stamp' in data:
        other = data['stamp']
        if (type(other.get('sec')) is not int or type(other.get('nanosec')) is not int or
                not 0 <= other['nanosec'] < 1_000_000_000 or
                other['sec'] * 1_000_000_000 + other['nanosec'] != stamp):
            raise ValueError('conflicting_stamp')
    return stamp


@dataclass(frozen=True)
class Safety:
    valid: bool = False
    front_distance: float = 0.
    forward_clear: bool = False


class RelativeFollower:
    """All clocks passed explicitly so replay, expiry and mode changes are testable."""
    def __init__(self, base_frame='r680_mapping_floor', max_age=.30,
                 follow_distance=1.4, approach_distance=.95,
                 max_linear=.20, max_angular=.45, emergency_distance=.60, local_planner=None):
        if (not base_frame or not 0 < max_age <= .30 or
                not 0 < max_linear <= .20 or not 0 < max_angular <= .45 or
                not .50 <= emergency_distance < approach_distance <= follow_distance):
            raise ValueError('invalid_control_configuration')
        self.local_planner = local_planner
        self.base_frame, self.max_age = base_frame, max_age
        self.follow_distance, self.approach_distance = follow_distance, approach_distance
        self.max_linear, self.max_angular = max_linear, max_angular
        self.emergency_distance = emergency_distance
        self.session, self.epoch, self.last_stamp = None, -1, 0
        self.retired_sessions = set()
        self.gate_enabled = False
        self.objects, self.expires_at = {}, -math.inf
        self.status_stamp, self.disabled_stamp = 0, 0
        self.request_id = self.mode = self.target_id = ''
        self.active, self.state, self.reason = False, 'idle', 'waiting_for_command'
        self.lease_until = -math.inf
        self.requests = {}
        self.arrival_count, self.arrival_stamp = 0, 0
        self.previous_v = self.previous_w = 0.
        self.previous_tick = None

    def _zero(self):
        self.previous_v = self.previous_w = 0.
        return 0., 0.

    def _terminate(self, state, reason):
        self.active, self.state, self.reason = False, state, reason
        if self.local_planner is not None:
            self.local_planner.reset()
        self.lease_until = -math.inf
        self.arrival_count, self.arrival_stamp = 0, 0
        self._zero()

    def _generation(self, session, epoch):
        if (not isinstance(session, str) or not session or ':' in session or len(session) > 128 or
                type(epoch) is not int or epoch < 0):
            raise ValueError('invalid_generation')
        if session in self.retired_sessions or (session == self.session and epoch < self.epoch):
            raise ValueError('retired_generation')
        changed = (session, epoch) != (self.session, self.epoch)
        if changed:
            if self.session is not None and session != self.session:
                if len(self.retired_sessions) >= 1024:
                    raise ValueError('too_many_detector_restarts')
                self.retired_sessions.add(self.session)
            if self.active:
                self._terminate('cancelled', 'detector_generation_changed')
            self.session, self.epoch = session, epoch
            self.objects, self.expires_at, self.last_stamp, self.status_stamp = {}, -math.inf, 0, 0
            self.disabled_stamp = 0
            self.gate_enabled = False
        return changed

    def receive(self, payload, now, ros_ns):
        """Accept one acquisition-stamped batch; malformed input revokes its evidence."""
        try:
            data = json.loads(payload) if isinstance(payload, str) else payload
            if (data.get('schema') != 'wla-relative-detections/v1' or
                    data.get('base_frame') != self.base_frame or type(data.get('enabled')) is not bool):
                raise ValueError('invalid_relative_envelope')
            stamp = source_stamp(data)
            age = (ros_ns - stamp) / 1e9
            if not -.05 <= age <= self.max_age:
                raise ValueError('expired_relative_frame')
            rows = data['objects']
            if not isinstance(rows, list) or len(rows) > 64:
                raise ValueError('invalid_objects')
            # Validate all rows before mutating the live detector generation.
            session, epoch = data['session'], data['epoch']
            prefix = f'{session}:{epoch}:'
            accepted = {}
            for row in rows:
                if (not isinstance(row, dict) or set(row) != {'detected','label','id','yaw','distance'} or
                        type(row.get('detected')) is not bool or
                        not isinstance(row.get('label'), str) or not row['label'] or
                        not isinstance(row.get('id'), str) or not row['id'].startswith(prefix) or
                        not row['id'][len(prefix):].isdigit() or row['id'] in accepted):
                    raise ValueError('invalid_relative_object')
                yaw, distance = row.get('yaw'), row.get('distance')
                if ((yaw is not None and (not finite(yaw) or abs(yaw) > math.pi)) or
                        (distance is not None and (not finite(distance) or not 0 < distance <= 20)) or
                        ((yaw is None) != (distance is None))):
                    raise ValueError('invalid_relative_geometry')
                accepted[row['id']] = dict(row)
            self._generation(session, epoch)
            if stamp <= self.last_stamp:
                raise ValueError('replayed_relative_frame')
            if data['enabled'] and stamp <= self.disabled_stamp:
                raise ValueError('frame_precedes_disable')
            self.last_stamp = stamp
            self.gate_enabled = data['enabled']
            self.objects = accepted if self.gate_enabled else {}
            self.expires_at = now + max(0., self.max_age - max(age, 0.))
            if not self.gate_enabled:
                if self.active:
                    self._terminate('cancelled', 'detection_disabled')
                else:
                    self._zero()
            return True
        except (ValueError, KeyError, TypeError, AttributeError):
            self.objects, self.expires_at = {}, -math.inf
            self._zero()
            return False

    def receive_status(self, payload, now, ros_ns):
        """A gate heartbeat may revoke execution; it never renews target evidence."""
        try:
            data = json.loads(payload) if isinstance(payload, str) else payload
            if (data.get('schema') != 'wla-relative-perception-status/v1' or
                    type(data.get('enabled')) is not bool):
                return False
            stamp = source_stamp(data)
            if not -.05 <= (ros_ns-stamp)/1e9 <= self.max_age:
                return False
            self._generation(data['session'], data['epoch'])
            if stamp <= self.status_stamp:
                return False
            self.status_stamp = stamp
            if not data['enabled']:
                self.disabled_stamp = stamp
                self.gate_enabled = False
                self.objects, self.expires_at = {}, -math.inf
                if self.active:
                    self._terminate('cancelled', 'detection_disabled')
                else:
                    self._zero()
            return True
        except (ValueError, KeyError, TypeError, AttributeError):
            return False

    def _expire(self, now):
        if self.active and now >= self.lease_until:
            self._terminate('expired', 'command_lease_expired')

    def command(self, command, request_id='', mode='', target_id='', lease_duration_s=1., now=0.):
        self._expire(now)
        if command == 'status':
            return True, 'status'
        if command == 'stop':
            if request_id and request_id != self.request_id:
                if not isinstance(request_id, str) or len(request_id) > 256:
                    return False, 'invalid_request_id'
                if request_id not in self.requests:
                    if len(self.requests) >= 10000:
                        return False, 'request_history_full_restart_required'
                    self.requests[request_id] = None  # A stop may overtake an in-flight start.
                return True, 'request_cancelled_before_start'
            self._terminate('stopped', 'stopped_by_agent')
            return True, 'stopped'
        if command not in ('start', 'heartbeat'):
            return False, 'unknown_command'
        if not isinstance(request_id, str) or not request_id or len(request_id) > 256:
            return False, 'invalid_request_id'
        if not finite(lease_duration_s) or not .1 <= lease_duration_s <= 2.:
            return False, 'invalid_lease_duration'
        if command == 'heartbeat':
            if not self.active or request_id != self.request_id:
                return False, 'no_matching_active_request'
            self.lease_until = now + lease_duration_s
            return True, 'lease_renewed'
        if mode not in ('follow', 'approach_interact'):
            return False, 'invalid_mode'
        if request_id in self.requests:
            if self.requests[request_id] is None:
                return False, 'request_already_terminal'
            if self.requests[request_id] != (mode, target_id):
                return False, 'request_id_conflict'
            if self.active and request_id == self.request_id:
                return True, 'already_active'
            return False, 'request_already_terminal'
        if len(self.requests) >= 10000:
            return False, 'request_history_full_restart_required'
        row = self.objects.get(target_id)
        if not self.gate_enabled:
            return False, 'detection_disabled'
        if now >= self.expires_at:
            return False, 'observations_stale'
        if row is None or not row['detected'] or row['label'] != 'person':
            return False, 'person_id_not_visible'
        if row['yaw'] is None or row['distance'] is None:
            return False, 'person_geometry_unavailable'
        if self.local_planner is not None:
            self.local_planner.reset()
        self.request_id, self.mode, self.target_id = request_id, mode, target_id
        self.requests[request_id] = (mode, target_id)
        self.active, self.state, self.reason = True, 'waiting', 'waiting_for_safe_depth'
        self.lease_until = now + lease_duration_s
        self.arrival_count, self.arrival_stamp = 0, 0
        self.previous_tick = now
        self._zero()
        return True, 'started'

    def tick(self, now, safety=None, scene=None):
        self._expire(now)
        previous_tick, self.previous_tick = self.previous_tick, now
        dt = .05 if previous_tick is None else max(.001, min(.10, now-previous_tick))
        if not self.active:
            return self._zero()
        row = self.objects.get(self.target_id)
        reason = ('detection_disabled' if not self.gate_enabled else
                  'observations_stale' if now >= self.expires_at else
                  'target_not_visible' if row is None or not row['detected'] or row['label'] != 'person' else
                  'target_geometry_unavailable' if row['yaw'] is None or row['distance'] is None else
                  'depth_safety_unavailable' if safety is None or not safety.valid else
                  'emergency_obstacle' if safety.front_distance < self.emergency_distance else
                  'local_scene_unavailable' if self.local_planner is not None and scene is None else '')
        if reason:
            self.state, self.reason = 'waiting', reason
            self.arrival_count = 0
            return self._zero()
        yaw, distance = row['yaw'], row['distance']
        if distance < self.emergency_distance:
            self.state, self.reason = 'waiting', 'target_too_close'
            self.arrival_count = 0
            return self._zero()
        target_distance = self.follow_distance if self.mode == 'follow' else self.approach_distance
        arrived = distance <= target_distance+.10 and abs(yaw) <= .12
        if self.mode == 'approach_interact':
            if self.last_stamp != self.arrival_stamp:
                self.arrival_count = self.arrival_count + 1 if arrived else 0
                self.arrival_stamp = self.last_stamp
            if self.arrival_count >= 3:
                self._terminate('arrived', 'interaction_distance_reached')
                return self._zero()
        if self.local_planner is not None:
            v, w = self.local_planner.command(scene, yaw, distance, target_distance,
                                               self.max_linear, self.max_angular)
            if not self.local_planner.last.path:
                self.state, self.reason = 'waiting', self.local_planner.last.reason
                return self._zero()
        else:
            v = max(0., min(self.max_linear, .45*(distance-target_distance)))
            if distance <= target_distance+.10 or abs(yaw) >= .55 or not safety.forward_clear:
                v = 0.
            w = 0. if abs(yaw) <= .08 else max(-self.max_angular, min(self.max_angular, 1.2*yaw))
        # Stops are immediate; only nonzero commands ramp upward from rest.
        v = min(v, self.previous_v+.4*dt) if v else 0.
        if w:
            if w*self.previous_w < 0:
                self.previous_w = 0.
            w = max(self.previous_w-.8*dt, min(self.previous_w+.8*dt, w))
        if self.local_planner is not None and not scene.command_clear(v, w):
            v = 0.
            if not scene.command_clear(v, w):
                w = 0.
        self.previous_v, self.previous_w = v, w
        self.state = 'approaching' if self.mode == 'approach_interact' else 'following'
        self.reason = (self.local_planner.last.reason if self.local_planner is not None else
                       'obstacle_blocks_forward' if not safety.forward_clear and distance > target_distance+.10 else 'relative_control')
        return v, w

    def snapshot(self, now):
        self._expire(now)
        return dict(request_id=self.request_id, mode=self.mode, target_id=self.target_id,
                    enabled=self.active, state=self.state, reason=self.reason,
                    session=self.session or '', epoch=self.epoch,
                    detection_enabled=self.gate_enabled,
                    observations_fresh=now < self.expires_at,
                    lease_remaining_s=max(0., self.lease_until-now))
