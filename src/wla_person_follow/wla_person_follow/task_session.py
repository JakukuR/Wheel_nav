"""Target task ledger: explicit identity, bounded leases, no motion controller."""
from .task_modes import normalize_mode
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


class TaskSession:
    """Identity, detector epoch, idempotence and lease ledger; no controller."""
    def __init__(self, base_frame='r680_mapping_floor', max_age=.30):
        if not base_frame or not 0 < max_age <= .45:  # 0.40 since 2026-10-30
            raise ValueError('invalid_task_configuration')
        self.base_frame, self.max_age = base_frame, max_age
        self.session, self.epoch, self.last_stamp = None, -1, 0
        self.retired_sessions = set()
        self.gate_enabled = False
        self.objects, self.expires_at = {}, -math.inf
        self.status_stamp, self.disabled_stamp = 0, 0
        self.request_id = self.mode = self.target_id = self.target_label = ''
        self.active, self.state, self.reason = False, 'idle', 'waiting_for_command'
        self.lease_until = -math.inf
        self.requests = {}

    def _terminate(self, state, reason):
        self.active, self.state, self.reason = False, state, reason
        self.lease_until = -math.inf

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
            return True
        except (ValueError, KeyError, TypeError, AttributeError):
            self.objects, self.expires_at = {}, -math.inf
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
            # A client's finally/cleanup stop must not overwrite the first cause.
            if self.active or not self.request_id:
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
        try:
            mode = normalize_mode(mode)
        except ValueError as exc:
            return False, str(exc)
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
        valid_label = row is not None and ((row['label'] == 'person') if mode != '2' else row['label'] != 'person')
        if not self.gate_enabled:
            return False, 'detection_disabled'
        if now >= self.expires_at:
            return False, 'observations_stale'
        if row is None or not row['detected'] or not valid_label:
            return False, 'target_id_not_visible'
        if row['yaw'] is None or row['distance'] is None:
            return False, 'target_geometry_unavailable'
        self.target_label = row['label']
        self.request_id, self.mode, self.target_id = request_id, mode, target_id
        self.requests[request_id] = (mode, target_id)
        self.active, self.state, self.reason = True, 'waiting', 'waiting_for_safe_depth'
        self.lease_until = now + lease_duration_s
        return True, 'started'

    def snapshot(self, now):
        self._expire(now)
        return dict(request_id=self.request_id, mode=self.mode, target_id=self.target_id, target_label=self.target_label,
                    enabled=self.active, state=self.state, reason=self.reason,
                    session=self.session or '', epoch=self.epoch,
                    detection_enabled=self.gate_enabled,
                    observations_fresh=now < self.expires_at,
                    lease_remaining_s=max(0., self.lease_until-now))
