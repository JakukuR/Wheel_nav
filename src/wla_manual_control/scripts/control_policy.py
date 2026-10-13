"""Sole chassis arbiter policy; no estimator or map dependency."""
import math


class Policy:
    STATUS_SCHEMA = 'wla-person-status/v1'
    COMMAND_SCHEMA = 'wla-person-candidate/v1'
    PERSON_COMMAND_AGE = .15
    PERSON_STATUS_AGE = .20

    def __init__(self, linear=.3, reverse=.3, angular=.5):
        self.limits = (linear, reverse, angular)
        self.mode = 'MANUAL'; self.armed = False
        self.manual_ready_after = -1e9
        self.estop_latched = False; self.estop_reason = ''
        self.gamepad_stop_asserted = False
        self.manual = (0., 0.); self.auto = (0., 0.)
        self.manual_at = self.auto_at = self.status_at = self.wheel_at = -1e9
        self.status = 'DISCONNECTED'; self.wheel = (0., 0.); self.auto_after = -1e9
        self.automatic_source = 'NAV'
        self.source_generation = 0; self.person_revoke_generation = 0
        self.source_after = -1e9; self.source_ros_ns = 0; self.auto_ros_ns = 0
        self.person = (0., 0.); self.person_at = self.person_status_at = -1e9
        self.person_until = self.person_status_until = -1e9
        self.person_stamp_ns = self.person_status_stamp_ns = 0
        self.person_request_id = ''; self.person_enabled = False
        self.person_session = ''; self.person_activation_ns = 0
        self.person_candidate_only = True; self.person_motion_permitted = False
        self.person_fault = False
        self.blocked_person_request_id = ''
        self.blocked_person_session = ''
        self.object_channel = ObjectChannel(self)

    @staticmethod
    def zero(v):
        return max(abs(x) for x in v) < .01

    def clear_person(self):
        self.person = (0., 0.); self.person_at = self.person_status_at = -1e9
        self.person_until = self.person_status_until = -1e9
        self.person_stamp_ns = self.person_status_stamp_ns = 0
        self.person_request_id = ''; self.person_enabled = False
        self.person_session = ''; self.person_activation_ns = 0
        self.person_candidate_only = True; self.person_motion_permitted = False
        self.person_fault = False

    def revoke_person(self):
        if self.person_request_id:
            self.blocked_person_request_id = self.person_request_id
            self.blocked_person_session = self.person_session
        self.person_revoke_generation += 1
        self.clear_person()

    def set_manual(self):
        if self.mode == 'AUTO' and self.automatic_source in ('PERSON','OBJECT'):
            self.tracking_channel().revoke_person()
        self.mode = 'MANUAL'

    def manual_input(self, v, t):
        if not all(math.isfinite(x) for x in v):
            self.latch_stop('invalid_manual_input'); return False
        self.manual = v; self.manual_at = t
        takeover = self.mode == 'AUTO' and self.manual_ready(t) and not self.zero(v)
        if takeover:
            self.set_manual()
        return takeover

    def gamepad_status_input(self, status, t):
        was_ready = self.status in ('ACTIVE', 'SPIN', 'STANDBY') and t - self.status_at < .5
        self.status = status; self.status_at = t
        ready = status in ('ACTIVE', 'SPIN', 'STANDBY')
        if not ready or not was_ready:
            self.clear_manual()
            self.manual_ready_after = t
        if status == 'ESTOP':
            self.gamepad_stop_asserted = True
            self.latch_stop('gamepad_estop')
        elif ready or status == 'ARMING':
            # A fresh released-button status clears the input, never the mux latch.
            self.gamepad_stop_asserted = False
        elif status != 'DISCONNECTED':
            self.latch_stop('invalid_gamepad_status')

    def clear_manual(self):
        self.armed = False; self.manual = (0., 0.); self.manual_at = -1e9

    def manual_ready(self, t):
        return (not self.estop_latched and self.connected(t)
                and self.manual_at > self.manual_ready_after)

    def manual_requested(self, t):
        return self.manual_ready(t) and not self.zero(self.manual)

    def latch_stop(self, reason):
        self.estop_latched = True; self.estop_reason = reason
        self.disarm()
        self.auto = (0., 0.); self.auto_at = -1e9

    def set_estop(self, enabled, t):
        if enabled:
            self.latch_stop('explicit_estop')
            return True, 'stop_latched; explicit reset and AUTO authorization required'
        if not self.estop_latched:
            return True, 'stop_already_clear'
        if self.gamepad_stop_asserted:
            return False, 'gamepad_estop_still_asserted'
        if not self.stopped(t):
            return False, 'chassis_not_stopped'
        if self.connected(t) and not self.zero(self.manual):
            return False, 'manual_input_not_neutral'
        if self.estop_latched:
            self.estop_latched = False; self.estop_reason = ''
            self.disarm()
        return True, 'stop_cleared; MANUAL; explicit AUTO required'

    def automatic_input(self, v, t):
        if all(math.isfinite(x) for x in v):
            self.auto = v; self.auto_at = t
        else:
            self.auto = (0., 0.); self.auto_at = -1e9

    @staticmethod
    def valid_stamp(message, now_ns, max_age):
        stamp = message.get('stamp_ns')
        return (type(stamp) is int and stamp > 0
                and -.05 <= (now_ns - stamp) * 1e-9 <= max_age)

    def person_status_input(self, message, t, now_ns):
        valid = (isinstance(message, dict)
                 and message.get('schema') == self.STATUS_SCHEMA
                 and self.valid_stamp(message, now_ns, self.PERSON_STATUS_AGE)
                 and isinstance(message.get('request_id'), str)
                 and isinstance(message.get('follow_session'), str) and bool(message['follow_session'])
                 and ('fault' not in message or type(message['fault']) is bool)
                 and all(type(message.get(k)) is bool for k in
                         ('enabled', 'candidate_only', 'motion_permitted')))
        if not valid:
            self.person_status_at = self.person_status_until = -1e9
            return False
        stamp = message['stamp_ns']
        if stamp <= self.source_ros_ns or stamp <= self.person_status_stamp_ns:
            return False
        request_id = message['request_id']
        session = message['follow_session']
        if message['enabled'] and (not request_id or (session, request_id) ==
                                   (self.blocked_person_session, self.blocked_person_request_id)):
            self.person_status_at = self.person_status_until = -1e9
            return False
        if (session, request_id) != (self.person_session, self.person_request_id):
            self.person = (0., 0.); self.person_at = self.person_until = -1e9; self.person_stamp_ns = 0
            self.person_activation_ns = stamp
        self.person_session = session
        self.person_request_id = request_id if message['enabled'] else ''
        self.person_status_at = t; self.person_status_stamp_ns = stamp
        self.person_status_until = t + self.PERSON_STATUS_AGE - max(0., (now_ns - stamp) * 1e-9)
        self.person_enabled = message['enabled']
        self.person_candidate_only = message['candidate_only']
        self.person_motion_permitted = message['motion_permitted']
        self.person_fault = message.get('fault', False) is True
        if not self.person_enabled or self.person_candidate_only:
            self.person = (0., 0.); self.person_at = self.person_until = -1e9
        return True

    def person_ready(self, t):
        return (self.person_enabled and not self.person_candidate_only
                and bool(self.person_request_id)
                and t <= self.person_status_until)

    def person_input(self, message, t, now_ns):
        values = (message.get('linear'), message.get('angular')) if isinstance(message, dict) else ()
        valid = (isinstance(message, dict)
                 and message.get('schema') == self.COMMAND_SCHEMA
                 and self.valid_stamp(message, now_ns, self.PERSON_COMMAND_AGE)
                 and message.get('request_id') == self.person_request_id
                 and message.get('follow_session') == self.person_session
                 and self.person_ready(t)
                 and len(values) == 2
                 and all(type(v) in (int, float) and math.isfinite(v) for v in values)
                 and abs(values[0]) <= .20 + 1e-9 and abs(values[1]) <= .45 + 1e-9)
        if not valid:
            self.person = (0., 0.); self.person_at = self.person_until = -1e9
            return False
        stamp = message['stamp_ns']
        if (stamp <= self.source_ros_ns or stamp <= self.person_stamp_ns
                or stamp < self.person_activation_ns):
            return False
        self.person = values; self.person_at = t; self.person_stamp_ns = stamp
        self.person_until = t + self.PERSON_COMMAND_AGE - max(0., (now_ns - stamp) * 1e-9)
        return True

    def disarm(self):
        self.set_manual(); self.clear_manual()

    def connected(self, t):
        return (t - self.status_at < .5 and self.status in ('ACTIVE', 'SPIN', 'STANDBY')
                and t - self.manual_at < .15)

    def stopped(self, t):
        return t - self.wheel_at <= .2 and self.zero(self.wheel)

    def tracking_channel(self):
        return self.object_channel if self.automatic_source=='OBJECT' else self

    def object_status_input(self,message,t,now_ns):
        return self.object_channel.person_status_input(message,t,now_ns)

    def object_input(self,message,t,now_ns):
        return self.object_channel.person_input(message,t,now_ns)

    def set_person_source(self, enabled, t, now_ns):
        return self.set_tracking_source('PERSON' if enabled else 'NAV',t,now_ns)

    def set_object_source(self, enabled, t, now_ns):
        return self.set_tracking_source('OBJECT' if enabled else 'NAV',t,now_ns)

    def set_tracking_source(self, source, t, now_ns):
        if source not in ('NAV','PERSON','OBJECT'):return False,'invalid_automatic_source'

        if source == self.automatic_source:
            return True, 'automatic_source_already_' + source.lower()
        if self.estop_latched:
            return False, 'estop_latched'
        if self.manual_requested(t):
            return False, 'manual_takeover_active'
        if not self.stopped(t):
            return False, 'chassis_not_stopped'
        if self.automatic_source in ('PERSON','OBJECT'):
            self.tracking_channel().revoke_person()
        self.mode = 'MANUAL'; self.automatic_source = source
        self.source_generation += 1; self.source_after = t; self.source_ros_ns = now_ns
        self.auto = (0., 0.); self.auto_at = -1e9; self.clear_person(); self.object_channel.clear_person()
        return True, 'automatic_source_' + source.lower() + '; MANUAL; explicit AUTO required'

    def set_auto(self, t, now_ns=0):
        if self.estop_latched:
            return False, 'estop_latched'
        if self.manual_requested(t):
            return False, 'manual_takeover_active'
        if not self.stopped(t):
            return False, 'chassis_not_stopped'
        if self.automatic_source in ('PERSON','OBJECT'):
            channel=self.tracking_channel()
            if (not channel.person_ready(t) or t > channel.person_until
                    or channel.person_at <= self.source_after or not self.zero(channel.person)):
                return False, self.automatic_source.lower()+'_not_fresh_enabled_and_stopped'
        elif t - self.auto_at > .15 or self.auto_at <= self.source_after or not self.zero(self.auto):
            return False, 'navigation_not_fresh_and_stopped'
        self.mode = 'AUTO'; self.auto_after = t; self.auto_ros_ns = now_ns
        return True, 'AUTO ' + self.automatic_source + ': stick input or A stop returns to MANUAL'

    def output(self, t):
        if self.estop_latched:
            # Retain new manual samples so reset can reject a held joystick.
            self.armed = False
            return (0., 0.)
        self.armed = self.manual_ready(t)  # Compatibility field: manual readiness only.
        if not self.armed:
            self.clear_manual()
        if self.mode == 'AUTO':
            if self.automatic_source in ('PERSON','OBJECT'):
                channel=self.tracking_channel()
                if t > channel.person_status_until or channel.person_fault:
                    self.set_manual(); return (0., 0.)
                if not channel.person_ready(t):
                    return (0., 0.)
                if (not channel.person_motion_permitted or t > channel.person_until
                        or channel.person_at <= self.auto_after or channel.person_stamp_ns <= self.auto_ros_ns):
                    return (0., 0.)
                v = channel.person
            else:
                if t - self.auto_at > .15 or self.auto_at <= self.auto_after:
                    return (0., 0.)
                v = self.auto
        else:
            if not self.armed:
                return (0., 0.)
            v = self.manual
        forward, reverse, turn = self.limits
        return (max(-reverse, min(forward, v[0])), max(-turn, min(turn, v[1])))


class ObjectChannel:
    """Reuse the exact leased-candidate validation, with independent state.

    The internal person_* names retain the established tested validator API;
    schemas, sessions, revocations and selected output are object-specific.
    """
    STATUS_SCHEMA='wla-object-status/v1'
    COMMAND_SCHEMA='wla-object-candidate/v1'
    PERSON_COMMAND_AGE=Policy.PERSON_COMMAND_AGE
    PERSON_STATUS_AGE=Policy.PERSON_STATUS_AGE
    valid_stamp=staticmethod(Policy.valid_stamp)
    clear_person=Policy.clear_person
    revoke_person=Policy.revoke_person
    person_ready=Policy.person_ready
    person_status_input=Policy.person_status_input
    person_input=Policy.person_input

    def __init__(self,owner):
        self.owner=owner;self.person_revoke_generation=0
        self.blocked_person_request_id='';self.blocked_person_session=''
        self.clear_person()

    @property
    def source_ros_ns(self):return self.owner.source_ros_ns
