"""Modeless velocity priority: manual > unified target task > navigation."""
import math
from control_policy import Policy


class VelocityPolicy(Policy):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.commands = {}
        self.last_stamps = {}
        self.manual_override = False
        self.resume_after = -math.inf

    def automatic_input(self, values, t):
        self._store('NAV', values, t, t+.15)

    def command_fresh(self, source, t):
        _, received, until = self.commands.get(source, ((0., 0.), -math.inf, -math.inf))
        return received > self.resume_after and t <= until

    def _store(self, source, values, t, until):
        if (len(values) != 2 or not all(type(v) in (int, float) and math.isfinite(v) for v in values)):
            self.commands.pop(source, None)
            return False
        self.commands[source] = (tuple(values), t, until)
        return True

    def candidate(self, source, message, t, now_ns):
        schema = 'wla-'+source.lower()+'-candidate/v1'
        if (not isinstance(message, dict) or message.get('schema') != schema or
                not self.valid_stamp(message, now_ns, .15)):
            self.commands.pop(source, None)
            return False
        stamp = message['stamp_ns']
        if stamp <= self.last_stamps.get(source, 0):
            return False
        self.last_stamps[source] = stamp
        return self._store(source, (message.get('linear'), message.get('angular')), t,
                           t+.15-max(0., (now_ns-stamp)/1e9))

    def person_input(self, message, t, now_ns):
        return self.candidate('PERSON', message, t, now_ns)

    def manual_input(self, values, t):
        if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
            self.latch_stop('invalid_manual_input')
            return False
        self.manual, self.manual_at = tuple(values), t
        return False  # Temporary priority, no task revocation or mode transition.

    def set_manual(self):
        # Legacy explicit stop request: clear caches, never latch a drive mode.
        self.commands.clear()
        self.revoke_person()
        self.mode = 'AUTO'

    def set_auto(self, t, now_ns=0):
        if self.estop_latched:
            return False, 'estop_latched'
        return True, 'velocity priority active; no AUTO switch required'

    def set_tracking_source(self, source, t, now_ns):
        if source not in ('NAV', 'PERSON'):
            return False, 'invalid_automatic_source'
        # Compatibility hint for older producers only; output ignores this field.
        self.automatic_source = source
        self.mode = 'AUTO'
        return True, 'compatibility hint only; velocity priority does not select sources'

    def output(self, t):
        self.output_source = 'NONE'
        if self.estop_latched:
            self.commands.clear()
            self.armed = False
            return 0., 0.
        self.armed = self.manual_ready(t)
        override = self.manual_requested(t)
        if self.manual_override and not override:
            # After release, wait for a command received after the handover.
            self.resume_after = t
        self.manual_override = override
        if override:
            values = self.manual
            self.output_source = 'MANUAL'
        else:
            values = (0., 0.)
            for source in ('PERSON', 'NAV'):
                command, received, until = self.commands.get(source, ((0., 0.), -math.inf, -math.inf))
                if received > self.resume_after and t <= until and not self.zero(command):
                    values, self.output_source = command, source
                    break
        forward, reverse, turn = self.limits
        return max(-reverse, min(forward, values[0])), max(-turn, min(turn, values[1]))
