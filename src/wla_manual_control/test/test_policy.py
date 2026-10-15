import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from control_policy import Policy


class TestTakeover(unittest.TestCase):
    def ready(self, p, t, v=(0., 0.)):
        p.gamepad_status_input('ACTIVE', t - .000001); p.manual_input(v, t)
        return p.output(t)

    def arm(self, p):
        self.ready(p, 0); self.ready(p, .4); self.ready(p, .8)
        self.assertTrue(p.armed)

    def test_manual_requires_producer_ready_and_new_input_after_reconnect(self):
        p = Policy()
        p.gamepad_status_input('ARMING', 0.)
        p.manual_input((.2, 0.), .01)
        self.assertEqual(p.output(.02), (0., 0.))
        self.assertEqual(self.ready(p, .4, (.2, 0.)), (.2, 0.))
        p.gamepad_status_input('DISCONNECTED', .5)
        self.assertEqual(p.output(.51), (0., 0.))
        p.manual_input((.2, 0.), .6)
        p.gamepad_status_input('ACTIVE', .61)
        self.assertEqual(p.output(.62), (0., 0.))
        self.assertEqual(self.ready(p, .63, (.2, 0.)), (.2, 0.))

    def test_navigation_failure_manual_override_and_resume(self):
        p = Policy(); self.arm(p); p.wheel_at = .8; p.automatic_input((0., 0.), .8)
        self.assertTrue(p.set_auto(.81)[0]); p.automatic_input((.1, .1), .82)
        self.assertEqual(p.output(.83), (.1, .1))
        self.ready(p, .98); self.assertEqual(p.output(1.), (0., 0.))
        self.assertFalse(p.manual_input((.2, 0.), 1.01)); p.status_at = 1.01
        self.assertEqual(p.output(1.02), (.2, 0.)); self.ready(p, 1.1)
        p.automatic_input((.3, 0.), 1.11); self.assertEqual(p.output(1.12), (.3, 0.))
        self.assertEqual(p.output_source, 'NAV')

    def test_estop_and_invalid_data(self):
        p = Policy(); self.arm(p); p.gamepad_status_input('ESTOP', .85)
        self.assertEqual(p.output(.9), (0., 0.)); self.assertFalse(p.armed)
        p.manual_input((float('nan'), 0.), 1.); self.assertFalse(p.armed)

    def test_limits_and_auto_release(self):
        p = Policy(); self.arm(p)
        self.assertEqual(self.ready(p, .9, (4., -3.)), (.3, -.5)); self.assertTrue(p.set_auto(.91)[0])
        self.ready(p, 1.); p.wheel_at = 1.; p.automatic_input((.2, 0.), 1.)
        self.assertTrue(p.set_auto(1.01)[0])
        p.automatic_input((0., 0.), 1.01); self.assertTrue(p.set_auto(1.02)[0])
        self.assertEqual(p.output(1.03), (0., 0.))


class TestPersonSource(TestTakeover):
    @staticmethod
    def ns(t):
        return round((100. + t) * 1e9)

    def status_message(self, t, request='job-a', enabled=True, preview=False, permitted=True):
        return {'schema': 'wla-person-status/v1', 'stamp_ns': self.ns(t),
                'follow_session': 'follow-process-a',
                'request_id': request, 'enabled': enabled, 'candidate_only': preview,
                'motion_permitted': permitted}

    def command_message(self, t, request='job-a', values=(.1, .2)):
        return {'schema': 'wla-person-candidate/v1', 'stamp_ns': self.ns(t),
                'follow_session': 'follow-process-a',
                'request_id': request, 'linear': values[0], 'angular': values[1]}

    def feed_person(self, p, t, values=(.1, .2), request='job-a', permitted=True):
        self.ready(p, t); p.wheel_at = t
        self.assertTrue(p.person_status_input(self.status_message(t, request, permitted=permitted), t, self.ns(t)))
        self.assertTrue(p.person_input(self.command_message(t, request, values), t, self.ns(t)))

    def selected(self):
        p = Policy(); self.arm(p); p.wheel_at = .8
        self.assertTrue(p.set_person_source(True, .81, self.ns(.81))[0])
        self.assertEqual(p.mode, 'MANUAL')
        self.feed_person(p, .82, (0., 0.), permitted=False)
        return p

    def running(self):
        p = self.selected()
        self.assertTrue(p.set_auto(.83, self.ns(.83))[0])
        self.feed_person(p, .84)
        self.assertEqual(p.output(.85), (.1, .2))
        return p

    def test_person_requires_explicit_source_then_auto_and_new_sample(self):
        p = self.selected()
        self.assertEqual(p.output(.825), (0., 0.))
        self.assertTrue(p.set_auto(.83, self.ns(.83))[0])
        self.assertEqual(p.output(.835), (0., 0.))
        self.feed_person(p, .84, permitted=False)
        self.assertEqual(p.output(.845), (0., 0.))
        self.feed_person(p, .85)
        self.assertEqual(p.output(.86), (.1, .2))

    def test_nav_never_wins_in_person_mode(self):
        p = self.running(); p.automatic_input((.3, -.5), .86)
        self.assertEqual(p.output(.87), (.1, .2))
        p.person_at = p.person_until = -1e9
        self.assertEqual(p.output(.88), (0., 0.))

    def test_source_switch_requires_neutral_and_stopped_and_flushes(self):
        p = self.running(); self.ready(p, .86, (.1, 0.)); p.wheel_at = .86
        self.assertFalse(p.set_person_source(False, .87, self.ns(.87))[0])
        self.ready(p, .88); p.wheel = (.05, 0.)
        self.assertFalse(p.set_person_source(False, .89, self.ns(.89))[0])
        p.wheel = (0., 0.); p.automatic_input((0., 0.), .90)
        self.assertTrue(p.set_person_source(False, .91, self.ns(.91))[0])
        self.assertEqual((p.automatic_source, p.mode), ('NAV', 'AUTO'))
        self.assertEqual(p.output(.92), (0., 0.))  # Source switch clears old commands.
        p.automatic_input((0., 0.), .93); self.assertTrue(p.set_auto(.94, self.ns(.94))[0])
        self.assertEqual(p.output(.945), (0., 0.))
        p.automatic_input((.2, 0.), .95); self.assertEqual(p.output(.96), (.2, 0.))

    def test_preview_cannot_arm_and_nonzero_prearm_rejected(self):
        p = self.selected()
        p.person_status_input(self.status_message(.83, preview=True), .83, self.ns(.83))
        self.assertFalse(p.person_input(self.command_message(.83), .83, self.ns(.83)))
        self.assertFalse(p.set_auto(.84, self.ns(.84))[0])
        self.feed_person(p, .85)
        self.assertFalse(p.set_auto(.86, self.ns(.86))[0])

    def test_request_id_mismatch_and_epoch_replacement_do_not_replay(self):
        p = self.running()
        self.assertFalse(p.person_input(self.command_message(.86, 'old-session:job'), .86, self.ns(.86)))
        self.assertEqual(p.output(.87), (0., 0.))
        p.person_status_input(self.status_message(.88, 'new-session:job'), .88, self.ns(.88))
        self.assertEqual(p.output(.89), (0., 0.))
        self.assertFalse(p.person_input(self.command_message(.90), .90, self.ns(.90)))
        self.assertTrue(p.person_input(self.command_message(.91, 'new-session:job'), .91, self.ns(.91)))
        self.assertEqual(p.output(.92), (.1, .2))

    def test_timestamp_source_and_auto_barriers(self):
        p = self.selected()
        self.assertFalse(p.person_status_input(self.status_message(.80), .83, self.ns(.83)))
        self.assertTrue(p.set_auto(.84, self.ns(.84))[0])
        self.assertTrue(p.person_input(self.command_message(.83), .85, self.ns(.85)))
        self.assertEqual(p.output(.86), (0., 0.))
        for stamp in (.50, 1.20):
            self.assertFalse(p.person_input(self.command_message(stamp), .87, self.ns(.87)))
        self.assertEqual(p.output(.88), (0., 0.))

    def test_candidate_bounds_finite_schema_and_timestamp_types(self):
        for updates in ({'linear': .21}, {'angular': -.46}, {'linear': float('nan')},
                        {'angular': float('inf')}, {'linear': True}, {'schema': 'bad'},
                        {'stamp_ns': '100860000000'}, {'stamp_ns': True}):
            p = self.running(); message = self.command_message(.86); message.update(updates)
            self.assertFalse(p.person_input(message, .86, self.ns(.86)))
            self.assertEqual(p.output(.87), (0., 0.))
        p = self.running()
        self.assertFalse(p.person_input(None, .86, self.ns(.86)))

    def test_candidate_timeout_zero_status_timeout_revokes(self):
        p = self.running(); self.ready(p, .99)
        p.person_status_input(self.status_message(1.), 1., self.ns(1.))
        self.assertEqual(p.output(1.01), (0., 0.)); self.assertEqual(p.mode, 'AUTO')
        self.ready(p, 1.19)
        self.assertEqual(p.output(1.21), (0., 0.)); self.assertEqual(p.mode, 'MANUAL')
        self.assertEqual(p.person_revoke_generation, 1)
        self.assertFalse(p.person_status_input(self.status_message(1.22), 1.22, self.ns(1.22)))

    def test_transit_delay_consumes_command_freshness_budget(self):
        p = self.running(); self.ready(p, .97)
        p.person_status_input(self.status_message(.98), .98, self.ns(.98))
        self.assertTrue(p.person_input(self.command_message(.85), .98, self.ns(.98)))
        self.assertEqual(p.output(.99), (.1, .2))
        self.assertEqual(p.output(1.01), (0., 0.))
        self.assertEqual(p.mode, 'AUTO')

    def test_transit_delay_consumes_status_freshness_budget(self):
        p = self.running(); self.ready(p, 1.02)
        p.person_status_input(self.status_message(.85), 1.03, self.ns(1.03))
        self.assertEqual(p.output(1.06), (0., 0.)); self.assertEqual(p.mode, 'MANUAL')

    def test_manual_takeover_and_explicit_manual_revoke_previous_request(self):
        for explicit in (False, True):
            p = self.running()
            if explicit:
                p.set_manual()
            else:
                self.assertTrue(p.manual_input((.1, 0.), .86))
            self.assertEqual(p.person_revoke_generation, 1)
            self.assertFalse(p.person_status_input(self.status_message(.87), .87, self.ns(.87)))
            self.ready(p, .88); p.wheel_at = .88
            self.assertFalse(p.set_auto(.89, self.ns(.89))[0])
            self.feed_person(p, .90, (0., 0.), 'job-b', permitted=False)
            self.assertTrue(p.set_auto(.91, self.ns(.91))[0])

    def test_disconnect_preserves_person_authorization_until_real_takeover(self):
        p = self.running(); p.gamepad_status_input('DISCONNECTED', .86)
        self.assertEqual(p.output(.87), (.1, .2)); self.assertFalse(p.armed)
        self.assertEqual(p.person_revoke_generation, 0)
        p.gamepad_status_input('ARMING', .88)
        self.assertFalse(p.manual_input((.2, 0.), .89))
        self.assertEqual(p.output(.9), (.1, .2)); self.assertEqual(p.mode, 'AUTO')
        self.assertEqual(self.ready(p, .91, (.2, 0.)), (.2, 0.))
        self.assertEqual(p.mode, 'MANUAL'); self.assertEqual(p.person_revoke_generation, 1)
        self.assertFalse(p.person_status_input(self.status_message(.92), .92, self.ns(.92)))

    def test_idempotent_selection_does_not_drop_running_person_task(self):
        p = self.running(); generation = p.source_generation
        self.assertTrue(p.set_person_source(True, .86, self.ns(.86))[0])
        self.assertEqual(p.source_generation, generation); self.assertEqual(p.mode, 'AUTO')
        self.assertEqual(p.output(.87), (.1, .2))

    def test_session_change_same_request_does_not_replay_old_command(self):
        p = self.running(); status = self.status_message(.86)
        status['follow_session'] = 'follow-process-b'
        self.assertTrue(p.person_status_input(status, .86, self.ns(.86)))
        self.assertEqual(p.output(.865), (0., 0.))
        self.assertFalse(p.person_input(self.command_message(.87), .87, self.ns(.87)))
        command = self.command_message(.88); command['follow_session'] = 'follow-process-b'
        self.assertTrue(p.person_input(command, .88, self.ns(.88)))
        self.assertEqual(p.output(.89), (.1, .2))

    def test_candidate_before_new_task_activation_rejected(self):
        p = self.running()
        p.person_status_input(self.status_message(.87, 'job-b'), .87, self.ns(.87))
        self.assertFalse(p.person_input(self.command_message(.86, 'job-b'), .88, self.ns(.88)))
        self.assertEqual(p.output(.89), (0., 0.))

    def test_fresh_idle_holds_auto_and_new_task_can_run(self):
        p = self.running()
        self.assertTrue(p.person_status_input(self.status_message(.86, enabled=False), .86, self.ns(.86)))
        self.assertEqual(p.person_request_id, '')
        self.assertEqual(p.output(.87), (0., 0.)); self.assertEqual(p.mode, 'AUTO')
        self.assertEqual(p.person_revoke_generation, 0)
        self.feed_person(p, .88, (.1, -.2), 'job-b')
        self.assertEqual(p.output(.89), (.1, -.2))

    def test_fresh_fault_latches_manual(self):
        p = self.running(); status = self.status_message(.86)
        status['fault'] = True
        self.assertTrue(p.person_status_input(status, .86, self.ns(.86)))
        self.assertEqual(p.output(.87), (0., 0.)); self.assertEqual(p.mode, 'MANUAL')
        self.assertEqual(p.person_revoke_generation, 1)

    def test_initial_manual_status_does_not_revoke_preview_task(self):
        p = Policy(); self.arm(p)
        p.person_status_input(self.status_message(.82, preview=True), .82, self.ns(.82))
        p.set_manual(); p.disarm()
        self.assertEqual(p.person_revoke_generation, 0)


class TestAutonomyWithoutGamepad(unittest.TestCase):
    def nav(self):
        p = Policy(); p.wheel_at = 10.
        p.automatic_input((0., 0.), 10.)
        self.assertTrue(p.set_auto(10.01)[0])
        p.automatic_input((.1, .2), 10.02)
        return p

    def test_no_gamepad_navigation_works_without_mode_switch(self):
        p = Policy(); p.automatic_input((.1, .2), 10.)
        self.assertEqual(p.output(10.01), (.1, .2))
        p = self.nav()
        self.assertEqual(p.output(10.03), (.1, .2))
        self.assertFalse(p.armed); self.assertEqual(p.mode, 'AUTO')
        self.assertEqual(p.output(10.2), (0., 0.))

    def test_no_gamepad_person_source_and_task_authorization(self):
        p = Policy(); p.wheel_at = 1.
        self.assertTrue(p.set_person_source(True, 1., 1000000000)[0])
        def feed(t, v):
            stamp = round(t * 1e9)
            self.assertTrue(p.person_status_input(dict(schema='wla-person-status/v1',
                stamp_ns=stamp, follow_session='p', request_id='job', enabled=True,
                candidate_only=False, motion_permitted=True), t, stamp))
            self.assertTrue(p.person_input(dict(schema='wla-person-candidate/v1',
                stamp_ns=stamp, follow_session='p', request_id='job', linear=v,
                angular=0.), t, stamp))
        feed(1.01, 0.)
        self.assertTrue(p.set_auto(1.02, 1020000000)[0])
        self.assertEqual(p.output(1.025), (0., 0.))
        feed(1.03, .1)
        self.assertEqual(p.output(1.04), (.1, 0.))
        self.assertFalse(p.armed)
        self.assertEqual(p.output(1.24), (0., 0.)); self.assertEqual(p.mode, 'MANUAL')
        self.assertEqual(p.person_revoke_generation, 1)

    def test_navigation_does_not_need_a_zero_speed_mode_handshake(self):
        for wheel_at, wheel, command in [(0., (0., 0.), (0., 0.)),
                (10., (.1, 0.), (0., 0.)), (10., (0., 0.), (.1, 0.))]:
            p = Policy(); p.wheel_at = wheel_at; p.wheel = wheel
            p.automatic_input(command, 10.)
            self.assertTrue(p.set_auto(10.01)[0])
            self.assertEqual(p.output(10.02), command)
        p = self.nav()
        p.set_manual(); p.wheel_at = 10.3
        self.assertTrue(p.set_auto(10.3)[0])
        self.assertEqual(p.output(10.31), (0., 0.))

    def test_disconnection_silence_and_arming_do_not_cancel_auto(self):
        for status in ('DISCONNECTED', 'ARMING'):
            p = self.nav(); p.gamepad_status_input(status, 10.03)
            self.assertFalse(p.manual_input((.2, 0.), 10.04))
            self.assertEqual(p.output(10.05), (.1, .2))
            self.assertEqual(p.mode, 'AUTO')
        p = self.nav(); p.gamepad_status_input('ACTIVE', 9.)
        self.assertFalse(p.manual_input((.2, 0.), 10.04))
        self.assertEqual(p.output(10.05), (.1, .2))

    def test_qualified_override_disconnect_then_fresh_nav_resumes(self):
        p = self.nav(); p.gamepad_status_input('ACTIVE', 10.03)
        self.assertFalse(p.manual_input((.2, 0.), 10.04))
        self.assertEqual(p.output(10.05), (.2, 0.))
        p.gamepad_status_input('DISCONNECTED', 10.06)
        self.assertEqual(p.output(10.07), (0., 0.))
        p.automatic_input((.2, 0.), 10.08)
        self.assertEqual(p.output(10.09), (.2, 0.))

    def test_global_estop_latches_without_gamepad_and_reset_never_restarts(self):
        p = self.nav(); self.assertTrue(p.set_estop(True, 10.03)[0])
        self.assertEqual(p.output(10.04), (0., 0.)); self.assertTrue(p.estop_latched)
        self.assertFalse(p.set_auto(10.05)[0])
        self.assertFalse(p.set_person_source(True, 10.05, 1)[0])
        self.assertFalse(p.set_estop(False, 10.5)[0])  # stale wheel feedback
        p.wheel_at = 10.5; p.wheel = (.1, 0.)
        self.assertFalse(p.set_estop(False, 10.51)[0])
        p.wheel = (0., 0.)
        self.assertTrue(p.set_estop(False, 10.52)[0])
        self.assertEqual(p.output(10.53), (0., 0.))  # old NAV command was cleared
        self.assertTrue(p.set_auto(10.54)[0])
        p.automatic_input((0., 0.), 10.55)
        self.assertTrue(p.set_auto(10.56)[0])
        self.assertEqual(p.output(10.57), (0., 0.))
        p.automatic_input((.1, 0.), 10.58)
        self.assertEqual(p.output(10.59), (.1, 0.))

    def test_gamepad_stop_cannot_be_cleared_by_disconnect_or_reconnect(self):
        p = self.nav(); p.gamepad_status_input('ESTOP', 10.03)
        p.gamepad_status_input('DISCONNECTED', 10.04)
        self.assertEqual(p.output(10.05), (0., 0.))
        self.assertFalse(p.set_estop(False, 10.06)[0])
        p.gamepad_status_input('ARMING', 10.07)
        self.assertTrue(p.estop_latched)
        self.assertTrue(p.set_estop(False, 10.08)[0])
        self.assertEqual(p.output(10.09), (0., 0.))

    def test_invalid_manual_input_stops_autonomy(self):
        p = self.nav(); p.manual_input((float('nan'), 0.), 10.03)
        self.assertTrue(p.estop_latched)
        self.assertEqual(p.output(10.04), (0., 0.))

    def test_stop_output_preserves_nonzero_manual_input_for_reset_rejection(self):
        p = self.nav(); p.set_estop(True, 10.03)
        p.gamepad_status_input('ACTIVE', 10.04)
        p.manual_input((.2, 0.), 10.05)
        self.assertEqual(p.output(10.06), (0., 0.))
        self.assertFalse(p.set_estop(False, 10.07)[0])
        p.manual_input((0., 0.), 10.08)
        self.assertTrue(p.set_estop(False, 10.09)[0])


if __name__ == '__main__':
    unittest.main()
