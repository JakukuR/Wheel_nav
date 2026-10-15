"""NAV/manual coexistence without a driver or ROS graph."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from control_policy import Policy

class TestSharedNav(unittest.TestCase):
    def running(self):
        p = Policy(nav_manual_coexist=True)
        self.manual(p, 1., (0., 0.)); p.wheel_at = 1.
        p.automatic_input((0., 0.), 1.)
        self.assertTrue(p.set_auto(1.01)[0])
        p.automatic_input((.12, .2), 1.02)
        self.assertEqual(p.output(1.03), (.12, .2))
        return p

    def manual(self, p, t, v):
        p.gamepad_status_input('ACTIVE', t - .000001)
        return p.manual_input(v, t)

    def test_neutral_manual_does_not_mask_nav(self):
        p = self.running()
        self.manual(p, 1.04, (0., 0.))
        self.assertEqual(p.output(1.05), (.12, .2))
        self.assertEqual(p.output_source, 'NAV')

    def test_manual_override_does_not_revoke_or_add(self):
        for manual_first in (True, False):
            p = self.running()
            commands = [lambda: self.manual(p, 1.04, (.08, -.1)),
                        lambda: p.automatic_input((.12, .2), 1.04)]
            for f in commands if manual_first else reversed(commands): f()
            self.assertEqual(p.output(1.05), (.08, -.1))
            self.assertEqual((p.mode, p.output_source), ('AUTO', 'MANUAL'))

    def test_release_requires_post_release_nav_sample(self):
        p = self.running(); self.manual(p, 1.04, (.08, 0.)); p.output(1.05)
        p.automatic_input((.12, .2), 1.06)
        self.manual(p, 1.07, (0., 0.))
        self.assertEqual(p.output(1.08), (0., 0.))
        p.automatic_input((.11, .1), 1.09)
        self.assertEqual(p.output(1.10), (.11, .1))
        self.assertEqual(p.mode, 'AUTO')

    def test_disconnect_and_timeout_do_not_replay_nav(self):
        for disconnected in (True, False):
            p = self.running(); self.manual(p, 1.04, (.08, 0.)); p.output(1.05)
            p.automatic_input((.12, .2), 1.20)
            if disconnected: p.gamepad_status_input('DISCONNECTED', 1.21)
            self.assertEqual(p.output(1.22), (0., 0.))
            p.automatic_input((.09, 0.), 1.23)
            self.assertEqual(p.output(1.24), (.09, 0.))

    def test_stale_nav_does_not_block_manual(self):
        p = self.running(); self.manual(p, 1.30, (.08, .1))
        self.assertEqual(p.output(1.31), (.08, .1))
        self.manual(p, 1.32, (0., 0.))
        self.assertEqual(p.output(1.33), (0., 0.))

    def test_estop_stops_both_and_reset_does_not_restore_auto(self):
        p = self.running(); self.manual(p, 1.04, (.08, 0.)); p.output(1.05)
        p.set_estop(True, 1.06); p.automatic_input((.12, .2), 1.07)
        self.assertEqual(p.output(1.08), (0., 0.))
        self.manual(p, 1.085, (.08, 0.))  # Held stick sends a new sample after stop.
        p.wheel_at = 1.09
        self.assertFalse(p.set_estop(False, 1.10)[0])
        self.manual(p, 1.11, (0., 0.)); self.assertTrue(p.set_estop(False, 1.12)[0])
        p.automatic_input((.12, .2), 1.13)
        self.assertEqual(p.output(1.14), (0., 0.)); self.assertEqual(p.mode, 'MANUAL')

    def test_nonfinite_manual_latches_stop(self):
        p = self.running(); self.manual(p, 1.04, (float('nan'), 0.))
        self.assertTrue(p.estop_latched); self.assertEqual(p.output(1.05), (0., 0.))

    def test_manual_limits_and_explicit_revoke(self):
        p = self.running(); self.manual(p, 1.04, (9., -9.))
        self.assertEqual(p.output(1.05), (.3, -.5))
        p.set_manual(); self.manual(p, 1.06, (0., 0.))
        p.automatic_input((.12, .2), 1.07)
        self.assertEqual(p.output(1.08), (0., 0.))

    def test_shared_mode_never_implies_startup_authorization(self):
        p = Policy(nav_manual_coexist=True); p.automatic_input((.2, .2), 1.)
        self.assertEqual(p.output(1.01), (0., 0.)); self.assertEqual(p.mode, 'MANUAL')

if __name__ == '__main__': unittest.main()
