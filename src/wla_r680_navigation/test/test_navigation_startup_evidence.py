#!/usr/bin/env python3
"""Regression for the 2026-10-14 static-map false success and late startup error."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from wla_r680_navigation_py.startup_evidence import StartupEvidence


class StartupLogTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'bringup.log'
        self.path.touch()

    def append(self, text):
        with self.path.open('a') as stream:
            stream.write(text)

    def test_only_openvins_startup_response_completes_openvins(self):
        check = StartupEvidence(self.path, 'openvins')
        self.append('[lifecycle_manager_map]: Managed nodes are active\n')
        self.assertFalse(check.read())
        self.append('[lifecycle_manager_navigation]: Managed nodes are active\n')
        self.assertFalse(check.read())
        self.append('[openvins_nav_startup]: OpenVINS Nav2 startup completed; motion gates remain independent\n')
        self.assertTrue(check.read())

    def test_other_frontends_require_navigation_manager(self):
        check = StartupEvidence(self.path, 'cuvslam')
        self.append('[lifecycle_manager_map]: Managed nodes are active\n')
        self.assertFalse(check.read())
        self.append('[lifecycle_manager_navigation]: Managed nodes are active\n')
        self.assertTrue(check.read())

    def test_error_arriving_during_readiness_checks_overrides_success(self):
        check = StartupEvidence(self.path, 'openvins')
        self.append('[openvins_nav_startup]: OpenVINS Nav2 startup completed; motion gates remain independent\n')
        self.assertTrue(check.read())
        self.append('[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.\n')
        with self.assertRaises(SystemExit):
            check.read()
        with self.assertRaises(SystemExit):
            check.read()

    def test_split_failure_line(self):
        check = StartupEvidence(self.path, 'openvins')
        self.append('[openvins_nav_startup]: OpenVINS Nav2 startup fai')
        self.assertFalse(check.read())
        self.append('led: lifecycle manager rejected STARTUP\n')
        with self.assertRaisesRegex(SystemExit, 'rejected STARTUP'):
            check.read()


if __name__ == '__main__':
    unittest.main()
