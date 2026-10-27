#!/usr/bin/env python3
"""Regression for the 2026-10-14 static-map false success and late startup error."""
import os
import subprocess
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

    def test_critical_process_exit_fails_closed(self):
        check = StartupEvidence(self.path, 'cuvslam')
        self.append('[lifecycle_manager_navigation]: Managed nodes are active\n')
        self.assertTrue(check.read())
        # A respawned frontend exit is not fatal; RTAB-Map has no respawn.
        self.append('[ERROR] [cuvslam_odometry-4]: process has died [pid 1, exit code -6]\n')
        self.assertTrue(check.read())
        self.append('[ERROR] [rtabmap-6]: process has died [pid 2, exit code -6, cmd ...]\n')
        with self.assertRaisesRegex(SystemExit, 'rtabmap-6'):
            check.read()

    def test_split_failure_line(self):
        check = StartupEvidence(self.path, 'openvins')
        self.append('[openvins_nav_startup]: OpenVINS Nav2 startup fai')
        self.assertFalse(check.read())
        self.append('led: lifecycle manager rejected STARTUP\n')
        with self.assertRaisesRegex(SystemExit, 'rejected STARTUP'):
            check.read()


    def test_launch_argument_failure_is_reported(self):
        check = StartupEvidence(self.path, 'cuvslam')
        self.append("malformed launch argument 'openvins_config_dir:=', expected format '<name>:=<value>'\n")
        with self.assertRaisesRegex(SystemExit, 'openvins_config_dir'):
            check.read()

    def test_launch_exception_is_reported(self):
        self.append('[ERROR] [launch]: Caught exception in launch: missing package\n')
        with self.assertRaisesRegex(SystemExit, 'missing package'):
            StartupEvidence(self.path, 'cuvslam').read()

    def test_live_process_still_needs_navigation_activation(self):
        check = StartupEvidence(self.path, 'cuvslam', bringup_pid=os.getpid())
        self.assertFalse(check.read())
        self.append('[lifecycle_manager_navigation]: Managed nodes are active\n')
        self.assertTrue(check.read())

    def test_silent_process_exit_overrides_success(self):
        child = subprocess.Popen([sys.executable, '-c', 'pass'])
        child.wait(timeout=5)
        self.append('[lifecycle_manager_navigation]: Managed nodes are active\n')
        with self.assertRaisesRegex(SystemExit, 'pid='):
            StartupEvidence(self.path, 'cuvslam', bringup_pid=child.pid).read()

    def test_zombie_process_is_not_alive(self):
        child = subprocess.Popen([sys.executable, '-c', 'pass'])
        try:
            os.waitid(os.P_PID, child.pid, os.WEXITED | os.WNOWAIT)
            os.kill(child.pid, 0)  # Positive control: this alone misses the exit.
            self.append('last launch failure without newline')
            with self.assertRaisesRegex(SystemExit, 'last launch failure without newline'):
                StartupEvidence(self.path, 'cuvslam', bringup_pid=child.pid).read()
        finally:
            child.wait(timeout=5)


class LaunchArgumentsTest(unittest.TestCase):
    def launch_arguments(self, frontend, initial_pose=''):
        # Execute only the real entry's argument builder, never launch or service commands.
        entry = Path(__file__).resolve().parents[3] / 'r680_nav.sh'
        source = entry.read_text()
        block = source[source.index('LAUNCH_ARGS=('):source.index('setsid ros2 launch')]
        env = dict(os.environ, ODOM_SOURCE=frontend,
                   DB_PATH='/tmp/user map/rtabmap.db', MAP_YAML='/tmp/user map/map.yaml',
                   NAV2_PARAMS_FILE='/tmp/nav2.yaml', SCAN_OBSTACLES='true',
                   START_DYNAMIC_OBSTACLES='false', START_CHASSIS='false',
                   USE_D455_IMU='false', OPENVINS_CONFIG='/tmp/openvins config' if frontend == 'openvins' else '',
                   OPENVINS_ALLOW_COMMISSIONING='false', NAV_RUN_DIR='/tmp/nav run',
                   AUTO_VIO_INIT='false', ENABLE_MOTION='false', AGENT_CONTROL='false',
                   MAP_DIR='/tmp/user map', INITIAL_POSE=initial_pose,
                   MOUNT_TF='false', DISABLE_DEGRADED_ODOM='false')
        result = subprocess.run(['bash', '--noprofile', '--norc', '-c',
            block + '\nprintf "%s\\0" "${LAUNCH_ARGS[@]}"'], env=env,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, timeout=5)
        from ros2launch.api.api import parse_launch_arguments
        return dict(parse_launch_arguments(result.stdout.decode().rstrip('\0').split('\0')))

    def test_cuvslam_and_rgbd_omit_openvins_config(self):
        for frontend in ('cuvslam', 'rgbd'):
            with self.subTest(frontend=frontend):
                args = self.launch_arguments(frontend)
                self.assertNotIn('openvins_config_dir', args)
                self.assertEqual(args['odom_source'], frontend)
                self.assertEqual(args['enable_hardware_output'], 'false')
                self.assertEqual(args['navigation_map_yaml'], '/tmp/user map/map.yaml')
                self.assertNotIn('initial_pose', args)
                self.assertEqual(args['publish_mount_tf'], 'false')
                if frontend == 'cuvslam':
                    self.assertEqual(args['degraded_odom_params_file'], '/tmp/nav run/degraded_odometry.yaml')

    def test_openvins_keeps_config_and_initial_pose(self):
        args = self.launch_arguments('openvins', '0 0 0 0 0 0')
        self.assertEqual(args['openvins_config_dir'], '/tmp/openvins config')
        self.assertEqual(args['initial_pose'], '0 0 0 0 0 0')


if __name__ == '__main__':
    unittest.main()
