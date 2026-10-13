"""Isolated source-arbiter test. Runs only a mux; never launches a chassis driver."""
import json
import os
import signal
import subprocess
import tempfile
import time

assert os.environ.get('ROS_DOMAIN_ID') == '198', 'isolated domain 198 required'

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class Fixture(Node):
    def __init__(self):
        super().__init__('relative_mux_fixture')
        self.outputs = []; self.states = []; self.cancellations = 0
        self.manual = self.create_publisher(Twist, '/r680_manual/cmd_vel', 1)
        self.nav = self.create_publisher(Twist, '/r680_nav/chassis_cmd_vel', 1)
        self.gamepad = self.create_publisher(String, '/gamepad_status', 10)
        self.wheel = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.person = self.create_publisher(String, '/r680/person_follow/command_candidate', 1)
        self.person_status = self.create_publisher(String, '/r680/person_follow/status', 1)
        self.create_subscription(Twist, '/r680_base/cmd_vel',
                                 lambda m: self.outputs.append((m.linear.x, m.angular.z)), 10)
        self.create_subscription(String, '/r680_base/control_state',
                                 lambda m: self.states.append(json.loads(m.data)), 10)
        self.create_service(CancelGoal, '/navigate_to_pose/_action/cancel_goal', self.cancel)
        self.automatic = self.create_client(SetBool, '/r680_base/set_automatic')
        self.source = self.create_client(SetBool, '/r680_base/set_person_source')
        self.request_id = 'integration-task-a'; self.follow_session = 'integration-process-a'
        self.task_enabled = True; self.preview = False; self.task_fault = False

    def cancel(self, req, res):
        self.cancellations += 1; res.return_code = 0
        return res

    def feed(self, seconds, manual=0., nav=0., person=.12, gamepad='ACTIVE',
             send_person=True, send_status=True, wrong_session=False, send_manual=True, send_nav=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if send_manual:
                command = Twist(); command.linear.x = manual; self.manual.publish(command)
            if send_nav:
                command = Twist(); command.linear.x = nav; self.nav.publish(command)
            self.gamepad.publish(String(data='[' + gamepad + '] fixture'))
            wheel = Odometry(); wheel.header.stamp = self.get_clock().now().to_msg(); self.wheel.publish(wheel)
            stamp = self.get_clock().now().nanoseconds
            state = self.states[-1] if self.states else {}
            permitted = state.get('mode') == 'AUTO' and state.get('automatic_source') == 'PERSON'
            if send_status:
                self.person_status.publish(String(data=json.dumps({
                    'schema': 'wla-person-status/v1', 'stamp_ns': stamp,
                    'request_id': self.request_id, 'follow_session': self.follow_session,
                    'enabled': self.task_enabled, 'candidate_only': self.preview,
                    'motion_permitted': permitted, 'fault': self.task_fault,
                    'state': 'following' if self.task_enabled else 'idle',
                })))
            if send_person:
                self.person.publish(String(data=json.dumps({
                    'schema': 'wla-person-candidate/v1', 'stamp_ns': stamp,
                    'request_id': self.request_id,
                    'follow_session': 'old-process' if wrong_session else self.follow_session,
                    'linear': person if permitted and self.task_enabled else 0., 'angular': 0.,
                })))
            rclpy.spin_once(self, timeout_sec=.01)
            time.sleep(.01)

    def call(self, client, data, expected=True, **feed_kwargs):
        assert client.wait_for_service(timeout_sec=2.)
        req = SetBool.Request(); req.data = data; future = client.call_async(req)
        deadline = time.monotonic() + 2.
        while not future.done() and time.monotonic() < deadline:
            self.feed(.03, person=0., **feed_kwargs)
        assert future.done(), 'service timeout'
        result = future.result()
        assert result.success is expected, result.message
        return result


def main():
    rclpy.init(); fixture = Fixture()
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen(['ros2', 'run', 'wla_manual_control', 'command_mux.py'],
                                   stdout=log, stderr=log, start_new_session=True)
        try:
            fixture.feed(2., manual=.2)
            assert fixture.outputs and all(v == (0., 0.) for v in fixture.outputs), 'boot safety'
            fixture.feed(1.1); assert fixture.states[-1]['armed']
            fixture.feed(.1, manual=.1)
            fixture.call(fixture.source, True, expected=False, manual=.1)
            fixture.feed(.3)
            fixture.call(fixture.source, True)
            fixture.feed(.4, nav=.25)
            assert fixture.states[-1]['mode'] == 'MANUAL'
            assert fixture.states[-1]['automatic_source'] == 'PERSON'
            assert fixture.cancellations > 0, 'source switch must cancel Nav2'
            assert fixture.outputs[-1] == (0., 0.)
            fixture.call(fixture.automatic, True)
            fixture.feed(.5, nav=.25)
            assert abs(fixture.outputs[-1][0] - .12) < 1e-6, 'exclusive PERSON source'
            fixture.feed(.3, nav=.25, wrong_session=True)
            assert fixture.outputs[-1] == (0., 0.), 'old process session rejection'
            fixture.feed(.3)
            assert abs(fixture.outputs[-1][0] - .12) < 1e-6
            fixture.feed(.3, send_person=False, nav=.25)
            assert fixture.outputs[-1] == (0., 0.), 'candidate timeout cannot fall back to NAV'
            fixture.task_enabled = False; fixture.feed(.3, nav=.25)
            assert fixture.outputs[-1] == (0., 0.) and fixture.states[-1]['mode'] == 'AUTO'
            fixture.request_id = 'integration-task-b'; fixture.task_enabled = True; fixture.feed(.4)
            assert abs(fixture.outputs[-1][0] - .12) < 1e-6, 'new task after idle'
            fixture.feed(.3, manual=.18, nav=.25)
            assert abs(fixture.outputs[-1][0] - .18) < 1e-6
            fixture.feed(.3, nav=.25)
            assert fixture.outputs[-1] == (0., 0.) and fixture.states[-1]['mode'] == 'MANUAL'
            assert fixture.states[-1]['person_revoke_generation'] > 0
            fixture.call(fixture.automatic, True, expected=False)
            fixture.request_id = 'integration-task-c'; fixture.feed(.3)
            fixture.call(fixture.automatic, True); fixture.feed(.4)
            assert abs(fixture.outputs[-1][0] - .12) < 1e-6
            fixture.feed(.5, send_status=False)
            assert fixture.outputs[-1] == (0., 0.) and fixture.states[-1]['mode'] == 'MANUAL'
            fixture.request_id = 'integration-task-d'; fixture.feed(.3)
            fixture.call(fixture.automatic, True); fixture.feed(.4)
            fixture.feed(.4, gamepad='DISCONNECTED')
            assert fixture.outputs[-1] == (0., 0.) and not fixture.states[-1]['armed']
            fixture.feed(.4, manual=.2)
            assert fixture.outputs[-1] == (0., 0.), 'reconnect requires neutral'
            fixture.feed(1.1)
            fixture.call(fixture.source, False); fixture.feed(.3)
            assert fixture.states[-1]['automatic_source'] == 'NAV'
            fixture.call(fixture.automatic, True); fixture.feed(.4, nav=.21)
            assert abs(fixture.outputs[-1][0] - .21) < 1e-6
            # Retained from the retired NAV-only test, using current mux limits.
            fixture.feed(.4, send_nav=False)
            assert fixture.outputs[-1] == (0., 0.), 'stale NAV must stop'
            fixture.feed(.3, nav=.21)
            assert abs(fixture.outputs[-1][0] - .21) < 1e-6
            fixture.feed(.3, manual=.2, gamepad='ESTOP')
            assert fixture.outputs[-1] == (0., 0.) and not fixture.states[-1]['armed']
            fixture.feed(.5, manual=.2)
            assert fixture.outputs[-1] == (0., 0.), 'estop release still requires neutral'
            fixture.feed(1.1)
            fixture.feed(.3, manual=3.)
            assert abs(fixture.outputs[-1][0] - 1.0) < 1e-6, 'current forward clamp'
            fixture.feed(.3, manual=-3.)
            assert abs(fixture.outputs[-1][0] + .3) < 1e-6, 'reverse clamp'
            fixture.feed(.4, send_manual=False)
            assert fixture.outputs[-1] == (0., 0.) and not fixture.states[-1]['armed'], 'stale joystick'
            publishers = fixture.get_publishers_info_by_topic('/r680_base/cmd_vel')
            assert len(publishers) == 1 and publishers[0].node_name == 'r680_command_mux'
            assert not any('wheeltec_robot' in name for name in fixture.get_node_names())
            print('PASS: source exclusivity, explicit AUTO, session binding, stale candidate/status, '
                  'idle-to-new-task, manual revoke, hotplug, Nav2 cancel, stale NAV/manual, '
                  'estop rearm, forward/reverse limits; isolated sole mux publisher')
        finally:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10)
            log.seek(0); details = log.read()
            if process.returncode not in (0, 130):
                print(details)
                raise AssertionError('mux unclean shutdown: ' + str(process.returncode))
    fixture.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
