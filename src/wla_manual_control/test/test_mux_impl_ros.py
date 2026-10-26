"""Run one mux executable in isolated domain 198 with synthetic inputs (no chassis).

Checks priority, expiry, manual override, estop service, clamp, control_state keys
and final zero on SIGINT, then measures mux CPU under production-like rates:
200 Hz /wheel/odom, 50 Hz /r680_manual/cmd_vel, 5 Hz /gamepad_status.
Usage: ROS_DOMAIN_ID=198 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 test_mux_impl_ros.py <path to command_mux or command_mux.py>
"""
import json
import os
import signal
import statistics
import subprocess
import sys
import time

assert os.environ.get('ROS_DOMAIN_ID') == '198', 'isolated domain 198 required'
assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool

exe = sys.argv[1]
rclpy.init()
n = rclpy.create_node('mux_check_fixture')
received, arrivals, states = [], [], []
n.create_subscription(Twist, '/r680_base/cmd_vel',
                      lambda m: (received.append((m.linear.x, m.angular.z)), arrivals.append(time.monotonic())), 10)
n.create_subscription(String, '/r680_base/control_state', lambda m: states.append(json.loads(m.data)), 10)
manual = n.create_publisher(Twist, '/r680_manual/cmd_vel', 1)
nav = n.create_publisher(Twist, '/r680_nav/chassis_cmd_vel', 1)
people = n.create_publisher(String, '/r680/person_follow/command_candidate', 1)
pad = n.create_publisher(String, '/gamepad_status', 1)
wheel = n.create_publisher(Odometry, '/wheel/odom', 1)


def pump(seconds, person=None, navigation=None, hand=None, pad_status='DISCONNECTED', old=False,
         odom_hz=33, manual_hz=33, pad_hz=33):
    end = time.monotonic() + seconds
    due = {'odom': 0., 'manual': 0., 'pad': 0., 'cmd': 0.}
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= due['odom']:
            due['odom'] = now + 1 / odom_hz
            w = Odometry(); w.header.stamp = n.get_clock().now().to_msg(); wheel.publish(w)
        if now >= due['pad']:
            due['pad'] = now + 1 / pad_hz
            pad.publish(String(data=pad_status))
        if now >= due['manual'] and hand is not None:
            due['manual'] = now + 1 / manual_hz
            t = Twist(); t.linear.x = hand; manual.publish(t)
        if now >= due['cmd']:
            due['cmd'] = now + .03
            stamp = n.get_clock().now().nanoseconds - (int(1e9) if old else 0)
            if person is not None:
                people.publish(String(data=json.dumps(dict(schema='wla-person-candidate/v1', stamp_ns=stamp,
                                                           linear=person, angular=0.))))
            if navigation is not None:
                t = Twist(); t.linear.x = navigation; nav.publish(t)
        rclpy.spin_once(n, timeout_sec=.001)


def call_estop(value, expected, **inputs):
    client = n.create_client(SetBool, '/r680_base/set_estop')
    assert client.wait_for_service(timeout_sec=3.)
    future = client.call_async(SetBool.Request(data=value))
    end = time.monotonic() + 2.
    while not future.done() and time.monotonic() < end:
        pump(.03, **inputs)
    assert future.done() and future.result().success is expected, future.result()
    n.destroy_client(client)


def cpu_seconds(pid):
    fields = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')


log = open(f'/tmp/mux_check_{os.path.basename(exe)}.log', 'w')
process = subprocess.Popen([exe], stdout=log, stderr=log, start_new_session=True)
try:
    pump(2.0, person=.65)
    assert received[-1] == (.65, 0.), received[-3:]
    pump(.3, person=.4, navigation=.1)
    assert received[-1] == (.4, 0.)
    pump(.3, person=0., navigation=.1)
    assert received[-1] == (.1, 0.), 'zero releases PERSON priority'
    pump(.3)
    assert received[-1] == (0., 0.), 'all commands expired'
    pump(.3, person=.4, old=True)
    assert received[-1] == (0., 0.), 'stale source stamp'
    pump(.3, person=float('nan'))
    assert received[-1] == (0., 0.), 'non-finite candidate'
    pump(.3, person=.4, hand=.08, pad_status='ACTIVE')
    assert received[-1] == (.08, 0.), 'manual priority'
    pump(.3, person=.4, hand=0., pad_status='ACTIVE')
    assert received[-1] == (.4, 0.), 'follower resumes after release'
    call_estop(True, True)
    pump(.3, person=.4, navigation=.2)
    assert received[-1] == (0., 0.) and states[-1]['estop_latched']
    pump(.3, pad_status='ESTOP')
    call_estop(False, False, pad_status='ESTOP')
    pump(.3, pad_status='ACTIVE', hand=0.)
    call_estop(False, True, pad_status='ACTIVE', hand=0.)
    pump(.3, person=4.)
    assert received[-1] == (1., 0.), 'forward clamp'
    pump(.3, person=-4.)
    assert received[-1] == (-.3, 0.), 'reverse clamp'
    keys = sorted(states[-1])
    # Production-like load: driver odom, gamepad zero stream, status only.
    pump(3., hand=0., odom_hz=200, manual_hz=50, pad_hz=5)
    del arrivals[:]
    c0, t0 = cpu_seconds(process.pid), time.monotonic()
    pump(20., hand=0., odom_hz=200, manual_hz=50, pad_hz=5)
    cpu = 100 * (cpu_seconds(process.pid) - c0) / (time.monotonic() - t0)
    gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
    print(json.dumps(dict(exe=os.path.basename(exe), checks='PASS', cpu_percent=round(cpu, 1),
                          cmd_hz=round(len(gaps) / sum(gaps), 2), gap_ms_p50=round(1e3 * statistics.median(gaps), 2),
                          gap_ms_max=round(1e3 * max(gaps), 2), gap_ms_std=round(1e3 * statistics.pstdev(gaps), 3),
                          state_keys=keys)))
    del received[:]
finally:
    os.killpg(process.pid, signal.SIGINT)
    end = time.monotonic() + 3
    while time.monotonic() < end and process.poll() is None:
        rclpy.spin_once(n, timeout_sec=.01)
    process.wait(timeout=10)
    pump(.2)
    print(json.dumps(dict(exit_code=process.returncode, zeros_after_sigint=sum(v == (0., 0.) for v in received))))
n.destroy_node()
rclpy.shutdown()
