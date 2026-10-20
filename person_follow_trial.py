#!/usr/bin/env python3
"""Explicit-ID follow client with timed or continuous mode; never publishes velocity."""
import argparse
import json
import math
import signal
import time
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from wla_behavior_interfaces.srv import PersonCommand


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('operation', choices=['list', 'status', 'follow'])
    p.add_argument('--target-id', default='')
    duration = p.add_mutually_exclusive_group()
    duration.add_argument('--seconds', type=float, default=30., help='Timed follow duration, 1–120 s (default: 30)')
    duration.add_argument('--continuous', action='store_true', help='Keep renewing the task until Ctrl+C or a backend stop/failure')
    p.add_argument('--allow-motion', action='store_true', help='Allow a trial against an already motion-enabled backend')
    args = p.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        p.error('--seconds must be between 1 and 120')
    if args.continuous and args.operation != 'follow':
        p.error('--continuous requires follow')
    if args.operation == 'follow' and not args.target_id:
        p.error('follow requires the full --target-id from list; no automatic selection')
    rclpy.init()
    node = rclpy.create_node('person_follow_trial_' + uuid.uuid4().hex[:8])
    seen = {}
    stopped = False

    def interrupt(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    def receive(key, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                seen[key] = (value, time.monotonic())
        except ValueError:
            pass

    for key, topic in [('status', '/r680/person_follow/status'),
                       ('objects', '/r680/perception/relative_objects')]:
        node.create_subscription(String, topic, lambda m, k=key: receive(k, m), qos_profile_sensor_data)
    client = node.create_client(PersonCommand, '/r680/person_follow/command')
    request_id = 'trial-' + uuid.uuid4().hex
    start_attempted = False

    def fresh(key):
        value, received = seen.get(key, ({}, 0.))
        age = (node.get_clock().now().nanoseconds-value.get('stamp_ns', 0))/1e9
        if time.monotonic()-received > .5 or not -.05 <= age <= .5:
            raise RuntimeError(key + '_unavailable_or_stale')
        return value

    def command(kind):
        request = PersonCommand.Request(command=kind, request_id=request_id,
            mode='follow', target_id=args.target_id, lease_duration_s=1.5)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=.7)
        if not future.done():
            raise RuntimeError(kind + '_response_unknown; lease will expire without renewal')
        response = future.result()
        if response is None or not response.accepted:
            raise RuntimeError(kind + '_rejected:' + (response.reason if response else 'no_response'))
        return response

    try:
        end = time.monotonic()+3.
        while not stopped and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        if stopped:
            return 0
        if args.operation != 'follow':
            data = fresh('objects' if args.operation == 'list' else 'status')
            print(json.dumps(data, ensure_ascii=False, indent=2), flush=True)
            return 0
        state = fresh('status')
        if not args.allow_motion and state.get('candidate_only') is not True:
            raise RuntimeError('backend_can_move; preview requires enable_motion:=false')
        if state.get('enabled'):
            raise RuntimeError('another_task_is_active; stop it through its owner first')
        if state.get('fault') or state.get('direct_publisher_conflict'):
            raise RuntimeError('backend_fault_or_output_conflict')
        if not client.wait_for_service(timeout_sec=2.):
            raise RuntimeError('command_service_unavailable')
        start_attempted = True
        response = command('start')
        print(json.dumps({'request_id': request_id, 'target_id': args.target_id,
            'candidate_only': state.get('candidate_only'),
            'continuous': args.continuous, 'seconds': None if args.continuous else args.seconds,
            'reason': response.reason}, ensure_ascii=False), flush=True)
        end = None if args.continuous else time.monotonic()+args.seconds
        next_heartbeat, next_print = time.monotonic()+.4, 0.
        while not stopped and (end is None or time.monotonic() < end):
            rclpy.spin_once(node, timeout_sec=.03)
            if stopped:
                break
            state = fresh('status')
            if not args.allow_motion and state.get('candidate_only') is not True:
                raise RuntimeError('backend_motion_mode_changed')
            if state.get('request_id') == request_id and not state.get('enabled'):
                raise RuntimeError('task_ended:' + state.get('reason', 'unknown'))
            if time.monotonic() >= next_heartbeat:
                command('heartbeat')
                next_heartbeat = time.monotonic()+.4
            if state.get('request_id') == request_id and time.monotonic() >= next_print:
                print(json.dumps({k: state.get(k) for k in ('state', 'target_id', 'safety_reason',
                    'motion_permitted', 'linear_mps', 'angular_rps', 'compute_ms',
                    'result_remaining_ms', 'output_blockers', 'planner_candidates',
                    'target_tracking', 'resolved_target_id', 'reid')}, ensure_ascii=False), flush=True)
                next_print = time.monotonic()+1.
        return 0
    except (RuntimeError, ValueError) as exc:
        print(str(exc), flush=True)
        return 1
    finally:
        if start_attempted:
            try:
                command('stop')
                print('Task stop acknowledged; this is not a physical braking measurement.', flush=True)
            except Exception as exc:
                print('Stop response unavailable: ' + str(exc) + '; no further heartbeat.', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
