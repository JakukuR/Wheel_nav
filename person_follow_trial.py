#!/usr/bin/env python3
"""Follow with a full/short target ID or an interactive person picker."""
import argparse
import json
import math
import signal
import select
import sys
import time
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from wla_behavior_interfaces.srv import PersonCommand


def visible_people(data, now_ns, label='person'):
    """A choice binds to the current full ID, never a future reused short ID."""
    if (data.get('schema')!='wla-relative-detections/v1' or data.get('enabled') is not True
            or not isinstance(data.get('session'),str) or not data['session']
            or type(data.get('epoch')) is not int or data['epoch']<0
            or not isinstance(data.get('objects'),list)
            or type(data.get('stamp_ns')) is not int
            or not -.05 <= (now_ns-data['stamp_ns'])/1e9 <= .30):
        raise RuntimeError('人物检测不可用或已过期，请刷新人物列表')
    prefix=f"{data['session']}:{data['epoch']}:";people=[];ids=set()
    for row in data.get('objects',[]):
        if not isinstance(row,dict):continue
        ident=row.get('id','')
        if (row.get('detected') is not True or (row.get('label')=='person' if label=='object' else row.get('label')!=label)
                or not isinstance(ident,str) or not ident.startswith(prefix)
                or not ident[len(prefix):].isdigit()):continue
        if ident in ids:raise RuntimeError('人物 ID 重复，无法唯一选择')
        ids.add(ident)
        distance,yaw=row.get('distance'),row.get('yaw')
        if (type(distance) not in (int,float) or not math.isfinite(distance) or distance<=0
                or type(yaw) not in (int,float) or not math.isfinite(yaw) or abs(yaw)>math.pi):continue
        people.append(row)
    return sorted(people,key=lambda r:int(r['id'].rsplit(':',1)[1]))


def resolve_target(selector, people):
    selector=selector.strip()
    matches=[row for row in people if row['id']==selector or
        (selector.isdecimal() and row['id'].rsplit(':',1)[1]==selector) or
        (selector.count(':')==1 and row['id'].endswith(':'+selector))]
    if len(matches)!=1:
        raise RuntimeError('目标 ID 不存在或不唯一；请用 list 查看当前人物，或不填 --target-id 交互选择')
    return matches[0]['id']


def show_people(people):
    print('\n序号  画面 ID       类别          距离      方位',flush=True)
    for index,row in enumerate(people,1):
        yaw=math.degrees(row['yaw']);side='左' if yaw>=0 else '右'
        print(f"{index:>3}   {row['id'].rsplit(':',2)[-2]+':'+row['id'].rsplit(':',1)[-1]:<12} {row['label']:<12} {row['distance']:.2f} m   {side} {abs(yaw):.0f}°",flush=True)
    if not people:print('当前没有位置有效的人物。',flush=True)
    print('输入序号后回车；r 刷新；q 取消：',end='',flush=True)


def choose_interactively(people_now,pump,is_stopped,stdin=None):
    stdin=sys.stdin if stdin is None else stdin
    if not stdin.isatty():raise RuntimeError('交互选择需要终端；脚本调用请传 --target-id 短ID或完整ID')
    people=people_now();show_people(people)
    while not is_stopped():
        # Keep ROS subscriptions alive while the operator reads the list.
        pump()
        if is_stopped():return None
        if not select.select([stdin],[],[],0)[0]:continue
        choice=stdin.readline()
        if choice=='':return None
        choice=choice.strip().lower()
        if choice=='q':return None
        if choice=='r':people=people_now();show_people(people);continue
        if not choice.isdecimal() or not 1<=int(choice)<=len(people):
            print('请输入表中的序号，或 r/q：',end='',flush=True);continue
        selected=people[int(choice)-1]['id']
        # Preserve the displayed choice. Never reinterpret its index/short ID
        # against a later detector epoch or reordered candidate list.
        try:return resolve_target(selected,people_now())
        except RuntimeError:
            print('该人物已不在当前检测中，请 r 刷新后重新选择：',end='',flush=True)
    return None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('operation', choices=['list', 'status', 'follow'])
    p.add_argument('--target-id', default='', help='Full ID, short ID (e.g. 190), or epoch:ID; omit for interactive selection')
    p.add_argument('--mode', choices=['0','1','2'], default='0',
                   help='0 人物跟随；1 人物靠近并停车；2 物体靠近并停车')
    duration = p.add_mutually_exclusive_group()
    duration.add_argument('--seconds', type=float, default=30., help='Timed follow duration, 1–120 s (default: 30)')
    duration.add_argument('--continuous', action='store_true', help='Keep renewing the task until Ctrl+C or a backend stop/failure')
    p.add_argument('--allow-motion', action='store_true', help='Allow a trial against an already motion-enabled backend')
    args = p.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        p.error('--seconds must be between 1 and 120')
    if args.continuous and args.operation != 'follow':
        p.error('--continuous requires follow')
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
            mode=args.mode, target_id=args.target_id, lease_duration_s=1.5)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=.7)
        if not future.done():
            raise RuntimeError(kind + '_response_unknown; lease will expire without renewal')
        response = future.result()
        if response is not None and kind=='heartbeat' and response.request_id==request_id and response.state=='arrived':
            return response
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
        target_label = 'object' if args.mode == '2' else 'person'
        people_now=lambda:visible_people(fresh('objects'),node.get_clock().now().nanoseconds,target_label)
        if args.target_id:
            args.target_id=resolve_target(args.target_id,people_now())
        else:
            args.target_id=choose_interactively(people_now,
                lambda:rclpy.spin_once(node,timeout_sec=.03),lambda:stopped)
            if args.target_id is None:return 0
        if stopped:return 0
        if not client.wait_for_service(timeout_sec=2.):
            raise RuntimeError('command_service_unavailable')
        # Operator input can take minutes. Recheck current authority and the
        # exact chosen full ID immediately before attempting start.
        state=fresh('status')
        if not args.allow_motion and state.get('candidate_only') is not True:
            raise RuntimeError('backend_can_move; preview requires enable_motion:=false')
        if state.get('enabled'):raise RuntimeError('another_task_is_active; stop it through its owner first')
        if state.get('fault') or state.get('direct_publisher_conflict'):
            raise RuntimeError('backend_fault_or_output_conflict')
        args.target_id=resolve_target(args.target_id,people_now())
        print(('已选择物体：' if args.mode == '2' else '已选择人物：')+args.target_id,flush=True)
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
                if state.get('state') == 'arrived':
                    print(json.dumps(state.get('arrival'),ensure_ascii=False),flush=True)
                    return 0
                raise RuntimeError('task_ended:' + state.get('reason', 'unknown'))
            if time.monotonic() >= next_heartbeat:
                response=command('heartbeat')
                if response.state=='arrived':
                    print('任务到达并结束：'+response.reason,flush=True)
                    return 0
                next_heartbeat = time.monotonic()+.4
            if state.get('request_id') == request_id and time.monotonic() >= next_print:
                print(json.dumps({k: state.get(k) for k in ('state', 'target_id', 'safety_reason',
                    'motion_permitted', 'linear_mps', 'angular_rps', 'compute_ms',
                    'result_remaining_ms', 'output_blockers', 'planner_candidates',
                    'target_tracking', 'resolved_target_id', 'reid', 'stamp_ns', 'diagnostic_sequence',
                    'target_observation', 'target_mask', 'recovery_state', 'planner_reason',
                    'measured_motion', 'actual_published_command', 'diagnostics')}, ensure_ascii=False), flush=True)
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
