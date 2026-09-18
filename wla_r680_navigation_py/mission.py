"""R680 mission: explore, scan, return to graph-anchored home, and archive."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from .archive import save_archive
from .mission_core import HomeAnchor, MissionResult, arrived, significant_frontiers, stopped_odometry


def load_config(path):
    import yaml
    value = yaml.safe_load(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError(f'invalid YAML mapping: {path}')
    return value


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import PoseStamped
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid, Odometry
    from nav_msgs.srv import GetMap as GetGrid
    from rtabmap_msgs.srv import GetMap as GetGraph
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Empty, Trigger
    from explore_lite_msgs.msg import ExploreStatus
    from tf2_ros import Buffer, TransformListener, TransformException
    from ament_index_python.packages import get_package_prefix
    import yaml

    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config-dir', type=Path, required=True)
    args = parser.parse_args()
    config = load_config(args.config_dir/'mission.yaml')
    if any(isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) or v <= 0 for v in config.values()):
        raise ValueError('mission parameters must be finite and positive')
    run = json.loads((args.output.parent/'run.json').read_text())
    if not run.get('initial_origin_valid'):
        raise RuntimeError('new mapping mission requires a new SLAM session')
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('wla_mapping_mission', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=False)])
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    data, states, futures, polls = {}, {}, {}, {}
    stop_samples = dict(since=None, last=None, received=None)
    ready_samples = dict(since=None, last=None)
    result = MissionResult()
    report = dict(session_id=args.output.parent.name, start_wall_s=time.time(), history=[],
                  coverage_validated=False, completion_basis='frontier_rechecks_not_ground_truth')
    flags = dict(finish=False, cancel=False, move=False)
    def interrupted(*_):
        flags.update(cancel=True, move=False)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    child = None
    stop_confirmed = False
    anchor = None
    permission = node.create_publisher(Bool, '/r680_nav/mission_motion_allowed', 10)
    status_pub = node.create_publisher(String, '/r680_nav/mission_status', retained)
    resume_pub = node.create_publisher(Bool, '/explore/resume', 10)
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    cancel_client = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
    spin_cancel_client = node.create_client(CancelGoal, '/spin/_action/cancel_goal')
    lifecycle = {name: node.create_client(GetState, '/'+name+'/get_state') for name in
                 ('bt_navigator','controller_server','planner_server')}
    subscriptions = []
    def receive(name, msg):
        now = time.monotonic()
        data[name] = (msg, now)
        if name == 'explore':
            report.setdefault('exploration_events', []).append(dict(status=msg.status, monotonic_s=now))
            if result.phase == 'EXPLORING' and msg.status == 'recovery_exhausted':
                report['local_recovery_exhaustions'] = report.get('local_recovery_exhaustions', 0)+1
                node.get_logger().warn('Local recovery exhausted; explorer will try other targets, mission remains EXPLORING')
        if name == 'ready':
            if not msg.data:
                ready_samples['since'] = None
            elif ready_samples['last'] is None or now-ready_samples['last'] > .5 or ready_samples['since'] is None:
                ready_samples['since'] = now
            ready_samples['last'] = now
        if name == 'wheel_odom':
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            stationary = stopped_odometry(msg, node.get_clock().now().nanoseconds/1e9,
                config['stopped_linear_mps'], config['stopped_angular_rps'])
            gap = stop_samples['last'] is None or not 0 < stamp-stop_samples['last'] <= .5 or now-stop_samples['received'] > .5
            if not stationary:
                stop_samples['since'] = None
            elif gap or stop_samples['since'] is None:
                stop_samples['since'] = stamp
            stop_samples.update(last=stamp,received=now)
    for name, cls, topic, qos in [
        ('ready',Bool,'/r680_nav/localization_ready',10),
        ('wheel_odom',Odometry,'/wheel/odom',qos_profile_sensor_data),
        ('map',OccupancyGrid,'/r680/d455/map',retained),
        ('explore',ExploreStatus,'/explore/status',retained),
        ('spins',GoalStatusArray,'/spin/_action/status',retained),
        ('goals',GoalStatusArray,'/navigate_to_pose/_action/status',retained)]:
        subscriptions.append(node.create_subscription(cls, topic,
            lambda msg, key=name: receive(key,msg), qos))

    def snapshot():
        value = dict(report, **result.snapshot())
        value['success'] = result.success and not result.reason and not flags['cancel']
        return value

    def heartbeat():
        permission.publish(Bool(data=flags['move']))
        status_pub.publish(String(data=json.dumps(snapshot(), allow_nan=False)))
    timer = node.create_timer(.1, heartbeat)

    def trigger(key, request, response):
        if result.phase in ('SAVING','SUCCEEDED','FAILED','CANCELED'):
            response.success, response.message = False, 'mission already finalizing'
        else:
            flags[key] = True
            response.success, response.message = True, key+' requested'
        return response
    services = [node.create_service(Trigger, '/r680_nav/finish_exploration',
                    lambda req,res: trigger('finish',req,res)),
                node.create_service(Trigger, '/r680_nav/cancel_mapping',
                    lambda req,res: trigger('cancel',req,res))]

    def phase(value):
        result.phase = value
        report['history'].append(dict(phase=value, monotonic_s=time.monotonic()))
        node.get_logger().info('Mapping phase: '+value)
        heartbeat()
        args.output.write_text(json.dumps(snapshot(), indent=2)+'\n')

    def spin():
        if not rclpy.ok():
            raise RuntimeError('ROS context stopped')
        rclpy.spin_once(node, timeout_sec=.05)

    def wait_until(predicate, timeout, allow_cancel=True):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            spin()
            if allow_cancel and flags['cancel']:
                raise RuntimeError('operator_canceled')
            if predicate():
                return
        raise TimeoutError('timeout in '+result.phase)

    def call(client, request, timeout=None):
        timeout = timeout or config['save_timeout_s']
        try:
            wait_until(client.service_is_ready, timeout, False)
        except TimeoutError as exc:
            raise TimeoutError('service unavailable: '+client.srv_name) from exc
        future = client.call_async(request)
        try:
            wait_until(future.done, timeout, False)
        except TimeoutError as exc:
            raise TimeoutError('service response timeout: '+client.srv_name) from exc
        response = future.result()
        if response is None:
            raise RuntimeError('empty service response')
        return response

    def pose():
        transform = buffer.lookup_transform('map','r680_mapping_floor',rclpy.time.Time())
        stamp = transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
        if not 0 <= node.get_clock().now().nanoseconds/1e9-stamp <= .5:
            raise ValueError('stale TF age='+str(node.get_clock().now().nanoseconds/1e9-stamp))
        p,q = transform.transform.translation, transform.transform.rotation
        out = (p.x,p.y,yaw(q))
        if not all(math.isfinite(v) for v in out):
            raise ValueError('invalid pose')
        return out

    def healthy():
        for name,client in lifecycle.items():
            future = futures.get(name)
            if future is not None and future.done():
                response = future.result()
                states[name] = (response.current_state.id if response else 0,time.monotonic())
                futures.pop(name)
            if name not in futures and time.monotonic()-polls.get(name,0) >= .5 and client.service_is_ready():
                polls[name] = time.monotonic()
                futures[name] = client.call_async(GetState.Request())
        report['readiness'] = dict(lifecycle={k:v[0] for k,v in states.items()}, ready=data['ready'][0].data if 'ready' in data else False, ready_age=time.monotonic()-data['ready'][1] if 'ready' in data else None)
        if any(name not in states or states[name][0] != 3 or time.monotonic()-states[name][1]>2 for name in lifecycle):
            return False
        if 'ready' not in data or not data['ready'][0].data or time.monotonic()-data['ready'][1]>.5 or 'map' not in data:
            return False
        try:
            pose()
        except (TransformException, ValueError) as exc:
            report['readiness']['tf_error'] = str(exc)
            return False
        return True

    def stopped():
        if 'wheel_odom' not in data or time.monotonic()-data['wheel_odom'][1] > .5:
            return False
        msg = data['wheel_odom'][0]
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not 0 <= node.get_clock().now().nanoseconds/1e9-stamp <= .5:
            return False
        v = msg.twist.twist
        report['stop_observation'] = dict(source='/wheel/odom', linear_mps=math.hypot(v.linear.x,v.linear.y), angular_rps=abs(v.angular.z), age_s=node.get_clock().now().nanoseconds/1e9-stamp)
        return stopped_odometry(msg, node.get_clock().now().nanoseconds/1e9,
            config['stopped_linear_mps'], config['stopped_angular_rps'])

    def graph():
        client = node.create_client(GetGraph, '/d455_slam/rtabmap/get_map_data')
        try:
            response = call(client, GetGraph.Request(global_map=True, optimized=True, graph_only=True))
            g = response.data.graph
            if response.data.header.frame_id != 'map' or g.header.frame_id not in ('', 'map') or len(g.poses_id) != len(g.poses):
                raise ValueError('invalid graph frames '+repr((response.data.header.frame_id,g.header.frame_id)))
            return {i:(p.position.x,p.position.y,yaw(p.orientation)) for i,p in zip(g.poses_id,g.poses) if i > 0}
        finally:
            node.destroy_client(client)

    def terminate_explorer():
        nonlocal child
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                wait_until(lambda: child.poll() is not None, 5., False)
            except TimeoutError:
                os.killpg(child.pid, signal.SIGTERM)
                wait_until(lambda: child.poll() is not None, 3., False)
        child = None

    def cancel_and_stop():
        flags['move'] = False
        heartbeat()
        terminate_explorer()  # no late explorer destructor canceling the return goal
        response = call(cancel_client, CancelGoal.Request(), config['cancel_timeout_s'])
        pending_ids = {bytes(g.goal_id.uuid) for g in response.goals_canceling}
        spin_response = call(spin_cancel_client, CancelGoal.Request(), config['cancel_timeout_s'])
        pending_spins = {bytes(g.goal_id.uuid) for g in spin_response.goals_canceling}
        requested = time.monotonic()
        def all_done():
            if not stopped():
                return False
            if 'spins' not in data:
                if pending_spins:
                    return False
            else:
                spins, received_spin = data['spins']
                if pending_spins and received_spin < requested:
                    return False
                if any(s.status in (1,2,3) for s in spins.status_list):
                    return False
            if 'goals' not in data:
                return not pending_ids
            statuses, received = data['goals']
            if pending_ids and received < requested:
                return False
            return all(s.status not in (1,2,3) for s in statuses.status_list)
        try:
            wait_until(all_done, config['cancel_timeout_s'], False)
        except TimeoutError as exc:
            raise TimeoutError('cancel/stop not confirmed: '+repr(dict(stopped=stopped(), pending=len(pending_ids), goals=[g.status for g in data['goals'][0].status_list] if 'goals' in data else None))) from exc
        # A full stopped interval, not one lucky zero-velocity observation.
        stop_samples['since'] = None
        def settled():
            return stopped() and stop_samples['since'] is not None and stop_samples['last']-stop_samples['since'] >= config['settle_s']
        try:
            wait_until(settled, config['cancel_timeout_s'], False)
        except TimeoutError as exc:
            raise TimeoutError('measured stop did not remain stable') from exc

    try:
        phase('PREPARING')
        def startup_ready():
            return (healthy() and stopped() and ready_samples['since'] is not None
                    and time.monotonic()-ready_samples['since'] >= config['settle_s']
                    and stop_samples['since'] is not None
                    and stop_samples['last']-stop_samples['since'] >= config['settle_s'])
        wait_until(startup_ready, config['startup_timeout_s'])
        initial = pose()
        anchor = HomeAnchor.capture(initial, graph())
        report.update(home_initial=initial, home_anchor=asdict(anchor))
        phase('EXPLORING')
        executable = Path(get_package_prefix('explore_lite'))/'lib/explore_lite/explore'
        report['explore_sha256'] = hashlib.sha256(executable.read_bytes()).hexdigest()
        # Read generated geometry, including independent navigation overrides.
        import yaml
        nav_config = yaml.safe_load((args.output.parent/'nav2.yaml').read_text())
        costmap = nav_config['local_costmap']['local_costmap']['ros__parameters']
        footprint = json.loads(costmap['footprint'])
        padding = costmap['footprint_padding']
        scan_radius = max(math.hypot(abs(x)+padding, abs(y)+padding) for x,y in footprint)
        front = max(x for x,y in footprint)
        child = subprocess.Popen([str(executable),'--ros-args','--params-file',
            str(args.config_dir/'exploration.yaml'), '-p', f'scan_radius:={scan_radius}',
            '-p', f'footprint_front:={front}'], start_new_session=True)
        flags['move'] = True
        deadline = time.monotonic()+config['exploration_timeout_s']
        complete_since, rechecks, last_gain = None, 0, None
        result.exploration = 'running'
        while time.monotonic() < deadline and not flags['finish']:
            spin()
            if flags['cancel']:
                raise RuntimeError('operator_canceled')
            if not healthy():
                # Sensor outlet closes independently; persistently unhealthy mission aborts.
                flags['move'] = False
                wait_until(healthy, 2.)
                flags['move'] = True
                continue
            if child.poll() is not None:
                raise RuntimeError('exploration process exited')
            done = 'explore' in data and data['explore'][0].status in (ExploreStatus.EXPLORATION_COMPLETE, 'exploration_blocked')
            if done:
                complete_since = complete_since or time.monotonic()
                if time.monotonic()-complete_since >= config['complete_hold_s']:
                    known = sum(v >= 0 for v in data['map'][0].data)
                    rechecks = rechecks+1 if last_gain == known else 0
                    last_gain = known
                    if rechecks >= 2:
                        remaining = significant_frontiers(data['map'][0])
                        report['remaining_raw_frontiers'] = remaining
                        blocked = data['explore'][0].status == 'exploration_blocked'
                        result.exploration = 'partial' if remaining or blocked else 'complete'
                        report['exploration_stop_reason'] = 'unserved_raw_frontiers' if remaining or blocked else 'stable_frontier_exhaustion'
                        break
                    data.pop('explore',None)
                    complete_since = None
                    resume_pub.publish(Bool(data=True))
            else:
                complete_since = None
        if result.exploration != 'complete':
            result.exploration = 'partial'
            report.setdefault('exploration_stop_reason', 'user_finish' if flags['finish'] else 'budget_exhausted')
        phase('STOPPING_EXPLORATION')
        cancel_and_stop()
        if flags['cancel']:
            raise RuntimeError('operator_canceled')
        phase('RETURNING')
        result.return_home = 'running'
        for attempt in range(2):
            home = anchor.resolve(graph())
            report['home_target'] = home
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x,goal.pose.pose.position.y = home[:2]
            goal.pose.pose.orientation.z,goal.pose.pose.orientation.w = math.sin(home[2]/2),math.cos(home[2]/2)
            wait_until(lambda: nav.server_is_ready() and healthy(), config['cancel_timeout_s'])
            flags['move'] = True
            future = nav.send_goal_async(goal)
            wait_until(future.done, config['cancel_timeout_s'])
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('return goal rejected')
            future = handle.get_result_async()
            def return_finished():
                if not healthy():
                    flags['move'] = False
                    wait_until(healthy, 2.)
                    flags['move'] = True
                return future.done()
            wait_until(return_finished, config['return_timeout_s'])
            status = future.result().status
            flags['move'] = False
            if status != 4:
                raise RuntimeError('return failed with status '+str(status))
            home = anchor.resolve(graph())
            wait_until(healthy, 2.)
            if arrived(pose(),home,config['xy_tolerance_m'],config['yaw_tolerance_rad']):
                result.return_home = 'succeeded'
                break
        if result.return_home != 'succeeded':
            raise RuntimeError('return pose outside optimized home tolerance')
        phase('SETTLING')
        cancel_and_stop()
        stop_confirmed = True
    except KeyboardInterrupt:
        flags['cancel'] = True
        result.reason = 'operator_canceled'
    except Exception as exc:
        result.reason = str(exc)
        if result.return_home == 'running':
            result.return_home = 'failed'
        if result.exploration == 'running':
            result.exploration = 'partial'
    finally:
        flags['move'] = False
        try:
            if not stop_confirmed:
                phase('STOPPING')
                cancel_and_stop()
            phase('SAVING')
            result.save = 'running'
            pause = node.create_client(Empty, '/d455_slam/rtabmap/pause')
            call(pause, Empty.Request())
            grid_client = node.create_client(GetGrid, '/d455_slam/rtabmap/get_map')
            msg = call(grid_client, GetGrid.Request()).map
            frozen = dict(frame_id=msg.header.frame_id,width=msg.info.width,height=msg.info.height,
                resolution=msg.info.resolution,origin=[msg.info.origin.position.x,msg.info.origin.position.y,
                yaw(msg.info.origin.orientation)],data=list(msg.data))
            backup = Path(run['database']+'.back')
            previous = backup.stat().st_mtime_ns if backup.exists() else None
            backup_client = node.create_client(Empty, '/d455_slam/rtabmap/backup')
            call(backup_client, Empty.Request())
            if not backup.is_file() or backup.stat().st_mtime_ns == previous:
                raise RuntimeError('RTAB-Map did not produce a fresh closed backup')
            report['map_directory'] = str(args.output.parent/'map_archive')
            config_snapshot = {p.stem:load_config(p) for p in args.config_dir.glob('*.yaml')}
            config_snapshot['nav2'] = load_config(args.output.parent/'nav2.yaml')
            metadata = snapshot()
            metadata['save_outcome'] = 'succeeded'
            metadata['success'] = result.exploration == 'complete' and result.return_home == 'succeeded' and not result.reason and not flags['cancel']
            metadata['phase'] = ('CANCELED' if flags['cancel'] else 'SUCCEEDED' if metadata['success'] else 'COMPLETED_PARTIAL' if result.exploration == 'partial' and result.return_home == 'succeeded' and not result.reason else 'FAILED')
            save_archive(report['map_directory'],frozen,backup,metadata,config_snapshot)
            result.save = 'succeeded'
        except Exception as exc:
            result.save = 'failed'
            report['save_error'] = str(exc)
        partial_ok = result.exploration == 'partial' and result.return_home == 'succeeded' and result.save == 'succeeded' and not result.reason
        result.phase = 'CANCELED' if flags['cancel'] else ('SUCCEEDED' if result.success and not result.reason else ('COMPLETED_PARTIAL' if partial_ok else 'FAILED'))
        report['wall_elapsed_s'] = time.time()-report['start_wall_s']
        phase(result.phase)
        try:
            terminate_explorer()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0 if result.success and not result.reason else 1


if __name__ == '__main__':
    raise SystemExit(main())
