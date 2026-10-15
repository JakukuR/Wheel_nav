#!/usr/bin/env python3
"""Real guard + mux in isolated domain 198; no chassis driver."""
import os,sys,time,signal,subprocess,json
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='198'
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,String
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatus,GoalStatusArray,GoalInfo
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root.parent/'wla_manual_control/scripts'))
from command_mux import Mux
from control_policy import Policy
rclpy.init();node=Node('handoff_fixture');mux=Mux();ex=SingleThreadedExecutor()
ex.add_node(node);ex.add_node(mux)
wheel=node.create_publisher(Odometry,'/wheel/odom',qos_profile_sensor_data)
health=node.create_publisher(Bool,'/r680_nav/localization_ready',10)
cmd=node.create_publisher(Twist,'/r680_nav/cmd_vel_collision_checked',10)
gamepad=node.create_publisher(String,'/gamepad_status',10)
manual=node.create_publisher(Twist,'/r680_manual/cmd_vel',1)
statuses=node.create_publisher(GoalStatusArray,'/navigate_to_pose/_action/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
states=[];base=[];cancelled=[];permissions=[]
node.create_subscription(String,'/r680_nav/navigation_motion_state',lambda m:states.append((time.monotonic(),m.data)),10)
node.create_subscription(Twist,'/r680_base/cmd_vel',lambda m:base.append((time.monotonic(),m.linear.x,m.angular.z)),10)
node.create_subscription(Bool,'/r680_nav/mission_motion_allowed',lambda m:permissions.append((time.monotonic(),m.data)),10)
goal=GoalInfo();goal.goal_id.uuid=[9]*16
def cancel(req,res):
    cancelled.append(time.monotonic());res.return_code=CancelGoal.Response.ERROR_NONE
    res.goals_canceling=[goal];return res
node.create_service(CancelGoal,'/navigate_to_pose/_action/cancel_goal',cancel)

def spin(seconds,callback):
    end=time.monotonic()+seconds
    while time.monotonic()<end:callback();ex.spin_once(timeout_sec=.005)

results=[]
cases=os.environ.get('WLA_HANDOFF_CASES','normal,manual_takeover,estop,process_killed,delayed_health,cancel_health_drop,waiting_health,stale_health').split(',')
try:
 for case in cases:
    states.clear();base.clear();cancelled.clear();permissions.clear();mux.p=Policy(1.,.3,1.5)
    guard=subprocess.Popen([str(Path(os.environ['WLA_NAV_BIN'])/'command_guard'),'--ros-args',
        '-p','hardware_output_enabled:=false','-p','require_frontend_motion_permission:=false'],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    session=subprocess.Popen([sys.executable,str(root/'scripts/navigation_motion_session.py')],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    start=time.monotonic();enabled=None;triggered=False;health_stopped=False;stale_sent=False
    try:
     def fixture():
        global enabled,triggered,health_stopped,stale_sent
        now=time.monotonic()
        hits=[t for t,s in states if s.startswith('navigation_enabled;')]
        if hits and enabled is None:enabled=hits[0]
        age=now-enabled if enabled is not None else -1.
        stamp=node.get_clock().now().to_msg()
        o=Odometry();o.header.stamp=stamp;o.pose.pose.orientation.w=1.
        v,w=base[-1][1:] if base else (0.,0.)
        o.twist.twist.linear.x=v;o.twist.twist.angular.z=w;wheel.publish(o)
        h=not(case=='normal' and .6<age<1.2)
        if case=='delayed_health':h=now-start>6.5
        if case=='cancel_health_drop' and cancelled:h=now-cancelled[0]>1.2
        if case=='waiting_health':h=False
        if case=='stale_health':
            if not stale_sent:
                # Send one true after discovery, then go silent. A never-received
                # sample would only duplicate waiting_health, not test freshness.
                send_true=bool(permissions) and now-start>1.2
                health.publish(Bool(data=send_true));stale_sent=send_true
        else:health.publish(Bool(data=h))
        # Old commands exist before handoff, but must not leak through. Once
        # canceled, no new goal is sent until 0.3 seconds after authorization.
        if not cancelled or age>.3:
            c=Twist();c.linear.x=.12;cmd.publish(c)
        gamepad.publish(String(data='ACTIVE' if case=='manual_takeover' else 'DISCONNECTED'))
        m=Twist()
        if case=='manual_takeover' and .6<age<.72:m.linear.x=.05
        manual.publish(m)
        status=GoalStatus();status.goal_info=goal
        finished=bool(cancelled) and (case!='cancel_health_drop' or now-cancelled[0]>.4)
        status.status=5 if finished else 2
        statuses.publish(GoalStatusArray(status_list=[status]))
        if case=='estop' and age>.65 and not triggered:
            mux.p.latch_stop('test_estop');triggered=True
        if case=='process_killed' and age>.65 and not triggered:
            session.kill();triggered=True
        if case=='normal' and 1.0<age<1.15:
            recent=[r for r in base if r[0]>now-.08]
            if recent and all(abs(r[1])<1e-8 for r in recent):health_stopped=True
     spin(10. if case=='delayed_health' else 8. if case=='stale_health' else 6.,fixture)
     if case in ('waiting_health','stale_health'):
        assert session.poll() is None,(case,'waiting session exited',states)
        assert enabled is None,(case,'navigation enabled without current health')
        assert permissions and not any(v for _,v in permissions),(case,'permission granted while waiting')
        assert base and all(abs(v)+abs(w)<1e-8 for _,v,w in base),(case,'waiting output nonzero')
        if case=='stale_health':
            assert stale_sent and any('localization_report_stale' in s for _,s in states),(case,'stale true was not observed',states)
        session.send_signal(signal.SIGTERM);spin(.8,fixture)
        assert session.poll() is not None,(case,'waiting did not cancel')
        results.append(dict(case=case,waited_without_exit=True,no_permission=True,final_zero=True))
        print('PASS '+case,flush=True)
        continue
     assert enabled is not None,(case,states,session.poll(),session.stderr.read().decode() if session.poll() is not None else '')
     assert cancelled and cancelled[0]<enabled,(case,'old goal not canceled')
     before=[r for r in base if r[0]<enabled+.25]
     assert before and all(abs(r[1])<1e-8 for r in before),(case,'stale command replay')
     assert any(r[1]>.1 for r in base),(case,'new navigation did not pass guard and mux')
     if case=='delayed_health':assert enabled-start>6.5,(case,'enabled before health recovery')
     if case=='cancel_health_drop':assert enabled-cancelled[0]>1.2,(case,'enabled during cancellation health loss')
     if case=='normal':
        assert health_stopped,'Invalid localization did not stop output'
        assert any(r[1]>.1 for r in base if r[0]>enabled+1.4),'Healthy fresh stream did not recover'
     if case=='manual_takeover':
        assert session.poll() is None,(case,'manual input ended the navigation session')
        assert any(abs(v-.05)<1e-8 for _,v,_ in base),(case,'manual input did not override NAV')
        assert any(v>.1 for t,v,_ in base if t>enabled+1.0),(case,'NAV did not resume after release')
     if case in ('normal','manual_takeover','delayed_health','cancel_health_drop'):session.send_signal(signal.SIGTERM)
     spin(.8,fixture)
     if case!='process_killed':
        assert session.poll() is not None,(case,'session did not exit')
        assert permissions and permissions[-1][1] is False,(case,'navigation permission not released')
     assert base and all(abs(r[1])+abs(r[2])<1e-8 for r in base[-10:]),(case,'nonzero final output')
     results.append(dict(case=case,cancelled_old_goal=True,new_command_passed=True,final_zero=True))
     print('PASS '+case,flush=True)
    finally:
     for p in (session,guard):
        if p.poll() is None:
            p.send_signal(signal.SIGTERM)
            try:p.wait(timeout=3.)
            except subprocess.TimeoutExpired:p.kill();p.wait()
 print(json.dumps(results,indent=2))
 print('PASS handoff cases: '+','.join(cases))
finally:
 for n in (mux,node):ex.remove_node(n);n.destroy_node()
 ex.shutdown();rclpy.shutdown()
