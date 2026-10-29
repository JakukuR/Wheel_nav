#!/usr/bin/env python3
"""Exercise one-shot motion with the real C++ mux, fake sensors, and no driver."""
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
from types import SimpleNamespace

assert os.environ.get('ROS_DOMAIN_ID')=='198', 'Test only on isolated domain 198'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Imu, Image

root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from mux_process import MuxProcess
script=root/'scripts/openvins_motion_init.py'
spec=importlib.util.spec_from_file_location('motion_init',script)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
for bad in [(float('nan'),.5,1),(.21,.5,1),(.2,.51,1),(.2,.5,1.01),(-.1,.5,1)]:
    try:module.Arc(*bad)
    except ValueError:pass
    else:raise AssertionError('Unbounded motion accepted')
arc=module.Arc();assert arc.command(100)==(0.,0.)
assert arc.command(100.3)==(.2,.5)
assert arc.command(101)==(0.,0.) and arc.command(102)==(0.,0.)
arc=module.Arc();arc.observe(0,0,math.pi-.02);arc.observe(.1,0,-math.pi+.02)
assert abs(arc.yaw-.04)<1e-8
try:arc.observe(.31,0,-math.pi+.02)
except module.Abort:pass
else:raise AssertionError('Wheel path budget ignored')

# Execute the real entry-point readiness branch with an otherwise-ready Nav2.
# It must never report ready while the arc is running or before failure cleanup.
entry=(root.parents[1]/'r680_nav.sh').read_text()
branch=entry[entry.index('    motion_ok = not motion_init'):entry.index('    if time.monotonic() - last_report',entry.index('    motion_ok = not motion_init'))]
# The real readiness branch can continue its outer polling loop.
branch=compile('for _ in range(1):\n'+textwrap.indent(textwrap.dedent(branch),'    '),
               'r680_nav.sh readiness','exec')
with tempfile.TemporaryDirectory(prefix='init-readiness-') as folder:
    result=Path(folder)/'result.json'
    for state in (None,'waiting_inputs','moving','stopping','failed','succeeded','already_initialized'):
        if state is not None:result.write_text(json.dumps({'state':state,'reason':'test'}))
        env=dict(motion_init=True,Path=Path,os=SimpleNamespace(environ={'WLA_INIT_MOTION_RESULT':str(result)}),json=json,
                 ready_count=5,last_ready=time.monotonic(),time=time,startup=SimpleNamespace(read=lambda:True),
                 map_ok=True,tf_ok=True,scan_ok=True,auto_init=False,init_state='succeeded',states={'controller':3},
                 stack_ready=False,map_mode='mapless',
                 node=SimpleNamespace(destroy_node=lambda:None),rclpy=SimpleNamespace(shutdown=lambda:None))
        code=None
        try:exec(branch,env)
        except SystemExit as error:code=error.code
        if state in ('succeeded','already_initialized'):assert code==0,(state,code)
        elif state=='failed':assert isinstance(code,str) and 'test' in code,(state,code)
        else:assert code is None,(state,code)

rclpy.init();probe=Node('motion_init_test');guard=Node('r680_command_guard');mux=MuxProcess(probe,'/tmp/openvins-motion-init-mux.log')
guard.declare_parameter('hardware_output_enabled',False)
guard_pub=guard.create_publisher(Twist,'/r680_nav/chassis_cmd_vel',1) # deliberately silent
executor=SingleThreadedExecutor()
for n in (probe,guard):executor.add_node(n)
pub_pose=probe.create_publisher(PoseWithCovarianceStamped,'/openvins/poseimu',10)
pub_imu=probe.create_publisher(Imu,'/r680/d455/imu',qos_profile_sensor_data)
pub_image=probe.create_publisher(Image,'/r680/d455/infra1/image_rect_raw',qos_profile_sensor_data)
pub_gamepad=probe.create_publisher(String,'/gamepad_status',10)
pub_manual=probe.create_publisher(Twist,'/r680_manual/cmd_vel',1)
pub_wheel=probe.create_publisher(Odometry,'/wheel/odom',qos_profile_sensor_data)
received=[];preview=[];base=[]
probe.create_subscription(Twist,'/r680_nav/chassis_cmd_vel',lambda m:received.append((time.monotonic(),m.linear.x,m.angular.z)),10)
probe.create_subscription(Twist,'/r680_nav/init_motion_preview',lambda m:preview.append((time.monotonic(),m.linear.x,m.angular.z)),10)
probe.create_subscription(Twist,'/r680_base/cmd_vel',lambda m:base.append((time.monotonic(),m.linear.x,m.angular.z)),10)
results=[]

def case(name,execute=True,guard_enabled=False):
    received.clear();preview.clear();base.clear()
    mux.restart(lambda:executor.spin_once(timeout_sec=.01))  # fresh policy state per case
    guard.set_parameters([Parameter('hardware_output_enabled',value=guard_enabled)])
    with tempfile.TemporaryDirectory(prefix='openvins-init-test-') as folder:
        result=Path(folder)/'result.json'
        proc=subprocess.Popen([sys.executable,str(script),'--result',str(result)]+(['--execute'] if execute else []),stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        start=last=time.monotonic();first=None;sent_signal=False;x=y=yaw=0.
        try:
            while time.monotonic()-start<18 and proc.poll() is None:
                now=time.monotonic();dt=min(.05,now-last);last=now
                nonzero=[r for r in (received if execute else preview) if r[1] or r[2]]
                if nonzero and first is None:first=nonzero[0][0]
                since=now-first if first else -1
                v,w=(base[-1][1:] if base else (0.,0.))
                x+=v*math.cos(yaw)*dt;y+=v*math.sin(yaw)*dt;yaw+=w*dt
                stamp=probe.get_clock().now().to_msg()
                odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='wheel_odom';odom.child_frame_id='base_link'
                odom.pose.pose.position.x=x;odom.pose.pose.position.y=y
                odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2)
                odom.twist.twist.linear.x=v;odom.twist.twist.angular.z=w;pub_wheel.publish(odom)
                pub_gamepad.publish(String(data='DISCONNECTED' if name=='no_gamepad' else 'ACTIVE'))
                manual=Twist()
                if name=='manual_takeover' and .15<since<.25:manual.linear.x=.05
                pub_manual.publish(manual)
                imu_age=-9.489059448242188e-5 if name=='no_gamepad' else .005
                image_age=.02
                # Leave transport margin beyond the allowed 50 ms clock lead.
                if name=='future_sensor' and since>.15:imu_age=-.20
                if name=='stale_sensor' and since>.15:imu_age=.101
                if name=='stale_image' and since>.15:image_age=.351
                ns=probe.get_clock().now().nanoseconds
                imu=Imu();imu.header.stamp=rclpy.time.Time(nanoseconds=ns-int(imu_age*1e9)).to_msg()
                image=Image();image.header.stamp=rclpy.time.Time(nanoseconds=ns-int(image_age*1e9)).to_msg()
                if not (name=='sensor_report_lost' and since>.15):
                    pub_imu.publish(imu);pub_image.publish(image)
                if since>1.6:
                    pose=PoseWithCovarianceStamped();pose.header.stamp=stamp
                    pose.header.frame_id='global';pose.pose.pose.orientation.w=1.;pub_pose.publish(pose)
                if name=='sigterm' and since>.15 and not sent_signal:
                    proc.send_signal(signal.SIGTERM);sent_signal=True
                executor.spin_once(timeout_sec=.01)
            if proc.poll() is None:
                raise AssertionError(name+' did not finish')
            # Drain final zero messages and the real mux timeout.
            deadline=time.monotonic()+.4
            while time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
            data=json.loads(result.read_text())
            commands=[r for r in received if r[1] or r[2]]
            assert all(0<=r[1]<=.2 and 0<=r[2]<=.5 for r in commands),name
            assert base and all(r[1:]==(0.,0.) for r in base[-5:]),name+' nonzero final chassis output'
            assert mux.state.get('linear')==0. and mux.state.get('angular')==0.,name+' mux output remained nonzero'
            if name in ('positive','no_gamepad','preview'):
                assert proc.returncode==0 and data['state']=='succeeded',(name,data,proc.stderr.read().decode())
                if execute:
                    assert commands and max(r[1] for r in commands)>.19
                    assert commands[-1][0]-commands[0][0]<=1.05
                    assert data['manual_release_confirmed']
                else:
                    assert not received and preview and any(r[1] for r in preview)
            else:
                assert proc.returncode!=0 and data['state']=='failed',(name,data)
                if guard_enabled:assert not commands and not data['motion_requested']
                else:assert commands and commands[-1][0]-commands[0][0]<1.0
            results.append(dict(case=name,state=data['state'],reason=data['reason'],nonzero=len(commands)))
            print('PASS '+name,flush=True)
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=3)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()

try:
    cases=os.environ.get('WLA_INIT_CASES',
        'positive,no_gamepad,manual_takeover,sensor_report_lost,sigterm,future_sensor,stale_sensor,stale_image,guard_enabled,preview').split(',')
    for name in cases:
        case(name,execute=name!='preview',guard_enabled=name=='guard_enabled')
    print(json.dumps(results,indent=2));print('PASS one bounded arc, real mux takeover, fault stop, SIGTERM cleanup, disabled Nav2 output and preview isolation')
finally:
    mux.stop()
    for n in (guard,probe):executor.remove_node(n);n.destroy_node()
    executor.shutdown();rclpy.shutdown()
