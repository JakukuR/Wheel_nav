"""Isolated integration test; never run in a hardware ROS domain."""
import os,time,subprocess,signal,json,tempfile
assert os.environ.get('ROS_DOMAIN_ID')=='86'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool

rclpy.init();n=Node('manual_mux_contract');outputs=[];states=[]
m=n.create_publisher(Twist,'/r680_manual/cmd_vel',1)
a=n.create_publisher(Twist,'/r680_nav/chassis_cmd_vel',1)
s=n.create_publisher(String,'/gamepad_status',10)
w=n.create_publisher(Odometry,'/wheel/odom',10)
n.create_subscription(Twist,'/r680_base/cmd_vel',lambda msg:outputs.append((msg.linear.x,msg.angular.z)),10)
n.create_subscription(String,'/r680_base/control_state',lambda msg:states.append(json.loads(msg.data)),10)
def feed(seconds,manual=0.,auto=0.,status='ACTIVE',send_auto=True,send_manual=True):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        if send_manual:
            v=Twist();v.linear.x=manual;m.publish(v)
        if send_auto:
            v=Twist();v.linear.x=auto;a.publish(v)
        s.publish(String(data='['+status+'] fixture'))
        msg=Odometry();msg.header.stamp=n.get_clock().now().to_msg();w.publish(msg)
        rclpy.spin_once(n,timeout_sec=.01);time.sleep(.01)
with tempfile.TemporaryFile(mode='w+') as log:
    p=subprocess.Popen(['ros2','run','wla_manual_control','command_mux.py'],stdout=log,stderr=log,start_new_session=True)
    try:
        feed(2.,manual=.2);assert outputs and all(v==(0.,0.) for v in outputs)
        feed(1.2);assert states[-1]['armed']
        client=n.create_client(SetBool,'/r680_base/set_automatic');assert client.wait_for_service(timeout_sec=3.)
        request=SetBool.Request();request.data=True;f=client.call_async(request)
        rclpy.spin_until_future_complete(n,f,timeout_sec=2.);assert f.result().success
        feed(.4,auto=.15);assert abs(outputs[-1][0]-.15)<1e-6
        feed(.4,send_auto=False);assert outputs[-1]==(0.,0.)
        feed(.4,manual=.18,auto=.25);assert abs(outputs[-1][0]-.18)<1e-6
        feed(.4,auto=.25);assert outputs[-1]==(0.,0.) and states[-1]['mode']=='MANUAL'
        feed(.3,manual=.2,status='ESTOP');assert outputs[-1]==(0.,0.)
        feed(.5,manual=.2);assert outputs[-1]==(0.,0.)
        feed(1.1);feed(.3,manual=3.);assert abs(outputs[-1][0]-.3)<1e-6
        feed(.4,send_manual=False);assert outputs[-1]==(0.,0.)
        print('PASS boot neutral, explicit auto, navigation timeout, manual takeover latch, estop, reconnect neutral, limits, stale joystick')
    finally:
        os.killpg(p.pid,signal.SIGINT);p.wait(timeout=10)
        if p.returncode not in (0,130):log.seek(0);print(log.read())
n.destroy_node();rclpy.shutdown()
