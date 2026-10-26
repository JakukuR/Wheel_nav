"""Real C++ mux process with synthetic publishers in domain 198; no chassis."""
import json
import os
import signal
import subprocess
import tempfile
import time


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='198', 'isolated domain 198 required'
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    rclpy.init();n=Node('velocity_mux_fixture')
    received=[];states=[]
    n.create_subscription(Twist,'/r680_base/cmd_vel',lambda m:received.append((m.linear.x,m.angular.z)),10)
    n.create_subscription(String,'/r680_base/control_state',lambda m:states.append(json.loads(m.data)),10)
    manual=n.create_publisher(Twist,'/r680_manual/cmd_vel',1)
    nav=n.create_publisher(Twist,'/r680_nav/chassis_cmd_vel',1)
    people=n.create_publisher(String,'/r680/person_follow/command_candidate',1)
    pad=n.create_publisher(String,'/gamepad_status',1)
    wheel=n.create_publisher(Odometry,'/wheel/odom',1)

    def pump(seconds, person=None, navigation=None, hand=None, pad_status='DISCONNECTED', old=False):
        end=time.monotonic()+seconds;next_at=0.
        while time.monotonic()<end:
            if time.monotonic()>=next_at:
                next_at=time.monotonic()+.03
                stamp=n.get_clock().now().nanoseconds
                pad.publish(String(data=pad_status))
                w=Odometry();w.header.stamp=n.get_clock().now().to_msg();wheel.publish(w)
                if person is not None:
                    people.publish(String(data=json.dumps(dict(schema='wla-person-candidate/v1',
                        stamp_ns=stamp-(int(1e9) if old else 0),linear=person,angular=0.))))
                for pub,value in [(nav,navigation),(manual,hand)]:
                    if value is not None:
                        twist=Twist();twist.linear.x=value;pub.publish(twist)
            rclpy.spin_once(n,timeout_sec=.003)

    def estop(value, expected=True, **inputs):
        client=n.create_client(SetBool,'/r680_base/set_estop')
        assert client.wait_for_service(timeout_sec=2.)
        future=client.call_async(SetBool.Request(data=value))
        end=time.monotonic()+2.
        while not future.done() and time.monotonic()<end:pump(.03,**inputs)
        assert future.done() and future.result().success is expected
        n.destroy_client(client)

    with tempfile.TemporaryFile(mode='w+') as log:
        process=subprocess.Popen(['ros2','run','wla_manual_control','command_mux'],
            stdout=log,stderr=log,start_new_session=True)
        try:
            pump(1.5,person=.65)
            assert received[-1]==(.65,0.),'PERSON without status, source selection or AUTO'
            assert states[-1]['velocity_passthrough'] and not states[-1]['mode_required']
            pump(.3,person=.4,navigation=.1)
            assert received[-1]==(.4,0.)
            pump(.3,person=0.,navigation=.1)
            assert received[-1]==(.1,0.),'zero releases PERSON priority'
            pump(.3)
            assert received[-1]==(0.,0.),'all commands expired'
            pump(.3,person=.4,old=True)
            assert received[-1]==(0.,0.),'source timestamp expiry'
            pump(.3,person=float('nan'))
            assert received[-1]==(0.,0.),'invalid values'
            pump(.3,person=.4,hand=.08,pad_status='ACTIVE')
            assert received[-1]==(.08,0.),'manual priority'
            pump(.3,person=.4,hand=0.,pad_status='ACTIVE')
            assert received[-1]==(.4,0.),'fresh follower resumes without a mode switch'
            estop(True)
            pump(.3,person=.4,navigation=.2)
            assert received[-1]==(0.,0.)
            pump(.3,pad_status='ESTOP')
            estop(False,expected=False,pad_status='ESTOP')
            pump(.3,pad_status='ACTIVE',hand=0.)
            estop(False,pad_status='ACTIVE',hand=0.)
            pump(.1)
            assert received[-1]==(0.,0.),'no cached replay after reset'
            pump(.3,person=4.)
            assert received[-1]==(1.,0.),'configured forward clamp'
            pump(.3,person=-4.)
            assert received[-1]==(-.3,0.),'reverse clamp'
            pump(.3)
            assert received[-1]==(0.,0.)
            owners=n.get_publishers_info_by_topic('/r680_base/cmd_vel')
            assert len(owners)==1 and owners[0].node_name=='r680_command_mux'
            assert not any('wheeltec' in name for name in n.get_node_names())
            print('PASS: no source/AUTO/status gate; priority, zero release, stale/invalid stop, manual resume, estop, clamp; one mock-domain publisher')
        finally:
            os.killpg(process.pid,signal.SIGINT);process.wait(timeout=10)
            if process.returncode not in (0,130):
                log.seek(0);print(log.read());raise AssertionError('mux shutdown failed')
    n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
