"""Isolated OBJECT mux handshake; output remapped, no chassis driver."""
import json,os,time,signal,subprocess,tempfile
import pytest

@pytest.mark.skipif(os.environ.get('RUN_RELATIVE_ROS')!='1',reason='isolated opt-in required')
def test_object_source_ros_handshake_revoke_and_timeout():
    assert os.environ.get('ROS_DOMAIN_ID')=='198'
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    rclpy.init();fixture=Node('object_mux_fixture');outputs=[];states=[]
    fixture.create_subscription(Twist,'/object_test/cmd',lambda m:outputs.append((m.linear.x,m.angular.z)),10)
    fixture.create_subscription(String,'/r680_base/control_state',lambda m:states.append(json.loads(m.data)),10)
    wheel=fixture.create_publisher(Odometry,'/wheel/odom',10)
    pubs={}
    for source in ['object','person']:
        pubs[source]=[fixture.create_publisher(String,'/r680/'+source+'_follow/'+suffix,1) for suffix in ['status','command_candidate']]
    nav=fixture.create_publisher(Twist,'/r680_nav/chassis_cmd_vel',1)
    select=fixture.create_client(SetBool,'/r680_base/set_object_source')
    automatic=fixture.create_client(SetBool,'/r680_base/set_automatic')
    request='object-a'
    def feed(duration,status=True,candidate=True,wrong_session=False):
        until=time.monotonic()+duration
        while time.monotonic()<until:
            odom=Odometry();odom.header.stamp=fixture.get_clock().now().to_msg();wheel.publish(odom)
            command=Twist();command.linear.x=.19;nav.publish(command)
            ns=fixture.get_clock().now().nanoseconds
            authorized=bool(states and states[-1]['mode']=='AUTO' and states[-1]['automatic_source']=='OBJECT')
            for source in ['object','person']:
                if status or source=='person':
                    pubs[source][0].publish(String(data=json.dumps(dict(schema='wla-'+source+'-status/v1',stamp_ns=ns,
                        request_id=request,follow_session=source,enabled=True,candidate_only=False,
                        motion_permitted=authorized))))
                if candidate or source=='person':
                    pubs[source][1].publish(String(data=json.dumps(dict(schema='wla-'+source+'-candidate/v1',stamp_ns=ns,
                        request_id=request,follow_session='retired' if wrong_session else source,
                        linear=(.12 if source=='object' else .18) if authorized else 0.,angular=0.))))
            for _ in range(8):rclpy.spin_once(fixture,timeout_sec=.001)
            time.sleep(.01)
    def call(client,value,expected=True):
        assert client.wait_for_service(timeout_sec=2.)
        req=SetBool.Request();req.data=value;future=client.call_async(req)
        until=time.monotonic()+2.
        while not future.done() and time.monotonic()<until:feed(.02)
        assert future.done() and future.result().success is expected
    with tempfile.TemporaryFile(mode='w+') as log:
        process=subprocess.Popen(['ros2','run','wla_manual_control','command_mux.py','--ros-args',
            '-r','/r680_base/cmd_vel:=/object_test/cmd'],stdout=log,stderr=log,start_new_session=True)
        try:
            ready_until=time.monotonic()+5.
            while time.monotonic()<ready_until and not outputs:
                assert process.poll() is None,'mux exited during startup'
                feed(.05)
            assert outputs and all(v==(0.,0.) for v in outputs)
            call(select,True);feed(.2)
            assert states[-1]['automatic_source']=='OBJECT' and states[-1]['mode']=='MANUAL'
            call(automatic,True);feed(.3);assert outputs[-1]==(.12,0.)
            feed(.2,wrong_session=True);assert outputs[-1]==(0.,0.)
            feed(.2);assert outputs[-1]==(.12,0.)
            feed(.25,candidate=False);assert outputs[-1]==(0.,0.)
            feed(.3,status=False);assert outputs[-1]==(0.,0.) and states[-1]['mode']=='MANUAL'
            generation=states[-1]['object_revoke_generation'];assert generation>0
            feed(.2);call(automatic,True,expected=False)
            request='object-b';feed(.2);call(automatic,True);feed(.3);assert outputs[-1]==(.12,0.)
            call(select,False);feed(.2)
            assert outputs[-1]==(0.,0.) and states[-1]['automatic_source']=='NAV'
            assert states[-1]['object_revoke_generation']>generation
            assert fixture.count_publishers('/r680_base/cmd_vel')==0
            assert not any('wheeltec' in name for name in fixture.get_node_names())
        finally:
            os.killpg(process.pid,signal.SIGINT);process.wait(timeout=10)
            log.seek(0);details=log.read()
            fixture.destroy_node();rclpy.try_shutdown()
            assert process.returncode in (0,130),details
