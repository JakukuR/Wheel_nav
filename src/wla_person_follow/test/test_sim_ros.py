"""ROS integration in domain 198 with mock command topics; never chassis I/O."""
import json
import os
import time
import numpy as np
import pytest
pytestmark=pytest.mark.skipif(os.environ.get('RUN_SIM_ROS')!='1',reason='isolated ROS opt-in required')


@pytest.fixture
def graph():
    assert os.environ.get('ROS_DOMAIN_ID')=='198'
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    from wla_person_follow.sim_node import SimulationFollow
    rclpy.init()
    fixture=Node('sim_port_fixture')
    direct='/sim_test/base/cmd_vel'
    follower=SimulationFollow(parameter_overrides=[Parameter('direct_topic',value=direct),Parameter('enable_motion',value=True)])
    executor=SingleThreadedExecutor();executor.add_node(fixture);executor.add_node(follower)
    received=[]
    fixture.create_subscription(Twist,direct,lambda m:received.append((m.linear.x,m.angular.z)),10)
    def pump(duration):
        until=time.monotonic()+duration
        while time.monotonic()<until:
            executor.spin_once(timeout_sec=.005)
    try:yield fixture,follower,pump,received
    finally:
        follower.close();executor.remove_node(follower);executor.remove_node(fixture)
        follower.destroy_node();fixture.destroy_node();executor.shutdown();rclpy.try_shutdown()


def seed(follower,command=(.12,.05)):
    """Inject an already-checked solver result to isolate the output/lease gate."""
    now=time.monotonic();ns=follower.now_ns();t=ns/1e9
    with follower.guard:
        c=follower.core
        c.receive(dict(schema='wla-relative-detections/v1',stamp_ns=ns,session='fixture',epoch=0,
            enabled=True,base_frame='r680_mapping_floor',objects=[dict(detected=True,label='person',id='fixture:0:1',yaw=0.,distance=1.5)]),now,ns)
        if not c.active:
            assert c.command('start','task','follow','fixture:0:1',1.,now)[0]
        c.expires_at=now+.3
        follower.motion.update(t,(0.,0.,0.),t);follower.wheel_received=now
        follower.result=dict(key=follower.key(),stamp_ns=ns,valid_until=now+.15,command=np.array(command),
            target_state='visible',recovery_state='follow',wide_offset=None,compute_ms=1.,
            pose=np.zeros(3),path=np.array([[0.,0.],[1.5,0.]]),map_lower=np.array([-2,-2]),resolution=.1,grid=np.zeros((4,4),np.int8))


def test_direct_output_stale_stop_and_conflict(graph):
    from geometry_msgs.msg import Twist
    fixture,f,pump,received=graph
    pump(1.2)
    seed(f);pump(.08)
    assert any(v>0 for v,w in received)
    pump(.30)
    assert received[-1]==(0.,0.)
    other=fixture.create_publisher(Twist,'/sim_test/base/cmd_vel',1)
    pump(.3)
    assert f.conflict_latched and not f.core.active
    before=len(received);pump(.2)
    assert len(received)==before  # no repeated competing zeros after latching
    fixture.destroy_publisher(other)


def test_estop_does_not_resurrect_task(graph):
    from std_srvs.srv import SetBool
    _,f,pump,received=graph;pump(1.1);seed(f);pump(.06)
    req=SetBool.Request();req.data=True;f.on_estop(req,SetBool.Response());pump(.06)
    assert received[-1]==(0.,0.) and not f.core.active
    req.data=False;f.on_estop(req,SetBool.Response())
    assert not f.core.active
    assert not f.core.command('heartbeat','task',lease_duration_s=1.,now=time.monotonic())[0]


def test_real_depth_tf_wheel_adapter_preview_only():
    assert os.environ.get('ROS_DOMAIN_ID')=='198'
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo,Image
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from wla_person_follow.sim_node import SimulationFollow
    rclpy.init();f=SimulationFollow(parameter_overrides=[Parameter('depth_stride',value=2)])
    fixture=Node('sim_depth_fixture');ex=SingleThreadedExecutor();ex.add_node(f);ex.add_node(fixture)
    broadcaster=StaticTransformBroadcaster(fixture)
    tr=TransformStamped();tr.header.frame_id='r680_mapping_floor';tr.child_frame_id='test_optical'
    tr.transform.translation.z=.5
    tr.transform.rotation.x=-.5;tr.transform.rotation.y=.5;tr.transform.rotation.z=-.5;tr.transform.rotation.w=.5
    broadcaster.sendTransform(tr)
    pub_depth=fixture.create_publisher(Image,f.p['depth_topic'],qos_profile_sensor_data)
    pub_info=fixture.create_publisher(CameraInfo,f.p['camera_info_topic'],qos_profile_sensor_data)
    pub_wheel=fixture.create_publisher(Odometry,f.p['wheel_topic'],qos_profile_sensor_data)
    pub_rel=fixture.create_publisher(String,f.p['relative_topic'],1)
    try:
        end=time.monotonic()+3.;next_at=0.;computed=False
        while time.monotonic()<end:
            if time.monotonic()>=next_at:
                next_at=time.monotonic()+.04
                stamp=fixture.get_clock().now().to_msg();ns=fixture.get_clock().now().nanoseconds
                info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='test_optical'
                info.width=64;info.height=48;info.k=[60.,0.,32.,0.,60.,24.,0.,0.,1.]
                depth=Image();depth.header=info.header;depth.width=64;depth.height=48
                depth.encoding='16UC1';depth.step=128;depth.data=(np.ones((48,64),dtype='<u2')*2000).tobytes()
                wheel=Odometry();wheel.header.stamp=stamp;wheel.child_frame_id='r680_mapping_floor'
                pub_wheel.publish(wheel);pub_info.publish(info);pub_depth.publish(depth)
                pub_rel.publish(String(data=json.dumps(dict(schema='wla-relative-detections/v1',stamp_ns=ns,
                    session='depth-fixture',epoch=0,enabled=True,base_frame='r680_mapping_floor',objects=[]))))
            ex.spin_once(timeout_sec=.002)
            with f.guard:computed=computed or f.result is not None
        assert f.direct_pub is None and f.count_publishers('/r680_base/cmd_vel')==0
        assert computed, f.reason
        assert f.result is not None
        assert f.result['grid'].max()==100
        assert np.any(f.result['grid']==-1)  # no manufactured full-room free space
    finally:
        f.close();ex.remove_node(f);ex.remove_node(fixture);f.destroy_node();fixture.destroy_node();ex.shutdown();rclpy.try_shutdown()
