"""Source-time RGB-D/object integration, no actuator or global navigation."""
import json,os,time
import numpy as np
import pytest

@pytest.mark.skipif(os.environ.get('RUN_RELATIVE_ROS')!='1',reason='isolated opt-in required')
def test_object_ros_source_join_preview_and_agent_endpoint():
    assert os.environ.get('ROS_DOMAIN_ID')=='198'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo,Image
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from wla_object_follow.node import ObjectFollow
    from wla_behavior_interfaces.srv import ObjectCommand
    rclpy.init();node=ObjectFollow();fixture=Node('object_fixture')
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(fixture)
    states=[];candidates=[]
    fixture.create_subscription(String,'/r680/object_follow/status',lambda m:states.append(json.loads(m.data)),10)
    fixture.create_subscription(String,'/r680/object_follow/command_candidate',lambda m:candidates.append(json.loads(m.data)),10)
    relative=fixture.create_publisher(String,'/r680/perception/relative_objects',1)
    raw=fixture.create_publisher(String,'/r680/perception/detections',1)
    depth_pub=fixture.create_publisher(Image,'/r680/d455/aligned_depth_to_color/image_raw',qos_profile_sensor_data)
    info_pub=fixture.create_publisher(CameraInfo,'/r680/d455/color/camera_info',qos_profile_sensor_data)
    wheel=fixture.create_publisher(Odometry,'/wheel/odom',qos_profile_sensor_data)
    tf=StaticTransformBroadcaster(fixture);transform=TransformStamped()
    transform.header.frame_id='r680_mapping_floor';transform.child_frame_id='d455_color_optical_frame'
    transform.transform.translation.z=.3
    q=transform.transform.rotation;q.x=-.5;q.y=.5;q.z=-.5;q.w=.5
    tf.sendTransform(transform)
    client=fixture.create_client(ObjectCommand,'/wla_agent/object_command')
    native=fixture.create_client(ObjectCommand,'/r680/object_follow/command')
    last_batch=None

    def pump(duration,kind='normal',distance=.8,observed=True):
        nonlocal last_batch
        deadline=time.monotonic()+duration;next_frame=0.
        while time.monotonic()<deadline:
            if time.monotonic()>=next_frame:
                next_frame=time.monotonic()+.04
                ns=fixture.get_clock().now().nanoseconds
                stamp=fixture.get_clock().now().to_msg()
                info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='d455_color_optical_frame'
                info.width,info.height=64,48;info.k=[60.,0.,32.,0.,60.,24.,0.,0.,1.]
                depth=Image();depth.header=info.header;depth.width,depth.height=64,48
                depth.encoding='16UC1';depth.step=128
                pixels=np.full((48,64),2000,dtype='<u2');pixels[8:32,25:39]=int(distance*1000)
                depth.data=pixels.tobytes();info_pub.publish(info);depth_pub.publish(depth)
                odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='wheel_odom'
                odom.child_frame_id='r680_mapping_floor';wheel.publish(odom)
                relative.publish(String(data=json.dumps(dict(schema='wla-relative-detections/v1',
                    stamp_ns=ns,session='object-test',epoch=0,enabled=True,base_frame='r680_mapping_floor',
                    objects=[dict(detected=True,label='bottle',id='object-test:0:7',yaw=0.,distance=distance)] if observed else []))))
                batch=dict(schema='wla-detections/v1',stamp_ns=ns,session='object-test',epoch=0,
                    frame_id='d455_color_optical_frame',width=64,height=48,
                    objects=[dict(track_id=7,label='bottle',confidence=.9,bbox_xyxy=[25.,8.,39.,32.])] if observed else [])
                if kind=='wrong_frame':batch['frame_id']='wrong_camera'
                if kind=='wrong_id':batch['objects'][0]['track_id']=8
                if kind=='wrong_epoch':batch['epoch']=1
                if kind=='replay':batch=last_batch
                elif kind!='none':last_batch=batch
                if kind!='none':raw.publish(String(data=json.dumps(batch)))
            executor.spin_once(timeout_sec=.002);time.sleep(.001)

    def command(kind,request_id='object-job',mode='follow_object',endpoint=client):
        req=ObjectCommand.Request();req.command=kind;req.request_id=request_id
        req.mode=mode;req.target_id='object-test:0:7';req.lease_duration_s=2.
        future=endpoint.call_async(req);deadline=time.monotonic()+2.
        while not future.done() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.005)
        assert future.done();return future.result()

    try:
        until=time.monotonic()+3.
        while time.monotonic()<until:
            pump(.1)
            if node.safety.valid and node.scene is not None:break
        assert node.safety.valid,node.safety_reason
        assert client.wait_for_service(timeout_sec=1.) and native.wait_for_service(timeout_sec=1.)
        result=command('start');assert result.accepted,result.reason
        pump(.3)
        assert any(s['linear_mps']>.05 for s in states),states[-3:]
        assert node.core.surface_gap==pytest.approx(.56,abs=.01)
        assert candidates and all(c['linear']==c['angular']==0. for c in candidates)
        assert fixture.count_publishers('/r680_base/cmd_vel')==0
        assert fixture.count_publishers('/r680_nav/chassis_cmd_vel')==0
        for kind in ['wrong_frame','wrong_id','replay','none']:
            assert command('heartbeat').accepted
            pump(.35,kind=kind)
            assert states[-1]['linear_mps']==states[-1]['angular_rps']==0.,(kind,states[-1])
            assert not states[-1]['object_surface_fresh'],(kind,states[-1])
            pump(.15)
            assert node.core.geometry is not None,kind
        assert command('heartbeat').accepted
        pump(.15,observed=False)
        assert states[-1]['linear_mps']==0. and not states[-1]['object_surface_fresh']
        pump(.1,distance=.39)
        assert command('start','approach-job','approach_object',native).accepted
        pump(.5,distance=.39)
        assert node.core.state=='arrived',states[-3:]
        assert not node.core.active
        subscribers=fixture.get_subscriber_names_and_types_by_node(node.get_name(),node.get_namespace())
        assert not any(name in ('/map','/odom') for name,_ in subscribers)
    finally:
        node.close();assert not node.worker.is_alive()
        executor.remove_node(node);executor.remove_node(fixture)
        node.destroy_node();fixture.destroy_node();executor.shutdown();rclpy.try_shutdown()
