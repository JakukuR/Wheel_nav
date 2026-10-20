"""Isolated adapter integration; all commands remain previews in domain 198."""
import json
import os
import time
import numpy as np
import pytest
pytestmark=pytest.mark.skipif(os.environ.get('RUN_SIM_ROS')!='1',reason='isolated ROS opt-in')


def test_rebind_keeps_request_and_stops_on_ambiguous_candidates():
    assert os.environ.get('ROS_DOMAIN_ID')=='198'
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster
    from wla_person_follow.sim_node import SimulationFollow
    rclpy.init()
    f=SimulationFollow(parameter_overrides=[Parameter('enable_motion',value=False),Parameter('reid_enabled',value=True),
        Parameter('reid_model_id',value='test'),Parameter('local_assumed_free_radius_m',value=.5)])
    fixture=Node('reid_fixture');ex=SingleThreadedExecutor();ex.add_node(f);ex.add_node(fixture)
    broadcaster=StaticTransformBroadcaster(fixture)
    tr=TransformStamped();tr.header.frame_id='r680_mapping_floor';tr.child_frame_id='test_optical'
    tr.header.stamp=fixture.get_clock().now().to_msg();tr.transform.translation.z=.5
    tr.transform.rotation.x=-.5;tr.transform.rotation.y=.5;tr.transform.rotation.z=-.5;tr.transform.rotation.w=.5
    broadcaster.sendTransform(tr)
    depth_pub=fixture.create_publisher(Image,f.p['depth_topic'],qos_profile_sensor_data)
    info_pub=fixture.create_publisher(CameraInfo,f.p['camera_info_topic'],qos_profile_sensor_data)
    wheel_pub=fixture.create_publisher(Odometry,f.p['wheel_topic'],qos_profile_sensor_data)
    relative_pub=fixture.create_publisher(String,f.p['relative_topic'],1)
    feature_pub=fixture.create_publisher(String,'/r680/person_follow/reid_features',1)
    seen=[];requests=[]
    fixture.create_subscription(String,'/r680/person_follow/reid_request',lambda m:requests.append(json.loads(m.data)),1)
    begin=time.monotonic();next_at=0.;next_feature=0.;started=None;captured=[]
    try:
        while time.monotonic()-begin<10.:
            now=time.monotonic();phase=0 if started is None else now-started
            if now>=next_at:
                next_at=now+.04;ns=fixture.get_clock().now().nanoseconds;stamp=fixture.get_clock().now().to_msg()
                ids=['s:1:1'] if phase<1.5 else (['s:1:2','s:1:3'] if phase<3.5 else ['s:1:2'])
                objects=[dict(id=i,label='person',detected=True,yaw=0.,distance=1.5) for i in ids]
                relative_pub.publish(String(data=json.dumps(dict(schema='wla-relative-detections/v1',stamp_ns=ns,session='s',epoch=1,enabled=True,base_frame='r680_mapping_floor',objects=objects))))
                info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='test_optical';info.width=64;info.height=48
                info.k=[60.,0.,32.,0.,60.,24.,0.,0.,1.]
                depth=Image();depth.header=info.header;depth.width=64;depth.height=48;depth.encoding='16UC1';depth.step=128
                depth.data=(np.ones((48,64),dtype='<u2')*4000).tobytes()
                wheel=Odometry();wheel.header.stamp=stamp;wheel.child_frame_id='r680_mapping_floor'
                info_pub.publish(info);depth_pub.publish(depth);wheel_pub.publish(wheel)
                if started is not None and now>=next_feature:
                    next_feature=now+.20
                    feature_pub.publish(String(data=json.dumps(dict(schema='wla-reid-features/v1',stamp_ns=ns,request_id='test-task',
                        session='s',epoch=1,model_id='test',base_frame='r680_mapping_floor',objects=[dict(id=i,feature=[1.]+[0.]*511,yaw=0.,distance=1.5) for i in ids]))))
            ex.spin_once(timeout_sec=.003)
            with f.guard:
                if started is None and now-begin>.5 and 's:1:1' in f.core.objects:
                    ok,_=f.core.command('start','test-task','follow','s:1:1',1.5,now)
                    if ok:started=now
                if started is not None:f.core.command('heartbeat','test-task',lease_duration_s=1.5,now=now)
                if f.map_result and f.map_result.get('reid',{}).get('state'):
                    r=f.map_result;captured.append((phase,r['reid'].copy(),r.get('resolved_target_id')))
            if phase>5 and any(r[1].get('resolved_id')=='s:1:2' and r[1].get('state')=='tracking' for r in captured):break
        assert any(r[1]['state']=='tracking' and r[2]=='s:1:1' for r in captured)
        assert any(r[1]['state']=='ambiguous' and r[2] is None for r in captured)
        assert any(r[1]['state']=='tracking' and r[2]=='s:1:2' for r in captured)
        assert f.core.target_id=='s:1:1' and f.core.request_id=='test-task'
        assert any(r['allowed_ids']==['s:1:1'] for r in requests)
        assert f.direct_pub is None and f.count_publishers('/r680_base/cmd_vel')==0
    finally:
        f.close();ex.remove_node(f);ex.remove_node(fixture);f.destroy_node();fixture.destroy_node();ex.shutdown();rclpy.try_shutdown()
