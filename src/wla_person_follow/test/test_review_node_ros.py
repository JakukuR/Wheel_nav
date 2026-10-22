"""Actual node compute, callbacks, worker and output; synthetic sensors only."""
import json
import os
import time
import numpy as np
import pytest
pytestmark=pytest.mark.skipif(os.environ.get('RUN_SIM_ROS')!='1',reason='isolated ROS opt-in')

@pytest.mark.parametrize('break_kind',['jump','missing'])
def test_geometry_loss_and_delayed_reacquisition_node(break_kind):
    assert os.environ.get('ROS_DOMAIN_ID')=='198'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
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
    from wla_person_follow.follow_config import NEW_PRESET_FLAGS
    rclpy.init()
    options=dict(enable_motion=False,follow_preset='first_batch_v2',reid_enabled=True,
                 reid_model_id='test',local_assumed_free_radius_m=.5,**dict.fromkeys(NEW_PRESET_FLAGS,True))
    f=SimulationFollow(parameter_overrides=[Parameter(k,value=v) for k,v in options.items()])
    fixture=Node('geometry_reid_regression');ex=SingleThreadedExecutor();ex.add_node(f);ex.add_node(fixture)
    broadcaster=StaticTransformBroadcaster(fixture)
    tr=TransformStamped();tr.header.frame_id=f.p['base_frame'];tr.child_frame_id='regression_optical'
    tr.transform.translation.z=.5
    tr.transform.rotation.x=-.5;tr.transform.rotation.y=.5;tr.transform.rotation.z=-.5;tr.transform.rotation.w=.5
    broadcaster.sendTransform(tr)
    dp=fixture.create_publisher(Image,f.p['depth_topic'],qos_profile_sensor_data)
    ip=fixture.create_publisher(CameraInfo,f.p['camera_info_topic'],qos_profile_sensor_data)
    wp=fixture.create_publisher(Odometry,f.p['wheel_topic'],qos_profile_sensor_data)
    rp=fixture.create_publisher(String,f.p['relative_topic'],1)
    fp=fixture.create_publisher(String,'/r680/person_follow/reid_features',1)
    statuses=[]
    fixture.create_subscription(String,'/r680/person_follow/status',lambda m:statuses.append(json.loads(m.data)),20)
    started=False;phase='seed';phase_at=time.monotonic();next_input=next_feature=0.
    deadline=time.monotonic()+16.;captured=[];last_result=None
    try:
        while time.monotonic()<deadline:
            now=time.monotonic();ns=f.now_ns()
            if now>=next_input:
                next_input=now+.04
                if phase=='seed':ids=['s:1:1'];distance=1.72
                elif phase=='break':ids=['s:1:1'] if break_kind=='jump' else [];distance=5.
                else:ids=['s:1:2'];distance=1.72
                rp.publish(String(data=json.dumps(dict(schema='wla-relative-detections/v1',stamp_ns=ns,session='s',epoch=1,enabled=True,
                    base_frame=f.p['base_frame'],objects=[dict(id=i,label='person',detected=True,yaw=0.,distance=distance) for i in ids]))))
                info=CameraInfo();info.header.stamp=fixture.get_clock().now().to_msg();info.header.frame_id=tr.child_frame_id
                info.width=64;info.height=48;info.k=[60.,0.,32.,0.,60.,24.,0.,0.,1.]
                depth=Image();depth.header=info.header;depth.width=64;depth.height=48;depth.encoding='16UC1';depth.step=128
                depth.data=np.full((48,64),4000,dtype='<u2').tobytes()
                wheel=Odometry();wheel.header.stamp=info.header.stamp;wheel.child_frame_id=f.p['base_frame']
                ip.publish(info);dp.publish(depth);wp.publish(wheel)
                if started and phase!='break' and now>=next_feature:
                    next_feature=now+.2
                    # Acquisition is deliberately older than the previous control tick.
                    feature_ns=ns-int(.18e9) if phase=='recover' else ns
                    fp.publish(String(data=json.dumps(dict(schema='wla-reid-features/v1',stamp_ns=feature_ns,request_id='regression-task',
                        session='s',epoch=1,model_id='test',base_frame=f.p['base_frame'],
                        objects=[dict(id=i,feature=[1.]+[0.]*511,yaw=0.,distance=distance) for i in ids]))))
            ex.spin_once(timeout_sec=.003)
            with f.guard:
                if not started and 's:1:1' in f.core.objects:
                    started=f.core.command('start','regression-task','follow','s:1:1',1.5,now)[0]
                if started:f.core.command('heartbeat','regression-task',lease_duration_s=1.5,now=now)
                result=f.map_result
                if result is not None and result is not last_result:
                    last_result=result;captured.append((phase,result))
                    if phase=='seed' and (result.get('identity_supervision') or {}).get('completed_enrollment'):
                        phase='break';phase_at=now
                    elif phase=='break' and now-phase_at>1.3:
                        phase='recover';phase_at=now
                    elif phase=='recover' and result.get('resolved_target_id')=='s:1:2':break
        broken=[r for p,r in captured if p=='break' and (r.get('geometry_binding') or {}).get('state')=='lost']
        assert broken,'No lost policy reached: '+str(f.reason)
        assert all(r['resolved_target_id'] is None for r in broken)
        assert all(r['command'][0]==0. and r['candidates'].get('forward_total',0)==0 for r in broken)
        assert any((s.get('geometry_binding') or {}).get('state')=='lost' and s['linear_mps']==0. for s in statuses)
        recovered=[r for p,r in captured if p=='recover' and r.get('resolved_target_id')=='s:1:2']
        assert recovered,'Delayed real ReID frames never recovered: '+str(f.reason)
        assert recovered[-1]['identity_supervision']['reacquired']==1
        assert f.direct_pub is None and not f.p['enable_motion']
        assert f.count_publishers('/r680_base/cmd_vel')==0
    finally:
        f.close();ex.remove_node(f);ex.remove_node(fixture);f.destroy_node();fixture.destroy_node();ex.shutdown();rclpy.try_shutdown()
