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
    follower=SimulationFollow(parameter_overrides=[Parameter('output_mode',value='direct'),Parameter('direct_topic',value=direct),Parameter('enable_motion',value=True)])
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
            assert c.command("start","task",0,'fixture:0:1',1.,now)[0]
        c.expires_at=now+.3
        follower.motion.update(t,(0.,0.,0.),t);follower.wheel_received=now
        follower._tracking_update_locked()
        snapshot=follower.tracker.snapshot(t,np.zeros(3))
        follower.result=dict(policy_snapshot=follower._tracking_policy(snapshot),geometry_binding=snapshot,key=follower.key(),stamp_ns=ns,valid_until=now+.15,command=np.array(command),
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
    from sensor_msgs.msg import CameraInfo,Image,PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from wla_person_follow.sim_node import SimulationFollow
    rclpy.init();f=SimulationFollow(parameter_overrides=[Parameter('depth_stride',value=2),Parameter('startup_clear_radius_m',value=0.),Parameter('local_assumed_free_radius_m',value=.5)])
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
    clouds=[]
    fixture.create_subscription(PointCloud2,f.p['prefix']+'/depth_obstacles',clouds.append,qos_profile_sensor_data)
    try:
        end=time.monotonic()+3.;next_at=0.;captured=None
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
            with f.guard:
                # Idle observations legitimately clear the next solver result;
                # inspect a fresh result while it exists, not at an arbitrary end tick.
                if f.result is not None and time.monotonic()<f.result['valid_until']:
                    assert 0<=(f.now_ns()-f.result['stamp_ns'])/1e9<=f.p['max_sensor_age_s']
                    captured=f.result
        assert f.direct_pub is None and f.count_publishers('/r680_base/cmd_vel')==0
        assert captured is not None, f.reason
        assert captured['startup_prior'] is None and not captured['startup_prior_consumed']
        assert captured['local_free_prior']['state']=='continuous'
        assert captured['local_free_prior']['radius_m']==.5
        assert captured['local_free_prior']['assumed_cells']>0
        assert clouds and captured['obstacle_points']>0
        pts=read_points_numpy(clouds[-1],field_names=('x','y','z'))
        # Local floor may be below zero: 8cm is only the fallback cutoff.
        from wla_person_follow.sim_runtime import packet_from_depth
        expected=packet_from_depth(np.full((48,64),2.),[60,60,32,24],
            np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]]),[0,0,.5],np.zeros(3),stride=2)
        np.testing.assert_array_equal(pts,expected['obstacle_points'].astype(np.float32))
        assert np.all(pts[:,2]<=.5) and np.all(pts[:,2]>=-.08)
        assert clouds[-1].header.stamp.sec>0
        assert clouds[-1].header.frame_id=='r680_mapping_floor'
        assert captured['grid'].max()==100
        assert np.any(captured['grid']==-1)  # no manufactured full-room free space
    finally:
        f.close();ex.remove_node(f);ex.remove_node(fixture);f.destroy_node();fixture.destroy_node();ex.shutdown();rclpy.try_shutdown()


def test_default_mux_modeless_output_and_estop():
    """Exercise the deployed C++ mux process, with no hardware in domain 198."""
    assert os.environ.get('ROS_DOMAIN_ID') == '198'
    import signal
    import subprocess
    import tempfile
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from wla_person_follow.sim_node import SimulationFollow
    log = tempfile.TemporaryFile(mode='w+')
    mux = subprocess.Popen(['ros2', 'run', 'wla_manual_control', 'command_mux'],
                           stdout=log, stderr=log, start_new_session=True)
    rclpy.init()
    f = SimulationFollow(parameter_overrides=[Parameter('enable_motion', value=True)])
    fixture = Node('sim_mux_fixture')
    ex = SingleThreadedExecutor()
    for node in (f, fixture):ex.add_node(node)
    velocities = [];states = []
    fixture.create_subscription(Twist, '/r680_base/cmd_vel', lambda m:velocities.append(m.linear.x), 10)
    fixture.create_subscription(String, '/r680_base/control_state', lambda m:states.append(json.loads(m.data)), 10)
    wheel = fixture.create_publisher(Odometry, '/wheel/odom', 10)
    feed = False

    def pump(seconds):
        end = time.monotonic()+seconds;next_input = 0.
        while time.monotonic()<end:
            if time.monotonic()>=next_input:
                next_input=time.monotonic()+.03
                msg=Odometry();msg.header.stamp=fixture.get_clock().now().to_msg()
                msg.child_frame_id='r680_mapping_floor';wheel.publish(msg)
                if feed:
                    seed(f);f.core.lease_until=time.monotonic()+1.
            ex.spin_once(timeout_sec=.003)

    def call(name, value):
        client=fixture.create_client(SetBool,name)
        assert client.wait_for_service(timeout_sec=1.)
        future=client.call_async(SetBool.Request(data=value))
        end=time.monotonic()+1.
        while not future.done() and time.monotonic()<end:pump(.02)
        assert future.done() and future.result().success
        fixture.destroy_client(client)

    try:
        end = time.monotonic()+5.
        while not states and time.monotonic()<end:pump(.1)
        assert states, 'mux process did not come up'
        assert f.p['output_mode']=='mux' and f.direct_pub is None
        assert [p.node_name for p in fixture.get_publishers_info_by_topic('/r680_base/cmd_vel')]==['r680_command_mux']
        assert states[-1]['mode']=='AUTO' and states[-1]['automatic_source']=='NAV'
        feed=True;pump(.5)
        assert any(v>0. for v in velocities), 'No source or AUTO service was called'
        assert states[-1]['output_source']=='PERSON'
        feed=False
        call('/r680_base/set_estop',True)
        pump(.3)
        assert velocities[-1]==0. and not f.core.active
        assert not f.core.command('heartbeat','task',lease_duration_s=1.,now=time.monotonic())[0]
    finally:
        f.close()
        for node in (f,fixture):ex.remove_node(node);node.destroy_node()
        ex.shutdown();rclpy.try_shutdown()
        os.killpg(mux.pid, signal.SIGINT);mux.wait(timeout=10)
        log.seek(0);output=log.read();log.close()
        assert mux.returncode in (0, 130), output


def test_status_distinguishes_expired_result_from_planner_clear(graph):
    from std_msgs.msg import String
    fixture,f,pump,_=graph
    statuses=[]
    sub=fixture.create_subscription(String,'/r680/person_follow/status',
        lambda m:statuses.append(json.loads(m.data)),5)
    pump(1.1)
    seed(f)
    with f.guard:
        f.reason='progress_or_clear'
        f.result['valid_until']=time.monotonic()-.001
    pump(.06)
    latest=statuses[-1]
    assert latest['planner_reason']=='progress_or_clear'
    assert latest['safety_reason']=='planner_result_expired'
    assert 'planner_result_expired' in latest['output_blockers']
    assert latest['result_remaining_ms']<0
    assert not latest['motion_permitted'] and latest['linear_mps']==0.
    assert latest['calibration']['status']=='pending_calibration'
    for name in ('binding_jump_radius_m','binding_jump_growth_mps','binding_jump_radius_max_m',
                 'reid_confirm_frames','neighbor_ambiguity_radius_m'):
        assert latest['calibration']['parameters'][name]==f.p[name]
    fixture.destroy_subscription(sub)


def test_result_event_publishes_without_periodic_tick_and_retains_stop_gate(graph):
    _,f,pump,received=graph
    pump(1.1);f.control_timer.cancel();received.clear()
    seed(f)
    f.result_ready.trigger();pump(.025)
    assert received and received[-1][0]>0
    with f.guard:f.result['valid_until']=time.monotonic()-.001
    f.result_ready.trigger();pump(.025)
    assert received[-1]==(0.,0.)


@pytest.mark.parametrize('reid_enabled',[False,True])
@pytest.mark.parametrize('bbox_filter',[False,True])
def test_mask_wire_pairs_depth_and_filters_published_cloud_preview_only(reid_enabled,bbox_filter):
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
    from wla_r680_navigation_py.target_masks import encode_mask
    from wla_person_follow.sim_node import SimulationFollow
    assert os.environ['ROS_DOMAIN_ID']=='198'
    options=dict(depth_stride=2,output_mode='direct',reid_enabled=reid_enabled,reid_model_id='test',
                 target_bbox_filter_enabled=bbox_filter)
    rclpy.init();f=SimulationFollow(parameter_overrides=[Parameter(k,value=v) for k,v in options.items()])
    fixture=Node('mask_depth_fixture');ex=SingleThreadedExecutor();ex.add_node(f);ex.add_node(fixture)
    broadcaster=StaticTransformBroadcaster(fixture)
    tr=TransformStamped();tr.header.frame_id='r680_mapping_floor';tr.child_frame_id='mask_optical'
    tr.transform.translation.z=.5
    tr.transform.rotation.x=-.5;tr.transform.rotation.y=.5;tr.transform.rotation.z=-.5;tr.transform.rotation.w=.5
    broadcaster.sendTransform(tr)
    pd=fixture.create_publisher(Image,f.p['depth_topic'],qos_profile_sensor_data)
    pi=fixture.create_publisher(CameraInfo,f.p['camera_info_topic'],qos_profile_sensor_data)
    pw=fixture.create_publisher(Odometry,f.p['wheel_topic'],qos_profile_sensor_data)
    pr=fixture.create_publisher(String,f.p['relative_topic'],1)
    pm=fixture.create_publisher(String,'/r680/perception/target_mask',1)
    mask=np.zeros((48,64),bool);mask[8:42,18:45]=True;bits=encode_mask(mask)
    try:
        end=time.monotonic()+4.;next_at=0.;captured=None
        while time.monotonic()<end:
            if time.monotonic()>=next_at:
                next_at=time.monotonic()+.04
                stamp=fixture.get_clock().now().to_msg();ns=stamp.sec*10**9+stamp.nanosec
                info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='mask_optical'
                info.width=64;info.height=48;info.k=[60.,0.,32.,0.,60.,24.,0.,0.,1.]
                depth=Image();depth.header=info.header;depth.width=64;depth.height=48
                depth.encoding='16UC1';depth.step=128
                values=np.full((48,64),2000,dtype='<u2');values[mask]=1200;depth.data=values.tobytes()
                wheel=Odometry();wheel.header.stamp=stamp;wheel.child_frame_id='r680_mapping_floor'
                pw.publish(wheel);pi.publish(info);pd.publish(depth)
                pr.publish(String(data=json.dumps(dict(schema='wla-relative-detections/v1',stamp_ns=ns,
                    session='mask',epoch=0,enabled=True,base_frame='r680_mapping_floor',
                    objects=[dict(detected=True,label='person',id='mask:0:1',yaw=0.,distance=1.2)]))))
                with f.guard:
                    if 'mask:0:1' in f.core.objects:
                        if not f.core.active:
                            assert f.core.command('start','mask-task','0','mask:0:1',1.,time.monotonic())[0]
                        else:f.core.command('heartbeat','mask-task',lease_duration_s=1.,now=time.monotonic())
                pm.publish(String(data=json.dumps(dict(schema='wla-target-mask/v1',stamp_ns=ns,depth_stamp_ns=ns,
                    session='mask',epoch=0,frame_id='mask_optical',width=64,height=48,
                    target_id='mask:0:1',requested_target_id='mask:0:1',target_revision=0,
                    target_present=True,request_id='mask-task',bits=bits,
                    bbox_xyxy=[16.,6.,47.,44.]))))
            ex.spin_once(timeout_sec=.002)
            with f.guard:
                if f.map_result and f.map_result.get('target_mask',{}).get('applied'):
                    captured=f.map_result;break
        assert captured is not None
        assert captured['target_mask']['removed_pixels']==(38*31 if bbox_filter else int(mask.sum()))
        assert captured['target_mask']['source']==('target_bbox' if bbox_filter else 'target_segmentation')
        assert captured['target_mask']['target_ghost_points']>0
        assert captured['obstacle_points']>0  # Background remains measured.
        assert f.direct_pub is None and not f.p['enable_motion']
    finally:
        f.close();ex.remove_node(f);ex.remove_node(fixture)
        f.destroy_node();fixture.destroy_node();ex.shutdown();rclpy.try_shutdown()


def test_diagnostic_status_and_independent_recorder(graph,tmp_path):
    import subprocess
    import sys
    _,f,pump,_=graph
    assert os.environ.get('ROS_DOMAIN_ID')=='198'
    proc=subprocess.Popen([sys.executable,'-m','wla_person_follow.status_recorder',
        '--directory',str(tmp_path)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    try:
        pump(1.5);seed(f)
        now=f.now_ns()/1e9
        f.computation_diagnostic=dict(key=f.key(),control_stamp_s=now,
            target_mask=dict(applied=False,reason='no_confirmed_target_mask'),
            front_blockage=dict(evaluated=False,reason='fixture_no_front_check'),
            resolved_target_source=dict(source='fixture',accepted_id=None))
        pump(.6)
    finally:
        proc.terminate();proc.wait(timeout=5)
    records=[]
    for path in tmp_path.glob('*.jsonl'):
        records.extend(json.loads(line) for line in path.read_text().splitlines())
    assert records
    matching=[r for r in records if r.get('target_observation',{}).get('id')=='fixture:0:1']
    assert matching
    record=next(r for r in matching if r.get('diagnostics'))
    assert record['measured_motion']['pose_source'].startswith('LocalMotion')
    assert 'actual_published_command' in record and record['diagnostic_sequence']>0
    assert record['diagnostics']['target_mask']['applied'] is False
    assert record['_recording']['source']=='/r680/person_follow/status'
