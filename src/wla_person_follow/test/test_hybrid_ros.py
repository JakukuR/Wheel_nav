"""Actual C++/Python IPC and ROS graph, exclusively domain 198, synthetic sensors."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('RUN_SIM_ROS') != '1', reason='isolated ROS opt-in required')


@pytest.fixture
def hybrid(tmp_path,request):
    assert os.environ.get('ROS_DOMAIN_ID') == '198'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped, Twist
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from ament_index_python.packages import get_package_prefix
    from wla_behavior_interfaces.srv import PersonCommand
    from wla_r680_navigation_py.target_masks import encode_mask

    settings=getattr(request,'param',60.)
    settings=settings if isinstance(settings,dict) else dict(focal=settings)
    full_chain=settings.get('full_chain',False)
    executable = Path(get_package_prefix('wla_follow_transport'))/'lib/wla_follow_transport/follow_transport'
    log = (tmp_path/'transport.log').open('w')
    options = dict(prefix='/hybrid_test', enable_motion=True, output_mode='mux', reid_enabled=False,
                   depth_topic='/hybrid_test/depth', camera_info_topic='/hybrid_test/info',
                   wheel_topic='/hybrid_test/wheel', relative_topic='/hybrid_test/relative',
                   detection_status_topic='/r680_nav/perception/status', local_assumed_free_radius_m=.5,
                   depth_stride=2)
    if full_chain:options.update(reid_enabled=True,reid_model_id='test')
    cmd = [str(executable), '--ros-args']
    for key, value in options.items():
        cmd += ['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}']
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = Node('hybrid_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    broadcaster = StaticTransformBroadcaster(node)
    tr = TransformStamped();tr.header.frame_id='r680_mapping_floor';tr.child_frame_id='hybrid_optical'
    tr.transform.translation.z=.5
    tr.transform.rotation.x=-.5;tr.transform.rotation.y=.5;tr.transform.rotation.z=-.5;tr.transform.rotation.w=.5
    broadcaster.sendTransform(tr)
    pd=node.create_publisher(Image,options['depth_topic'],qos_profile_sensor_data)
    pi=node.create_publisher(CameraInfo,options['camera_info_topic'],qos_profile_sensor_data)
    pw=node.create_publisher(Odometry,options['wheel_topic'],qos_profile_sensor_data)
    pr=node.create_publisher(String,options['relative_topic'],1)
    ps=node.create_publisher(String,options['detection_status_topic'],1)
    pc=None if full_chain else node.create_publisher(String,'/r680_base/control_state',5)
    pm=node.create_publisher(String,'/r680/perception/target_mask',1)
    pf=node.create_publisher(String,'/hybrid_test/reid_features',1)
    pad=node.create_publisher(String,'/gamepad_status',1)
    manual=node.create_publisher(Twist,'/r680_manual/cmd_vel',1)
    states=[];candidates=[];clouds=[];outputs=[];controls=[];extra=[];calls=[];checks=[]
    node.create_subscription(String,'/hybrid_test/status',lambda m:states.append(json.loads(m.data)),30)
    node.create_subscription(String,'/hybrid_test/command_candidate',lambda m:candidates.append((time.monotonic(),json.loads(m.data))),30)
    node.create_subscription(PointCloud2,'/hybrid_test/depth_obstacles',clouds.append,qos_profile_sensor_data)
    node.create_subscription(Twist,'/hybrid_test/chassis_cmd_vel',lambda m:outputs.append((time.monotonic(),m.linear.x,m.angular.z)),50)
    node.create_subscription(String,'/r680_base/control_state',lambda m:controls.append(json.loads(m.data)),30)
    command=node.create_client(PersonCommand,'/hybrid_test/agent_command' if full_chain else '/hybrid_test/command')
    estop=node.create_client(SetBool,'/hybrid_test/set_estop')
    mask=np.zeros((48,64),bool);mask[8:42,18:45]=True;bits=encode_mask(mask)
    # Wider synthetic FOV supplies observed free-space rays around the object's
    # conservative footprint. The original narrow scene remains a hold test.
    focal=float(settings.get('focal',60.))

    class Fixture:
        next_at=0.;request='';detector_enabled=True;source='PERSON';revoke=0;wheels=True;depths=True
        task_mode='0';target_label='person';detected_ids=('hybrid:0:1',);target_distance=1.4
        manual_velocity=None;next_feature=0.;expect_transport_exit=False
        def pump(self, duration):
            until=time.monotonic()+duration
            while time.monotonic()<until:
                if not self.expect_transport_exit:assert proc.poll() is None, (tmp_path/'transport.log').read_text()
                for child in extra:assert child.poll() is None, (tmp_path/'transport.log').read_text()
                if time.monotonic()>=self.next_at:
                    self.next_at=time.monotonic()+.02
                    stamp=node.get_clock().now().to_msg();ns=stamp.sec*10**9+stamp.nanosec
                    info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='hybrid_optical'
                    info.width=64;info.height=48;info.k=[focal,0.,32.,0.,focal,24.,0.,0.,1.]
                    depth=Image();depth.header=info.header;depth.width=64;depth.height=48;depth.encoding='16UC1';depth.step=128
                    values=np.full((48,64),2000,dtype='<u2');values[mask]=round(self.target_distance*1000);depth.data=values.tobytes()
                    wheel=Odometry();wheel.header.stamp=stamp;wheel.child_frame_id='r680_mapping_floor'
                    if self.wheels:pw.publish(wheel)
                    if self.depths:pi.publish(info);pd.publish(depth)
                    envelope=dict(stamp_ns=ns,session='hybrid',epoch=0,enabled=self.detector_enabled)
                    pr.publish(String(data=json.dumps(dict(envelope,schema='wla-relative-detections/v1',base_frame='r680_mapping_floor',
                        objects=[dict(detected=True,label=self.target_label,id=ident,yaw=0.,distance=self.target_distance) for ident in self.detected_ids]))))
                    ps.publish(String(data=json.dumps(dict(envelope,schema='wla-relative-perception-status/v1'))))
                    if pc is not None:
                        pc.publish(String(data=json.dumps(dict(stamp_ns=ns,control_session='hybrid-control',mode='AUTO',automatic_source=self.source,
                            source_generation=self.revoke,person_revoke_generation=self.revoke,velocity_passthrough=False,estop_latched=False))))
                    if full_chain:
                        pad.publish(String(data='ACTIVE' if self.manual_velocity is not None else 'DISCONNECTED'))
                        if self.manual_velocity is not None:
                            velocity=Twist();velocity.linear.x=self.manual_velocity;manual.publish(velocity)
                    revision=states[-1].get('geometry_binding') if states else None
                    resolved=revision['resolved_id'] if revision else 'hybrid:0:1'
                    pm.publish(String(data=json.dumps(dict(schema='wla-target-mask/v1',stamp_ns=ns,depth_stamp_ns=ns,
                        session='hybrid',epoch=0,frame_id='hybrid_optical',width=64,height=48,target_id=resolved,
                        requested_target_id=resolved,target_revision=revision['revision'] if revision else 0,
                        target_present=resolved in self.detected_ids,request_id=self.request,bits=bits,bbox_xyxy=[16.,6.,47.,44.]))))
                    if full_chain and time.monotonic()>=self.next_feature:
                        self.next_feature=time.monotonic()+.2
                        pf.publish(String(data=json.dumps(dict(schema='wla-reid-features/v1',stamp_ns=ns,
                            request_id=self.request,session='hybrid',epoch=0,model_id='test',
                            target_revision=revision['revision'] if revision else 0,base_frame='r680_mapping_floor',
                            objects=[dict(id=ident,feature=[1.]+[0.]*511,yaw=0.,distance=self.target_distance) for ident in self.detected_ids]))))
                executor.spin_once(timeout_sec=.001)

        def call(self, operation, request='', wait=True):
            if operation=='start':self.request=request
            record=dict(command=operation,request_id=request,mode=self.task_mode,sent=time.monotonic())
            calls.append(record)
            future=command.call_async(PersonCommand.Request(command=operation,request_id=request,mode=self.task_mode,target_id='hybrid:0:1',lease_duration_s=1.))
            if not wait:return future
            until=time.monotonic()+2.
            while not future.done() and time.monotonic()<until:self.pump(.01)
            assert future.done(), (states[-1] if states else 'no status')
            record.update(received=time.monotonic(),accepted=future.result().accepted,state=future.result().state,reason=future.result().reason)
            return future.result()

        def start_task(self, request):
            assert self.call('start',request).accepted
            for _ in range(5):
                self.pump(.2)
                assert self.call('heartbeat',request).accepted

        def run_task(self, request):
            self.start_task(request)
            assert any(s.get('motion_permitted') for s in states), states[-1]
            assert any(abs(c['linear'])+abs(c['angular'])>0 for _,c in candidates), states[-1]

        def set_estop(self, data):
            start=time.monotonic();future=estop.call_async(SetBool.Request(data=data))
            while not future.done() and time.monotonic()-start<.5:self.pump(.005)
            assert future.done() and future.result().success
            return time.monotonic()-start

    f=Fixture();f.states=states;f.candidates=candidates;f.clouds=clouds;f.proc=proc
    f.outputs=outputs;f.controls=controls;f.node=node;f.extra=extra;f.checks=checks
    try:
        if full_chain:
            mux=Path(get_package_prefix('wla_manual_control'))/'lib/wla_manual_control/command_mux'
            api=Path(get_package_prefix('wla_behavior_bridge'))/'lib/wla_behavior_bridge/agent_person_api'
            extra.append(subprocess.Popen([str(mux),'--ros-args',
                '-r','/r680/person_follow/command_candidate:=/hybrid_test/command_candidate',
                '-r','/r680/person_follow/status:=/hybrid_test/status',
                '-r','/wheel/odom:=/hybrid_test/wheel',
                '-r','/r680_base/cmd_vel:=/hybrid_test/chassis_cmd_vel'],stdout=log,stderr=log))
            extra.append(subprocess.Popen([str(api),'--ros-args',
                '-r','/r680/person_follow/command:=/hybrid_test/command',
                '-r','/wla_agent/person_command:=/hybrid_test/agent_command'],stdout=log,stderr=log))
        until=time.monotonic()+8.
        while time.monotonic()<until:
            f.pump(.05)
            if states and states[-1].get('numeric_backend') and clouds and command.service_is_ready() and (not full_chain or outputs):break
        assert states and clouds, ((states[-1] if states else None),(tmp_path/'transport.log').read_text())
        assert states[-1]['transport_backend']=='cpp_ros_python_compute'
        f.worker=states[-1]['transport_worker_pid']
        assert command.service_is_ready()
        if full_chain:
            assert outputs and controls
            assert [p.node_name for p in node.get_publishers_info_by_topic('/hybrid_test/chassis_cmd_vel')]==['r680_command_mux']
            assert node.count_publishers('/r680_base/cmd_vel')==0
            assert len(node.get_publishers_info_by_topic('/r680_base/control_state'))==1
        yield f
    finally:
        # Preserve complete planner/status evidence even when pytest truncates an assertion.
        (tmp_path/'status-history.json').write_text(json.dumps(states,ensure_ascii=False,indent=2))
        (tmp_path/'candidate-history.json').write_text(json.dumps(candidates,ensure_ascii=False,indent=2))
        (tmp_path/'chain-history.json').write_text(json.dumps(dict(outputs=outputs,controls=controls,calls=calls,checks=checks),ensure_ascii=False,indent=2))
        if states and states[-1].get('transport_worker_pid'):
            try:os.kill(states[-1]['transport_worker_pid'],signal.SIGCONT)
            except ProcessLookupError:pass
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=2)
        # Also clean up the worker if a test killed its C++ parent.
        if states and states[-1].get('transport_worker_pid'):
            try:os.kill(states[-1]['transport_worker_pid'],signal.SIGTERM)
            except ProcessLookupError:pass
        for child in reversed(extra):
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
        executor.remove_node(node);node.destroy_node();executor.shutdown();rclpy.try_shutdown();log.close()


def test_actual_compute_cancel_and_late_start(hybrid):
    f=hybrid;f.run_task('normal')
    assert f.call('stop','normal').accepted
    f.pump(.10)
    assert f.candidates[-1][1]['linear']==0 and f.candidates[-1][1]['angular']==0
    assert not f.call('heartbeat','normal').accepted
    assert f.call('stop','future-task').accepted
    assert not f.call('start','future-task').accepted


def test_hung_worker_expires_and_cpp_estop_stays_responsive(hybrid):
    f=hybrid;f.run_task('hang')
    os.kill(f.worker,signal.SIGSTOP)
    try:
        f.pump(.25)
        assert f.candidates[-1][1]['linear']==0 and f.candidates[-1][1]['angular']==0
        assert f.set_estop(True)<.2
        assert f.set_estop(False)<.2
    finally:os.kill(f.worker,signal.SIGCONT)
    f.pump(.15)
    assert not f.call('heartbeat','hang').accepted
    assert not f.call('start','hang').accepted


def test_authority_revocation_does_not_resume_on_restore(hybrid):
    f=hybrid;f.run_task('authority')
    f.source='NAV';f.revoke+=1;f.pump(.10)
    assert f.candidates[-1][1]['linear']==0 and not f.states[-1]['transport_task_active']
    f.source='PERSON';f.pump(.10)
    assert not f.call('heartbeat','authority').accepted
    assert f.call('stop','authority').state=='cancelled'
    f.pump(.10)
    assert f.states[-1]['state']=='cancelled'


def test_stale_wheel_and_detector_disable(hybrid):
    f=hybrid;f.run_task('sensors')
    f.wheels=False;f.pump(.25)
    assert f.candidates[-1][1]['linear']==0 and f.candidates[-1][1]['angular']==0
    f.wheels=True;f.detector_enabled=False;f.pump(.10)
    assert not f.states[-1]['transport_task_active']
    f.detector_enabled=True;f.pump(.10)
    assert not f.call('heartbeat','sensors').accepted


def test_stop_overtakes_pending_start(hybrid):
    f=hybrid
    os.kill(f.worker,signal.SIGSTOP)
    try:
        future=f.call('start','delayed-start',wait=False)
        f.pump(.04)
        assert not future.done()
        assert f.call('stop','delayed-start').accepted
    finally:os.kill(f.worker,signal.SIGCONT)
    f.pump(.20)
    assert future.done() and not future.result().accepted
    assert not f.states[-1]['transport_task_active']
    assert all(c['linear']==0 and c['angular']==0 for _,c in f.candidates)
    assert not f.call('start','delayed-start').accepted


def test_worker_crash_terminates_transport(hybrid):
    f=hybrid
    os.kill(f.worker,signal.SIGKILL)
    assert f.proc.wait(timeout=3)!=0


@pytest.mark.parametrize('hybrid,mode,label',[(60.,'1','person'),(20.,'2','chair')],indirect=['hybrid'])
def test_approach_modes_use_the_same_transport(hybrid,mode,label):
    f=hybrid;f.task_mode=mode;f.target_label=label;f.pump(.1)
    f.run_task('approach-'+mode)
    assert f.states[-1]['mode']==mode and f.states[-1]['target_label']==label
    if mode=='2':
        assert any(s.get('target_mask',{}).get('reason')=='object_cloud_preserved' for s in f.states if s.get('target_mask'))
        assert any(s.get('approach_goal') for s in f.states)
    assert f.call('stop','approach-'+mode).accepted


def test_object_approach_plans_through_unseen_space_like_person_approach(hybrid):
    # The forward camera never observes the robot's flanks, so only the assumed
    # free disc is known there. Like person approach, unknown cells no longer
    # block the route; the MPC keeps only its near horizon in known free space.
    f=hybrid;f.task_mode='2';f.target_label='chair';f.pump(.1)
    f.start_task('object-unseen')
    planned=[s for s in f.states if s.get('motion_permitted') and s.get('approach_goal')]
    assert planned, f.states[-1]
    assert all(s['target_mask']['surface_extracted'] and s['target_mask']['reason']=='object_cloud_preserved' and
               s['diagnostics']['compute_error'] is None for s in planned)
    assert not any(s.get('reason')=='object_goal_unavailable' for s in planned)
    assert f.call('stop','object-unseen').accepted
