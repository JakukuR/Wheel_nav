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
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from ament_index_python.packages import get_package_prefix
    from wla_behavior_interfaces.srv import PersonCommand
    from wla_r680_navigation_py.target_masks import encode_mask

    executable = Path(get_package_prefix('wla_follow_transport'))/'lib/wla_follow_transport/follow_transport'
    log = (tmp_path/'transport.log').open('w')
    options = dict(prefix='/hybrid_test', enable_motion=True, output_mode='mux', reid_enabled=False,
                   depth_topic='/hybrid_test/depth', camera_info_topic='/hybrid_test/info',
                   wheel_topic='/hybrid_test/wheel', relative_topic='/hybrid_test/relative',
                   detection_status_topic='/r680_nav/perception/status', local_assumed_free_radius_m=.5,
                   depth_stride=2)
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
    pc=node.create_publisher(String,'/r680_base/control_state',5)
    pm=node.create_publisher(String,'/r680/perception/target_mask',1)
    states=[];candidates=[];clouds=[]
    node.create_subscription(String,'/hybrid_test/status',lambda m:states.append(json.loads(m.data)),30)
    node.create_subscription(String,'/hybrid_test/command_candidate',lambda m:candidates.append((time.monotonic(),json.loads(m.data))),30)
    node.create_subscription(PointCloud2,'/hybrid_test/depth_obstacles',clouds.append,qos_profile_sensor_data)
    command=node.create_client(PersonCommand,'/hybrid_test/command')
    estop=node.create_client(SetBool,'/hybrid_test/set_estop')
    mask=np.zeros((48,64),bool);mask[8:42,18:45]=True;bits=encode_mask(mask)
    # Wider synthetic FOV supplies observed free-space rays around the object's
    # conservative footprint. The original narrow scene remains a hold test.
    focal=float(getattr(request,'param',60.))

    class Fixture:
        next_at=0.;request='';detector_enabled=True;source='PERSON';revoke=0;wheels=True;depths=True
        task_mode='0';target_label='person'
        def pump(self, duration):
            until=time.monotonic()+duration
            while time.monotonic()<until:
                assert proc.poll() is None, (tmp_path/'transport.log').read_text()
                if time.monotonic()>=self.next_at:
                    self.next_at=time.monotonic()+.02
                    stamp=node.get_clock().now().to_msg();ns=stamp.sec*10**9+stamp.nanosec
                    info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='hybrid_optical'
                    info.width=64;info.height=48;info.k=[focal,0.,32.,0.,focal,24.,0.,0.,1.]
                    depth=Image();depth.header=info.header;depth.width=64;depth.height=48;depth.encoding='16UC1';depth.step=128
                    values=np.full((48,64),2000,dtype='<u2');values[mask]=1400;depth.data=values.tobytes()
                    wheel=Odometry();wheel.header.stamp=stamp;wheel.child_frame_id='r680_mapping_floor'
                    if self.wheels:pw.publish(wheel)
                    if self.depths:pi.publish(info);pd.publish(depth)
                    envelope=dict(stamp_ns=ns,session='hybrid',epoch=0,enabled=self.detector_enabled)
                    pr.publish(String(data=json.dumps(dict(envelope,schema='wla-relative-detections/v1',base_frame='r680_mapping_floor',
                        objects=[dict(detected=True,label=self.target_label,id='hybrid:0:1',yaw=0.,distance=1.4)]))))
                    ps.publish(String(data=json.dumps(dict(envelope,schema='wla-relative-perception-status/v1'))))
                    pc.publish(String(data=json.dumps(dict(stamp_ns=ns,control_session='hybrid-control',mode='AUTO',automatic_source=self.source,
                        source_generation=self.revoke,person_revoke_generation=self.revoke,velocity_passthrough=False,estop_latched=False))))
                    revision=states[-1].get('geometry_binding') if states else None
                    pm.publish(String(data=json.dumps(dict(schema='wla-target-mask/v1',stamp_ns=ns,depth_stamp_ns=ns,
                        session='hybrid',epoch=0,frame_id='hybrid_optical',width=64,height=48,target_id='hybrid:0:1',
                        requested_target_id='hybrid:0:1',target_revision=revision['revision'] if revision else 0,
                        target_present=True,request_id=self.request,bits=bits,bbox_xyxy=[16.,6.,47.,44.]))))
                executor.spin_once(timeout_sec=.001)

        def call(self, operation, request='', wait=True):
            if operation=='start':self.request=request
            future=command.call_async(PersonCommand.Request(command=operation,request_id=request,mode=self.task_mode,target_id='hybrid:0:1',lease_duration_s=1.))
            if not wait:return future
            until=time.monotonic()+2.
            while not future.done() and time.monotonic()<until:self.pump(.01)
            assert future.done(), (states[-1] if states else 'no status')
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
    try:
        until=time.monotonic()+8.
        while time.monotonic()<until:
            f.pump(.05)
            if states and states[-1].get('numeric_backend') and clouds:break
        assert states and clouds, ((states[-1] if states else None),(tmp_path/'transport.log').read_text())
        assert states[-1]['transport_backend']=='cpp_ros_python_compute'
        f.worker=states[-1]['transport_worker_pid']
        yield f
    finally:
        # Preserve complete planner/status evidence even when pytest truncates an assertion.
        (tmp_path/'status-history.json').write_text(json.dumps(states,ensure_ascii=False,indent=2))
        (tmp_path/'candidate-history.json').write_text(json.dumps(candidates,ensure_ascii=False,indent=2))
        if states and states[-1].get('transport_worker_pid'):
            try:os.kill(states[-1]['transport_worker_pid'],signal.SIGCONT)
            except ProcessLookupError:pass
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=2)
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


def test_object_approach_holds_without_connected_free_route(hybrid):
    f=hybrid;f.task_mode='2';f.target_label='chair';f.pump(.1)
    f.start_task('object-hold')
    held=[s for s in f.states if s.get('motion_permitted') and s.get('reason')=='object_goal_unavailable']
    assert held, f.states[-1]
    assert all(s['approach_goal'] is None and s['target_mask']['surface_extracted'] and
               s['target_mask']['reason']=='object_cloud_preserved' and
               s['diagnostics']['compute_error'] is None for s in held)
    assert all(c['linear']==0 and c['angular']==0 for _,c in f.candidates)
    assert f.call('stop','object-hold').accepted
