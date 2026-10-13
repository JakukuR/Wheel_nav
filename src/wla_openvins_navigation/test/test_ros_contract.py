#!/usr/bin/env python3
"""Run only in an explicitly isolated ROS domain. No hardware topics outside it."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Image, CameraInfo
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtabmap_msgs.msg import Info
from std_msgs.msg import String, Bool
from tf2_msgs.msg import TFMessage
from wla_openvins_navigation.msg import EstimatorStatus
import yaml


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='86', 'use isolated ROS_DOMAIN_ID=86'
    root=Path(sys.argv[1]).resolve()
    gate_command=[sys.executable,str(root/'scripts/quality_gate.py')]
    rclpy.init();n=Node('openvins_contract_test');statuses=[];odoms=[];tfs=[];permits=[];mapped=[]
    n.create_subscription(EstimatorStatus,'/contract/estimator_status',statuses.append,10)
    n.create_subscription(Odometry,'/contract/odom',odoms.append,qos_profile_sensor_data)
    n.create_subscription(TFMessage,'/tf',tfs.append,10)
    imu=n.create_publisher(Imu,'/contract/imu',qos_profile_sensor_data)
    images=[n.create_publisher(Image,f'/contract/image{i}',qos_profile_sensor_data) for i in range(2)]
    infos=[n.create_publisher(CameraInfo,f'/contract/info{i}',qos_profile_sensor_data) for i in range(2)]
    processes=[]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    with tempfile.TemporaryDirectory(prefix='openvins-contract-') as d:
        params=yaml.safe_load((root/'config/synthetic/frontend.yaml').read_text())
        params['/**']['ros__parameters'].update(config_path=str(root/'config/synthetic/estimator_config.yaml'),
            imu_topic='/contract/imu',left_topic='/contract/image0',right_topic='/contract/image1',
            right_info_frame='d455_infra1_optical_frame',
            left_info_topic='/contract/info0',right_info_topic='/contract/info1')
        p=Path(d)/'params.yaml';p.write_text(yaml.safe_dump(params))
        log=open(Path(d)/'process.log','w+')
        try:
            processes.append(subprocess.Popen(['ros2','run','wla_openvins_navigation','openvins_frontend',
                '--ros-args','-r','__ns:=/contract','--params-file',str(p)],stdout=log,stderr=log,start_new_session=True))
            spin(3)
            assert statuses and statuses[-1].mode==EstimatorStatus.INITIALIZING
            end=time.monotonic()+1.
            while time.monotonic()<end:
                ts=n.get_clock().now().to_msg()
                msg=Imu();msg.header.stamp=ts;msg.header.frame_id='gyro_link';msg.linear_acceleration.z=9.81
                imu.publish(msg)
                for k in range(2):
                    info=CameraInfo();info.header.stamp=ts;info.header.frame_id='d455_infra1_optical_frame'
                    info.width=640;info.height=480;info.p=[400.,0.,320.,0.,0.,400.,240.,0.,0.,0.,1.,0.]
                    infos[k].publish(info)
                    image=Image();image.header.stamp=ts;image.header.frame_id=f'd455_infra{k+1}_optical_frame';image.width=640;image.height=480
                    image.encoding='mono8';image.step=640;image.data=bytes(640*480);images[k].publish(image)
                spin(.025)
            assert statuses[-1].image_age_s>=0 and statuses[-1].mode==EstimatorStatus.INITIALIZING
            spin(.25)
            assert statuses[-1].mode==EstimatorStatus.INITIALIZING, 'warm-up silence must not latch an odometry fault before initialization'
            msg=Imu();msg.header.stamp=n.get_clock().now().to_msg();msg.header.frame_id='gyro_link'
            msg.linear_acceleration.z=9.81;msg.angular_velocity.x=float('nan');imu.publish(msg);spin(.3)
            assert statuses[-1].mode==EstimatorStatus.INVALID
            msg.angular_velocity.x=0.;msg.header.stamp=n.get_clock().now().to_msg();imu.publish(msg);spin(.2)
            assert statuses[-1].mode==EstimatorStatus.INVALID, 'invalid input must latch'
            reset=n.create_client(Trigger,'/contract/reset');assert reset.wait_for_service(timeout_sec=3)
            f=reset.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=3)
            assert f.result().success;spin(.2)
            assert statuses[-1].generation==1 and statuses[-1].mode==EstimatorStatus.INITIALIZING
            assert statuses[-1].header.frame_id.endswith('_g1')
            assert not odoms and not tfs, 'no fabricated odom or default TF before initialization'

            # Test quality gate using explicit fixture messages in the same isolated domain.
            tracking=[]
            n.create_subscription(Bool,'/gate/tracking_healthy',lambda m:tracking.append(m.data),10)
            status=n.create_publisher(EstimatorStatus,'/gate/estimator_status',10)
            contact=n.create_publisher(String,'/gate/contact',10)
            odom=n.create_publisher(Odometry,'/gate/odom',10)
            n.create_subscription(Bool,'/gate/motion_ready',lambda m:permits.append(m.data),10)
            n.create_subscription(Odometry,'/gate/mapping_odom',mapped.append,10)
            map_pose=n.create_publisher(PoseWithCovarianceStamped,'/d455_slam/localization_pose',10)
            map_info=n.create_publisher(Info,'/d455_slam/info',10)
            processes.append(subprocess.Popen(gate_command+[
                '--ros-args','-r','__ns:=/gate','-p','odom_topic:=/gate/odom',
                '-p','contact_topic:=/gate/contact','-p','mapping_odom_topic:=/gate/mapping_odom'],
                stdout=log,stderr=log,start_new_session=True))
            spin(2)
            def fixture(mode,ground,gen=0,seconds=.4,map_kind=None,state_age=0):
                end=time.monotonic()+seconds
                while time.monotonic()<end:
                    s=EstimatorStatus();s.header.stamp=n.get_clock().now().to_msg();s.header.frame_id='test_odom'
                    s.state_stamp=n.get_clock().now().to_msg();s.state_stamp.sec-=state_age;s.instance_id='fixture';s.generation=gen;s.mode=mode
                    s.initialized=s.state_valid=s.uncertainty_acceptable=True
                    s.visual_update_age_s=.1;s.recovery_updates=3
                    status.publish(s);contact.publish(String(data=ground));spin(.02)
                    o=Odometry();o.header=s.header;o.child_frame_id='test_base';o.pose.pose.orientation.w=1.0
                    odom.publish(o);spin(.02)
                    if map_kind:
                        pose=PoseWithCovarianceStamped();pose.header.stamp=s.header.stamp;pose.header.frame_id='map'
                        pose.pose.pose.orientation.w=1.0
                        map_pose.publish(pose)
                        if map_kind!='pose_only':
                            info=Info();info.header.stamp=s.header.stamp
                            info.loop_closure_id=0 if map_kind=='no_match' else 42
                            if map_kind=='stale_match':info.header.stamp.sec-=1
                            map_info.publish(info)
                        spin(.02)
            fixture(EstimatorStatus.VIO_TRACKING,'UNKNOWN');assert permits and not permits[-1] and not mapped
            assert tracking and tracking[-1], 'ground permission must not define VIO health'
            fixture(EstimatorStatus.INERTIAL_ONLY,'GROUNDED');assert not permits[-1] and not mapped
            assert not tracking[-1], 'inertial propagation is not healthy VIO'
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED');assert permits[-1] and mapped
            fixture(EstimatorStatus.VIO_TRACKING,'CARRIED');assert not permits[-1]
            assert tracking[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED',state_age=1);assert not permits[-1] and not tracking[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED');assert permits[-1] and tracking[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED',gen=1);assert not permits[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED');assert not permits[-1], 'generation lock must persist'
            status=n.create_publisher(EstimatorStatus,'/local_gate/estimator_status',10)
            contact=n.create_publisher(String,'/local_gate/contact',10)
            odom=n.create_publisher(Odometry,'/local_gate/odom',10)
            local_mapped=[];local_permits=[]
            n.create_subscription(Odometry,'/local_gate/mapping_odom',local_mapped.append,10)
            n.create_subscription(Bool,'/local_gate/motion_ready',lambda m:local_permits.append(m.data),10)
            processes.append(subprocess.Popen(gate_command+[
                '--ros-args','-r','__ns:=/local_gate','-p','mode:=localization','-p','odom_topic:=/local_gate/odom',
                '-p','contact_topic:=/local_gate/contact','-p','mapping_odom_topic:=/local_gate/mapping_odom'],
                stdout=log,stderr=log,start_new_session=True))
            spin(2)
            fixture(EstimatorStatus.RECOVERING,'CARRIED');assert local_mapped and not local_permits[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED',map_kind='pose_only');assert not local_permits[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED',map_kind='no_match');assert not local_permits[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED',map_kind='stale_match');assert not local_permits[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED',map_kind='matched');assert local_permits[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED',map_kind='pose_only');assert local_permits[-1]
            fixture(EstimatorStatus.INERTIAL_ONLY,'GROUNDED');assert not local_permits[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED',map_kind='pose_only');assert not local_permits[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED',map_kind='matched');assert local_permits[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED',seconds=3.3);assert not local_permits[-1]
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED',map_kind='pose_only');assert not local_permits[-1]
            # An explicitly unvalidated commissioning session can never authorize motion.
            status=n.create_publisher(EstimatorStatus,'/commission_gate/estimator_status',10)
            contact=n.create_publisher(String,'/commission_gate/contact',10)
            odom=n.create_publisher(Odometry,'/commission_gate/odom',10)
            commission_permits=[];commission_mapped=[]
            n.create_subscription(Bool,'/commission_gate/motion_ready',lambda m:commission_permits.append(m.data),10)
            n.create_subscription(Odometry,'/commission_gate/mapping_odom',commission_mapped.append,10)
            processes.append(subprocess.Popen(gate_command+[
                '--ros-args','-r','__ns:=/commission_gate','-p','motion_output_enabled:=false',
                '-p','odom_topic:=/commission_gate/odom','-p','contact_topic:=/commission_gate/contact',
                '-p','mapping_odom_topic:=/commission_gate/mapping_odom'],stdout=log,stderr=log,start_new_session=True))
            spin(2)
            fixture(EstimatorStatus.VIO_TRACKING,'GROUNDED');assert commission_mapped and not commission_permits[-1]
            print(json.dumps({'passed':['initialization_status','stereo_image_and_camera_info_input','startup_silence_waits_without_fault','nonfinite_imu_latched','explicit_reset_generation',
                'no_fake_odom','no_default_tf','unknown_and_carried_no_drive','inertial_no_mapping','raw_vio_independent_of_contact','fresh_report_cannot_mask_stale_state',
                'tracking_grounded_permission','generation_change_latched','recovering_can_relocalize_without_drive',
                'pose_only_and_stale_info_cannot_confirm_map','matched_pose_confirms_map',
                'visual_loss_and_backend_timeout_revoke_map','commissioning_never_authorizes_motion']}))
        except Exception:
            log.flush();log.seek(0);print(log.read()[-12000:]);raise
        finally:
            for process in processes:
                try:os.killpg(process.pid,signal.SIGINT)
                except ProcessLookupError:pass
            for process in processes:
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            log.close();n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
