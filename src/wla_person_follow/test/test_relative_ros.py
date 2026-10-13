"""Isolated ROS smoke. Explicit RUN_RELATIVE_ROS=1, never a production domain."""
import json
import os
import time
import unittest


@unittest.skipUnless(os.environ.get('RUN_RELATIVE_ROS') == '1', 'isolated ROS opt-in required')
class RelativeRosTests(unittest.TestCase):
    def test_service_depth_preview_gate_and_shutdown(self):
        if os.environ.get('ROS_DOMAIN_ID') != '198':
            self.fail('relative ROS test requires isolated ROS_DOMAIN_ID=198')
        import numpy as np
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        from wla_behavior_interfaces.srv import PersonCommand
        from wla_person_follow.node import PersonFollow
        from wla_person_follow.relative_control import Safety

        rclpy.init()
        follower, fixture = PersonFollow(), Node('relative_follow_fixture')
        executor = SingleThreadedExecutor()
        executor.add_node(follower)
        executor.add_node(fixture)
        statuses, candidates = [], []
        fixture.create_subscription(String,'/r680/person_follow/status',lambda m: statuses.append(json.loads(m.data)),10)
        fixture.create_subscription(String,'/r680/person_follow/command_candidate',lambda m: candidates.append(json.loads(m.data)),10)
        relative=fixture.create_publisher(String,'/r680/perception/relative_objects',1)
        gate=fixture.create_publisher(String,'/r680_nav/perception/status',1)
        depths=fixture.create_publisher(Image,'/r680/d455/aligned_depth_to_color/image_raw',qos_profile_sensor_data)
        infos=fixture.create_publisher(CameraInfo,'/r680/d455/color/camera_info',qos_profile_sensor_data)
        broadcaster=StaticTransformBroadcaster(fixture)
        transform=TransformStamped()
        transform.header.frame_id='r680_mapping_floor'
        transform.child_frame_id='d455_color_optical_frame'
        transform.transform.translation.z=.3
        transform.transform.rotation.x=-.5
        transform.transform.rotation.y=.5
        transform.transform.rotation.z=-.5
        transform.transform.rotation.w=.5
        broadcaster.sendTransform(transform)
        client=fixture.create_client(PersonCommand,'/r680/person_follow/command')
        wheel=fixture.create_publisher(Odometry,'/wheel/odom',qos_profile_sensor_data)
        enabled=True

        def pump(duration, publish=False, distance=2., identifier=7, motion=True):
            deadline=time.monotonic()+duration
            next_publish=0.
            while time.monotonic()<deadline:
                if publish and time.monotonic()>=next_publish:
                    next_publish=time.monotonic()+.04
                    ns=fixture.get_clock().now().nanoseconds
                    stamp=fixture.get_clock().now().to_msg()
                    info=CameraInfo()
                    info.header.frame_id='d455_color_optical_frame'
                    info.header.stamp=stamp
                    info.width,info.height=64,48
                    info.k=[60.,0.,32.,0.,60.,24.,0.,0.,1.]
                    depth=Image()
                    depth.header=info.header
                    depth.width,depth.height=64,48
                    depth.encoding='16UC1'
                    depth.step=128
                    depth.data=(np.ones((48,64),dtype='<u2')*2000).tobytes()
                    odom=Odometry();odom.header.stamp=stamp
                    odom.header.frame_id='wheel_odom';odom.child_frame_id='r680_mapping_floor'
                    if motion: wheel.publish(odom)
                    infos.publish(info)
                    depths.publish(depth)
                    relative.publish(String(data=json.dumps(dict(schema='wla-relative-detections/v1',
                        stamp_ns=ns, session='ros-test',epoch=0,enabled=enabled,base_frame='r680_mapping_floor',
                        objects=[dict(detected=True,label='person',id=f'ros-test:0:{identifier}',yaw=0.,distance=distance)] if enabled else []))))
                    gate.publish(String(data=json.dumps(dict(schema='wla-relative-perception-status/v1',
                        stamp_ns=ns,session='ros-test',epoch=0,enabled=enabled))))
                executor.spin_once(timeout_sec=.002)
                time.sleep(.001)

        def command(kind, request_id='', mode='', target='', lease=1.):
            req=PersonCommand.Request()
            req.command,req.request_id,req.mode,req.target_id=kind,request_id,mode,target
            req.lease_duration_s=lease
            future=client.call_async(req)
            until=time.monotonic()+2.
            while not future.done() and time.monotonic()<until:
                executor.spin_once(timeout_sec=.01)
            self.assertTrue(future.done(),'service response timed out')
            return future.result()

        try:
            pump(.25)
            self.assertTrue(statuses)
            self.assertTrue(all(c['linear']==c['angular']==0. for c in candidates))
            self.assertEqual(fixture.count_publishers('/r680_base/cmd_vel'),0)
            self.assertEqual(fixture.count_publishers('/r680_nav/chassis_cmd_vel'),0)
            # DDS discovery and first timestamp-aligned wheel/depth pair have
            # no fixed 500 ms startup guarantee. Wait on the actual readiness
            # condition with a bounded deadline, retaining failure diagnostics.
            ready_until=time.monotonic()+2.
            while time.monotonic()<ready_until:
                pump(.1,publish=True)
                if follower.safety.valid and follower.scene is not None:break
            self.assertTrue(follower.safety.valid,json.dumps(dict(reason=follower.safety_reason,
                statuses=statuses[-3:],motion=list(follower.local_motion.samples)[-3:]),default=str))
            self.assertTrue(client.wait_for_service(timeout_sec=1.))
            result=command('start','follow-test','follow','ros-test:0:7')
            self.assertTrue(result.accepted,result.reason)
            self.assertTrue(result.enabled)
            pump(.2,publish=True)
            self.assertTrue(any(s['linear_mps']>0 for s in statuses),
                json.dumps(dict(last_statuses=statuses[-5:], core=follower.core.snapshot(time.monotonic()),
                    safety=vars(follower.safety), safety_reason=follower.safety_reason),default=str))
            self.assertTrue(all(c['linear']==c['angular']==0. for c in candidates),'preview leaked motion')
            self.assertTrue(all(s['motion_permitted'] is False for s in statuses))
            pump(.30,publish=True,motion=False)
            self.assertFalse(statuses[-1]['local_motion_fresh'])
            self.assertEqual((statuses[-1]['linear_mps'],statuses[-1]['angular_rps']),(0.,0.))
            pump(.15,publish=True)
            self.assertTrue(statuses[-1]['local_motion_fresh'])
            self.assertTrue(command('heartbeat','follow-test').accepted)
            pump(.12,publish=True,identifier=8)
            self.assertEqual(statuses[-1]['target_tracking'],'predicted')
            self.assertEqual(statuses[-1]['reason'],'target_prediction_last_seen')
            self.assertEqual(follower.core.target_id,'ros-test:0:7')
            pump(.12,publish=True)
            self.assertEqual(statuses[-1]['target_tracking'],'measured')
            self.assertTrue(command('heartbeat','follow-test').accepted)
            cancelled=command('stop','late-start')
            self.assertTrue(cancelled.accepted)
            self.assertEqual(cancelled.request_id,'late-start')
            self.assertEqual(cancelled.state,'stopped')
            self.assertFalse(cancelled.enabled)
            self.assertTrue(follower.core.active)
            rejected=command('start','late-start','follow','ros-test:0:7')
            self.assertFalse(rejected.accepted)
            self.assertEqual(rejected.request_id,'late-start')
            self.assertEqual(rejected.state,'rejected')
            self.assertEqual(follower.core.request_id,'follow-test')
            self.assertFalse(any((('/odom' in name and name!='/wheel/odom') or name=='/map') for name,_ in
                fixture.get_subscriber_names_and_types_by_node(follower.get_name(),follower.get_namespace())))
            pump(.05,publish=True,distance=1.)
            approached=command('start','approach-test','approach_interact','ros-test:0:7')
            self.assertTrue(approached.accepted,approached.reason)
            pump(.2,publish=True,distance=1.)
            arrived=command('heartbeat','approach-test')
            self.assertFalse(arrived.accepted)
            self.assertEqual(arrived.state,'arrived')
            self.assertEqual(arrived.request_id,'approach-test')
            self.assertFalse(arrived.enabled)
            pump(.05,publish=True)
            resumed=command('start','follow-after-arrival','follow','ros-test:0:7')
            self.assertTrue(resumed.accepted,resumed.reason)
            pump(.35)
            self.assertEqual(follower.core.tick(time.monotonic(),Safety(True,10.,True)),(0.,0.))
            pump(.1,publish=True,identifier=8)
            self.assertEqual(follower.core.target_id,'ros-test:0:7')
            self.assertEqual(follower.core.tick(time.monotonic(),Safety(True,10.,True)),(0.,0.))
            enabled=False
            pump(.1,publish=True)
            self.assertFalse(follower.core.active)
            self.assertEqual(follower.core.reason,'detection_disabled')
        finally:
            follower.close()
            executor.remove_node(follower)
            executor.remove_node(fixture)
            follower.destroy_node()
            fixture.destroy_node()
            executor.shutdown()
            rclpy.try_shutdown()


if __name__=='__main__': unittest.main()
