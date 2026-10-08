#!/usr/bin/env python3
"""Run only map_server/planner in domain 74; no controller/chassis or velocity output."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from tf2_ros import StaticTransformBroadcaster


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '74'
    children = []
    logs = []
    rclpy.init()
    node = rclpy.create_node('global_obstacle_replan_regression')
    tf = StaticTransformBroadcaster(node)
    transform = TransformStamped(); transform.header.frame_id = 'map'
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.child_frame_id = 'r680_mapping_floor'; transform.transform.rotation.w = 1.0
    tf.sendTransform(transform)
    cloud = node.create_publisher(PointCloud2, '/r680_nav/d455/points', 10)
    health = node.create_publisher(Bool, '/r680_nav/localization_ready', 10)
    client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    with tempfile.TemporaryDirectory(prefix='wla-global-plan-test-') as temporary:
        root = Path(temporary)
        (root/'map.pgm').write_bytes(b'P5\n100 100\n255\n'+bytes([254])*10000)
        (root/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=0.05,
            origin=[-2.5,-2.5,0.0], negate=0, occupied_thresh=0.65, free_thresh=0.25)))
        params = yaml.safe_load((Path(get_package_share_directory('wla_r680_navigation'))/
            'config/nav2_recovery_test.yaml').read_text())
        params = {k: params[k] for k in ['planner_server', 'global_costmap']}
        (root/'nav.yaml').write_text(yaml.safe_dump(params))
        def spawn(command, name):
            stream = (root/(name+'.log')).open('w+'); logs.append((name,stream))
            children.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                             start_new_session=True))
        spawn(['ros2','run','nav2_map_server','map_server','--ros-args','-p',
               'yaml_filename:='+str(root/'map.yaml'), '-r','__node:=map_server',
               '-r','map:=/r680/d455/map'], 'map')
        spawn(['ros2','run','nav2_planner','planner_server','--ros-args',
               '--params-file',str(root/'nav.yaml')], 'planner')
        spawn(['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args',
               '-r','__node:=test_lifecycle','-p','autostart:=true',
               '-p','node_names:=[map_server,planner_server]'], 'lifecycle')
        def tick(mark):
            health.publish(Bool(data=True))
            if mark:
                h = Header(); h.frame_id = 'r680_mapping_floor'; h.stamp = node.get_clock().now().to_msg()
                # Wall across the straight route, with room on either side.
                cloud.publish(point_cloud2.create_cloud_xyz32(h,
                    [(0.70,float(y)/100,0.20) for y in range(-40,41,5)]))
            rclpy.spin_once(node,timeout_sec=0.02); time.sleep(0.02)
            if any(p.poll() is not None for p in children):
                raise AssertionError('planning test node exited')
        def wait(future, mark, limit=10):
            end=time.monotonic()+limit
            while not future.done() and time.monotonic()<end: tick(mark)
            assert future.done(), 'planner action timed out'
            return future.result()
        def plan(mark):
            goal=ComputePathToPose.Goal(); goal.use_start=True; goal.planner_id='GridBased'
            for p in [goal.start,goal.goal]:
                p.header.frame_id='map'; p.header.stamp=node.get_clock().now().to_msg()
                p.pose.orientation.w=1.0
            goal.goal.pose.position.x=1.70
            handle=wait(client.send_goal_async(goal),mark); assert handle.accepted
            result=wait(handle.get_result_async(),mark)
            assert result.status==4 and result.result.path.poses, f'planner rejected fixture: {result.status}'
            return result.result.path
        try:
            end=time.monotonic()+12
            while not client.server_is_ready() and time.monotonic()<end: tick(True)
            assert client.server_is_ready(), 'planner not active'
            end=time.monotonic()+1.5
            while time.monotonic()<end: tick(True)
            detour=plan(True)
            lateral=max(abs(p.pose.position.y) for p in detour.poses)
            assert lateral>0.50, f'global planner ignored dynamic wall: lateral={lateral}'
            end=time.monotonic()+3
            while time.monotonic()<end: tick(False)
            cleared=plan(False)
            straight=max(abs(p.pose.position.y) for p in cleared.poses)
            assert straight<0.08, f'expired dynamic wall still affects planner: {straight}'
            print(f'PASS: real SmacPlanner2D detour={lateral:.3f}m; after 2s TTL route={straight:.3f}m')
        except BaseException:
            for name,stream in logs:
                stream.flush(); stream.seek(0); print(name,stream.read()[-5000:])
            raise
        finally:
            for p in children:
                if p.poll() is None: os.killpg(p.pid,signal.SIGINT)
            for p in children:
                try: p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL);p.wait()
            for _,stream in logs: stream.close()
            node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':
    main()
