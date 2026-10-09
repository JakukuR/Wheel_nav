#!/usr/bin/env python3
"""Isolated synthetic mapping readiness test; no drivers or motion publishers."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '74', 'Synthetic test must be isolated'
    prefix = Path(get_package_prefix('wla_r680_navigation'))/'lib/wla_r680_navigation'
    rclpy.init(); node = Node('mapping_cuvslam_synthetic')
    log = open('/tmp/mapping-cuvslam-regression.log', 'w')
    processes = [subprocess.Popen([str(prefix/name), '--ros-args', '-p', parameter]
                                 + (['-p','require_mapping_odom:=true'] if name=='interface_monitor' else []),
                                 stdout=log, stderr=subprocess.STDOUT)
                 for name, parameter in [('mapping_odom_gate', 'require_initialization_supervisor:=true'),
                                         ('interface_monitor', 'require_vio_health:=true')]]
    def restart_gate():
        processes[0].terminate(); processes[0].wait(timeout=5)
        processes[0]=subprocess.Popen([str(prefix/'mapping_odom_gate'),'--ros-args','-p',
                                     'require_initialization_supervisor:=true'],stdout=log,stderr=subprocess.STDOUT)
    count, states = [0], []
    node.create_subscription(Odometry, '/r680_nav/mapping_odom', lambda m: count.__setitem__(0,count[0]+1), 10)
    node.create_subscription(Bool, '/r680_nav/localization_ready', lambda m: states.append(m.data), 10)
    pubs = {topic: node.create_publisher(typ,topic,10) for topic,typ in [
        ('/d455_slam/odom',Odometry), ('/r680_nav/vio_tracking_healthy',Bool),
        ('/r680_nav/vio_init_state',String), ('/r680/d455/color/image_raw',Image),
        ('/r680/d455/aligned_depth_to_color/image_raw',Image), ('/r680/d455/color/camera_info',CameraInfo),
        ('/r680_nav/d455/points',PointCloud2)]}
    retained = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid,'/r680/d455/map',retained)
    tf = TransformBroadcaster(node)
    def lifecycle(req,res): res.current_state.id=3; return res
    node.create_service(GetState,'/velocity_smoother/get_state',lifecycle)
    node.create_service(GetState,'/collision_monitor/get_state',lifecycle)
    def drive(seconds, healthy=False, init='moving', send_health=True, valid=True, map_enabled=False):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            stamp=node.get_clock().now().to_msg()
            odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id='d455_floor_odom'
            odom.child_frame_id='r680_mapping_floor'; odom.pose.pose.orientation.w=1.0
            if not valid: odom.pose.pose.position.x=float('nan')
            pubs['/d455_slam/odom'].publish(odom)
            if send_health: pubs['/r680_nav/vio_tracking_healthy'].publish(Bool(data=healthy))
            pubs['/r680_nav/vio_init_state'].publish(String(data=init))
            image=Image(); image.header.stamp=stamp
            pubs['/r680/d455/color/image_raw'].publish(image)
            pubs['/r680/d455/aligned_depth_to_color/image_raw'].publish(image)
            info=CameraInfo(); info.header.stamp=stamp; info.k=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
            pubs['/r680/d455/color/camera_info'].publish(info)
            cloud=PointCloud2(); cloud.header.stamp=stamp; cloud.header.frame_id='r680_mapping_floor'; cloud.point_step=12
            pubs['/r680_nav/d455/points'].publish(cloud)
            if map_enabled:
                grid=OccupancyGrid(); grid.header.stamp=stamp; grid.header.frame_id='map'
                grid.info.width=grid.info.height=2; grid.info.resolution=0.05; grid.data=[0,0,0,100]
                grid.info.origin.orientation.w=1.0; map_pub.publish(grid)
                transform=TransformStamped(); transform.header.stamp=stamp; transform.header.frame_id='map'
                transform.child_frame_id='r680_mapping_floor'; transform.transform.rotation.w=1.0
                tf.sendTransform(transform)
            rclpy.spin_once(node,timeout_sec=0.01)
            assert all(p.poll() is None for p in processes[:2]), 'test node exited'
    try:
        drive(1.5); assert count[0]==0 and states and not states[-1]
        drive(1,True); assert count[0]==0, 'initialization motion was written to mapping odometry'
        drive(1,True,'waiting_localization'); assert count[0]>10 and states[-1]
        drive(.4,False,'succeeded'); before=count[0]
        drive(.6,False,'succeeded'); assert count[0]==before and not states[-1]
        drive(.6,True,'succeeded'); assert count[0]==before and not states[-1], 'lost inertial session silently resumed mapping or control'
        restart_gate(); drive(.5,True,'succeeded')
        drive(.4,True,'failed'); before=count[0]
        drive(.6,True,'failed'); assert count[0]==before, 'failed initialization resumed map writes'
        restart_gate()
        drive(.4,True,'succeeded',valid=False); before=count[0]
        drive(.6,True,'succeeded',valid=False); assert count[0]==before, 'invalid pose was forwarded'
        drive(1,True,'succeeded'); before=count[0]
        drive(.4,True,'succeeded',send_health=False); before=count[0]
        drive(.6,True,'succeeded',send_health=False); assert count[0]==before and not states[-1]
        drive(.6,True,'succeeded'); assert count[0]==before, 'stale session resumed without restart'
        restart_gate()
        with tempfile.TemporaryDirectory(prefix='mapping-ready-') as directory:
            waiter=subprocess.Popen([str(prefix/'wait_mapping_ready'),'--frontend','cuvslam','--auto-init',
                                      '--run-dir',directory,'--timeout','8'],stdout=log,stderr=subprocess.STDOUT)
            processes.append(waiter)
            drive(1.0,True,'succeeded',map_enabled=False)
            assert waiter.poll() is None, 'mapping opened without a map and TF'
            drive(2,True,'succeeded',map_enabled=True)
            waiter.wait(timeout=2); assert waiter.returncode==0
            result=json.loads((Path(directory)/'mapping_readiness.json').read_text())
            assert result['ready'] and result['mapping_odom_ready'] and result['map_to_base']
        with tempfile.TemporaryDirectory(prefix='mapping-not-ready-') as directory:
            waiter=subprocess.Popen([str(prefix/'wait_mapping_ready'),'--frontend','cuvslam','--auto-init',
                                      '--run-dir',directory,'--timeout','5'],stdout=log,stderr=subprocess.STDOUT)
            processes.append(waiter)
            drive(1.5,True,'failed')
            waiter.wait(timeout=2); assert waiter.returncode!=0
            result=json.loads((Path(directory)/'mapping_readiness.json').read_text())
            assert not result['ready'] and result['initialization']=='failed'
        print('PASS: provisional/lost/stale/cancelled/invalid VIO blocked; mapping opens after map + TF + lifecycle, without an old-map match')
    finally:
        node.destroy_node(); rclpy.shutdown()
        for p in processes:
            if p.poll() is None:
                p.terminate()
                try: p.wait(timeout=5)
                except subprocess.TimeoutExpired: p.kill(); p.wait()
        log.close()


if __name__=='__main__': main()
