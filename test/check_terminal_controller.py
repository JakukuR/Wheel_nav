#!/usr/bin/env python3
"""Actual Nav2 controller, synthetic delayed chassis in domain 74. No hardware."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import math
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from tf2_ros import TransformBroadcaster


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '74', 'isolated domain required'
    map_correction_mode='--map-correction' in sys.argv
    rclpy.init(); node=rclpy.create_node('terminal_controller_regression')
    tf=TransformBroadcaster(node)
    odom_pub=node.create_publisher(Odometry,'/d455_slam/odom',10)
    outputs=[]; latest=Twist(); x=0.0; yaw=0.0; v=0.08; w=0.30
    queue=[]; last_tick=time.monotonic(); first_yaw_motion=None
    correction_started=None;map_offset=0.0;goal_count=0
    children=[]; streams=[]
    def received(msg):
        nonlocal latest,first_yaw_motion
        latest=msg; outputs.append((time.monotonic(),msg.linear.x,msg.angular.z))
        if abs(msg.angular.z)>1e-6 and first_yaw_motion is None:
            first_yaw_motion=(v,w)
    sub=node.create_subscription(Twist,'/cmd_vel',received,10)
    client=ActionClient(node,FollowPath,'/follow_path')
    lifecycle=node.create_client(GetState,'/controller_server/get_state')
    with tempfile.TemporaryDirectory(prefix='wla-terminal-controller-test-') as directory:
        root=Path(directory)
        config=yaml.safe_load((Path(get_package_share_directory('wla_r680_navigation'))/
            'config/nav2_recovery_test.yaml').read_text())
        config={key:config[key] for key in ['controller_server','local_costmap']}
        local=config['local_costmap']['local_costmap']['ros__parameters']
        local['plugins']=['inflation_layer'];local.pop('obstacle_layer')
        (root/'params.yaml').write_text(yaml.safe_dump(config))
        def spawn(args,name):
            stream=(root/(name+'.log')).open('w+');streams.append((name,stream))
            children.append(subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
        spawn(['ros2','run','nav2_controller','controller_server','--ros-args','--params-file',
               str(root/'params.yaml')],'controller')
        spawn(['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args',
               '-r','__node:=terminal_test_lifecycle','-p','autostart:=true',
               '-p','node_names:=[controller_server]'],'lifecycle')
        def tick():
            nonlocal last_tick,x,yaw,v,w,correction_started,map_offset
            now=time.monotonic();dt=min(.1,now-last_tick);last_tick=now
            if map_correction_mode and first_yaw_motion is not None and correction_started is None:
                correction_started=now;map_offset=-.13
            queue.append((now+.40,latest.linear.x,latest.angular.z))
            desired=(0.0,0.0)
            # Preserve the most recent arrived command across ticks.
            while queue and queue[0][0]<=now:
                tick.arrived=(queue.pop(0)[1:])
            desired=tick.arrived
            v+=(desired[0]-v)*min(1.0,dt/.5)
            w+=(1.4*desired[1]-w)*min(1.0,dt/.5)
            x+=v*math.cos(yaw)*dt;yaw+=w*dt
            stamp=node.get_clock().now().to_msg()
            transform=TransformStamped();transform.header.stamp=stamp
            transform.header.frame_id='d455_floor_odom';transform.child_frame_id='r680_mapping_floor'
            transform.transform.translation.x=x
            transform.transform.rotation.z=math.sin(yaw/2);transform.transform.rotation.w=math.cos(yaw/2)
            tf.sendTransform(transform)
            if map_correction_mode:
                global_transform=TransformStamped();global_transform.header.stamp=stamp
                global_transform.header.frame_id='map';global_transform.child_frame_id='d455_floor_odom'
                global_transform.transform.translation.x=map_offset
                global_transform.transform.rotation.w=1.0;tf.sendTransform(global_transform)
            odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='d455_floor_odom'
            odom.child_frame_id='r680_mapping_floor';odom.pose.pose.position.x=x
            odom.pose.pose.orientation=transform.transform.rotation
            odom.twist.twist.linear.x=v;odom.twist.twist.angular.z=w
            # A short visual-twist disturbance while the body remains stopped
            # reproduces the intermediate stopping phase from the recorded run.
            if correction_started is not None and now-correction_started<.2:
                odom.twist.twist.linear.x=.04
            odom.twist.covariance[0]=.001;odom.twist.covariance[35]=.001
            odom_pub.publish(odom)
            rclpy.spin_once(node,timeout_sec=.01);time.sleep(.01)
            assert all(p.poll() is None for p in children),'test node exited'
        tick.arrived=(0.0,0.0)
        def wait(future,limit):
            end=time.monotonic()+limit
            while not future.done() and time.monotonic()<end:tick()
            assert future.done(),'action timed out'
            return future.result()
        def goal(angle):
            nonlocal goal_count
            goal_count+=1
            path=RosPath();path.header.frame_id='map' if map_correction_mode else 'd455_floor_odom'
            target=.17 if map_correction_mode and goal_count==1 else x+map_offset+.05 if map_correction_mode else .05
            for xx in [x+map_offset,target]:
                p=PoseStamped();p.header=path.header;p.pose.position.x=xx
                p.pose.orientation.z=math.sin(angle/2);p.pose.orientation.w=math.cos(angle/2)
                path.poses.append(p)
            request=FollowPath.Goal();request.path=path;request.controller_id='FollowPath'
            request.goal_checker_id='goal_checker';request.progress_checker_id='progress_checker'
            handle=wait(client.send_goal_async(request),5);assert handle.accepted
            result=wait(handle.get_result_async(),25)
            assert result.status==4,f'goal failed: {result.status}'
            assert abs(math.remainder(angle-yaw,2*math.pi))<=.18,'outside yaw tolerance'
            assert abs(w)<=.04 and abs(v)<=.025,'success declared before body stopped'
            end=time.monotonic()+.7;start=len(outputs)
            while time.monotonic()<end:tick()
            assert all(abs(vv)<1e-6 and abs(ww)<1e-6 for _,vv,ww in outputs[start:]),'did not hold zero'
        try:
            end=time.monotonic()+12
            while not client.server_is_ready() and time.monotonic()<end:tick()
            assert client.server_is_ready(),'controller not ready'
            state=wait(lifecycle.call_async(GetState.Request()),5)
            end=time.monotonic()+10
            while state.current_state.id!=3 and time.monotonic()<end:
                tick();state=wait(lifecycle.call_async(GetState.Request()),5)
            assert state.current_state.id==3,'controller lifecycle not active'
            # Reintroduce motion so first command must stop and confirm feedback.
            v=.08;w=.30
            goal(.9)
            assert first_yaw_motion is not None
            assert abs(first_yaw_motion[0])<=.025 and abs(first_yaw_motion[1])<=.04,'rotated before stopping'
            if map_correction_mode:
                assert correction_started is not None,'map correction was not injected'
                assert .2<abs(.17-map_offset-x)<.28,'correction did not cross XY tolerance inside exit distance'
            goal(-.8) # A new target must reset the terminal latch.
            assert all(abs(vv)<1e-6 for _,vv,_ in outputs),'terminal translated'
            for name,stream in streams:
                stream.flush();stream.seek(0)
                Path('/tmp/wla-terminal-'+('map-correction-' if map_correction_mode else 'baseline-')+name+'.log').write_text(stream.read())
            print('PASS: real controller and StoppedGoalChecker, delayed chassis, stop-before-align, two goals, zero hold'+
                  (', map correction across 0.20m retained alignment without translation' if map_correction_mode else ''))
        except BaseException:
            for name,stream in streams:
                stream.flush();stream.seek(0);print(name,stream.read()[-6500:])
            raise
        finally:
            for p in children:
                if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
            for p in children:
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            for _,stream in streams:stream.close()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
