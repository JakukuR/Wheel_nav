#!/usr/bin/env python3
"""Record actual odom gaps; never labels continuity as accuracy."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


def main():
    p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--namespace',default='/openvins_trial')
    a,ros=p.parse_known_args();rclpy.init(args=ros);n=Node('openvins_metrics');last=[None]
    with open(a.output,'x',encoding='utf-8') as f:
        def write(x): f.write(json.dumps(x)+'\n');f.flush()
        def odom(m):
            t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            write(dict(type='odom',wall=time.time(),stamp=t,gap=None if last[0] is None else t-last[0],
                       frame=m.header.frame_id,xyz=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]))
            last[0]=t
        n.create_subscription(Odometry,a.namespace+'/odom',odom,qos_profile_sensor_data)
        try:rclpy.spin(n)
        except KeyboardInterrupt:pass
    n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
