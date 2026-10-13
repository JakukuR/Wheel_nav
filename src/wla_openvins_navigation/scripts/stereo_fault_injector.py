#!/usr/bin/env python3
"""Replay-only stereo fault injection on distinct output topics; preserves stamps."""
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Injector(Node):
    def __init__(self):
        super().__init__('openvins_stereo_fault_injector')
        self.mode=self.declare_parameter('fault','black').value
        self.start=self.declare_parameter('start_after_s',10.).value
        self.duration=self.declare_parameter('duration_s',10.).value
        if self.mode not in ('black','drop') or self.start<0 or self.duration<=0:
            raise ValueError('invalid fault window')
        self.origin=None;self.publishers_=[];self.subscriptions_=[]
        for i in range(2):
            source=self.declare_parameter(f'input_{i}',f'/replay/infra{i+1}/image_rect_raw').value
            target=self.declare_parameter(f'output_{i}',f'/openvins_fault/infra{i+1}/image_rect_raw').value
            if source==target:raise ValueError('input and output must differ')
            self.publishers_.append(self.create_publisher(Image,target,qos_profile_sensor_data))
            self.subscriptions_.append(self.create_subscription(Image,source,lambda m,k=i:self.on_image(m,k),qos_profile_sensor_data))

    def on_image(self,m,k):
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if self.origin is None:self.origin=t
        failed=self.start<=t-self.origin<self.start+self.duration
        if failed and self.mode=='drop':return
        if failed:
            m=copy.deepcopy(m);m.data=bytes(len(m.data))
        self.publishers_[k].publish(m)


def main():
    rclpy.init();n=Injector()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
