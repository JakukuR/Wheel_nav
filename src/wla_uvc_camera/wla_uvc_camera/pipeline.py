"""Read-only RGB crop and optional calibrated stereo depth; no TF or actuation."""
import argparse
from collections import deque
from copy import deepcopy
import json
import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from .stereo import StereoRig,raw_crop


def stamp(msg):
    return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec


def array(msg):
    if msg.encoding!='bgr8' or msg.step<msg.width*3 or len(msg.data)!=msg.height*msg.step:
        raise ValueError('expected_bgr8_raw_image')
    return np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)


def image(values,header,encoding):
    msg=Image();msg.header=deepcopy(header);msg.height,msg.width=values.shape[:2]
    msg.encoding=encoding;msg.is_bigendian=False;msg.step=msg.width*(3 if encoding=='bgr8' else 2)
    msg.data=values.tobytes();return msg


class Pipeline(Node):
    def __init__(self,args):
        super().__init__('uvc_rgb_stereo_depth')
        self.args=args;cv2.setNumThreads(1)
        self.rgb_mode=getattr(args,'rgb_mode','full')
        if self.rgb_mode not in ('full','crop'):raise ValueError('invalid_rgb_mode')
        self.output=(640,360) if self.rgb_mode=='full' else (640,480)
        self.rig=StereoRig(args.calibration,args.algorithm,args.disparities,args.block_size,rgb_mode=self.rgb_mode) if args.calibration else None
        self.colors=deque(maxlen=6);self.pairs={};self.last_used_rgb=0;self.last_pair=0
        self.pending=None;self.condition=threading.Condition();self.closed=False
        self.failure='';self.metrics={};self.last_output=0
        self.color_pub=self.create_publisher(Image,'/camera/processed/color/image_raw',qos_profile_sensor_data)
        self.info_pub=self.create_publisher(CameraInfo,'/camera/processed/color/camera_info',qos_profile_sensor_data)
        self.depth_pub=self.create_publisher(Image,'/camera/processed/aligned_depth_to_color/image_raw',qos_profile_sensor_data)
        self.status_pub=self.create_publisher(String,'/camera/processed/status',1)
        self.create_subscription(Image,'/camera/dicota/color/image_raw',self.rgb,qos_profile_sensor_data)
        self.create_subscription(Image,'/camera/sunplus/left/image_raw',lambda m:self.eye('left',m),qos_profile_sensor_data)
        self.create_subscription(Image,'/camera/sunplus/right/image_raw',lambda m:self.eye('right',m),qos_profile_sensor_data)
        self.worker=threading.Thread(target=self.work,name='uvc-depth',daemon=True);self.worker.start()
        self.create_timer(.5,self.status)

    def fresh(self,msg):
        return 0<stamp(msg) and -.05<=(self.get_clock().now().nanoseconds-stamp(msg))/1e9<=.3

    def rgb(self,msg):
        if msg.header.frame_id!='dicota_color_optical_frame' or (msg.width,msg.height)!=(1920,1080) or not self.fresh(msg):return
        if self.rig:
            self.colors.append(msg);self.match()
        else:
            self.publish_color(raw_crop(array(msg),output=self.output),msg.header)

    def eye(self,which,msg):
        if not self.rig or msg.header.frame_id!=f'sunplus_{which}_optical_frame' or (msg.width,msg.height)!=self.rig.input_size or not self.fresh(msg):return
        key=stamp(msg)
        if key<=self.last_pair:return
        self.pairs.setdefault(key,{})[which]=msg
        while len(self.pairs)>6:del self.pairs[min(self.pairs)]
        self.match()

    def match(self):
        complete=[s for s,p in self.pairs.items() if len(p)==2 and s>self.last_pair]
        available=[m for m in self.colors if stamp(m)>self.last_used_rgb]
        if not complete or not available:return
        # Pick the newest matched pair, bounded by host receive timestamp skew.
        for key in sorted(complete,reverse=True):
            rgb=min(available,key=lambda m:abs(stamp(m)-key))
            if abs(stamp(rgb)-key)>self.args.max_skew_ms*1e6:continue
            pair=self.pairs[key]
            with self.condition:
                self.pending=(rgb,pair['left'],pair['right']);self.condition.notify()
            self.last_used_rgb=stamp(rgb);self.last_pair=key
            for old in list(self.pairs):
                if old<=key:del self.pairs[old]
            return

    def publish_color(self,values,header):
        out=image(values,header,'bgr8');info=CameraInfo();info.header=deepcopy(header)
        info.width=out.width;info.height=out.height
        if self.rig:
            info.distortion_model='plumb_bob';info.d=[0.]*5
            info.k=self.rig.k.reshape(-1).tolist();info.r=np.eye(3).reshape(-1).tolist()
            info.p=np.column_stack([self.rig.k,np.zeros(3)]).reshape(-1).tolist()
        self.info_pub.publish(info);self.color_pub.publish(out);self.last_output=time.monotonic()

    def work(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.closed or self.pending is not None)
                if self.closed:return
                rgb,left,right=self.pending;self.pending=None
            if not self.fresh(rgb) or not self.fresh(left):continue
            start=time.monotonic()
            try:
                depth,_=self.rig.depth(array(left),array(right));color=self.rig.color(array(rgb))
                if not self.fresh(rgb) or not self.fresh(left):continue
                header=deepcopy(left.header);header.frame_id=rgb.header.frame_id
                # Keep stereo receive time on depth, RGB receive time on color.
                # These USB cameras do not provide a shared exposure timestamp.
                self.depth_pub.publish(image(depth,header,'16UC1'))
                self.publish_color(color,rgb.header)
                self.metrics=dict(compute_ms=(time.monotonic()-start)*1000,
                    host_skew_ms=(stamp(rgb)-stamp(left))/1e6,
                    valid_depth_fraction=float(np.count_nonzero(depth)/depth.size))
                self.failure=''
            except Exception as exc:
                self.failure=type(exc).__name__+': '+str(exc)
                self.get_logger().error(self.failure)

    def status(self):
        body=dict(schema='wla-uvc-depth/v1',calibrated=bool(self.rig),
            rgb_mode=self.rgb_mode,color_width=self.output[0],color_height=self.output[1],
            depth_enabled=bool(self.rig),mount_tf_published_by_this_node=False,
            reason=self.failure or ('calibrated_software_only' if self.rig else 'missing_calibration_2d_only'),
            color_fresh=bool(self.last_output and time.monotonic()-self.last_output<.3),
            depth_fresh=bool(self.rig and not self.failure and self.last_output and time.monotonic()-self.last_output<.3),
            timestamp_source='host_capture_receive_not_hardware_exposure',
            cross_camera_sync='approximate_host_receive',max_host_skew_ms=self.args.max_skew_ms,
            metrics=self.metrics)
        self.status_pub.publish(String(data=json.dumps(body,allow_nan=False)))

    def close(self):
        with self.condition:self.closed=True;self.pending=None;self.condition.notify_all()
        self.worker.join(timeout=3.)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rgb-mode',choices=['full','crop'],default='full',
        help='full keeps the complete RGB field at 640x360; crop uses central 1440x1080 -> 640x480')
    p.add_argument('--calibration',default='');p.add_argument('--algorithm',choices=['sgbm','bm'],default='sgbm')
    p.add_argument('--disparities',type=int,default=128);p.add_argument('--block-size',type=int,default=5)
    p.add_argument('--max-skew-ms',type=float,default=25.)
    args=p.parse_args()
    if not 0<args.max_skew_ms<=35:p.error('max-skew-ms must be within (0,35]')
    rclpy.init(args=[]);node=None
    try:node=Pipeline(args);rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if node is not None:node.close();node.destroy_node()
        rclpy.try_shutdown()
