"""UVC acquisition only: no depth, TF, motor command or guessed intrinsics."""
import argparse
import json
from pathlib import Path
import threading
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from .geometry import split_frame,calibration


class Camera(Node):
    def __init__(self,args):
        super().__init__('uvc_'+args.name)
        self.args=args;self.lock=threading.Lock();self.latest=None;self.sequence=0
        self.last_sequence=0;self.closed=threading.Event();self.failure='';self.started=time.monotonic()
        eyes=['color'] if args.layout=='mono' else ['left','right']
        self.eyes=eyes
        width=args.width if args.layout=='mono' else args.width//2
        paths=[args.left_calibration] if args.layout=='mono' else [args.left_calibration,args.right_calibration]
        if args.layout=='side_by_side' and bool(paths[0])!=bool(paths[1]):
            raise ValueError('stereo_calibration_requires_both_eyes')
        self.calibrations=[calibration(p,width,args.height) for p in paths]
        self.frames=[args.name+'_'+eye+'_optical_frame' for eye in eyes]
        self.images=[self.create_publisher(Image,args.prefix+'/'+eye+'/image_raw',qos_profile_sensor_data) for eye in eyes]
        self.infos=[self.create_publisher(CameraInfo,args.prefix+'/'+eye+'/camera_info',qos_profile_sensor_data) for eye in eyes]
        self.report=self.create_publisher(String,args.prefix+'/status',1)
        device=Path(args.device).resolve()
        if not device.is_char_device():raise ValueError('camera_device_is_not_character_device')
        # Capture continuously in a dedicated thread, keeping one newest frame.
        # This bounds application backlog; timestamps below are host receive time.
        self.capture=cv2.VideoCapture(str(device),cv2.CAP_V4L2)
        if not self.capture.isOpened():raise RuntimeError('cannot_open_uvc_camera')
        self.capture.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,args.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT,args.height)
        self.capture.set(cv2.CAP_PROP_FPS,args.capture_fps)
        # V4L2 needs spare queued buffers while MJPEG is decoded. One buffer
        # dropped every other frame on both cameras in the target-machine test.
        # The reader still retains only one newest decoded frame for ROS.
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE,3)
        self.reader=threading.Thread(target=self.read,name='uvc-acquire',daemon=True)
        self.reader.start();self.create_timer(1/args.publish_fps,self.publish)
        self.create_timer(.5,self.status)

    def read(self):
        while not self.closed.is_set():
            ok,frame=self.capture.read();received=time.monotonic();stamp=self.get_clock().now().nanoseconds
            if not ok:
                self.failure='capture_failed';self.closed.set();return
            if frame.shape!=(self.args.height,self.args.width,3):
                self.failure='negotiated_shape_differs_from_requested';self.closed.set();return
            with self.lock:
                self.sequence+=1;self.latest=(self.sequence,received,stamp,frame)

    def publish(self):
        with self.lock:latest=self.latest
        if latest is None:return
        seq,received,stamp,frame=latest
        if seq==self.last_sequence or time.monotonic()-received>.2:return
        self.last_sequence=seq
        for i,image in enumerate(split_frame(frame,self.args.layout)):
            msg=Image();msg.header.stamp.sec=stamp//1000000000;msg.header.stamp.nanosec=stamp%1000000000
            msg.header.frame_id=self.frames[i];msg.height,msg.width=image.shape[:2]
            msg.encoding='bgr8';msg.is_bigendian=False;msg.step=msg.width*3;msg.data=image.tobytes()
            info=CameraInfo();info.header=msg.header;info.width=msg.width;info.height=msg.height
            # K[0]==0 is the ROS uncalibrated convention. Never invent a focal length.
            values=self.calibrations[i]
            if values:
                for field,value in values.items():setattr(info,field,value)
            self.images[i].publish(msg);self.infos[i].publish(info)

    def status(self):
        with self.lock:latest=self.latest
        age=None if latest is None else time.monotonic()-latest[1]
        body=dict(schema='wla-uvc-camera/v1',device=self.args.device,layout=self.args.layout,
            capture_width=self.args.width,capture_height=self.args.height,frame_sequence=self.sequence,
            frame_age_s=age,fresh=age is not None and age<.2 and not self.failure,
            timestamp_source='host_capture_receive_not_hardware_exposure',
            calibrated=all(self.calibrations),depth_available=False,mount_tf_published_by_this_node=False,
            reason=self.failure or ('ready_raw_only' if latest else 'waiting_for_capture'))
        self.report.publish(String(data=json.dumps(body,allow_nan=False)))
        if self.failure or (latest is None and time.monotonic()-self.started>10):
            raise RuntimeError(self.failure or 'camera_startup_timeout')

    def close(self):
        self.closed.set();self.reader.join(timeout=2.)
        if not self.reader.is_alive():self.capture.release()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device',required=True);p.add_argument('--name',required=True)
    p.add_argument('--prefix',required=True);p.add_argument('--layout',choices=['mono','side_by_side'],required=True)
    p.add_argument('--width',type=int,required=True);p.add_argument('--height',type=int,required=True)
    p.add_argument('--capture-fps',type=float,default=30.);p.add_argument('--publish-fps',type=float,default=30.)
    p.add_argument('--left-calibration',default='');p.add_argument('--right-calibration',default='')
    args=p.parse_args()
    if not(args.width>0 and args.height>0 and 0<args.publish_fps<=args.capture_fps<=120):p.error('invalid size/rate')
    if args.layout=='side_by_side' and args.width%2:p.error('side-by-side width must be even')
    if not args.name.replace('_','').isalnum() or not args.prefix.startswith('/'):p.error('invalid name/prefix')
    rclpy.init(args=[]);node=None
    try:node=Camera(args);rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.close();node.destroy_node()
        rclpy.try_shutdown()
