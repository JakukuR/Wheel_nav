"""Read-only LAS2 adapter for the follower's existing native cloud front-end.

No alternative ground algorithm, odometry, map, target selection or control.
The ground reference is the existing joint/IMU support geometry. A dedicated
moving preview frame keeps this provisional reference out of the navigation TF.
"""
import json
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data as QOS
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String, Header
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from scipy.spatial.transform import Rotation
from _wla_sim_geometry import deproject, filter_points
from .camera import depth_meters
from .depth_proc import rotation_from_quaternion

FRAME = 'follow_support_floor_preview'
TOPIC = '/r680/person_follow/depth_obstacles'

def stamp_ns(h):
    return h.stamp.sec * 10**9 + h.stamp.nanosec

def support_transform(g, depth_time, now, rect_rotation):
    if g.get('schema') != 'robot-geometry/v1' or g.get('frame') != 'base_Link' or not g.get('available'):
        raise ValueError('support_unavailable')
    floor = g['ground']
    if not floor['available'] or floor['reason'] != 'operator_confirmed_both_contact_flat_ground':
        raise ValueError('support_contact_assumption_unavailable')
    jt, it = float(g['joint_stamp']), float(g['imu_stamp'])
    if not all(np.isfinite([jt,it])) or not all(-.02 <= now-t <= .2 for t in [jt,it]) or max(abs(jt-depth_time),abs(it-depth_time)) > .075:
        raise ValueError('support_time_mismatch')
    n=np.asarray(floor['normal'],dtype=float);h=float(floor['offset_m']);heights=np.asarray(floor['wheel_heights_m'],dtype=float)
    if n.shape!=(3,) or not np.isfinite(n).all() or abs(np.linalg.norm(n)-1)>.01 or not np.isfinite(h) or not 0<h<1 or heights.shape!=(2,) or not np.isfinite(heights).all() or abs(heights[0]-heights[1])>.05:
        raise ValueError('invalid_support_geometry')
    n=n/np.linalg.norm(n);x=np.array([1.,0.,0.])-n*n[0]
    if np.linalg.norm(x)<.9:raise ValueError('unsupported_heading')
    x/=np.linalg.norm(x);basis=np.array([x,np.cross(n,x),n])
    mount=next(f for f in g['frames'] if f['name']=='bottom_camera_L')
    t=np.asarray(mount['p'],dtype=float);q=np.asarray(mount['q'],dtype=float)
    if t.shape!=(3,) or not np.isfinite(t).all() or np.linalg.norm(t)>.3 or q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.01:
        raise ValueError('invalid_camera_mount')
    body_camera=rotation_from_quaternion(q)@np.array([[0,0,1],[-1,0,0],[0,-1,0]])@rect_rotation
    return basis@body_camera, basis@t+np.array([0,0,h]), basis, n, h

class FrontendPreview(Node):
    def __init__(self):
        super().__init__('follow_cloud_frontend_preview')
        self.depth=None;self.info=None;self.support=deque(maxlen=30);self.last_stamp=0
        self.tf=Buffer();self.listener=TransformListener(self.tf,self);self.broadcaster=TransformBroadcaster(self)
        self.cloud=self.create_publisher(PointCloud2,TOPIC,QOS)
        self.clearing=self.create_publisher(PointCloud2,'/r680/person_follow/depth_clearing_preview',QOS)
        self.status=self.create_publisher(String,'/r680/person_follow/frontend_status',QOS)
        self.subs=[self.create_subscription(Image,'/camera/stereo/depth/image_raw',lambda m:setattr(self,'depth',m),QOS),self.create_subscription(CameraInfo,'/camera/stereo/depth/camera_info',lambda m:setattr(self,'info',m),QOS),self.create_subscription(String,'/robot_state/geometry',self.geometry,QOS)]
        self.timer=self.create_timer(.1,self.tick)
    def geometry(self,m):
        try:
            g=json.loads(m.data)
            if not g.get('available'):self.support.clear()
            else:self.support.append(g)
        except (ValueError,TypeError):self.support.clear()
    def tick(self):
        # This preview must never compete with the full follower on its output.
        if self.count_publishers(TOPIC)>1:
            self.get_logger().error('Full follower/another publisher owns obstacle topic; preview exits')
            raise RuntimeError('preview_output_conflict')
        if not self.cloud.get_subscription_count() and not self.status.get_subscription_count() and not self.clearing.get_subscription_count():return
        started=time.monotonic();d=self.depth;k=self.info
        out={'schema':'follow-frontend-preview/v1','available':False,'backend':'_wla_sim_geometry.deproject/filter_points','motion_enabled':False,'person_removal_active':False,'frame':FRAME,'geometry_source':'existing_joint_imu_support_and_CAD_optical_axis_convention'}
        try:
            if d is None or k is None or not self.support:raise ValueError('waiting_for_inputs')
            ns=stamp_ns(d.header);t=ns/1e9;now=self.get_clock().now().nanoseconds/1e9
            if not -.02<=now-t<=.3:raise ValueError('stale_depth')
            if d.header.frame_id!='sunplus_left_rect_optical_frame' or k.header.frame_id!=d.header.frame_id or (k.width,k.height)!=(d.width,d.height) or abs(stamp_ns(k.header)-ns)>100_000_000:raise ValueError('depth_info_mismatch')
            if d.encoding!='16UC1' or d.is_bigendian or not 0<d.width*d.height<=2_000_000 or d.step!=d.width*2 or len(d.data)!=d.step*d.height or not np.isfinite(k.k).all() or k.k[0]<=0 or k.k[4]<=0:raise ValueError('depth_layout_or_intrinsics')
            if ns==self.last_stamp:return
            g=min(self.support,key=lambda g:max(abs(g.get('joint_stamp',0)-t),abs(g.get('imu_stamp',0)-t)))
            tr=self.tf.lookup_transform('sunplus_left_optical_frame',d.header.frame_id,Time(nanoseconds=ns)).transform
            if abs(tr.translation.x)+abs(tr.translation.y)+abs(tr.translation.z)>1e-6:raise ValueError('rect_translation')
            q=tr.rotation;rr=rotation_from_quaternion([q.x,q.y,q.z,q.w])
            r,trans,basis,n,h=support_transform(g,t,now,rr)
            # Exact functions used by sim_runtime.packet_from_depth. No new filter.
            points=deproject(depth_meters(d),np.array([k.k[0],k.k[4],k.k[2],k.k[5]]),r,trans,.15,5.,4)
            obstacles,clearing=filter_points(points)
            if (self.get_clock().now().nanoseconds-ns)/1e9>.3:raise ValueError('processing_expired')
            header=Header(stamp=d.header.stamp,frame_id=FRAME)
            tf=TransformStamped();tf.header=Header(stamp=d.header.stamp,frame_id='base_Link');tf.child_frame_id=FRAME
            xyz=-n*h;qq=Rotation.from_matrix(basis.T).as_quat()
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,xyz)
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,qq)
            self.broadcaster.sendTransform(tf)
            self.cloud.publish(create_cloud_xyz32(header,obstacles.astype(np.float32)))
            if self.clearing.get_subscription_count():self.clearing.publish(create_cloud_xyz32(header,clearing.astype(np.float32)))
            self.last_stamp=ns
            out.update(available=True,stamp_ns=ns,input_points=len(points),obstacle_points=len(obstacles),clearing_points=len(clearing),compute_ms=(time.monotonic()-started)*1000,support_height_m=h,reason='original_follow_frontend')
        except Exception as e:
            header=Header(stamp=self.get_clock().now().to_msg(),frame_id=FRAME)
            self.cloud.publish(create_cloud_xyz32(header,np.empty((0,3),np.float32)))
            out.update(reason=str(e)[:160])
        self.status.publish(String(data=json.dumps(out)))

def main():
    rclpy.init();node=FrontendPreview()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
