"""ROS check of terrain_analysis wiring: rendered D455 depth of a 20 deg ramp and a
box, the car's camera mount TF, level odometry -> obstacle cloud.

    ROS_DOMAIN_ID=<free> python3 test_terrain_node_ros.py <path to terrain_analysis executable>
"""
import subprocess, sys, time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from tf2_ros import StaticTransformBroadcaster

FX = 385.
# Car mount (tf2_echo r680_mapping_floor -> d455_depth_optical_frame, 2026-10-29).
T_MOUNT = np.array([0.184, 0.060, 0.502])
Q_MOUNT = (-0.537, 0.537, -0.460, 0.460)


def quat_matrix(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w); x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def height(x, y):
    g = np.tan(np.radians(20.))
    h = np.where(x > 2.5, (x - 2.5) * g, 0.)
    box = (x > 1.6) & (x < 2.0) & (np.abs(y + .6) < .2)
    return np.where(box, .25, h)


def render():
    r = quat_matrix(*Q_MOUNT)
    v, u = np.mgrid[0:480, 0:640]
    rays = np.stack(((u - 320.) / FX, (v - 240.) / FX, np.ones_like(u, float)), -1) @ r.T  # base frame dirs per unit optical z
    depth = np.zeros((480, 640), np.uint16)
    for s in np.arange(.15, 5., .005):
        p = T_MOUNT + rays * s
        hit = (p[..., 2] <= height(p[..., 0], p[..., 1])) & (depth == 0)
        depth[hit] = np.uint16(s * 1000)
    return depth


def main(exe):
    rclpy.init()
    node = rclpy.create_node('terrain_fixture')
    proc = subprocess.Popen([exe, '--ros-args', '-p', 'max_rate:=30.0'])
    try:
        tfb = StaticTransformBroadcaster(node)
        t = TransformStamped(); t.header.frame_id = 'r680_mapping_floor'; t.child_frame_id = 'cam_opt'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = T_MOUNT
        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = Q_MOUNT
        tfb.sendTransform(t)
        depth_pub = node.create_publisher(Image, '/r680/d455/aligned_depth_to_color/image_raw', qos_profile_sensor_data)
        info_pub = node.create_publisher(CameraInfo, '/r680/d455/color/camera_info', qos_profile_sensor_data)
        odom_pub = node.create_publisher(Odometry, '/r680_nav/terrain_odom', 20)
        got = []
        node.create_subscription(PointCloud2, '/r680_nav/d455/points', lambda m: got.append(m), 10)
        depth = render()
        end = time.time() + 12.
        while time.time() < end and len(got) < 6:
            stamp = node.get_clock().now().to_msg()
            info = CameraInfo(); info.header.stamp = stamp; info.header.frame_id = 'cam_opt'
            info.width, info.height = 640, 480; info.k = [FX, 0., 320., 0., FX, 240., 0., 0., 1.]
            img = Image(); img.header = info.header; img.width, img.height = 640, 480
            img.encoding = '16UC1'; img.step = 1280; img.data = depth.tobytes()
            odom = Odometry(); odom.header.stamp = stamp; odom.header.frame_id = 'terrain_odom'
            odom.child_frame_id = 'r680_mapping_floor'; odom.pose.pose.orientation.w = 1.
            odom_pub.publish(odom); info_pub.publish(info); depth_pub.publish(img)
            rclpy.spin_once(node, timeout_sec=.1)
        assert len(got) >= 3, 'no obstacle clouds published'
        pts = read_points_numpy(got[-1], field_names=('x', 'y', 'z'))
        box = pts[(pts[:, 0] > 1.55) & (pts[:, 0] < 2.05) & (np.abs(pts[:, 1] + .6) < .25)]
        ramp = pts[(pts[:, 0] > 2.1) & (pts[:, 0] < 4.6)]  # flat-to-ramp kink and the ramp up to the range limit
        other = pts[(pts[:, 0] < 1.45)]  # the cell right in front of the box may be a step edge
        print(f'obstacle points: total {len(pts)} box {len(box)} on ramp {len(ramp)} before box {len(other)}; '
              f'box height above ground median {np.median(box[:, 2]) if len(box) else float("nan"):.3f}')
        # Cells touching the box are its step edge (raised surface): allow 0.25 m round it;
        # two cells of the box's shadow on the ramp may also be judged (<= 3 points).
        grown = (pts[:, 0] >= 1.35) & (pts[:, 0] <= 2.25) & (np.abs(pts[:, 1] + .6) <= .45)
        print(f'outside the box grown by 0.25 m: {int(np.count_nonzero(~grown))}')
        assert len(box) >= 10 and np.count_nonzero(~grown) <= 3, 'wrong obstacle classification'
        assert np.all((pts[:, 2] >= .05) & (pts[:, 2] < .5)), 'z is not height above local ground'
        print('terrain node ROS check passed')
    finally:
        proc.terminate(); proc.wait(timeout=5)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1])
