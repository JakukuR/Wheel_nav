"""terrain_odometry on a simulated 20 deg climb: IMU gravity tilted nose-up,
wheels at 0.5 m/s for 4 s after the 2 s bias window -> z rises v*sin(20)*t.

    ROS_DOMAIN_ID=<free> python3 test_terrain_odometry_ros.py <terrain_odometry executable>
"""
import math, subprocess, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def main(exe):
    rclpy.init(); node = rclpy.create_node('odom_fixture')
    proc = subprocess.Popen([exe])
    try:
        imu_pub = node.create_publisher(Imu, '/wheel/imu/data_raw', qos_profile_sensor_data)
        wheel_pub = node.create_publisher(Odometry, '/wheel/odom_50hz', qos_profile_sensor_data)
        got = []
        node.create_subscription(Odometry, '/r680_nav/terrain_odom', lambda m: got.append(m), 50)
        time.sleep(1.)
        pitch = -math.radians(20.)  # nose up (REP-103)
        start = time.time(); next_wheel = start
        while time.time() - start < 6.5:
            now = node.get_clock().now().to_msg()
            imu = Imu(); imu.header.stamp = now; imu.header.frame_id = 'gyro_link'
            # specific force of gravity in the body frame for pitch p: (-g sin p, 0, g cos p)
            imu.linear_acceleration.x = -9.81 * math.sin(pitch); imu.linear_acceleration.z = 9.81 * math.cos(pitch)
            imu_pub.publish(imu)
            if time.time() >= next_wheel:
                w = Odometry(); w.header.stamp = now; w.child_frame_id = 'r680_mapping_floor'
                w.twist.twist.linear.x = .5 if time.time() - start > 2.5 else 0.
                wheel_pub.publish(w); next_wheel += .02
            rclpy.spin_once(node, timeout_sec=.004)
        last = got[-1]; p = last.pose.pose.position; q = last.pose.pose.orientation
        est_pitch = math.asin(max(-1., min(1., 2 * (q.w * q.y - q.z * q.x))))
        print(f'odom samples {len(got)}: x {p.x:.3f} z {p.z:.3f} pitch {math.degrees(est_pitch):.1f} deg; '
              f'expected z/x = tan(20) = {math.tan(math.radians(20)):.3f}, got {p.z / max(p.x, 1e-6):.3f}')
        assert abs(math.degrees(est_pitch) + 20.) < 1.5 and abs(p.z / p.x - math.tan(math.radians(20))) < .03
        assert last.pose.covariance[21] < 1.
        print('terrain odometry ROS check passed')
    finally:
        proc.terminate(); proc.wait(timeout=5); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1])
