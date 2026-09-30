#!/usr/bin/env python3
"""Bounded hardware-input test; explicitly disables hardware output and sends no goals."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, Imu
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory, get_package_prefix

parser = argparse.ArgumentParser()
parser.add_argument('--frontend', choices=['rgbd', 'cuvslam'], default='cuvslam')
parser.add_argument('--auto-init', action='store_true')
parser.add_argument('--own-chassis', action='store_true', help='read chassis sensors with a private remapped driver; hardware output remains disabled')
args = parser.parse_args()
if args.auto_init and args.frontend != 'cuvslam':
    parser.error('--auto-init requires --frontend cuvslam')
if args.frontend == 'cuvslam':
    get_package_prefix('wla_cuvslam_navigation')  # Resolve overlay before starting any camera/driver.
if args.own_chassis:
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit(): continue
        try: target = os.readlink(entry / 'exe')
        except OSError: continue
        if target.endswith('/wheeltec_robot_node'):
            parser.error('an operator chassis driver is already running; omit --own-chassis')
root = Path.home() / 'nav_run' / (args.frontend + '-nav-preview-' + time.strftime('%Y%m%d-%H%M%S'))
root.mkdir(parents=True)
database = root / 'rtabmap.db'
subprocess.run(['cp', '--reflink=auto', str(Path.home() / 'ros2_ws/maps/map-2026-09-29-1/rtabmap.db'), str(database)], check=True)
config = Path(get_package_share_directory('wla_r680_navigation')) / 'config'
command = ['ros2', 'launch', 'wla_r680_navigation', 'bringup.launch.py',
           'mode:=localization', 'odom_source:=' + args.frontend, 'start_d455:=true',
           'start_chassis:=' + str(args.own_chassis).lower(), 'start_nav2:=true', 'start_navigation_servers:=true',
           'obstacle_scan:=true', 'start_dynamic_obstacles:=false', 'start_web:=false',
           'use_chassis_imu:=true', 'publish_mount_tf:=true', 'enable_hardware_output:=false',
           'chassis_imu_topic:=' + ('/wheel/imu/data_raw' if args.own_chassis else '/imu/data_raw'),
           'chassis_odom_topic:=' + ('/wheel/odom' if args.own_chassis else '/odom'),
           'database_path:=' + str(database), 'nav_params_file:=' + str(config / 'nav2_test.yaml'),
           'cuvslam_statistics_path:=' + str(root / 'cuvslam_statistics.json')]
if args.auto_init:
    command += ['auto_vio_init:=true', 'vio_init_result_path:=' + str(root / 'vio_init.json')]
(root / 'command.json').write_text(json.dumps(command, indent=2))
log = (root / 'bringup.log').open('w')
process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
rclpy.init()
node = Node('cuvslam_navigation_preview_probe')
counts, stamps, frames, states, statuses = {}, {}, {}, {}, {}
health_true = {}
tf = Buffer()
listener = TransformListener(tf, node)

def observe(name):
    def callback(msg):
        counts[name] = counts.get(name, 0) + 1
        if hasattr(msg, 'header'):
            stamps.setdefault(name, []).append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            frames[name] = msg.header.frame_id
        if hasattr(msg, 'data') and isinstance(msg.data, bool):
            health_true[name] = health_true.get(name, 0) + int(msg.data)
        if isinstance(msg, String):
            statuses[msg.data] = statuses.get(msg.data, 0) + 1
    return callback

for name, typ in [('/d455_slam/odom', Odometry), ('/r680_nav/d455/scan', LaserScan),
                  ('/r680/d455/infra1/image_rect_raw', Image), ('/r680/d455/infra2/image_rect_raw', Image),
                  ('/wheel/imu/data_raw' if args.own_chassis else '/imu/data_raw', Imu), ('/r680/d455/map', OccupancyGrid),
                  ('/r680_nav/vio_tracking_healthy', Bool), ('/r680_nav/localization_ready', Bool),
                  ('/r680_nav/vio_status', String), ('/r680_nav/vo_watchdog_status', String),
                  ('/r680_nav/vio_init_state', String), ('/r680_nav/chassis_cmd_vel', Twist)]:
    node.create_subscription(typ, name, observe(name), qos_profile_sensor_data)
names = ['controller_server', 'planner_server', 'bt_navigator', 'velocity_smoother', 'collision_monitor']
clients = {n: node.create_client(GetState, '/' + n + '/get_state') for n in names}
try:
    print('Preview input test started:', root, flush=True)
    deadline = time.monotonic() + (55 if args.auto_init else 45)
    while time.monotonic() < deadline and process.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    for name, client in clients.items():
        if client.service_is_ready():
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=1)
            if future.done() and future.result(): states[name] = future.result().current_state.id
    rates = {name: (len(values)-1)/(values[-1]-values[0]) for name, values in stamps.items()
             if len(values) > 1 and values[-1] > values[0]}
    snapshot = {'counts': counts, 'rates_hz': rates, 'frames': frames, 'lifecycle': states,
                'health_true_counts': health_true, 'status_counts': statuses,
                'map_to_base': tf.can_transform('map', 'r680_mapping_floor', rclpy.time.Time()),
                'odom_to_base': tf.can_transform('d455_floor_odom', 'r680_mapping_floor', rclpy.time.Time()),
                'chassis_command_subscribers': node.count_subscribers('/r680_nav/chassis_cmd_vel'),
                'hardware_output_enabled': False, 'operator_driver_reused': not args.own_chassis,
                'frontend': args.frontend,
                'auto_init': args.auto_init,
                'nav_command_publishers': [i.node_name for i in node.get_publishers_info_by_topic('/cmd_vel_nav')],
                'nav_command_input_publishers': [i.node_name for i in node.get_publishers_info_by_topic('/r680_nav/nav_command_input')],
                'nodes': node.get_node_names_and_namespaces()}
    (root / 'probe.json').write_text(json.dumps(snapshot, indent=2))
    print(json.dumps(snapshot, indent=2), flush=True)
    assert counts.get('/r680_nav/chassis_cmd_vel', 0) == 0, 'preview emitted hardware commands'
    if args.auto_init:
        assert snapshot['nav_command_publishers'] == ['r680_vio_initializer'], 'initialization mux is not sole command publisher'
        assert counts.get('/r680_nav/vio_init_state', 0) > 0, 'initializer did not run'
finally:
    node.destroy_node()
    rclpy.shutdown()
    try: os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError: pass
    try: process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        try: os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError: pass
        process.wait()
    log.close()
    print('Owned navigation/camera/chassis nodes stopped; any operator driver left untouched:', root, flush=True)
