#!/usr/bin/env python3
"""Hardware-input-only mapping workflow; all chassis output explicitly disabled."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

parser=argparse.ArgumentParser(); parser.add_argument('--frontend',choices=['cuvslam','rgbd'],required=True)
args=parser.parse_args()
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit(): continue
    try: executable=os.readlink(entry/'exe')
    except OSError: continue
    if executable.endswith(('/wheeltec_robot_node','/realsense2_camera_node')):
        raise SystemExit('An operator driver/camera is running; do not interrupt it')
root=Path.home()/'ros2_ws/.runtime/mapping_cuvslam_validation'/time.strftime('%Y%m%d-%H%M%S')
root.mkdir(parents=True)
storage=root/'storage.yaml'
storage.write_text(yaml.safe_dump(dict(run_root=str(root/'runs'), maps_root=str(root/'maps'), map_prefix='map',active_map='latest')))
prefix=Path(get_package_prefix('wla_r680_navigation'))/'lib/wla_r680_navigation'
log=(root/'workflow.log').open('w')
if args.frontend=='cuvslam':
    command=['bash',str(prefix/'r680_mapping.sh'),'--cuvslam','--auto-vio-init','--dry-run',
             '--no-rviz','--no-semantics','--storage-config',str(storage)]
    duration=60
else:
    run=subprocess.check_output([str(prefix/'prepare_mapping_run'),'--storage-config',str(storage)],text=True).strip()
    command=['ros2','launch','wla_r680_navigation','bringup.launch.py','mode:=mapping',
             'odom_source:=rgbd','start_d455:=true','start_chassis:=true','start_nav2:=true',
             'start_navigation_servers:=false','start_dynamic_obstacles:=false',
             'use_chassis_imu:=true','publish_mount_tf:=true','enable_hardware_output:=false',
             'start_web:=false','start_semantics:=false','database_path:='+str(Path(run)/'rtabmap.db')]
    duration=30
(root/'command.json').write_text(json.dumps(command,indent=2))
rclpy.init(); node=Node('mapping_hardware_preview_probe'); counts={}; true_counts={}; statuses={}; stamps={}
def observe(name):
    def cb(msg):
        counts[name]=counts.get(name,0)+1
        if isinstance(msg,Bool): true_counts[name]=true_counts.get(name,0)+int(msg.data)
        if isinstance(msg,String): statuses[msg.data]=statuses.get(msg.data,0)+1
        if hasattr(msg,'header'): stamps.setdefault(name,[]).append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    return cb
for name,typ in [('/r680_nav/chassis_cmd_vel',Twist),('/d455_slam/odom',Odometry),
                 ('/r680_nav/mapping_odom',Odometry),('/wheel/imu/data_raw',Imu),
                 ('/r680/d455/map',OccupancyGrid),('/r680_nav/vio_tracking_healthy',Bool),
                 ('/r680_nav/vio_init_state',String),('/r680_nav/mapping_odom_ready',Bool)]:
    node.create_subscription(typ,name,observe(name),qos_profile_sensor_data)
process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
print('Hardware output DISABLED:',root,flush=True)
try:
    deadline=time.monotonic()+duration
    while time.monotonic()<deadline and process.poll() is None: rclpy.spin_once(node,timeout_sec=.1)
    if args.frontend=='rgbd':
        saver=subprocess.run([str(prefix/'save_manual_map'),'--run-dir',run,'--timeout','30'],
                             stdout=log,stderr=subprocess.STDOUT,timeout=65)
        assert saver.returncode==0,'fallback map save failed: inspect workflow.log'
    rates={name:(len(values)-1)/(values[-1]-values[0]) for name,values in stamps.items()
           if len(values)>1 and values[-1]>values[0]}
    report=dict(frontend=args.frontend,hardware_output_enabled=False,counts=counts,
                rates_hz=rates,health_true_counts=true_counts,status_counts=statuses,
                raw_command_messages=counts.get('/r680_nav/chassis_cmd_vel',0))
    (root/'probe.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
    assert report['raw_command_messages']==0,'preview published actual chassis velocity'
    if args.frontend=='cuvslam' and true_counts.get('/r680_nav/vio_tracking_healthy',0)==0:
        assert counts.get('/r680_nav/mapping_odom',0)==0 and counts.get('/r680/d455/map',0)==0, 'provisional VIO entered mapping'
    if args.frontend=='rgbd':
        result=json.loads((Path(run)/'navigation_map_result.json').read_text()); saved=Path(result['directory'])
        assert all((saved/name).is_file() for name in ['map.pgm','map.yaml','rtabmap.db','navigation.yaml','map_info.json','semantic.geojson'])
        print('Real RTAB backup and compact navigation archive PASS:',saved,flush=True)
finally:
    node.destroy_node(); rclpy.shutdown()
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try: process.wait(timeout=65)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM); process.wait(timeout=10)
    log.close()
    print('Owned workflow stopped:',root,flush=True)
