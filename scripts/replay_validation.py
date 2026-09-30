#!/usr/bin/env python3
"""Replay recorded sensors in an isolated ROS domain; starts no hardware nodes."""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--rate', type=float, default=.5)
    args = parser.parse_args()
    if not (args.bag/'metadata.yaml').is_file(): parser.error('bag metadata.yaml missing')
    if not 1 <= args.seconds <= 600 or not 0 < args.rate <= 1: parser.error('invalid replay duration/rate')
    # Keep prerecorded TF, odometry and camera messages away from the live robot graph.
    os.environ['ROS_DOMAIN_ID'] = '74'
    prefix = Path(get_package_prefix('wla_cuvslam_validation'))
    config = Path(get_package_share_directory('wla_cuvslam_validation'))/'config/validation.yaml'
    root = args.bag.parent/('replay-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    root.mkdir()
    original = json.loads((args.bag.parent/'run.json').read_text())
    topic_overrides = [arg for arg in original['command']
                       if arg.startswith(('imu_topic:=', 'wheel_odom_topic:='))]
    for use_imu in [False, True]:
        run = root/('inertial' if use_imu else 'stereo');run.mkdir()
        cmd = [str(prefix/'lib/wla_cuvslam_validation/cuvslam_validation'),'--ros-args',
            '--params-file',str(config),'-p','use_sim_time:=true',
            '-p',f'use_imu:={str(use_imu).lower()}', '-p','async_sba:=false',
            '-p',f'statistics_path:={run/"statistics.json"}']
        for value in topic_overrides: cmd += ['-p',value]
        play = ['ros2','bag','play',str(args.bag),'--clock','200','--rate',str(args.rate),
                '--playback-duration',str(args.seconds),'--disable-keyboard-controls']
        (run/'run.json').write_text(json.dumps({'tracker':cmd,'player':play,'ros_domain_id':74,
                                               'hardware_started':False},indent=2))
        with (run/'tracker.log').open('w') as log, (run/'player.log').open('w') as plog:
            child = subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
            player = None
            try:
                time.sleep(1.5)
                player = subprocess.Popen(play,stdout=plog,stderr=subprocess.STDOUT)
                player.wait(timeout=args.seconds/args.rate+20)
                if player.returncode: raise RuntimeError('bag playback failed; see player.log')
                time.sleep(.5)
            finally:
                for process in [player,child]:
                    if process is not None and process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try: process.wait(timeout=8)
                        except subprocess.TimeoutExpired: process.kill();process.wait()
        print(run,flush=True)
        stats = json.loads((run/'statistics.json').read_text())
        print(json.dumps(stats),flush=True)
    print(f'Comparison retained: {root}',flush=True)

if __name__ == '__main__': main()
