"""Actual C++ receiver over a PTY, isolated domain 211; no real device access."""
import argparse
import json
import math
import os
import pty
import struct
import subprocess
import tempfile
import time
import zlib

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def packet(sequence=1, boot=123, valid=True):
    h = struct.pack('<4sBBBBHH6IB3xQQQQ', b'TF4!', 2, 1, 8, 0, 460, 0,
                    boot, sequence, 9, 0, 0, 0, 147, 10000*sequence,
                    10000*sequence+100, 10000*sequence-100, 10000*sequence+200)
    body = struct.pack('<64h64H', *([1000]+[-1]*63), *([3]*64))
    body += bytes([5 if valid else 9]+[9]*63)+bytes([1]+[0]*63)
    data = h+body
    return data+struct.pack('<I', zlib.crc32(data))


def main():
    parser = argparse.ArgumentParser();parser.add_argument('--executable', required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '211', 'isolated ROS domain 211 required'
    directory = tempfile.TemporaryDirectory(prefix='wla-tof-pty-')
    master, slave = pty.openpty();path = directory.name+'/device'
    os.symlink(os.ttyname(slave),path)
    rclpy.init();node = Node('tof_pty_test')
    clouds = [];raw = [];statuses = []
    subscriptions = [
        node.create_subscription(PointCloud2, '/r680/tof/front_right/points_preview', clouds.append, qos_profile_sensor_data),
        node.create_subscription(String, '/r680/tof/front_right/raw', lambda m:raw.append(json.loads(m.data)), qos_profile_sensor_data),
        node.create_subscription(String, '/r680/tof/status', lambda m:statuses.append(json.loads(m.data)), 10)]
    proc = subprocess.Popen([args.executable, '--ros-args', '-p', 'device:='+path], stdout=subprocess.DEVNULL)
    def wait(condition, seconds=5):
        deadline = time.monotonic()+seconds
        while not condition() and time.monotonic()<deadline:
            assert proc.poll() is None, 'receiver exited'
            rclpy.spin_once(node, timeout_sec=.05)
        assert condition(), 'condition timed out'
    try:
        wait(lambda:bool(statuses))
        os.write(master, b'# HEALTH name=front_left mode=polling online=0 frames=0\r\n'+packet())
        wait(lambda:bool(clouds) and bool(raw))
        c = clouds[-1]
        assert (c.width,c.height,c.point_step,c.row_step)==(1,1,20,20)
        assert c.header.frame_id=='tof_front_right_preview_optical_frame'
        x,y,z,distance,sigma,zone = struct.unpack('<ffffHH', bytes(c.data))
        expected = math.tan(-.875*math.radians(45)/2)
        assert abs(x-expected)<1e-6 and abs(y-expected)<1e-6
        assert (z,distance,sigma,zone)==(1.,1.,3,0)
        assert raw[-1]['distance_mm'][1]==-1 and len(raw[-1]['status'])==64
        assert raw[-1]['timestamp_source']=='host_complete_frame_receive_not_exposure'
        os.write(master, packet(2,valid=False))
        wait(lambda:len(raw)>=2 and clouds[-1].width==0)
        os.write(master, packet(2))  # Duplicate sequence must not generate another cloud.
        bad = bytearray(packet(3));bad[100]^=1
        os.write(master, bytes(bad)+packet(4))
        wait(lambda:len(raw)>=3 and raw[-1]['sequence']==4)
        os.write(master, packet(1,boot=999))
        wait(lambda:len(raw)>=4 and raw[-1]['boot_id']==999)
        wait(lambda:statuses[-1]['channels'][1]['boot_changes']==1)
        channel=statuses[-1]['channels'][1]
        assert channel['duplicates_rejected']==1 and channel['sequence_gaps']==1
        assert statuses[-1]['crc_errors']==1 and not statuses[-1]['follow_input_enabled']
        assert 'online=0' in statuses[-1]['channels'][0]['firmware_health']
        count=len(clouds)
        wait(lambda:not statuses[-1]['channels'][1]['fresh'])
        assert len(clouds)==count and all(not c['fresh'] for c in statuses[-1]['channels'])
        os.close(master);master=-1
        os.close(slave);slave=-1
        wait(lambda:bool(statuses[-1]['serial_error']))
        master,slave = pty.openpty()
        os.symlink(os.ttyname(slave),path+'.new');os.replace(path+'.new',path)
        wait(lambda:not statuses[-1]['serial_error'])
        os.write(master,packet(1,boot=1000))
        wait(lambda:len(raw)>=5 and raw[-1]['boot_id']==1000)
        assert clouds[-1].width==1
        print(json.dumps(dict(checks='native receiver, CRC/recovery, optical projection, invalid unknown, duplicate, reboot, stale, health, USB-style reconnect, no production input', clouds=len(clouds),passed=True)))
    finally:
        proc.terminate();proc.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()
        if master>=0:os.close(master)
        if slave>=0:os.close(slave)
        directory.cleanup()
        del subscriptions


if __name__=='__main__':main()
