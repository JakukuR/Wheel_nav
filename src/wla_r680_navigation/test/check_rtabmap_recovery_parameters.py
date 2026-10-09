#!/usr/bin/env python3
"""Actual installed RTAB-Map parameter API; temporary DB, no hardware inputs."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '175'
    keys=['RGBD/ProximityBySpace','RGBD/LocalRadius','RGBD/ProximityAngle',
          'RGBD/ProximityMaxPaths','Rtabmap/LoopThr','RGBD/AggressiveLoopThr',
          'Reg/Strategy','Vis/EstimationType']
    rclpy.init(); node=rclpy.create_node('recovery_parameter_probe')
    with tempfile.TemporaryDirectory(prefix='wla-rtab-recovery-') as temporary:
        log=Path(temporary)/'rtab.log'
        with log.open('w') as out:
            process=subprocess.Popen([str(Path(get_package_prefix('rtabmap_slam'))/'lib/rtabmap_slam/rtabmap'),
                '--ros-args','-r','__ns:=/recovery_api_test','-r','__node:=rtabmap',
                '-p','database_path:='+str(Path(temporary)/'test.db'),
                '-p','Mem/IncrementalMemory:="false"'],stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
        client=AsyncParameterClient(node,'/recovery_api_test/rtabmap')
        def result(future):
            rclpy.spin_until_future_complete(node,future,timeout_sec=8)
            assert future.done(), 'RTAB parameter service timeout\n'+log.read_text()[-5000:]
            return future.result()
        try:
            assert client.wait_for_services(timeout_sec=15), log.read_text()[-3000:]
            original=result(client.get_parameters(keys)).values
            assert all(v.type==Parameter.Type.STRING.value for v in original)
            values=['true','1','90','3','1','1','0','0']
            reply=result(client.set_parameters([Parameter(k,value=v) for k,v in zip(keys,values)]))
            assert all(r.successful for r in reply.results)
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=0.05)
            current=result(client.get_parameters(keys)).values
            assert [v.string_value for v in current]==values
            text=log.read_text()
            assert 'Setting RTAB-Map parameter "Vis/EstimationType"="0"' in text, text[-3000:]
            reply=result(client.set_parameters([Parameter(k,value=v.string_value) for k,v in zip(keys,original)]))
            assert all(r.successful for r in reply.results)
            print('PASS: installed RTAB 0.22 parameter changes reached core, RGB-D registration switch and baseline restoration')
        finally:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
