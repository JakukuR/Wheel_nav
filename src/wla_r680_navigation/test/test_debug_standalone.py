"""Read-only launch-plan and local projection policy tests; no hardware start."""
import argparse
import importlib.machinery
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest import mock
import tempfile
import fcntl
from types import SimpleNamespace

base=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(base))
loader=importlib.machinery.SourceFileLoader('standalone',str(base/'scripts/perception_debug_standalone'))
spec=importlib.util.spec_from_loader(loader.name,loader);standalone=importlib.util.module_from_spec(spec);loader.exec_module(standalone)
from wla_r680_navigation_py.debug_viewer import DebugViewer
from sensor_msgs.msg import PointCloud2, Image


class StandaloneTest(unittest.TestCase):
    def test_second_launcher_cannot_race_camera_ownership(self):
        with tempfile.TemporaryDirectory() as folder:
            runtime=Path(folder)/'.runtime';runtime.mkdir()
            with (runtime/'perception-debug-197.lock').open('a') as lock:
                fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
                with mock.patch.dict('os.environ',{'WLA_ROS_WS':folder,'ROS_DOMAIN_ID':'197'}), \
                     mock.patch.object(sys,'argv',['standalone','--python','python','--model','m','--tracker','t']), \
                     mock.patch.object(standalone,'discover') as discover:
                    self.assertEqual(standalone.main(),2)
                    discover.assert_not_called()

    def test_launch_plan_contains_no_navigation_or_actuation(self):
        args=argparse.Namespace(python='/usr/bin/python3',model='m.pt',tracker='t.yaml',device='0',port=18081,
                                classes_file='classes.json',text_encoder='text.ts')
        plan=standalone.commands(args,dict(camera=False,mount=False,detector=False,points=False),False)
        self.assertEqual([x[0] for x in plan],['camera','mount','detector','local_obstacles','viewer'])
        text=' '.join(' '.join(row[1]) for row in plan)
        for bad in ['nav2','bringup.launch.py','r680_nav.sh','command_guard','wheeltec_robot','enable_motion:=true']:
            self.assertNotIn(bad,text)
        self.assertIn('/r680/perception/local_obstacles',text)
        self.assertIn('--classes-file classes.json',text)
        self.assertIn('--text-encoder text.ts',text)
        self.assertEqual(standalone.commands(args,dict(camera=True,mount=True,detector=True,points=True),True),[])

    def test_local_projection_rejects_world_frame_and_time_skew(self):
        fake=SimpleNamespace(args=SimpleNamespace(mode='local',base_frame='r680_mapping_floor'))
        msg=PointCloud2();msg.header.frame_id='map';msg.header.stamp.sec=10
        img=Image();img.header.frame_id='d455_color_optical_frame';img.header.stamp.sec=10
        with self.assertRaisesRegex(ValueError,'requires_robot_or_camera_frame'):
            DebugViewer.source_transform(fake,msg,img)
        msg.header.frame_id='r680_mapping_floor';msg.header.stamp.nanosec=31_000_000
        with self.assertRaisesRegex(ValueError,'skew_over_30ms'):
            DebugViewer.source_transform(fake,msg,img)


if __name__=='__main__':unittest.main()
