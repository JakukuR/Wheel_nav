"""Read-only launch-plan tests for the Kalo debug entry; no hardware start."""
import argparse
import importlib.machinery
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest import mock
import tempfile
import fcntl

base=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(base))
loader=importlib.machinery.SourceFileLoader('standalone',str(base/'scripts/perception_debug_standalone'))
spec=importlib.util.spec_from_loader(loader.name,loader);standalone=importlib.util.module_from_spec(spec);loader.exec_module(standalone)

ARGS=dict(python='/usr/bin/python3',model='m.pt',tracker='t.yaml',device='0',port=18081,
          classes_file='classes.json',text_encoder='text.ts',with_points=False)
NOTHING=dict(camera=True,mount=False,detector=False,points=False,jpeg=False)


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

    def test_launch_plan_is_read_only_and_starts_the_kalo_page(self):
        plan=standalone.commands(argparse.Namespace(**ARGS),NOTHING,False,False)
        self.assertEqual([x[0] for x in plan],['mount','detector','bridge','jpeg','web'])
        text=' '.join(' '.join(row[1]) for row in plan)
        for bad in ['nav2','bringup.launch.py','r680_nav.sh','command_guard','wheeltec_robot','enable_motion:=true',
                    'realsense2_camera','nav_perception_debug','depth_to_points']:
            self.assertNotIn(bad,text)
        self.assertIn('config/kalo_bridge.yaml',text)
        self.assertIn('kalo_jpeg',text)
        self.assertIn('kalo_debug_web --port 18081 --bridge-port 8765',text)
        self.assertIn('--classes-file classes.json',text)
        self.assertIn('--text-encoder text.ts',text)

    def test_camera_is_never_started_here(self):
        with self.assertRaisesRegex(RuntimeError,'r680_camera.sh run'):
            standalone.commands(argparse.Namespace(**ARGS),dict(NOTHING,camera=False),False,False)

    def test_depth_points_only_on_request_and_running_parts_are_reused(self):
        args=argparse.Namespace(**dict(ARGS,with_points=True))
        self.assertIn('local_obstacles',[x[0] for x in standalone.commands(args,NOTHING,False,False)])
        running=dict(camera=True,mount=True,detector=True,points=True,jpeg=True)
        self.assertEqual(standalone.commands(args,running,True,True),[])


if __name__=='__main__':unittest.main()
