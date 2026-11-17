"""Keep every D455 launch/VIO entry on the same chassis mounting transform."""
import ast
import math
from pathlib import Path
import re
import unittest

WS = Path(__file__).resolve().parents[3]
SRC = WS / 'src'


def launch_pose(path):
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.List):
            values = [item.value if isinstance(item, ast.Constant) else None for item in node.elts]
            if '--child-frame-id' in values and 'd455_link' in values:
                return [float(values[values.index('--' + key) + 1])
                        for key in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')]
    raise AssertionError('D455 mount argument list missing: ' + str(path))


class CameraMountConfiguration(unittest.TestCase):
    def test_chassis_mount_forward_axis_is_up_fifteen_degrees(self):
        pose = launch_pose(SRC / 'wla_r680_navigation/launch/d455_camera.launch.py')
        x, y, z, w = pose[3:]
        forward = [1 - 2*(y*y + z*z), 2*(x*y + w*z), 2*(x*z - w*y)]
        self.assertAlmostEqual(sum(v*v for v in pose[3:]), 1., places=12)
        self.assertAlmostEqual(forward[0], math.cos(math.radians(15)), places=12)
        self.assertAlmostEqual(forward[1], 0., places=12)
        self.assertAlmostEqual(forward[2], math.sin(math.radians(15)), places=12)
        for actual, expected in zip(pose[:3], [.184428484, .059803590, .501660007]):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_all_launch_and_vio_mount_copies_agree(self):
        reference = launch_pose(SRC / 'wla_r680_navigation/launch/d455_camera.launch.py')
        for relative in ['wla_r680_navigation/launch/bringup.launch.py',
                         'wla_r680_navigation/scripts/perception_debug_standalone']:
            with self.subTest(path=relative):
                self.assertEqual(launch_pose(SRC / relative), reference)
        paths = ['wla_r680_navigation/config/cuvslam.yaml',
                 'wla_cuvslam_navigation/config/cuvslam.yaml',
                 'wla_cuvslam_validation/config/validation.yaml',
                 'wla_cuvslam_navigation/src/cuvslam_odometry.cpp',
                 'wla_cuvslam_validation/src/cuvslam_validation.cpp']
        for relative in paths:
            with self.subTest(path=relative):
                tail = (SRC / relative).read_text().split('base_from_camera_link', 1)[1]
                match = re.search(r'[\[{]([^\]}]+)[\]}]', tail)
                values = [float(v.strip()) for v in match.group(1).split(',')]
                self.assertEqual(values, reference)


if __name__ == '__main__':
    unittest.main()
