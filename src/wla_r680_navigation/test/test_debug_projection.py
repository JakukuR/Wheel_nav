"""Geometry and identity regressions; no ROS graph or hardware required."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from wla_r680_navigation_py.debug_projection import Camera, FrameCache, matrix, transform, voxel_edges, path_edges, relative_lookup


class DebugGeometryTest(unittest.TestCase):
    def setUp(self):
        self.cam = Camera(640, 480, [100, 0, 320, 0, 100, 240, 0, 0, 1], [0]*5, 'plumb_bob')

    def test_optical_projection_and_extrinsics(self):
        mat = matrix([0, 0, 0, 1], [1, 0, 0])
        self.assertEqual(self.cam.points(transform([[0, 0, 2]], mat)), [[370., 240.]])
        self.assertEqual(self.cam.points([[0, 0, -1], [float('nan'), 0, 1], [100, 0, 1]]), [])

    def test_distortion_and_fisheye(self):
        distorted = Camera(640, 480, self.cam.k, [.1, 0, 0, 0, 0], 'plumb_bob')
        self.assertAlmostEqual(distorted.project([[1, 0, 2]])[0, 0], 371.25)
        fish = Camera(640, 480, self.cam.k, [0]*4, 'equidistant')
        self.assertAlmostEqual(fish.project([[1, 0, 1]])[0, 0], 320+100*np.pi/4)
        with self.assertRaises(ValueError):
            Camera(640, 480, self.cam.k, [], 'unknown')

    def test_near_plane_and_image_clipping(self):
        seg = self.cam.segments([[[0, 0, -1], [1, 0, 2]], [[-10, 0, 1], [10, 0, 1]], [[0, 0, -2], [1, 0, -1]]])
        self.assertEqual(len(seg), 2)
        self.assertEqual(seg[1], [0, 240, 639, 240])
        self.assertTrue(all(0 <= x < 640 and 0 <= y < 480 for s in seg for x, y in zip(s[::2], s[1::2])))

    def test_voxel_uses_floor_not_sample_center(self):
        edges, count = voxel_edges([[-.01, .04, .25], [-.02, .03, .24]], .1)
        self.assertEqual(count, 1)
        np.testing.assert_allclose(edges.min(axis=(0, 1)), [-.1, 0, .2])
        np.testing.assert_allclose(edges.max(axis=(0, 1)), [0, .1, .3])
        self.assertEqual(edges.shape, (12, 2, 3))
        self.assertEqual(voxel_edges([[i, 0, 1] for i in range(1000)], .1, 20)[1], 20)

    def test_no_path_bridge_across_invalid_point(self):
        edges = path_edges([[0, 0, 1], [float('nan'), 0, 1], [2, 0, 1]])
        self.assertEqual(len(edges), 0)
        self.assertEqual(len(path_edges([[0, 0, 1], [1, 0, 1]])), 10)

    def test_frame_cache_exact_and_bounded(self):
        cache = FrameCache(2, 10)
        cache.put((1, 'rgb'), 'one', 6)
        cache.put((2, 'rgb'), 'two', 6)
        self.assertIsNone(cache.get((1, 'rgb')))
        self.assertIsNone(cache.get((2, 'other')))
        cache.put((2, 'rgb'), 'new', 3)
        self.assertEqual(cache.bytes, 3)
        cache.put((3, 'rgb'), 'large', 11)
        self.assertIsNone(cache.get((3, 'rgb')))

    def test_relative_identity_requires_same_frame_and_epoch(self):
        raw = {'session':'s', 'epoch':1, 'stamp_ns':100}
        rel = dict(raw, enabled=True, objects=[{'id':'s:1:42','detected':True,'distance':1}])
        self.assertEqual(relative_lookup(raw, rel)['42']['distance'], 1)
        for key, value in [('session','other'), ('epoch',2), ('stamp_ns',101), ('enabled',False)]:
            self.assertEqual(relative_lookup(raw, dict(rel, **{key:value})), {})


if __name__ == '__main__':
    unittest.main()
