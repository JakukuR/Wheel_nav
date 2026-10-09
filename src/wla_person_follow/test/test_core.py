import unittest
import numpy as np

from wla_person_follow.controller import FollowController
from wla_person_follow.costmap import LocalCostmap
from wla_person_follow.target_lock import Observation, TargetLock


def observation(track_id, x, descriptor):
    return Observation(track_id, (0., 0., 100., 200.), np.asarray(descriptor, dtype=float),
                       np.array([x, 0.]), np.array([x, 0.]), .9)


class IdentityTests(unittest.TestCase):
    def test_never_auto_selects(self):
        lock = TargetLock()
        self.assertEqual(lock.update([observation(1, 2., [1., 0.])], 0.)[1], 'idle')

    def test_rejects_other_person_during_occlusion(self):
        lock = TargetLock(1.2)
        lock.select(observation(1, 2., [1., 0.]), 0.)
        xy, mode = lock.update([observation(2, 2.05, [0., 1.])], .2)
        self.assertEqual(mode, 'predicting')
        self.assertAlmostEqual(xy[0], 2., places=2)
        self.assertEqual(lock.track_id, 1)
        self.assertEqual(lock.update([], 1.3)[1], 'lost')
        self.assertIsNone(lock.position(1.3))

    def test_rebind_requires_three_unique_confirmations(self):
        lock = TargetLock()
        lock.select(observation(1, 2., [1., 0.]), 0.)
        for step in (1, 2):
            xy, mode = lock.update([observation(4, 2.02, [1., 0.])], step*.1)
            self.assertIsNone(xy)
            self.assertEqual(mode, 'recovering')
            self.assertEqual(lock.track_id, 1)
        xy, mode = lock.update([observation(4, 2.02, [1., 0.])], .3)
        self.assertEqual(mode, 'tracking')
        self.assertEqual(lock.track_id, 4)
        self.assertIsNotNone(xy)

    def test_ambiguous_never_moves(self):
        lock = TargetLock()
        lock.select(observation(1, 2., [1., 0.]), 0.)
        xy, mode = lock.update([observation(1, 2.01, [1., 0.]),
                                observation(2, 2.02, [1., 0.])], .1)
        self.assertIsNone(xy)
        self.assertEqual(mode, 'ambiguous')


class MotionTests(unittest.TestCase):
    def test_emergency_obstacle_stops(self):
        grid = LocalCostmap()
        grid.update(np.array([[.4, 0., .3]], dtype=np.float32))
        cmd = FollowController().command((2., 0.), grid, .05)
        self.assertEqual(cmd[:2], (0., 0.))

    def test_prediction_speed_is_bounded(self):
        grid = LocalCostmap()
        grid.update(np.empty((0, 3), dtype=np.float32))
        linear, angular, _ = FollowController().command((2.5, .2), grid, .05, True)
        self.assertLessEqual(linear, .08)
        self.assertLessEqual(abs(angular), .25)


if __name__ == '__main__':
    unittest.main()
