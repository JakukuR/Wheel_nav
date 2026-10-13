import unittest
from types import SimpleNamespace as N

import numpy as np

from wla_person_follow.camera import depth_meters
from wla_person_follow.depth_proc import points_to_base, rotation_from_quaternion


class RelativeGeometryTests(unittest.TestCase):
    def test_big_endian_and_stride(self):
        buffer = np.array([[1000,2000,999],[3000,4000,999]],dtype='>u2').tobytes()
        msg=N(encoding='16UC1',width=2,height=2,step=6,data=buffer,is_bigendian=True)
        np.testing.assert_allclose(depth_meters(msg),[[1.,2.],[3.,4.]])

    def test_bad_buffer_rejected(self):
        msg=N(encoding='16UC1',width=2,height=2,step=4,data=b'\x00',is_bigendian=False)
        with self.assertRaises(ValueError): depth_meters(msg)

    def test_float_invalid_depth_is_unknown(self):
        raw=np.array([[np.nan,-1.,np.inf,2.]],dtype='<f4')
        msg=N(encoding='32FC1',width=4,height=1,step=16,data=raw.tobytes(),is_bigendian=False)
        np.testing.assert_allclose(depth_meters(msg),[[0.,0.,0.,2.]])

    def test_camera_to_base_axes_only(self):
        # optical z forward, optical x right, optical y down.
        rotation=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        points=points_to_base(np.ones((2,2),dtype=np.float32)*2., (100.,100.,0.,0.),
            rotation,np.array([.1,0.,.3]),height_min=0.,height_max=2.,stride=1)
        self.assertTrue(np.allclose(points[:,0],2.1))
        self.assertTrue(np.all(points[:,1]<=0.))
        self.assertTrue(np.all(points[:,2]<=.3+1e-7))

    def test_invalid_quaternion_rejected(self):
        for q in ((0,0,0,0),(float('nan'),0,0,1),(0,0,0,2)):
            with self.assertRaises(ValueError): rotation_from_quaternion(q)


if __name__=='__main__': unittest.main()
