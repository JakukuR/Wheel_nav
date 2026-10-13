import hashlib
import json
from types import SimpleNamespace as NS
import numpy as np
import pytest
from wla_r680_navigation_py.perception_geometry import color_array,depth_array,bbox_point,transform_point



def test_padded_color_and_endian_depth():
    color = NS(encoding='rgb8',width=1,height=2,step=4,data=bytes([1,2,3,99,4,5,6,99]))
    assert color_array(color).tolist() == [[[3,2,1]],[[6,5,4]]]
    depth = NS(encoding='16UC1',width=1,height=2,step=4,is_bigendian=True,
               data=bytes([7,208,99,99,3,232,99,99]))
    assert depth_array(depth).tolist() == [[2000],[1000]]


def test_projection_matches_furniture_formula_and_rejects_bad_depth():
    d = np.full((48,64),2000,dtype=np.uint16)
    k = [40.,0.,32.,0.,40.,24.,0.,0.,1.]
    p = bbox_point([24,16,48,40],d,k)
    assert p == [.2,.2,2.]
    tf = NS(rotation=NS(x=0,y=0,z=0,w=1),translation=NS(x=1,y=2,z=3))
    assert transform_point(p,tf) == [1.2,2.2,5.]
    d[:] = 0
    assert bbox_point([24,16,48,40],d,k) is None
    d[:]=5000
    assert bbox_point([24,16,48,40],d,k) is None
    k[0]=0
    assert bbox_point([24,16,48,40],d,k) is None
