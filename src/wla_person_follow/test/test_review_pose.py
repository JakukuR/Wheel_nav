import numpy as np
from wla_person_follow.geometry_binding import GeometryBinding

def test_unavailable_pose_is_missing_measurement():
    b=GeometryBinding('s:1:1',0.)
    rows={'s:1:1':dict(detected=True,label='person',distance=1.72,yaw=0.)}
    b.update(rows,np.zeros(3),1.,1.)
    assert b.update(rows,None,1.1,1.1)=='s:1:1'
    assert b.state=='bound_coasting'
    assert b.snapshot(1.1)['measurement_reason']=='observation_pose_unavailable'
