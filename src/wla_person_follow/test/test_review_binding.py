import numpy as np
from wla_person_follow.geometry_binding import GeometryBinding
from wla_person_follow.identity_supervisor import IdentitySupervisor

ID='s:1:1'
def objects(distance=1.72):
    return {ID:dict(detected=True,label='person',distance=distance,yaw=0.)}
def test_jump_cannot_rebind_without_confirmation():
    b=GeometryBinding(ID,0.)
    b.update(objects(),np.zeros(3),1.,1.,enrollment_complete=True)
    assert b.update(objects(5.),np.zeros(3),1.1,1.1,enrollment_complete=True) is None
    assert b.rebind(ID,objects(5.),np.zeros(3),1.1,1.1) is None
    assert b.state=='lost'
def test_same_id_return_after_grace_requires_reacquisition():
    b=GeometryBinding(ID,0.)
    b.update(objects(),np.zeros(3),1.,1.,enrollment_complete=True)
    b.update({},np.zeros(3),1.6,1.6,enrollment_complete=True)
    assert b.update(objects(),np.zeros(3),1.7,1.7,enrollment_complete=True) is None
