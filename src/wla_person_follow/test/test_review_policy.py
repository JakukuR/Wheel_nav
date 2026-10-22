import numpy as np
from wla_person_follow.sim_runtime import SimulationEngine

def test_geometry_loss_overrides_planner_coast():
    from wla_person_follow.follow_policy import PolicySnapshot
    engine=SimulationEngine(np.zeros(3),1.)
    engine.mapping.log[:]=-2.;engine.mapping.stamp[:]=1.
    engine.step(np.zeros(3),np.zeros(2),[],(1.,np.array([3.,0.]),{}),1.)
    policy=PolicySnapshot(state='lost',last_seen=1.,lost_since=1.5)
    result=engine.step(np.zeros(3),np.zeros(2),[],(1.6,None,{}),1.6,policy=policy)
    assert result['command'][0]==0.
    assert result['target_state']=='lost'
    assert result['candidates']['forward_total']==0
    assert policy.limit_command([.2,.2],1.6)[0]==0.
    assert np.all(policy.limit_command([.2,.2],9.6)==0.)
