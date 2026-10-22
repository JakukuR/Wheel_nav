import numpy as np
from wla_person_follow.sim_runtime import measurements_from_relative,SimulationEngine

def test_suspect_target_remains_in_people_channel():
    ident='s:1:1';objects={ident:dict(detected=True,label='person',distance=1.72,yaw=0.)}
    target,people=measurements_from_relative(objects,np.zeros(3),ident,target_is_obstacle=True)
    assert ident in people
    engine=SimulationEngine(np.zeros(3),1.)
    result=engine.step(np.zeros(3),np.zeros(2),[],(1.,target,people,ident,True),1.)
    assert ident in engine.people.tracks
    assert ident in result['obstacle_track_ids']
    target,people=measurements_from_relative(objects,np.zeros(3),ident)
    engine.step(np.zeros(3),np.zeros(2),[],(1.1,target,people,ident,False),1.1)
    assert ident not in engine.people.tracks
