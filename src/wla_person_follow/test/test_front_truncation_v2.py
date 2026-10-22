import numpy as np
from wla_person_follow.sim_runtime import SimulationEngine, packet_from_filtered_points


def mapped_engine(points,t=1.):
    e=SimulationEngine(np.zeros(3),t,recovery_options=dict(
        front_check_truncation=True,front_check_max_m=1.4,front_check_target_offset_m=.87,
        retreat_disabled=False,reverse_policy_v2=True,reverse_policy_allow_retreat=False))
    e.mapping.local_assumed_free_radius=.5
    packet=packet_from_filtered_points(np.asarray(points,float),np.empty((0,3)),[.18,0,.5],np.zeros(3))
    e.step(np.zeros(3),np.zeros(2),[(t,packet)],None,t)
    return e,packet


def test_one_metre_surface_with_1_28_centre_does_not_block_target_route():
    e,_=mapped_engine([[1.,0.,.2]])
    target=np.array([1.28,0.])
    samples=e.controller.recovery._path_samples(np.zeros(3),target)
    assert .39<=samples[:,0].max()<=.41+1e-9
    assert not e.controller.recovery.forward_blocked(np.zeros(3),e.mapping,[],target=target)


def test_one_metre_centre_with_0_72_surface_does_not_block_target_route():
    e,_=mapped_engine([[.72,0.,.2]])
    target=np.array([1.,0.])
    samples=e.controller.recovery._path_samples(np.zeros(3),target)
    assert .10<=samples[:,0].max()<=.13+1e-9
    assert not e.controller.recovery.forward_blocked(np.zeros(3),e.mapping,[],target=target)


def test_real_obstacle_inside_the_truncated_supervisory_range_still_blocks():
    e,_=mapped_engine([[.20,0.,.2]])
    target=np.array([1.28,0.])
    assert e.controller.recovery.forward_blocked(np.zeros(3),e.mapping,[],target=target)
    assert e.controller.recovery.front_diagnostics['source']['kind']=='map_occupancy'
    diagnostics=e.controller.recovery.front_diagnostics
    assert diagnostics['front_check_truncation'] is True
    assert diagnostics['front_check_max_m']==1.4
    assert diagnostics['front_check_target_offset_m']==.87
    assert diagnostics['route_length_m']==target[0]
    assert abs(diagnostics['effective_check_length_m']-.41)<1e-9


def test_v2_persistent_front_block_holds_without_automatic_retreat():
    e,_=mapped_engine([[.20,0.,.2]])
    target=np.array([3.,0.])
    e.step(np.zeros(3),np.zeros(2),[],(3.1,target,{}),3.1)
    e.step(np.zeros(3),np.zeros(2),[],(5.11,target,{}),5.11)
    assert e.controller.recovery.reason=='reverse_policy_v2_no_auto_retreat'
    assert e.controller.recovery.state=='hold'
