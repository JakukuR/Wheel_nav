import math
import numpy as np
import pytest
from wla_person_follow.sim_runtime import SimulationEngine, measurements_from_relative, packet_from_depth
from wla_person_follow.sim_core.wide_turn import WideTurnTuning
from wla_person_follow.sim_core.common import rotation


def test_deployed_chassis_and_follow_clearances():
    c=WideTurnTuning()
    assert (c.step,c.horizon,c.max_v,c.max_w)==(.1,24,.65,.9)
    assert c.desired_gap==pytest.approx(.87)
    assert c.target_hard_gap==pytest.approx(.57)
    assert c.radius == .19
    assert c.target_stop_gap-c.radius-c.target_radius == pytest.approx(.36)
    assert c.desired_gap-c.radius-c.target_radius == pytest.approx(.40)
    assert c.target_hard_gap-c.radius-c.target_radius == pytest.approx(.10)
    assert c.margin==.1 and c.rear_yield and c.wide_turn


def test_unknown_start_is_not_fabricated():
    e=SimulationEngine(np.zeros(3),1.)
    assert not e.mapping.known.any()
    assert not e.controller.recovery.rear_clear(np.zeros(3),e.mapping,[])


def test_cpp_projection_matches_numpy_full_se3():
    from _wla_sim_geometry import deproject
    from scipy.spatial.transform import Rotation
    rng=np.random.default_rng(5)
    d=rng.uniform(.1,6.,(48,64));d[::5,::7]=np.nan
    k=np.array([61.,60.,31.,23.]);r=Rotation.from_euler('xyz',[.15,-.2,.35]).as_matrix()
    tr=np.array([.18,.06,.5]);stride=3
    v,u=np.mgrid[0:48:stride,0:64:stride];z=d[::stride,::stride]
    mask=np.isfinite(z)&(z>=.4)&(z<=5.)
    expected=np.column_stack(((u[mask]-k[2])*z[mask]/k[0],(v[mask]-k[3])*z[mask]/k[1],z[mask]))@r.T+tr
    actual=deproject(d,k,r,tr,.4,5.,stride)
    np.testing.assert_allclose(actual,expected,rtol=0,atol=2e-15)
    with pytest.raises(ValueError):deproject(d,[0,60,31,23],r,tr,.4,5.,3)


def test_packet_retains_obstacles_and_measured_origins():
    p=packet_from_depth(np.ones((8,8)),[10,10,4,4],np.eye(3),[.1,.2,.3],np.array([2,3,math.pi/2]))
    assert len(p['points'])==4 and not p['dynamic'].any()
    np.testing.assert_allclose(p['origin_odom'],[1.8,3.1,.3])
    np.testing.assert_allclose(p['points_odom'][:,:2],p['points'][:,:2]@rotation(math.pi/2).T+[2,3])


def test_relative_observation_id_and_reference():
    rows={key:dict(detected=True,label='person',yaw=0.,distance=1.) for key in ('s:0:1','s:0:2')}
    target,people=measurements_from_relative(rows,np.array([2,3,math.pi/2]),'s:0:1')
    np.testing.assert_allclose(target,[2,4.28]);assert set(people)=={'s:0:2'}
    target,_=measurements_from_relative(rows,np.zeros(3),'s:0:99');assert target is None


def test_repeated_observation_does_not_renew_tracking():
    e=SimulationEngine(np.zeros(3),1.)
    obs=(1.,np.array([1.6,0.]),{})
    for i in range(12):
        e.step(np.zeros(3),np.zeros(2),[],obs,1.+i*.1)
    assert e.target.seen==1.
    assert len(e.target.history)==2
    assert e.controller.recovery.search is not None


def test_no_feasible_fallback_is_not_forwarded():
    e=SimulationEngine(np.zeros(3),1.)
    def bad(*args,**kwargs):
        e.controller.last_infeasible=True;e.controller.last_wide=None
        e.controller.last_yield=False;e.controller.last_candidates={}
        return np.array([.4,.3])
    e.controller.command=bad
    result=e.step(np.zeros(3),np.array([.4,.3]),[],None,1.)
    assert np.array_equal(result['command'],np.zeros(2))
    assert result['reason']=='no_feasible_candidate'


def test_smaller_footprint_keeps_nearby_obstacle_evidence():
    from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
    mapping=RollingDepthMap(resolution=.01,size=100)
    pose=np.zeros(3)
    points=np.array([[.215,.005,.2]])
    cloud=dict(points=points,points_odom=points,origin=np.array([0.,0.,.1]),
               origin_odom=np.array([0.,0.,.1]),dynamic=np.array([False]))
    mapping.update(cloud,pose,1.)
    cell=np.floor(points[0,:2]/mapping.res).astype(int)-mapping.lower
    assert mapping.robot_radius == .19
    assert mapping.occupied[tuple(cell)]  # Outside new body; inside old 24cm body.


def test_bitmap_slots_match_row_sort_across_ring_wrap():
    from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
    rng=np.random.default_rng(487)
    for size in (10,120):
        mapping=RollingDepthMap(size=size)
        for origin in ([0.,0.],[-12.3,7.8],[24.6,-19.1]):
            mapping.slide(np.array(origin))
            points=rng.uniform(-size*.1,size*.1,(12000,2))+origin
            points=np.concatenate([points,points[:200],np.array([origin])])
            cells=np.floor(points/mapping.res).astype(int)
            valid=np.all((cells>=mapping.lower)&(cells<mapping.lower+size),axis=-1)
            expected=np.unique(cells[valid],axis=0)
            x,y=mapping._slots(points)
            np.testing.assert_array_equal(x,expected[:,0]%size)
            np.testing.assert_array_equal(y,expected[:,1]%size)
        x,y=mapping._slots(np.empty((0,2)))
        assert len(x)==len(y)==0


def test_unknown_forward_space_is_reported_without_changing_stop():
    e=SimulationEngine(np.zeros(3),1.)
    result=e.step(np.zeros(3),np.zeros(2),[],(1.,np.array([2.,0.]),{}),1.)
    assert result['command'][0]==0.
    assert result['candidates']['forward_total']>0
    assert result['candidates']['forward_known']==0
    assert result['reason']=='forward_space_unobserved'
