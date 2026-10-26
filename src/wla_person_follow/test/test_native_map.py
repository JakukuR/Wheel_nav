"""Native hot paths must preserve the NumPy occupancy and collision reference."""
import numpy as np
import pytest
from _wla_sim_geometry import ray_slots, grid_query
from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
import wla_person_follow.sim_core.rolling_depth_map as module


def reference_rays(m,ends,origin):
    delta=ends-origin;length=np.linalg.norm(delta,axis=1)
    distance=np.arange(0,7.,m.res*.45)
    points=origin+delta[:,None,:]/np.maximum(length[:,None,None],1e-9)*distance[None,:,None]
    mask=distance[None,:]<length[:,None]-.12
    return m._slots(points[mask])


def test_ray_slots_equivalence_boundaries_wrap_and_empty():
    rng=np.random.default_rng(542)
    for res in (.01,.1,.13):
        m=RollingDepthMap(resolution=res,size=120)
        for pos in ([0.,0.],[-11.34,5.67],[25.1,-36.8]):
            m.slide(np.array(pos));origin=np.asarray(pos)+[.18,.06]
            ends=np.vstack((rng.uniform(-7,7,(1300,2))+pos,
                np.asarray(pos)+np.array([[0.,0.],[.1,0.],[.12,0.],[7.,7.]])))
            for actual,expected in zip(ray_slots(ends,origin,m.lower,res,m.n),reference_rays(m,ends,origin)):
                np.testing.assert_array_equal(actual,expected)
    assert len(ray_slots(np.empty((0,2)),[0,0],[-60,-60],.1,120)[0])==0
    with pytest.raises(ValueError):ray_slots(np.ones((1,3)),[0,0],[0,0],.1,120)


def test_query_equivalence_all_shapes_edges_and_custom_field(monkeypatch):
    rng=np.random.default_rng(32);m=RollingDepthMap();m.slide([-2.5,4.1]);m.rebuild(1.)
    field=rng.uniform(0,3,(120,120))
    for shape in ((2,),(0,2),(325,3,2)):
        points=rng.uniform(-12,12,shape)
        expected=grid_query(points,field,m.lower,m.res)
        snapshot=dict(state='tracking',prediction=[2.,.5],position=[2.,.5],
                      velocity_mps=[0.,0.],history=[],last_seen=t)
        with monkeypatch.context() as patch:
            patch.setattr(module,'native_grid_query',None)
            actual=m.query(points,field=field)
        np.testing.assert_allclose(actual,expected,rtol=0,atol=2e-15)


def test_full_map_and_controller_parity(monkeypatch):
    from wla_person_follow.sim_runtime import SimulationEngine,packet_from_filtered_points
    rng=np.random.default_rng(92);pose=np.zeros(3)
    reference=SimulationEngine(pose,1.);native=SimulationEngine(pose,1.)
    for e in (reference,native):e.mapping.local_assumed_free_radius=.5
    for i in range(12):
        t=1.+i*.1;pose=np.array([i*.015,0.,i*.01])
        floor=np.column_stack((rng.uniform([.3,-2],[4,2],(2500,2)),np.zeros(2500)))
        hits=np.column_stack((rng.uniform([1.2,-.5],[1.5,.5],(300,2)),np.full(300,.2)))
        packet=packet_from_filtered_points(hits,np.vstack([floor,hits]),[.18,0,.5],pose)
        args=(pose,np.array([.2,.1]),[(t,packet)],(t,np.array([2.,.5]),{'p':np.array([1.5,1.])},'target'),t)
        snapshot=dict(state='tracking',prediction=[2.,.5],position=[2.,.5],
                      velocity_mps=[0.,0.],history=[],last_seen=t)
        with monkeypatch.context() as patch:
            patch.setattr(module,'native_grid_query',None);patch.setattr(module,'native_ray_slots',None)
            a=reference.step(*args,tracker_snapshot=snapshot)
        b=native.step(*args,tracker_snapshot=snapshot)
        for name in ('log','stamp','known','occupied','dist','free_dist'):
            np.testing.assert_array_equal(getattr(reference.mapping,name),getattr(native.mapping,name))
        np.testing.assert_array_equal(a['command'],b['command'])
        assert a['candidates']==b['candidates'] and a['reason']==b['reason']
