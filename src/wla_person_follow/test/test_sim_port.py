import math
import numpy as np
import pytest
from wla_person_follow.sim_runtime import SimulationEngine, measurements_from_relative, packet_from_depth
from wla_person_follow.sim_core.common import Tuning
from wla_person_follow.sim_core.common import rotation


def test_deployed_chassis_and_follow_clearances():
    c=Tuning()
    assert (c.step,c.horizon,c.max_v,c.max_w)==(.1,24,1.2,.9)
    assert c.desired_gap==pytest.approx(.87)
    assert c.target_hard_gap==pytest.approx(.57)
    assert c.radius == .19
    assert c.target_stop_gap-c.radius-c.target_radius == pytest.approx(.36)
    assert c.desired_gap-c.radius-c.target_radius == pytest.approx(.40)
    assert c.target_hard_gap-c.radius-c.target_radius == pytest.approx(.10)
    assert c.margin==.1


def test_unknown_start_is_not_fabricated():
    e=SimulationEngine(np.zeros(3),1.)
    assert not e.mapping.known.any()


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
    p=packet_from_depth(np.ones((8,8)),[10,20,4,4],np.array([[0,0,1],[-1,0,0],[0,-1,0]]),[.1,.2,.1],np.array([2,3,math.pi/2]))
    assert len(p['points'])==4 and not p['dynamic'].any()
    np.testing.assert_allclose(p['origin_odom'],[1.8,3.1,.1])
    np.testing.assert_allclose(p['points_odom'][:,:2],p['points'][:,:2]@rotation(math.pi/2).T+[2,3])


def test_relative_observation_id_and_reference():
    rows={key:dict(detected=True,label='person',yaw=0.,distance=1.) for key in ('s:0:1','s:0:2')}
    target,people=measurements_from_relative(rows,np.array([2,3,math.pi/2]),'s:0:1')
    np.testing.assert_allclose(target,[2,4.28]);assert set(people)=={'s:0:2'}
    target,_=measurements_from_relative(rows,np.zeros(3),'s:0:99');assert target is None




def test_no_feasible_fallback_is_not_forwarded():
    e=SimulationEngine(np.zeros(3),1.)
    def bad(*args,**kwargs):
        e.controller.last_infeasible=True;e.controller.last_candidates={}
        return np.array([.4,.3])
    e.controller.command=bad
    result=e.step(np.zeros(3),np.array([.4,.3]),[],None,1.)
    assert np.array_equal(result['command'],np.zeros(2))
    assert result['reason']=='no_feasible_candidate'
    assert not result['planner_feasible']


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
    snapshot=dict(state='tracking',prediction=[2.,0.],position=[2.,0.],
                  velocity_mps=[0.,0.],history=[],last_seen=1.)
    result=e.step(np.zeros(3),np.zeros(2),[],(1.,np.array([2.,0.]),{},'target'),1.,tracker_snapshot=snapshot)
    assert result['command'][0]==0.
    assert result['candidates']['forward_total']>0
    assert result['candidates']['forward_known']==0
    # Stationary candidates remain feasible; no forward candidate may survive.
    assert result['candidates']['forward']==0
    assert result['candidates']['feasible']>0
    assert result['planner_feasible'] and result['reason']=='tracker_follow'


def reference_display_filter(points):
    """Independent Python oracle for depth_to_points.cpp default policy."""
    from collections import defaultdict
    # Original display cloud stores float32 points and ground estimates.
    pts=np.asarray(points,dtype=np.float32)
    pts=pts[np.isfinite(pts).all(axis=1)]
    pts=pts[(pts[:,2].astype(float)>=-.08)&(pts[:,2].astype(float)<=.50)]
    samples=defaultdict(list)
    for x,y,z in pts:
        if not -.08<=float(z)<=.12:continue
        ix,iy=np.floor(np.array([x,y],float)/.20).astype(int)
        for dx in (-1,0,1):
            for dy in (-1,0,1):samples[ix+dx,iy+dy].append(z)
    ground={k:sorted(v)[int(.20*(len(v)-1))] for k,v in samples.items() if len(v)>=5}
    candidates=[]
    for pt in pts:
        key=tuple(np.floor(pt[:2].astype(float)/.20).astype(int))
        if (float(np.float32(pt[2]-ground[key]))>=.05 if key in ground else float(pt[2])>=.08):
            candidates.append(pt)
    candidates=np.asarray(candidates,dtype=np.float32).reshape(-1,3)
    cells={tuple(x) for x in np.floor(candidates[:,:2].astype(float)/.05).astype(int)}
    keep=set()
    while cells:
        start=cells.pop();component={start};todo=[start]
        while todo:
            x,y=todo.pop()
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    key=x+dx,y+dy
                    if key in cells:cells.remove(key);component.add(key);todo.append(key)
        if len(component)>=3:keep.update(component)
    mask=[tuple(x) in keep for x in np.floor(candidates[:,:2].astype(float)/.05).astype(int)]
    return candidates[np.asarray(mask,dtype=bool)].astype(float),pts.astype(float)


def test_native_filter_matches_display_ground_height_and_clusters():
    from _wla_sim_geometry import filter_points
    rng=np.random.default_rng(20261020)
    floor=np.column_stack((rng.uniform(-2,2,(4000,2)),rng.uniform(-.02,.03,4000)))
    objects=np.column_stack((rng.uniform(.5,1.,(2500,2)),rng.uniform(.06,.65,2500)))
    # Isolated cells stay rejected even with many duplicate returns.
    isolated=np.repeat([[3.01,3.01,.2]],30,axis=0)
    edges=np.array([[x,0.,z] for x in np.arange(.5,.8,.03)
                    for z in [-.080001,-.08,.04,.05,.08,.12,.5,.500001]])
    pts=np.vstack((floor,objects,isolated,edges,[[np.nan,0,0],[0,0,np.inf]]))
    actual=filter_points(pts);expected=reference_display_filter(pts)
    for a,b in zip(actual,expected):np.testing.assert_array_equal(a,b)
    assert len(actual[0])>0 and not np.any(actual[0][:,0]>3.)
    assert len(actual[1])>len(actual[0])


def test_filtered_ground_is_not_remarked_and_empty_depth_stays_unknown():
    from wla_person_follow.sim_runtime import packet_from_filtered_points
    from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
    pose=np.zeros(3);origin=np.array([.18,0,.5])
    mapping=RollingDepthMap()
    # Rejected by ground filter although above the legacy 8cm marking cutoff.
    points=np.array([[1.05,.05,.10]])
    packet=packet_from_filtered_points(np.empty((0,3)),points,origin,pose)
    mapping.update(packet,pose,1.)
    assert not mapping.occupied.any()
    empty=packet_from_depth(np.zeros((8,8)),[10,10,4,4],np.eye(3),origin,pose)
    assert empty['points'].shape==empty['marking_points_odom'].shape==(0,3)
    mapping=RollingDepthMap();mapping.update(empty,pose,1.)
    centers=(mapping.cells+.5)*mapping.res
    assert not mapping.known[np.linalg.norm(centers,axis=-1)>.19].any()


@pytest.fixture
def startup_worker(monkeypatch):
    import wla_person_follow.sim_runtime as runtime
    for name,value in [('_PROCESS_ENGINE',None),('_PROCESS_KEY',None),
                       ('_PROCESS_STARTUP_USED',False),('_PROCESS_STARTUP_PRIOR',None)]:
        monkeypatch.setattr(runtime,name,value)
    def step(t,active=False,request='',generation=0,packets=(),radius=.5,pose=None,motion_threshold=.03,local_radius=0.):
        return runtime.process_step(((request,'target' if active else '', 'fixture',0,generation),
            np.zeros(3) if pose is None else np.asarray(pose,dtype=float),np.zeros(2),packets,None,t,
            dict(active=active,radius=radius,motion_threshold=motion_threshold,local_radius=local_radius)))
    return runtime,step


def test_startup_prior_waits_for_translation_then_expires_without_reseeding(startup_worker):
    runtime,step=startup_worker
    idle=step(100.)
    assert idle['startup_prior'] is None and not idle['startup_prior_consumed']
    assert np.all(idle['grid']==-1)
    first=step(130.,True,'one')
    prior=first['startup_prior']
    assert prior['radius_m']==.5 and prior['stamp_s']==130.
    assert prior['expires_s'] is None and prior['remaining_s'] is None
    assert prior['state']=='waiting_for_translation'
    mapping=runtime._PROCESS_ENGINE.mapping
    centers=(mapping.cells+.5)*mapping.res
    inside=np.linalg.norm(centers,axis=-1)<.5
    np.testing.assert_array_equal(mapping.known,inside)
    assert mapping.query([[0.,0.]],unknown=True)[0]>.19
    # Stationary waiting, small drift and pure rotation do not start the clock.
    for t,pose in [(150.,[0,0,0]),(170.,[.01,.01,0]),(190.,[0,0,2.])]:
        held=step(t,True,'one',pose=pose)
        assert held['startup_prior']['expires_s'] is None
        np.testing.assert_array_equal(mapping.known,inside)
    # Task switches neither recenter nor rearm. Only wheel-pose translation counts.
    second=step(191.,True,'two',pose=[.031,0,2.])
    assert runtime._PROCESS_ENGINE.mapping is mapping
    assert second['startup_prior']['center_odom']==[0.,0.]
    assert second['startup_prior']['motion_started_s']==191.
    assert second['startup_prior']['expires_s']==203.
    still=step(202.9,True,'two',pose=[0,0,0])
    assert still['startup_prior']['state']=='counting_down'
    np.testing.assert_array_equal(mapping.known,inside)
    expired=step(203.1,True,'three')
    assert expired['startup_prior_consumed'] and expired['startup_prior']['remaining_s']==0.
    assert np.all(expired['grid']==-1)
    reset=step(204.,True,'four',generation=1)
    assert np.all(reset['grid']==-1) and reset['startup_prior']['state']=='reference_changed'


def test_held_startup_prior_does_not_restore_expired_observed_cells(startup_worker):
    from wla_person_follow.sim_runtime import packet_from_filtered_points
    runtime,step=startup_worker
    step(1.,True,'one')
    mapping=runtime._PROCESS_ENGINE.mapping
    obstacle=np.array([[.35,.05,.2]])
    packet=packet_from_filtered_points(obstacle,obstacle,[.18,0,.5],np.zeros(3))
    step(2.,True,'one',packets=[(2.,packet)])
    ij=np.floor(obstacle[0,:2]/mapping.res).astype(int)-mapping.lower
    assert mapping.occupied[tuple(ij)]
    held=step(20.,True,'one')
    # An actual observed obstacle can expire to unknown, never back to assumed free.
    assert held['startup_prior']['state']=='waiting_for_translation'
    assert not mapping.known[tuple(ij)]
    assert mapping.known.any()
    # Ring-buffer wrap must not transfer the held prior to unrelated cells.
    mapping.slide([20.,0.]);mapping.advance_startup_prior(21.);mapping.rebuild(21.)
    assert not mapping.known.any()


def test_reference_reset_before_motion_discards_hold(startup_worker):
    _,step=startup_worker
    step(1.,True,'one')
    result=step(30.,True,'two',generation=1)
    assert result['startup_prior']['state']=='reference_changed'
    assert result['startup_prior_consumed'] and np.all(result['grid']==-1)


def test_startup_prior_never_erases_old_or_new_obstacle_measurements(startup_worker):
    from wla_person_follow.sim_runtime import packet_from_filtered_points
    runtime,step=startup_worker
    old=np.array([[.35,.05,.2]])
    cloud=packet_from_filtered_points(old,old,[.18,0,.5],np.zeros(3))
    step(1.,packets=[(1.,cloud)])
    mapping=runtime._PROCESS_ENGINE.mapping
    def occupied(pt):
        ij=np.floor(pt[:2]/mapping.res).astype(int)-mapping.lower
        return mapping.occupied[tuple(ij)]
    assert occupied(old[0])
    step(1.1,True,'one')
    assert runtime._PROCESS_ENGINE.mapping is mapping and occupied(old[0])
    new=np.array([[.25,.25,.2]])
    measured=packet_from_filtered_points(new,new,[.18,0,.5],np.zeros(3))
    step(1.2,True,'one',packets=[(1.2,measured)])
    assert occupied(new[0])
    step(1.3,True,'two')
    assert occupied(new[0])  # Changing target/request cannot erase evidence.


def test_startup_prior_disabled_and_radius_validation(startup_worker):
    _,step=startup_worker
    result=step(1.,True,'one',radius=0.)
    assert not result['startup_prior_consumed'] and np.all(result['grid']==-1)
    for radius in [-.1,.51,float('nan')]:
        with pytest.raises(ValueError,match='invalid_startup_clear_radius'):
            step(2.,True,'one',radius=radius)


def test_startup_motion_threshold_validation(startup_worker):
    _,step=startup_worker
    for threshold in [0.,-.01,.11,float('nan')]:
        with pytest.raises(ValueError,match='invalid_startup_motion_threshold'):
            step(1.,True,'one',motion_threshold=threshold)


def test_continuous_free_circle_idle_follows_robot_without_trail(startup_worker):
    runtime,step=startup_worker
    for t,pose in [(1.,[0,0,0]),(40.,[0,0,1.]),(41.,[2,0,0])]:
        result=step(t,pose=pose,local_radius=.5)
        m=runtime._PROCESS_ENGINE.mapping
        inside=np.linalg.norm((m.cells+.5)*m.res-np.asarray(pose[:2]),axis=-1)<.5
        np.testing.assert_array_equal(m.known,inside)
        assert not m.occupied.any() and np.isneginf(m.stamp).all()
        assert result['local_free_prior']['state']=='continuous'
        assert result['startup_prior'] is None and not result['startup_prior_consumed']
    result=step(42.,pose=[2,0,0],local_radius=0.)
    assert np.all(result['grid']==-1)
    for invalid in [-.1,.51,float('nan')]:
        with pytest.raises(ValueError,match='invalid_local_assumed_free_radius'):
            step(43.,local_radius=invalid)


def test_continuous_prior_obstacles_win_including_inside_footprint(startup_worker):
    from wla_person_follow.sim_runtime import packet_from_filtered_points
    runtime,step=startup_worker
    step(1.,local_radius=.5)
    hits=np.array([[.35,.05,.2],[.05,.05,.2]])
    packet=packet_from_filtered_points(hits,np.empty((0,3)),[.18,0,.5],np.zeros(3))
    for t,packets in [(2.,[(2.,packet)]),(10.,[])]:
        step(t,packets=packets,local_radius=.5)
        m=runtime._PROCESS_ENGINE.mapping
        ij=np.floor(hits[:,:2]/m.res).astype(int)-m.lower
        assert m.occupied[ij[:,0],ij[:,1]].all()
        assert not m.assumed_free[ij[:,0],ij[:,1]].any()
    # Only the normal sensor TTL expires the hit; the assumption never refreshes it.
    step(15.,local_radius=.5)
    assert not m.occupied[ij[:,0],ij[:,1]].any()
    assert m.known[ij[:,0],ij[:,1]].all()


def _straight_corridor(width):
    from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
    m=RollingDepthMap();m.slide(np.zeros(2))
    xs=np.arange(-1,5,.02);h=width/2
    walls=np.r_[np.c_[xs,np.full_like(xs,h),np.full_like(xs,.5)],np.c_[xs,np.full_like(xs,-h),np.full_like(xs,.5)]]
    floor=np.array([[x,y,0.] for x in np.arange(-1,5,.05) for y in np.arange(-h+.03,h-.02,.05)])
    points=np.r_[walls,floor]
    m.update(dict(points=points,points_odom=points,dynamic=np.zeros(len(points),bool),
                  origin=np.array([0,0,.3]),origin_odom=np.array([0,0,.3])),np.zeros(3),0.)
    return m


@pytest.mark.parametrize('width,forward',[(1.0,True),(.8,True),(.6,False)])
def test_static_margin_is_a_cost_and_only_penetration_rejects(width,forward):
    from wla_person_follow.sim_core.hybrid_controller import HybridFollower
    f=HybridFollower(Tuning());hist=[np.array([2.+i*.1,0.]) for i in range(10)]
    cmd=f.command(np.zeros(3),np.zeros(2),np.array([3.,0.]),np.zeros(2),hist,_straight_corridor(width),[],'visible')
    # 0.8 m: inside the 0.10 m band (formerly all forward rollouts were rejected).
    assert (f.last_candidates['forward_static']>0 and cmd[0]>0)==forward


def _blind_zone_chair_map():
    """J05 geometry: camera 0.18 m ahead of centre at 0.5 m height, chair 2.1 m ahead.

    The forward camera never sees the robot's flanks or rear, so only the 0.5 m
    assumed-free disc is known there."""
    from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
    m=RollingDepthMap();m.slide(np.zeros(2))
    m.local_assumed_free_radius=.5;m.local_assumed_free_center=np.zeros(2)
    floor=np.array([[x,y,0.] for x in np.arange(.85,4.,.05) for y in np.arange(-1.5,1.5,.05)
                    if abs(np.arctan2(y,x))<.75 and not (2.05<x<2.5 and abs(y)<.25)])
    chair=np.array([[x,y,z] for x in np.arange(2.1,2.5,.05) for y in np.arange(-.2,.21,.05) for z in (.2,.45)])
    points=np.r_[floor,chair]
    m.update(dict(points=points,points_odom=points,dynamic=np.zeros(len(points),bool),
                  origin=np.array([.18,0.,.5]),origin_odom=np.array([.18,0.,.5])),np.zeros(3),0.)
    return m,chair[:,:2]


def test_object_approach_passes_the_camera_near_blind_zone_like_person_approach():
    from wla_person_follow.sim_core.approach_goal import ApproachGoal
    from wla_person_follow.sim_core.hybrid_controller import HybridFollower
    m,surface=_blind_zone_chair_map()
    from wla_person_follow.sim_core.approach_goal import SWEEP_RADIUS
    # The robot's own cell is closer to unknown space than the old route rule
    # (sweep radius + 0.014 m) allowed, so object approach could never start.
    assert m.query(np.zeros((1,2)),unknown=True)[0] < SWEEP_RADIUS+.014
    goal=ApproachGoal().select(np.zeros(3),surface,m,0.)
    assert goal is not None
    route=goal['path'];face=np.median(surface,axis=0)
    c=Tuning(desired_gap=0.,target_stop_gap=.025,target_hard_gap=0.,target_clearance_max=0.,max_v=.20)
    cmd=HybridFollower(c).command(np.zeros(3),np.zeros(2),goal['point'],np.zeros(2),[],m,[],'visible',
                                  reference_path=route,face_point=face)
    assert cmd[0]>0  # creeps forward into the blind strip; the near horizon stays in the assumed-free disc


def test_deployed_rolling_map_is_5cm_over_8m():
    e=SimulationEngine(np.zeros(3),1.)
    assert e.mapping.res==.05 and e.mapping.n==160
    assert e.mapping.n*e.mapping.res==pytest.approx(8.)


def _box_ahead_map():
    """Known floor all round, a 0.8 m wide box 1.0-1.4 m straight ahead (J02 18:27 stall)."""
    from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
    m=RollingDepthMap();m.slide(np.zeros(2))
    floor=np.array([[x,y,0.] for x in np.arange(-1.,4.,.05) for y in np.arange(-2.,2.01,.05)
                    if not (.95<x<1.45 and abs(y)<.45)])
    box=np.array([[x,y,z] for x in np.arange(1.,1.41,.05) for y in np.arange(-.4,.41,.05) for z in (.2,.45)])
    points=np.r_[floor,box]
    m.update(dict(points=points,points_odom=points,dynamic=np.zeros(len(points),bool),
                  origin=np.array([0,0,.3]),origin_odom=np.array([0,0,.3])),np.zeros(3),0.)
    return m


def test_blocked_straight_route_plans_round_the_obstacle_edge():
    from wla_person_follow.sim_core.hybrid_controller import HybridFollower
    f=HybridFollower(Tuning());target=np.array([3.,0.]);hist=[np.array([2.+i*.1,0.]) for i in range(10)]
    cmd=f.command(np.zeros(3),np.zeros(2),target,np.zeros(2),hist,_box_ahead_map(),[],'visible')
    d=f.last_candidates.get('detour') or {}
    assert d.get('active') and d.get('side') in ('left','right')
    # The reference bends past a box corner rather than through the box.
    assert abs(f.last_reference_path[1][1])>.45 and cmd[0]>0
    f.reset()
    assert f.detour.side is None and f.stuck_cycles==0


def test_clear_straight_route_does_not_detour():
    from wla_person_follow.sim_core.hybrid_controller import HybridFollower
    f=HybridFollower(Tuning());hist=[np.array([2.+i*.1,0.]) for i in range(10)]
    f.command(np.zeros(3),np.zeros(2),np.array([3.,0.]),np.zeros(2),hist,_straight_corridor(1.2),[],'visible')
    assert 'detour' not in f.last_candidates


def test_person_only_has_to_stay_in_the_image_not_near_its_centre():
    c=Tuning()
    # Hard limit ~35.8 deg (camera 38.7 deg less 0.05 rad) instead of the old 32.6 deg centre box.
    assert c.camera_half_angle-c.fov_edge_margin>float(np.arctan(.64))+.05 and c.centering_weight<4
    # ...but a weak pull to the centre and an edge cost from ~26 deg keep it away from
    # the edge at walking speed (J02 2026-10-29 15:37 lost the person near 35 deg).
    assert c.centering_weight==2. and abs(c.fov_soft_angle-.45)<1e-9 and c.fov_soft_angle<c.camera_half_angle-c.fov_edge_margin


def test_planner_result_ttl_is_020():
    import pathlib,yaml
    from wla_person_follow import sim_node
    cfg=yaml.safe_load((pathlib.Path(sim_node.__file__).resolve().parents[1]/'config'/'sim_follow.yaml').read_text()) if (pathlib.Path(sim_node.__file__).resolve().parents[1]/'config'/'sim_follow.yaml').exists() else None
    src=pathlib.Path(sim_node.__file__).read_text()
    assert 'result_ttl_s=.20,' in src
    if cfg is not None:
        assert next(iter(cfg.values()))['ros__parameters']['result_ttl_s']==.2


def test_grid_route_searches_a_0p1m_lattice_on_the_0p05m_map():
    from wla_person_follow.sim_core.detour import DetourPlanner, GRID_ROUTE_CELL_M
    from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
    m=RollingDepthMap(resolution=.05,size=160);m.slide(np.zeros(2))
    floor=np.array([[x,y,0.] for x in np.arange(-1.,4.,.05) for y in np.arange(-2.,2.01,.05)
                    if not (.95<x<1.45 and abs(y)<.45)])
    box=np.array([[x,y,z] for x in np.arange(1.,1.41,.05) for y in np.arange(-.4,.41,.05) for z in (.2,.45)])
    pts=np.r_[floor,box]
    m.update(dict(points=pts,points_odom=pts,dynamic=np.zeros(len(pts),bool),origin=np.array([0,0,.3]),origin_odom=np.array([0,0,.3])),np.zeros(3),0.)
    m.rebuild(0.)
    route=DetourPlanner()._grid_route(np.zeros(2),np.array([3.,0.]),m,Tuning())
    assert GRID_ROUTE_CELL_M==.1 and route is not None
    # Goes round the box (|y| beyond its half width) and never through it.
    assert np.max(np.abs(route[:,1]))>.45
    inside=(route[:,0]>.95)&(route[:,0]<1.45)&(np.abs(route[:,1])<.45)
    assert not inside.any()


def test_seed_follows_last_command_and_first_step_uses_real_interval():
    from wla_person_follow.sim_core.hybrid_controller import seed_command, limit_step
    c=Tuning()
    # Chassis lagging a little: keep ramping from the command, not the lagging wheel.
    np.testing.assert_allclose(seed_command([.50,.0],np.array([.60,.1]),10.,10.05),[.60,.1])
    # Large mismatch (blocked/slipping) or stale command: measured wins.
    np.testing.assert_allclose(seed_command([.20,.0],np.array([.60,.0]),10.,10.05),[.20,.0])
    np.testing.assert_allclose(seed_command([.55,.0],np.array([.60,.0]),10.,10.5),[.55,.0])
    np.testing.assert_allclose(seed_command([.55,.0],None,None,10.),[.55,.0])
    out=limit_step(np.array([1.,1.]),np.array([.6,0.]),.05,c)
    np.testing.assert_allclose(out,[.6+c.accel*.05,c.yaw_accel*.05])
    out=limit_step(np.array([1.,0.]),np.array([.6,0.]),.5,c)  # interval capped at the model step
    np.testing.assert_allclose(out[0],.6+c.accel*c.step)


def test_engine_keeps_zero_and_bounds_planned_changes_between_replans():
    from wla_person_follow.sim_runtime import SimulationEngine
    m=_straight_corridor(1.4)
    e=SimulationEngine(np.zeros(3),0.);e.mapping=m;e.map_stamp=0.
    snap=dict(state='tracking',prediction=[3.,0.],position=[3.,0.],velocity_mps=[0.,0.],
              history=[[2.+i*.1,0.] for i in range(10)],surface_offset_m=.28,last_seen=0.,history_sampling=None)
    first=e._plan(np.zeros(3),np.array([.6,0.]),None,1.,None,snap,'0',None)['command']
    second=e._plan(np.zeros(3),np.array([.6,0.]),None,1.05,None,snap,'0',None)['command']
    c=Tuning()
    assert abs(second[0]-first[0])<=c.accel*.05+1e-9
    e.controller.last_infeasible=False
    lost=dict(snap,state='lost',prediction=None,position=None)
    assert not np.any(e._plan(np.zeros(3),np.array([.6,0.]),None,1.1,None,lost,'0',None)['command'])


def test_extrapolated_position_places_cylinder_for_newer_depth():
    from wla_person_follow.follow_session import FollowTracker
    t=FollowTracker('s:1:1',0.)
    t.observations.append((1.0,np.array([2.,0.])))
    np.testing.assert_allclose(t.extrapolated_position(1.02),[2.,0.])   # exact match window
    np.testing.assert_allclose(t.extrapolated_position(1.2),[2.,0.])    # no track velocity yet
    assert t.extrapolated_position(1.5) is None and t.extrapolated_position(.9) is None


def test_control_rate_and_map_age_parameters():
    import pathlib,yaml
    from wla_person_follow import sim_node
    assert sim_node.CONTROL_PERIOD_S==.05 and .45<=sim_node.TARGET_FALLBACK_RADIUS_M<=.6
    src=pathlib.Path(sim_node.__file__).read_text()
    assert 'map_max_age_s=.45,' in src
    cfg=pathlib.Path(sim_node.__file__).resolve().parents[1]/'config'/'sim_follow.yaml'
    if cfg.exists():
        params=next(iter(yaml.safe_load(cfg.read_text()).values()))['ros__parameters']
        assert params['map_max_age_s']==.45 and params['max_sensor_age_s']==.40
