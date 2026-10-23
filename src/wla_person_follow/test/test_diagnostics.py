"""Read-only diagnostics: truthful sources, expiry and no map mutation."""
import json
import logging
from types import SimpleNamespace as NS
import numpy as np
from wla_person_follow.diagnostics import finite_json, occupancy_source, front_distances, observed_target
from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
from wla_person_follow.sim_core.wide_turn import WideTurnTuning
from wla_person_follow.sim_core.blocked_recovery import BlockedRecovery
from wla_person_follow.status_recorder import make_handler


def test_source_matches_padded_edt_and_does_not_touch_map():
    m=RollingDepthMap();m.slide(np.zeros(2));m.rebuild(10.)
    point=np.array([.05,.05]);before=[a.copy() for a in (m.log,m.stamp,m.dist)]
    assert occupancy_source(m,point,10.)['kind']=='padded_map_boundary'
    cell=np.array([.25,.05]);a,b=m._slots(cell[None,:]);m.log[a,b]=2.;m.stamp[a,b]=9.;m.rebuild(10.)
    before=[a.copy() for a in (m.log,m.stamp,m.dist)]
    result=occupancy_source(m,point,10.)
    assert result['kind']=='map_occupancy' and result['cell_reference']==[2,0]
    assert result['cell_age_s']==1. and result['semantic_owner']=='unattributed_occupancy'
    for x,y in zip(before,(m.log,m.stamp,m.dist)):np.testing.assert_array_equal(x,y)


def test_static_short_circuit_source_and_independent_nearest_scan():
    c=WideTurnTuning();r=BlockedRecovery(c)
    samples=r._path_samples(np.zeros(3),np.array([2.,0.]))
    queried=np.full(len(samples),1.);queried[-1]=.30
    tracks=[dict(id='person',p=np.array([.5,0.]),v=np.zeros(2),r=.28)]
    summary=front_distances(samples,queried,tracks,c,.5)
    assert summary['nearest_block_source']['id']=='person'
    assert len(summary['blocking_sources'])==2
    mapping=NS(query=lambda _:queried)
    assert r.forward_blocked(np.zeros(3),mapping,tracks,target=np.array([2.,0.]))
    # The original decision still short-circuits at static occupancy.
    assert r.front_diagnostics['reason']=='static_distance_field'
    assert r.front_diagnostics['person_evaluation']=='not_reached'
    assert r.front_diagnostics['all_sources_scan']['nearest_block_source']['id']=='person'






def test_missing_target_and_nonfinite_diagnostics_are_explicit():
    obj=observed_target({},'missing',10.,11.)
    assert obj['surface_distance_m'] is None and obj['age_s']==1. and obj['detected'] is False
    assert finite_json(dict(valid=True,bad=float('nan')))==dict(valid=True,bad=None)
    json.dumps(finite_json(dict(value=np.array([np.inf,.1]))),allow_nan=False)


def test_rotating_recorder_writes_full_snapshot_and_is_independent(tmp_path):
    path,h=make_handler(tmp_path);h.maxBytes=150
    logger=logging.getLogger('diagnostic_recorder_test');logger.setLevel(logging.INFO);logger.addHandler(h)
    try:
        for i in range(5):logger.info(json.dumps(dict(diagnostic_sequence=i,diagnostics={'payload':'x'*60})))
        h.flush()
    finally:logger.removeHandler(h);h.close()
    records=[]
    for p in tmp_path.iterdir():records.extend(json.loads(line) for line in p.read_text().splitlines())
    assert sorted(r['diagnostic_sequence'] for r in records)==list(range(5))
