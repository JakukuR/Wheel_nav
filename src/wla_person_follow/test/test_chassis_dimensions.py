"""Physical dimensions agree across Nav2 and the existing follow backends."""
import ast
from pathlib import Path
import numpy as np
import pytest
import yaml
from wla_person_follow.sim_core.common import ROBOT_LENGTH_M, ROBOT_WIDTH_M, ROBOT_RADIUS_M, Tuning
from wla_person_follow.sim_core.approach_goal import HALF_LENGTH, HALF_WIDTH, SWEEP_RADIUS, surface_gap


def test_footprint_encloses_rotated_corners_and_preserves_margin():
    assert (ROBOT_LENGTH_M, ROBOT_WIDTH_M) == (.29, .22)
    assert (HALF_LENGTH, HALF_WIDTH) == (.145, .11)
    assert SWEEP_RADIUS == ROBOT_RADIUS_M == Tuning().radius
    assert Tuning().margin == .10
    corners = np.array([[x,y] for x in [-HALF_LENGTH, HALF_LENGTH] for y in [-HALF_WIDTH, HALF_WIDTH]])
    for yaw in np.linspace(-np.pi, np.pi, 101):
        rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        assert np.all(np.linalg.norm(corners@rot.T,axis=1) <= ROBOT_RADIUS_M + 1e-14)


@pytest.mark.parametrize('yaw',[0., np.pi/2, -.7])
def test_surface_clearance_uses_length_and_width(yaw):
    origin=np.array([1.,2.]);c,s=np.cos(yaw),np.sin(yaw);rot=np.array([[c,-s],[s,c]])
    for local in [[.195,0.],[0.,.16]]:
        assert surface_gap([*origin,yaw],np.array([origin+rot@local])) == pytest.approx(.05)


def test_nav2_local_and_global_match_body():
    root=Path(__file__).resolve().parents[3]
    files=list((root/'src/wla_r680_navigation/config').glob('nav2*.yaml'))
    count=0
    def walk(v):
        nonlocal count
        if isinstance(v,dict):
            if 'footprint' in v:
                p=np.array(ast.literal_eval(v['footprint']))
                assert p.shape == (4,2)
                np.testing.assert_allclose(p.min(axis=0),[-.145,-.11],rtol=0,atol=1e-14)
                np.testing.assert_allclose(p.max(axis=0),[.145,.11],rtol=0,atol=1e-14)
                assert v['footprint_padding']==0
                count+=1
            for child in v.values():walk(child)
    for file in files:walk(yaml.safe_load(file.read_text()))
    assert count==8
