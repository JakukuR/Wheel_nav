"""Short-lived wheel-compensated observations, without global pose."""
import math
import numpy as np
import pytest
from wla_person_follow.local_memory import LocalMemory, LocalMotion, advance
from wla_person_follow.local_avoidance import scene_from_points

def test_motion_integrates_body_twist_and_bounds_extrapolation():
    motion=LocalMotion()
    assert motion.update(10., (.2,0.,0.),10.)
    assert motion.update(10.1,(.2,0.,0.),10.1)
    assert motion.pose_at(10.1)==pytest.approx((.02,0.,0.))
    assert motion.pose_at(10.14)==pytest.approx((.028,0.,0.))
    assert motion.pose_at(10.2) is None
    assert motion.pose_at(9.9) is None
    assert advance((0.,0.,math.pi/2),(.2,0.,0.),.1)==pytest.approx((0.,.02,math.pi/2))
    assert not motion.healthy(10.31)

@pytest.mark.parametrize('stamp,twist,now',[(9.,(.1,0.,0.),10.),(10.1,(.1,0.,0.),10.),(10.,(float('nan'),0.,0.),10.),(10.,(2.,0.,0.),10.)])
def test_invalid_motion_revokes_history(stamp,twist,now):
    motion=LocalMotion();motion.update(9.9,(.1,0.,0.),9.9)
    assert not motion.update(stamp,twist,now)
    assert not motion.samples and motion.generation==1

def test_gap_changes_generation_and_replay_does_not_move_pose():
    motion=LocalMotion();motion.update(10.,(.1,0.,0.),10.)
    assert not motion.update(10.,(.1,0.,0.),10.)
    assert motion.generation==0 and len(motion.samples)==1
    assert motion.update(10.4,(.1,0.,0.),10.4)
    assert motion.generation==1 and motion.pose_at(10.4)==(0.,0.,0.)

def test_remembered_obstacle_transforms_into_current_body_frame():
    memory=LocalMemory();raw=scene_from_points([[1.,0.,.2]])
    memory.apply(raw,(0.,0.,0.),10.,0)
    empty=scene_from_points([])
    moved=memory.apply(empty,(.2,0.,math.pi/2),10.1,0)
    assert moved.occupied[moved.cell(0.,-.8)]
    assert not moved.occupied[moved.cell(1.,0.)]

def test_memory_expires_without_recursive_refresh():
    memory=LocalMemory();raw=scene_from_points([[1.,0.,.2]])
    memory.apply(raw,(0.,0.,0.),10.,0)
    for stamp in np.arange(10.1,12.,.1):
        result=memory.apply(scene_from_points([]),(0.,0.,0.),float(stamp),0)
        assert result.occupied.any()
    result=memory.apply(scene_from_points([]),(0.,0.,0.),12.1,0)
    assert not result.occupied.any()

def test_motion_reset_removes_old_observations():
    memory=LocalMemory();memory.apply(scene_from_points([[1.,0.,.2]]),(0.,0.,0.),10.,0)
    result=memory.apply(scene_from_points([]),(0.,0.,0.),10.1,1)
    assert not result.occupied.any()

def test_new_obstacle_between_keyframes_is_retained_and_blocks_old_free():
    memory=LocalMemory();memory.apply(scene_from_points([[2.,0.,0.]]),(0.,0.,0.),10.,0)
    obstacle=scene_from_points([[1.,0.,.2]])
    current=memory.apply(obstacle,(0.,0.,0.),10.1,0)
    assert not current.clear(1.,0.)
    later=memory.apply(scene_from_points([]),(0.,0.,0.),10.2,0)
    assert later.occupied[later.cell(1.,0.)]


def test_control_time_reprojection_does_not_move_obstacle_with_robot():
    memory=LocalMemory();raw=scene_from_points([[1.,0.,.2]])
    memory.apply(raw,(0.,0.,0.),10.,0)
    moved=memory.project((.2,0.,math.pi/2),10.1,0)
    assert moved.occupied[moved.cell(0.,-.8)]
    assert not moved.occupied[moved.cell(1.,0.)]
    assert memory.project((.4,0.,0.),12.1,0) is None
    assert memory.project((0.,0.,0.),10.1,1) is None

def test_control_projection_keeps_latest_non_keyframe_free_observation():
    memory=LocalMemory();memory.apply(scene_from_points([]),(0.,0.,0.),10.,0)
    latest=scene_from_points([[2.,0.,0.]])
    memory.apply(latest,(0.,0.,0.),10.1,0)
    projected=memory.project((.1,0.,0.),10.15,0)
    assert projected.observed[projected.cell(1.9,0.)]
    assert memory.project((.1,0.,0.),12.2,0) is None
