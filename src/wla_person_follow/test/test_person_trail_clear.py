"""Hits the followed person left behind are forgotten; fixed obstacles are not."""
import numpy as np

from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
from wla_person_follow.sim_runtime import SimulationEngine


def _cloud(*, hits=()):
    hits = np.asarray(hits, dtype=float).reshape(-1, 3)
    empty = np.empty((0, 3), dtype=float)
    return dict(points=empty, points_odom=empty, origin=np.array([0., 0., .5]),
                origin_odom=np.array([0., 0., .5]), dynamic=np.empty(0, dtype=bool),
                marking_points_odom=hits)


def _cell(mapping, point):
    return tuple(np.floor(np.asarray(point)[:2] / mapping.res).astype(int) - mapping.lower)


def test_hit_marked_where_the_person_stood_becomes_unknown():
    mapping = RollingDepthMap(resolution=.1, size=40, ttl=12.)
    robot = np.zeros(3)
    body = (1.05, .05, .8)  # Box lagged: the person's own surface was marked.
    mapping.update(_cloud(hits=(body,)), robot, 1.)
    assert mapping.occupied[_cell(mapping, body)]
    retired = mapping.retire_person_trail([[1., 1.2, 0.]], 1.1)
    mapping.rebuild(1.1)
    assert retired == 1
    assert not mapping.occupied[_cell(mapping, body)]
    assert not mapping.known[_cell(mapping, body)]


def test_wall_seen_before_the_person_came_close_is_kept():
    mapping = RollingDepthMap(resolution=.1, size=40, ttl=12.)
    robot = np.zeros(3)
    wall = (1.05, .25, .8)
    mapping.update(_cloud(hits=(wall,)), robot, 1.)
    # Re-observed while the person walks past it: occupied since 1.0, not 3.0.
    mapping.update(_cloud(hits=(wall,)), robot, 3.)
    assert mapping.retire_person_trail([[3., 1.1, 0.]], 3.1) == 0
    mapping.rebuild(3.1)
    assert mapping.occupied[_cell(mapping, wall)]


def test_hit_far_from_the_person_or_at_another_time_is_kept():
    mapping = RollingDepthMap(resolution=.1, size=40, ttl=12.)
    robot = np.zeros(3)
    far = (1.05, .65, .8)
    later = (1.05, .05, .8)
    mapping.update(_cloud(hits=(far,)), robot, 1.)
    mapping.update(_cloud(hits=(later,)), robot, 4.)  # Person long gone by then.
    assert mapping.retire_person_trail([[1., 1.1, 0.], [1.5, 1.6, 0.]], 4.1) == 0
    mapping.rebuild(4.1)
    assert mapping.occupied[_cell(mapping, far)]
    assert mapping.occupied[_cell(mapping, later)]


def test_detection_gap_is_interpolated_around_the_corner():
    mapping = RollingDepthMap(resolution=.1, size=40, ttl=12.)
    robot = np.zeros(3)
    # Seen at (1, 0) and again at (1, 1.2) 1.2 s later; marked mid-gap at the turn.
    ghost = (1.05, .65, .8)
    mapping.update(_cloud(hits=(ghost,)), robot, 1.6)
    trail = [[1., 1., 0.], [2.2, 1., 1.2]]
    assert mapping.retire_person_trail(trail, 2.3) == 1
    # A gap longer than max_gap is not bridged.
    mapping.update(_cloud(hits=(ghost,)), robot, 5.)
    assert mapping.retire_person_trail([[3., 1., 0.], [7.5, 1., 1.2]], 7.6) == 0


def test_rehit_after_the_person_left_marks_the_obstacle_again():
    mapping = RollingDepthMap(resolution=.1, size=40, ttl=12.)
    robot = np.zeros(3)
    box = (1.05, .05, .8)
    mapping.update(_cloud(hits=(box,)), robot, 1.)
    trail = [[1., 1.1, 0.]]
    assert mapping.retire_person_trail(trail, 1.1) == 1
    mapping.update(_cloud(hits=(box,)), robot, 3.)
    assert mapping.retire_person_trail(trail, 3.1) == 0
    mapping.rebuild(3.1)
    assert mapping.occupied[_cell(mapping, box)]


def test_engine_collects_trail_only_for_person_modes_and_retires_ghosts():
    engine = SimulationEngine(np.zeros(3), 0.)
    engine.trail_clear_radius = .3
    body = (1.02, .02, .8)
    engine.update_map(np.zeros(3), [(1., _cloud(hits=(body,)))], 1.)
    assert engine.mapping.occupied[_cell(engine.mapping, body)]
    observation = (1., np.array([1.1, 0.]), {}, 'p')
    engine.step(np.zeros(3), np.zeros(2), [], observation, 1.1, task_mode='2', active=True)
    assert not len(engine.target_trail)
    engine.step(np.zeros(3), np.zeros(2), [], observation, 1.1, task_mode='0', active=True)
    assert len(engine.target_trail) == 1 and engine.trail_retired == 1
    assert not engine.mapping.occupied[_cell(engine.mapping, body)]


def test_hits_written_before_the_task_are_retired_round_the_person():
    """2026-11-10 trial: the person stood in view while no task ran, so the map
    held their body; at task start the straight route ended in that ghost."""
    mapping = RollingDepthMap(resolution=.05, size=80, ttl=12.)
    robot = np.zeros(3)
    body = [(1.0, y, .8) for y in (-.15, 0., .15)] + [(.85, 0., .8)]
    chair = (1.0, .75, .3)  # 0.75 m to the side of the person: kept.
    mapping.update(_cloud(hits=body + [chair]), robot, 1.)
    # The trail starts later than the hits, so the trail rule cannot match them.
    assert mapping.retire_person_trail([[3., 1., 0.]], 3.) == 0
    assert mapping.retire_disc((1.0, 0.), .5, 3.) == len(body)
    mapping.rebuild(3.)
    for point in body:
        assert not mapping.occupied[_cell(mapping, point)]
        assert not mapping.known[_cell(mapping, point)]  # unknown, not free
    assert mapping.occupied[_cell(mapping, chair)]
    # A later hit marks the cell again.
    mapping.update(_cloud(hits=body[:1]), robot, 3.2)
    mapping.rebuild(3.2)
    assert mapping.occupied[_cell(mapping, body[0])]


def test_engine_retires_the_disc_a_packet_requests():
    engine = SimulationEngine(np.zeros(3), 0.)
    body = (1.02, .02, .8)
    engine.update_map(np.zeros(3), [(1., _cloud(hits=(body,)))], 1.)
    assert engine.mapping.occupied[_cell(engine.mapping, body)]
    packet = dict(_cloud(), retire_disc_odom=np.array([1., 0., .5]))
    engine.update_map(np.zeros(3), [(1.1, packet)], 1.1)
    assert not engine.mapping.occupied[_cell(engine.mapping, body)]
