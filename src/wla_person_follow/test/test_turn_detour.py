"""Turn response and detour behaviour (J02 2026-10-29 17:47).

Planner |w| was a median 0.26 rad/s when ~0.9 was needed, 37 % of turn commands
fell in the chassis deadband (< 0.2 rad/s), the detour reset its side on one
clear frame, and the robot drove on at 0.65-0.79 m/s with the detour point
0.7-1.2 rad off until it pinned itself on the desk.
"""
import numpy as np
from wla_person_follow.sim_core.common import Tuning
from wla_person_follow.sim_core.detour import DetourPlanner
from wla_person_follow.sim_core.hybrid_controller import HybridFollower, chassis_yaw


def _box_ahead_map():
    """Known floor all round, a 0.8 m wide box 1.0-1.4 m straight ahead (as in test_sim_port)."""
    from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
    m = RollingDepthMap(); m.slide(np.zeros(2))
    floor = np.array([[x, y, 0.] for x in np.arange(-1., 4., .05) for y in np.arange(-2., 2.01, .05)
                      if not (.95 < x < 1.45 and abs(y) < .45)])
    box = np.array([[x, y, z] for x in np.arange(1., 1.41, .05) for y in np.arange(-.4, .41, .05) for z in (.2, .45)])
    points = np.r_[floor, box]
    m.update(dict(points=points, points_odom=points, dynamic=np.zeros(len(points), bool),
                  origin=np.array([0, 0, .3]), origin_odom=np.array([0, 0, .3])), np.zeros(3), 0.)
    return m


def _open_map(obstacle=None):
    from wla_person_follow.sim_core.rolling_depth_map import RollingDepthMap
    m = RollingDepthMap(); m.slide(np.zeros(2))
    floor = np.array([[x, y, 0.] for x in np.arange(-2., 4., .05) for y in np.arange(-2., 2.01, .05)])
    points = floor if obstacle is None else np.r_[floor, obstacle]
    m.update(dict(points=points, points_odom=points, dynamic=np.zeros(len(points), bool),
                  origin=np.array([0, 0, .3]), origin_odom=np.array([0, 0, .3])), np.zeros(3), 0.)
    return m


def test_turn_does_not_lock_while_following_straight_ahead():
    """Closed loop (J02 2026-10-30 16:45): starting with the chassis already
    turning left at 0.25 rad/s, the robot must come back onto the person walking
    straight ahead instead of holding the turn."""
    import math
    from wla_person_follow import sim_runtime as rt
    e = rt.SimulationEngine(np.zeros(3), 0.)
    floor = np.array([[x, y, 0.] for x in np.arange(-2., 8., .1) for y in np.arange(-3., 3.01, .1)])
    pose = np.zeros(3); act = np.array([0., .25]); dt = .05
    for k in range(80):
        t = k * dt; person = np.array([2. + .8 * t, 0.])
        packets = []
        if k % 2 == 0:
            near = floor[np.linalg.norm(floor[:, :2] - pose[:2], axis=1) < 4.5]
            c, s = math.cos(pose[2]), math.sin(pose[2]); d = near[:, :2] - pose[:2]
            base = np.c_[d @ np.array([[c, -s], [s, c]]), near[:, 2]]
            packets = [(t, rt.packet_from_filtered_points(np.empty((0, 3)), base, np.array([.18, 0, .5]), pose))]
        snap = dict(state='tracking', prediction=person.tolist(), position=person.tolist(), velocity_mps=[.8, 0.],
                    history=[(person - [.4 * i, 0.]).tolist() for i in range(8)][::-1], last_seen=t)
        cmd = np.asarray(e.step(pose.copy(), act.copy(), packets, (t, person, {}, 'target'), t,
                                tracker_snapshot=snap)['command'], float)
        goal = np.array([cmd[0], float(chassis_yaw(cmd[1], e.tuning))])
        act = act + (goal - act) * (1 - np.exp(-dt / np.array([.18, .40])))
        pose = pose + np.array([act[0] * dt * math.cos(pose[2]), act[0] * dt * math.sin(pose[2]), act[1] * dt])
    bearing = math.atan2(-pose[1], person[0] - pose[0]) - pose[2]
    assert abs(bearing) < .15 and abs(pose[2]) < .3 and pose[0] > 1.


def test_turns_in_place_when_the_person_is_far_off_the_heading():
    f = HybridFollower(Tuning()); target = np.array([.9, 1.8])  # 1.1 rad to the left
    hist = [target + [0., .02 * i] for i in range(5)]
    cmd = f.command(np.zeros(3), np.zeros(2), target, np.zeros(2), hist, _open_map(), [], 'visible')
    assert abs(cmd[0]) <= .01 and cmd[1] > .2


def test_drives_on_when_the_person_is_nearly_ahead():
    f = HybridFollower(Tuning()); target = np.array([3., .3])
    hist = [np.array([2. + .1 * i, .3]) for i in range(10)]
    cmd = f.command(np.zeros(3), np.zeros(2), target, np.zeros(2), hist, _open_map(), [], 'visible')
    assert cmd[0] > .05


def test_detour_is_held_through_brief_clear_frames():
    c = Tuning(); d = DetourPlanner(); target = np.array([3., 0.])
    route = d.plan(np.zeros(3), target, _box_ahead_map(), c)
    assert route is not None and d.held is not None
    clear = _open_map()
    for _ in range(d.exit_cycles - 1):
        held = d.plan(np.zeros(3), target, clear, c)
        assert held is not None and d.last.get('holding')
    assert d.plan(np.zeros(3), target, clear, c) is None and d.side is None


def test_one_blocked_frame_restarts_the_exit_count():
    c = Tuning(); d = DetourPlanner(); target = np.array([3., 0.]); box = _box_ahead_map(); clear = _open_map()
    d.plan(np.zeros(3), target, box, c)
    for _ in range(d.exit_cycles - 2): d.plan(np.zeros(3), target, clear, c)
    d.plan(np.zeros(3), target, box, c)
    assert d.clear_count == 0
    assert d.plan(np.zeros(3), target, clear, c) is not None


def _drive_trail(f, m):
    for x in np.arange(-1., .01, .1):
        f._remember(np.array([x, 0., 0.]))


def test_stuck_robot_backs_up_along_its_own_path():
    c = Tuning(); f = HybridFollower(c); m = _open_map(); _drive_trail(f, m)
    cmd = f._backup(np.zeros(3), np.zeros(2), m, c)
    assert cmd is not None and -c.backup_speed - 1e-9 <= cmd[0] < -.1 and abs(cmd[1]) < .05
    assert f.last_candidates.get('backing')


def test_backing_up_stops_at_a_known_obstacle_and_after_the_limit():
    c = Tuning(); f = HybridFollower(c)
    behind = np.array([[-.25, y, z] for y in np.arange(-.3, .31, .05) for z in (.2, .4)])
    m = _open_map(behind); _drive_trail(f, m)
    assert f._backup(np.zeros(3), np.zeros(2), m, c) is None
    f2 = HybridFollower(c); m2 = _open_map(); _drive_trail(f2, m2); f2.backed = c.backup_max_m
    assert f2._backup(np.zeros(3), np.zeros(2), m2, c) is None


def test_backed_distance_counts_only_reverse_motion():
    c = Tuning(); f = HybridFollower(c); m = _open_map(); _drive_trail(f, m)
    assert f._backup(np.zeros(3), np.zeros(2), m, c) is not None
    f._remember(np.array([-.1, 0., 0.]))
    assert abs(f.backed - .1) < 1e-9
    f._remember(np.array([.2, 0., 0.]))  # forward again: not counted
    assert abs(f.backed - .1) < 1e-9
