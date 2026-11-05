"""Turn response and detour behaviour (J02 2026-10-29 17:47).

Planner |w| was a median 0.26 rad/s when ~0.9 was needed, 37 % of turn commands
fell in the chassis deadband (< 0.2 rad/s), the detour reset its side on one
clear frame, and the robot drove on at 0.65-0.79 m/s with the detour point
0.7-1.2 rad off until it pinned itself on the desk.
"""
import numpy as np
import pytest
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


def _trail_follower(c):
    """Follower that has driven 0.6 m straight to the origin (its swept trail)."""
    f = HybridFollower(c)
    for x in np.arange(-.6, .001, .05):
        f._remember(np.array([x, 0., 0.]))
    return f


def test_backs_out_when_no_candidate_is_safe():
    """Wall inside the body radius ahead: stopping is unsafe too. After three plans
    without a safe option the robot reverses along its trail (J02 2026-10-30)."""
    c = Tuning(); f = _trail_follower(c)
    wall = np.array([[.16, y, z] for y in np.arange(-.6, .61, .05) for z in (.2, .4)])
    m = _open_map(wall); target = np.array([2., 0.]); hist = [np.array([1.5 + .1 * i, 0.]) for i in range(5)]
    cmds = [f.command(np.zeros(3), np.zeros(2), target, np.zeros(2), hist, m, [], "visible") for _ in range(4)]
    assert np.allclose(cmds[0], 0.) and f.last_infeasible is False
    assert cmds[-1][0] < -.05


def test_depth_recovery_never_drives_forward():
    c = Tuning(); m = _open_map(); hist = [np.array([1.5, 0.])]
    # With a swept trail: back out first.
    f = _trail_follower(c)
    cmd = f.command(np.zeros(3), np.zeros(2), np.array([1.5, -.8]), np.zeros(2), hist, m, [], "visible", recovery=True)
    assert cmd[0] < 0.
    # No trail: turn in place towards the person, never forward.
    f = HybridFollower(c)
    cmd = f.command(np.zeros(3), np.array([0., -.3]), np.array([1.5, -.8]), np.zeros(2), hist, m, [], "visible", recovery=True)
    assert cmd[0] == 0. and cmd[1] < 0.
    cmd = f.command(np.zeros(3), np.zeros(2), np.array([1.5, .05]), np.zeros(2), hist, m, [], "visible", recovery=True)
    np.testing.assert_allclose(cmd, [0., 0.])


def test_search_turns_to_the_exit_heading_at_the_search_rate():
    """At the last-seen point the robot turns to where the person went at up to
    search_turn_rate (was 0.3 rad/s; 3-12 s per loss on 2026-10-30)."""
    from wla_person_follow.follow_session import TrackingPolicy
    c = Tuning(); f = HybridFollower(c); m = _open_map()
    policy = TrackingPolicy(state="search_heading", last_seen=0., prediction_s=2., target_id="t", revision=0,
                            search_heading=1.2, search_started=None)
    cmd = np.zeros(2); peak = 0.
    for _ in range(6):
        cmd = f.command(np.zeros(3), cmd, np.array([.05, 0.]), np.zeros(2), [], m, [], "search_heading",
                        policy=policy, policy_time=10.)
        peak = max(peak, cmd[1])
    assert cmd[0] == 0. and peak > .45 and peak <= c.search_turn_rate + 1e-9


def test_clear_walk_is_followed_directly_not_along_the_footprints():
    """Robot 0.9 m beside the person's straight line with nothing in between:
    the reference goes straight to the person (no re-joining S-turn)."""
    c = Tuning(); f = HybridFollower(c); m = _open_map()
    hist = [np.array([x, .9]) for x in np.arange(-.5, 2.01, .25)]
    target = np.array([2., .9])
    f.command(np.zeros(3), np.zeros(2), target, np.array([.6, 0.]), hist, m, [], "visible")
    ref = f.last_reference_path
    np.testing.assert_allclose(ref[0], [0., 0.], atol=1e-9)
    assert np.linalg.norm(ref[1] - target) < .3


def test_shortcut_keeps_the_footprints_behind_an_obstacle():
    c = Tuning(); f = HybridFollower(c)
    wall = np.array([[x, y, z] for x in (1.0, 1.05) for y in np.arange(-1., .61, .05) for z in (.2, .4)])
    m = _open_map(wall)
    path = np.array([[0., 0.], [.5, 0.], [.5, 1.1], [1.5, 1.1], [2.5, 0.]])
    out = f._shortcut(np.zeros(3), path, m, c)
    assert len(out) >= 4 and np.linalg.norm(out[-1] - [2.5, 0.]) < 1e-9
    # Never a straight line through the wall.
    assert not np.allclose(out[1], [2.5, 0.]) and not np.allclose(out[1], [1.5, 1.1])


def test_target_filter_smooths_detection_jitter():
    """5 cm detection noise on a straight walk: the followed-person filter's
    velocity direction stays steady (2026-11-02 retune)."""
    from wla_person_follow import follow_session
    from wla_person_follow.sim_core.tracking import VisualTrack
    rng = np.random.default_rng(4); spread = {}
    for name, kw in (('old', {}), ('tuned', dict(meas_std=follow_session.TARGET_MEAS_STD_M,
                                                  accel_std=follow_session.TARGET_ACCEL_STD_MPS2))):
        tr = VisualTrack(timeout=2., **kw); h = []
        for k in range(80):
            z = np.array([2. + .8 * .07 * k, 0.]) + rng.normal(0, .035, 2)
            _, v, _ = tr.update(z, .07 * k)
            if k > 20: h.append(np.arctan2(v[1], v[0]))
        spread[name] = np.std(np.diff(h))
    assert spread['tuned'] < .6 * spread['old']



def test_holding_keeps_driving_on_the_last_trusted_track():
    """A rejected depth jump ('holding', J02 2026-10-30 ~1.8 s stops at corridor
    corners) keeps the coasting plan in person modes; object approach still stops."""
    from wla_person_follow.follow_session import TrackingPolicy
    from wla_person_follow.sim_runtime import SimulationEngine  # noqa: F401  (import check)
    for mode in ("0", "1"):
        policy = TrackingPolicy(state="holding", last_seen=0., prediction_s=2., target_id="t", revision=0, mode=mode)
        assert policy.translation_allowed(.5) and not policy.translation_allowed(2.5)
        assert np.allclose(policy.limit_command(np.array([.3, .4]), .5), [.3, .4])
    obj = TrackingPolicy(state="holding", last_seen=0., prediction_s=2., target_id="t", revision=0, mode="2")
    assert np.allclose(obj.limit_command(np.array([.2, .4]), .5), 0.)
    c = Tuning(); f = HybridFollower(c); m = _open_map()
    cmd = np.zeros(2)
    for _ in range(6):
        cmd = f.command(np.zeros(3), cmd, np.array([2.5, .6]), np.zeros(2), [], m, [], "coast",
                        policy=TrackingPolicy(state="holding", last_seen=0., prediction_s=2., target_id="t",
                                              revision=0, mode="0"), policy_time=.5)
    assert cmd[0] > .05 and cmd[1] > 0.



def test_search_turns_to_the_exit_side_first_then_sees_the_whole_circle():
    """At the last-seen point (before the corridor corner) the robot turns ~90 deg
    towards the side the person left on, then keeps rotating the same way, so a
    person who turned back behind it is seen too."""
    from wla_person_follow.follow_session import TrackingPolicy
    from wla_person_follow.sim_core.common import wrap
    c = Tuning(); m = _open_map()
    for side in (1., -1.):
        f = HybridFollower(c); pose = np.zeros(3); cmd = np.zeros(2); yaws = []
        policy = TrackingPolicy(state="search_heading", last_seen=0., prediction_s=2., target_id="t", revision=0,
                                search_heading=0., search_started=0., mode="0", search_side=side)
        turned = 0.
        for k in range(int(12./c.step)):
            cmd = f.command(pose.copy(), cmd, np.array([.2, 0.]), np.zeros(2), [], m, [], "search_heading",
                            policy=policy, policy_time=k*c.step)
            assert cmd[0] == 0.
            pose[2] = wrap(pose[2]+cmd[1]*c.step); turned += side*cmd[1]*c.step; yaws.append(turned)
        assert yaws[int(3./c.step)] > 1.3       # reached ~90 deg on the exit side by 3 s
        assert min(np.diff(yaws)) > -.01        # never swings back
        assert yaws[-1] > 2*np.pi               # the whole circle within 12 s


def test_image_edge_detection_keeps_bearing_and_predicted_range():
    from wla_person_follow.follow_session import FollowTracker
    tr = FollowTracker("t", 0.)
    pose = np.zeros(3)
    for k in range(8):
        s = .1*k
        assert tr.observe({"t": dict(detected=True, label="person", yaw=-.3, distance=1.2)}, pose, s, s+.01)
    # Person half out of the image: depth reads the wall 2.3 m away.
    s = .8
    assert tr.observe({"t": dict(detected=True, label="person", yaw=-.55, distance=2.3, image_edge=True)},
                      pose, s, s+.01)
    snap = tr.snapshot(s+.01, pose)
    assert snap["state"] == "tracking" and snap["search_side"] == -1.
    assert np.hypot(*snap["position"]) == pytest.approx(1.48, abs=.05)
    tr2 = FollowTracker("t", 0.)
    for k in range(8):
        s = .1*k
        tr2.observe({"t": dict(detected=True, label="person", yaw=-.3, distance=1.2)}, pose, s, s+.01)
    tr2.observe({"t": dict(detected=True, label="person", yaw=-.55, distance=2.3)}, pose, .8, .81)
    assert tr2.snapshot(.81, pose)["state"] == "holding"



def test_backing_reverses_at_once_and_steers_the_tail_onto_the_trail():
    """11-02 16:47: 0.2*cos(err) = 0.03-0.05 m/s did not move the chassis; 11-03
    18:07: turning the tail first alternated with search turns for 75 s and never
    reversed. Now it reverses at >= backup_min_speed straight away."""
    c = Tuning(); f = HybridFollower(c); m = _open_map(); _drive_trail(f, m)
    yaw = 1.4  # tail points 80 deg off the trail behind
    cmd = f._backup(np.array([0., 0., yaw]), np.array([0., -.5]), m, c, 0.)
    assert cmd is not None and cmd[0] <= -c.backup_min_speed + 1e-9 and cmd[1] < 0.
    cmd = f._backup(np.array([0., 0., .3]), np.zeros(2), m, c, 0.)
    assert cmd[0] <= -c.backup_min_speed + 1e-9


def test_backing_refuses_when_the_straight_back_line_is_blocked():
    c = Tuning(); f = HybridFollower(c)
    wall = np.array([[x, -.32, h] for x in np.arange(-.3, .31, .05) for h in (.3, .6)])
    m = _open_map(wall); _drive_trail(f, m)
    # Tail towards the wall: the trail is free, but the chassis would start into it.
    assert f._backup(np.array([0., 0., 1.4]), np.zeros(2), m, c, 0.) is None
    assert f._backup(np.zeros(3), np.zeros(2), m, c, 0.) is not None


def test_backing_gives_up_when_reversing_does_not_move():
    c = Tuning(); f = HybridFollower(c); m = _open_map(); _drive_trail(f, m)
    t = 0.
    while t < c.backup_stall_s - .05:
        assert f._backup(np.zeros(3), np.zeros(2), m, c, t)[0] < 0.; t += .1
    assert f._backup(np.zeros(3), np.zeros(2), m, c, t + .1) is None
    assert f.backed >= c.backup_max_m  # for the rest of this stuck episode
    # Progress keeps it going.
    f2 = HybridFollower(c); _drive_trail(f2, m)
    for k in range(20):
        p = np.array([-.03 * k, 0., 0.])
        assert f2._backup(p, np.zeros(2), m, c, .1 * k) is not None


def test_turns_in_place_inside_the_obstacle_margin():
    """11-02 16:50/16:52: nose inside the margin of a known obstacle, no backing
    left, person 0.3 rad off: zero command for 10-11 s. The round footprint can
    turn in place there."""
    c = Tuning(); hist = [np.array([1.5, 0.])]
    wall = np.array([[.12, y, z] for y in np.arange(-.6, .61, .05) for z in (.2, .4)])
    m = _open_map(wall)
    f = HybridFollower(c)
    cmd = f.command(np.zeros(3), np.array([0., .3]), np.array([1.5, .5]), np.zeros(2), hist, m, [], "visible",
                    recovery=True)
    assert cmd[0] == 0. and cmd[1] > .2
