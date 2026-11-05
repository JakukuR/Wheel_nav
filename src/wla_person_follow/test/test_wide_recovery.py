"""Recovery without a position gate (2026-11-02 16:09 follow run).

The walker kept going past the 1.5 m gate while the robot searched; a 0.75-0.78
match was reset by crop-overlap / low-confidence frames in between. Now the best
visible match wins (>= .55, leading the next by .08) after three matches each
within 2 s; frames without a usable feature neither count nor reset.
"""
import math
import numpy as np
from wla_person_follow.follow_session import FollowTracker, RecoveryIdentity
from test_review_expansion import feature, observe

FAR = np.array([2.28, -4.2])  # 4.2 m beside the last seen position (2.28, 0)


def lost():
    tracker = FollowTracker('target', 0.); identity = RecoveryIdentity()
    row = dict(id='target', feature=feature(), position=np.array([2.28, 0.]))
    for stamp in (1., 1.2): observe(identity, tracker, row, stamp)
    assert identity.complete and tracker.last_seen == 1.2
    return identity, tracker


def frame(identity, tracker, stamp, people):
    """people: list of (id, similarity, position); [] = no usable feature."""
    snapshot = tracker.snapshot(stamp, np.zeros(3))
    assert snapshot['state'] != 'tracking'
    rows = [dict(id=i, feature=feature(s), position=np.asarray(p, float)) for i, s, p in people]
    identity.observe(rows, stamp, {}, tracker, snapshot)


def geometry(point):
    r = float(np.linalg.norm(point))
    return dict(detected=True, label='person', yaw=math.atan2(point[1], point[0]), distance=r-.28)


def test_lone_person_uses_the_lower_bar():
    identity, tracker = lost()
    for stamp in (5., 5.1, 5.2): frame(identity, tracker, stamp, [('walker', .48, FAR)])
    assert identity.proposal is not None and identity.proposal['id'] == 'walker'
    identity, tracker = lost()
    for stamp in (5., 5.1, 5.2): frame(identity, tracker, stamp, [('walker', .44, FAR)])
    assert identity.proposal is None and identity.reason == 'no_unambiguous_recovery_candidate'


def test_lower_bar_is_off_when_anyone_else_is_in_view():
    identity, tracker = lost()
    far2 = np.array([1., 3.])
    for stamp in (5., 5.1, 5.2):
        frame(identity, tracker, stamp, [('walker', .50, FAR), ('other', .30, far2)])
    assert identity.proposal is None
    # The other person's crop is unusable, so only the diagnostics show them.
    identity, tracker = lost()
    for stamp in (5., 5.1, 5.2):
        snapshot = tracker.snapshot(stamp, np.zeros(3))
        rows = [dict(id='walker', feature=feature(.50), position=FAR)]
        identity.observe(rows, stamp, {'walker': 'feature_ready', 'other': 'crop_overlap'}, tracker, snapshot)
    assert identity.proposal is None
    # The old track reported as undetected is nobody in view.
    identity, tracker = lost()
    for stamp in (5., 5.1, 5.2):
        snapshot = tracker.snapshot(stamp, np.zeros(3))
        rows = [dict(id='walker', feature=feature(.50), position=FAR)]
        identity.observe(rows, stamp, {'walker': 'feature_ready', 'target': 'target_not_detected'}, tracker, snapshot)
    assert identity.proposal is not None


def test_far_best_match_is_recovered_without_a_position_gate():
    identity, tracker = lost()
    for stamp in (5., 5.1): frame(identity, tracker, stamp, [('walker', .72, FAR)])
    assert identity.proposal is None and identity.reason == 'confirming_recovery'
    frame(identity, tracker, 5.2, [('walker', .72, FAR)])
    assert identity.proposal['id'] == 'walker'
    assert tracker.rebind(identity.proposal, {'walker': geometry(FAR)}, np.zeros(3), 5.2, 5.2)
    assert tracker.target_id == 'walker' and tracker.reason == 'reid_reacquired'
    np.testing.assert_allclose(tracker.last_position, FAR, atol=1e-6)


def test_frames_without_a_usable_feature_do_not_reset_the_confirmation():
    """The 16:09 sequence: matches at 198.9, 199.1 and 201.0 s with bad frames between."""
    identity, tracker = lost()
    frame(identity, tracker, 8.9, [('walker', .75, FAR)])
    frame(identity, tracker, 9.0, [])
    frame(identity, tracker, 9.1, [('walker', .78, FAR)])
    for stamp in np.arange(9.2, 11.0, .1): frame(identity, tracker, float(stamp), [])
    assert identity.pending_count == 2
    frame(identity, tracker, 11.0, [('walker', .75, FAR)])
    assert identity.proposal is not None and identity.proposal['id'] == 'walker'


def test_matches_further_apart_than_two_seconds_start_over():
    identity, tracker = lost()
    for stamp in (5., 7.5, 10.): frame(identity, tracker, stamp, [('walker', .8, FAR)])
    assert identity.proposal is None and identity.pending_count == 1


def test_highest_similarity_wins_with_another_person_in_view():
    identity, tracker = lost()
    near = np.array([2.28, -.5])
    for stamp in (5., 5.1, 5.2):
        frame(identity, tracker, stamp, [('bystander', .50, near), ('walker', .74, FAR)])
    assert identity.proposal['id'] == 'walker'


def test_two_similar_people_or_a_weak_match_are_not_recovered():
    identity, tracker = lost()
    for stamp in (5., 5.1, 5.2, 5.3):
        frame(identity, tracker, stamp, [('a', .74, FAR), ('b', .70, [2.28, 1.])])
    assert identity.proposal is None and identity.reason == 'no_unambiguous_recovery_candidate'
    for stamp in (6., 6.1, 6.2, 6.3):
        # Alone in view the bar is .45, so a lone bystander at .49-.51 (measured
        # 11-02) IS re-acquired since 11-04; only a weaker match is refused.
        frame(identity, tracker, stamp, [('bystander', .44, FAR)])
    assert identity.proposal is None



def test_coverage_recovery_on_the_way_starts_the_search_at_once():
    """16:38: stuck 0.75 m short of a last-seen point next to an obstacle for 37 s."""
    _, tracker = lost()
    pose = np.array([2.28-.75, 0., 0.])
    assert tracker.snapshot(4., pose)['state'] == 'search_last_seen'
    tracker.report_planner('search_last_seen', 'search_last_seen', 4.05)
    assert tracker.snapshot(4.1, pose)['state'] == 'search_last_seen'
    tracker.report_planner('depth_coverage_recovery', 'search_last_seen', 4.15)
    snap = tracker.snapshot(4.2, pose)
    assert snap['state'] == 'search_heading' and snap['search_arrived_by'] == 'coverage'


def test_blocked_counts_two_seconds_in_total_and_turning_does_not():
    _, tracker = lost()
    pose = np.array([2.28-1.5, 0., 0.])
    tracker.snapshot(4., pose)
    t = 4.
    for _ in range(30):  # 3 s of normal driving / turning: never arrives
        t += .1; tracker.report_planner('search_last_seen', 'search_last_seen', t)
    pose = pose+[.1, 0., 0.]  # it was driving
    assert tracker.snapshot(t, pose)['state'] == 'search_last_seen'
    for reason in ('no_feasible_candidate',)*12 + ('search_last_seen',)*5 + ('backing_out',)*7:
        t += .1; tracker.report_planner(reason, 'search_last_seen', t)
    assert tracker.snapshot(t, pose)['state'] == 'search_last_seen'  # 1.1 + 0.6 s blocked
    for _ in range(4):
        t += .1; tracker.report_planner('backing_out', 'search_last_seen', t)
    snap = tracker.snapshot(t, pose)
    assert snap['state'] == 'search_heading' and snap['search_arrived_by'] == 'blocked'


def test_standing_still_just_outside_the_arrival_band_starts_the_search():
    _, tracker = lost()
    pose = np.array([2.28-.46, 0., 0.])  # arrival is within 0.45 m
    assert tracker.snapshot(4., pose)['state'] == 'search_last_seen'
    assert tracker.snapshot(6.9, pose)['state'] == 'search_last_seen'
    snap = tracker.snapshot(7., pose)
    assert snap['state'] == 'search_heading' and snap['search_arrived_by'] == 'stalled'


def test_driving_or_turning_towards_the_point_never_counts_as_stalled():
    _, tracker = lost()
    pose = np.array([2.28-2., 0., 0.]); t = 4.
    for k in range(100):  # 10 s: a slow creep, then turning on the spot
        t += .1; pose = pose+([.003, 0., 0.] if k < 50 else [0., 0., .004])
        assert tracker.snapshot(t, pose)['state'] == 'search_last_seen'


def test_planner_reports_outside_the_search_drive_are_ignored():
    _, tracker = lost()
    pose = np.array([2.28-1.5, 0., 0.])
    tracker.report_planner('depth_coverage_recovery', 'coasting', 4.)       # stale plan
    tracker.report_planner('depth_coverage_recovery', 'search_last_seen', 2.)  # still predicting
    assert tracker.snapshot(4.1, pose)['state'] == 'search_last_seen'
    tracker.mode = '2'
    tracker.report_planner('depth_coverage_recovery', 'search_last_seen', 4.2)
    tracker.mode = '0'
    assert tracker.snapshot(4.3, pose)['state'] == 'search_last_seen'


def lost_at_bearing(bearing, mode='0'):
    tracker = FollowTracker('target', 0., mode=mode)
    tracker.last_position, tracker.last_seen = np.array([2.28, 0.]), 1.2
    tracker.exit_bearing, tracker.exit_side = bearing, 1. if bearing >= 0 else -1.
    return tracker


def test_lost_at_the_image_edge_sweeps_towards_the_exit_without_driving_back():
    tracker = lost_at_bearing(-.5)
    pose = np.array([0., 0., 0.])
    assert tracker.snapshot(3.1, pose)['state'] == 'coasting'  # prediction still applies
    snap = tracker.snapshot(3.3, pose)
    assert snap['state'] == 'search_heading' and snap['search_arrived_by'] == 'edge'
    assert snap['search_side'] == -1.


def test_lost_near_the_image_centre_still_drives_to_the_last_seen_point():
    tracker = lost_at_bearing(.39)
    snap = tracker.snapshot(3.3, np.array([0., 0., 0.]))
    assert snap['state'] == 'search_last_seen' and snap['search_arrived_by'] is None


def test_object_approach_never_starts_the_edge_sweep():
    tracker = lost_at_bearing(.6, mode='2')
    snap = tracker.snapshot(3.3, np.array([0., 0., 0.]))
    assert snap['state'] == 'holding' and snap['search_arrived_by'] is None
