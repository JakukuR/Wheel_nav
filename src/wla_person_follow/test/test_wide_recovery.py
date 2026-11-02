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
    identity.observe(rows, stamp, {}, tracker, snapshot, people=len(rows))


def geometry(point):
    r = float(np.linalg.norm(point))
    return dict(detected=True, label='person', yaw=math.atan2(point[1], point[0]), distance=r-.28)


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
        frame(identity, tracker, stamp, [('bystander', .50, FAR)])
    assert identity.proposal is None
