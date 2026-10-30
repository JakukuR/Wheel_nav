"""Search-time recovery outside the geometry gate (J02 2026-10-29 17:25).

The walker passed the robot and stood 1.7-2.2 m from the frozen prediction; the
1.5 m gate never requested its appearance and the robot searched for a minute.
"""
import math
import numpy as np
from wla_person_follow.follow_session import FollowTracker, RecoveryIdentity
from test_review_expansion import feature, observe

FAR = np.array([2.28, -2.2])  # 2.2 m beside the last seen position (2.28, 0)


def lost():
    tracker = FollowTracker('target', 0.); identity = RecoveryIdentity()
    row = dict(id='target', feature=feature(), position=np.array([2.28, 0.]))
    for stamp in (1., 1.2): observe(identity, tracker, row, stamp)
    assert identity.complete and tracker.last_seen == 1.2
    return identity, tracker


def search(identity, tracker, stamps, *, similarity=.7, position=FAR, people=1, ident='walker'):
    for stamp in stamps:
        snapshot = tracker.snapshot(stamp, np.zeros(3))
        assert snapshot['state'] != 'tracking'
        row = dict(id=ident, feature=feature(similarity), position=np.asarray(position, float))
        identity.observe([row], stamp, {}, tracker, snapshot, people=people)


def geometry(point):
    r = float(np.linalg.norm(point))
    return dict(detected=True, label='person', yaw=math.atan2(point[1], point[0]), distance=r-.28)


def test_sole_far_person_is_recovered_after_five_strict_matches():
    identity, tracker = lost()
    stamps = [9. + .1*i for i in range(5)]  # gate 2.4 m by then
    search(identity, tracker, stamps[:4])
    assert identity.proposal is None and identity.reason == 'confirming_wide_recovery'
    search(identity, tracker, stamps[4:])
    assert identity.proposal['wide'] is True and identity.proposal['id'] == 'walker'
    now = stamps[-1]
    assert tracker.rebind(identity.proposal, {'walker': geometry(FAR)}, np.zeros(3), now, now)
    assert tracker.target_id == 'walker' and tracker.reason == 'reid_reacquired'
    np.testing.assert_allclose(tracker.last_position, FAR, atol=1e-6)


def test_far_match_needs_the_stricter_similarity():
    identity, tracker = lost()
    search(identity, tracker, [7. + .1*i for i in range(8)], similarity=.5)  # passes .40, not .55
    assert identity.proposal is None and identity.reason == 'no_unambiguous_recovery_candidate'
    assert identity.candidates[-1]['wide'] is False


def test_far_match_is_refused_with_another_person_in_view():
    identity, tracker = lost()
    search(identity, tracker, [7. + .1*i for i in range(8)], similarity=.95, people=2)
    assert identity.proposal is None


def test_no_wide_match_while_still_coasting():
    identity, tracker = lost()
    assert not tracker.wide_search(3.)
    search(identity, tracker, [2.5 + .1*i for i in range(6)], similarity=.95)
    assert identity.proposal is None


def test_gate_grows_from_jump_max_to_search_radius():
    identity, tracker = lost()
    # Continuous with the binding gate (.45 + .25/s since last seen), capped at 4 m.
    assert tracker.recovery_radius(3.3) == tracker.radius(3.3)
    assert tracker.radius(7.) == 1.5 and math.isclose(tracker.recovery_radius(7.), .45+.25*5.8)
    assert tracker.recovery_radius(60.) == 4.
    # 2.2 m away is outside the gate right after the search starts.
    search(identity, tracker, [3.3 + .1*i for i in range(8)], similarity=.95)
    assert identity.proposal is None


def test_near_candidates_keep_the_original_rule():
    identity, tracker = lost()
    near = np.array([2.28, -.6])
    search(identity, tracker, [7., 7.1, 7.2], similarity=.45, position=near)
    assert identity.proposal is not None and identity.proposal['wide'] is False


def test_wide_proposal_cannot_rebind_beyond_the_gate():
    identity, tracker = lost()
    search(identity, tracker, [9. + .1*i for i in range(5)])
    proposal = identity.proposal; assert proposal['wide']
    moved = np.array([2.28, -4.5])
    assert not tracker.rebind(proposal, {'walker': geometry(moved)}, np.zeros(3), 9.4, 9.4)
