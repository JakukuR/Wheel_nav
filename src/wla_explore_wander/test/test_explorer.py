import math
import numpy as np
import pytest
from wla_explore_wander.explorer import Explorer, Params
from wla_explore_wander.memory import ObjectMemory, SectorMemory, VisitHeatmap, WheelPose
from wla_explore_wander.policy import Weights, choose_object, grid_ray_measure, object_scores, wander_goal

AUTO = dict(mode='AUTO', estop_latched=False, automatic_source='PERSON')


class FakeIO:
    def __init__(self):
        self.calls = []

    def start_leg(self, rid, mode, target, lease):
        self.calls.append(('start', rid, mode, target))

    def heartbeat(self, rid, lease):
        self.calls.append(('heartbeat', rid))

    def stop_leg(self, rid):
        self.calls.append(('stop', rid))

    def starts(self):
        return [c for c in self.calls if c[0] == 'start']


def explorer(**kw):
    io = FakeIO(); e = Explorer(io, Params(**kw), seed=1)
    e.on_wheel(0., 0., 0., 0.); e.on_control(AUTO, 0.)
    return e, io


def arrive(e, now):
    rid = e.leg['request_id']
    e.on_leg_reply(rid, 'start', True, 'started', 'waiting', now)
    e.on_follow_status(dict(request_id=rid, state='arrived', enabled=False), now)


def open_sectors(e, now, reach=3.):
    e.sectors.reach[:] = reach; e.sectors.wide[:] = 1.; e.sectors.stamp[:] = now


def test_object_memory_clusters_by_label_and_position():
    m = ObjectMemory()
    a = m.observe('chair', [1., 0.], 0.); b = m.observe('chair', [1.3, .1], .5)
    c = m.observe('table', [1., 0.], .5); d = m.observe('chair', [3., 0.], .5)
    assert a is b and c is not a and d is not a and len(m.instances) == 3
    m.observe('chair', [1.2, 0.], 1.2)
    assert a.sightings == 2  # sightings counted at most once per second
    m.observe('chair', [1.6, 0.], 5.)
    assert a.moved_at == 5.
    m.touch(np.array([3.2, 0.]), 6.)
    assert d.touched_at == 6. and a.touched_at == -math.inf


def test_visit_heatmap_grows_and_fades():
    h = VisitHeatmap(tau=100.)
    for k in range(60):
        h.add([.2, .2], k, 1.)
    high = h.level([.2, .2], 60.)
    assert high > .8 and h.level([5., 5.], 60.) == 0.
    assert h.level([.2, .2], 400.) < .3


def test_curiosity_prefers_new_rare_near_objects_and_respects_cooldown():
    m = ObjectMemory(); pose = WheelPose()
    old = m.observe('cup', [1., 0.], 0.); old.sightings = 40; old.visits = 2; old.last_seen = 299.
    new = m.observe('plant', [1.5, .5], 290.)
    rows = object_scores(m, pose, 300., np.zeros(2), 4.)
    score = {inst.label: s for s, inst, _ in rows}
    assert score['plant'] > score['cup']
    new.cooldown_until = 400.
    assert [inst.label for _, inst, _ in object_scores(m, pose, 300., np.zeros(2), 4.)] == ['cup']
    far = m.observe('lamp', [6., 0.], 300.)
    assert far not in [inst for _, inst, _ in object_scores(m, pose, 300., np.zeros(2), 4.)]
    assert choose_object([(.1, old, {})], np.random.default_rng(0)) is None


def test_grid_rays_stop_at_obstacles_and_skip_the_near_blind_zone():
    res = .05; grid = np.zeros((160, 160), dtype=int)
    grid[60:100, 20:96] = 0
    grid[:, 120] = 100            # wall 2.0 m ahead (cells measured from x=-4 m)
    grid[78:83, 80:84] = -1       # unknown patch inside the blind zone ahead
    measure = grid_ray_measure(grid, res, (-4., -4.), 0.)
    reach, wide = measure(0.)
    assert 1.9 <= reach <= 2.05 and wide == 1.
    grid[:, 90] = 100              # obstacle 0.5 m ahead
    assert measure(0.)[0] <= .55


def test_wander_goal_picks_open_unvisited_sector_inside_boundary():
    s = SectorMemory(); v = VisitHeatmap(); pose = WheelPose(); rng = np.random.default_rng(0)
    s.reach[:] = .5; s.stamp[:] = 0.
    assert wander_goal(s, v, pose, 1., np.zeros(2), 4., rng) is None
    s.reach[3] = 3.; s.reach[0] = 3.; s.wide[:] = 1.
    for k in range(200):
        v.add([1.5, 0.], k*.1, 1.)   # straight ahead has been walked a lot
    score, index, goal = wander_goal(s, v, pose, 20., np.zeros(2), 4., rng)
    assert index == 3 and 1. <= np.linalg.norm(goal) <= 2.
    assert wander_goal(s, v, pose, 20., np.zeros(2), .8, rng) is None


def test_start_requires_auto_and_person_source_without_estop():
    io = FakeIO(); e = Explorer(io, Params(), seed=0)
    assert e.start(0.) == (False, 'control_state_unavailable')
    e.on_control(dict(AUTO, estop_latched=True), 0.)
    assert e.start(0.) == (False, 'estop_latched')
    e.on_control(dict(AUTO, automatic_source='NAV'), 0.)
    assert e.start(0.) == (False, 'person_source_not_selected')
    e.on_control(AUTO, 0.)
    assert e.start(0.) == (True, 'started') and e.state == 'look'
    assert io.starts()[-1][2:] == ('4', '6.483')


def test_round_wanders_when_nothing_is_interesting_then_looks_again():
    e, io = explorer(); e.start(0.)
    open_sectors(e, 10.)
    arrive(e, 10.)
    assert e.state == 'wander' and io.starts()[-1][2] == '3'
    dx, dy = map(float, io.starts()[-1][3].split(','))
    assert 1. <= math.hypot(dx, dy) <= 2.
    e.on_leg_reply(e.leg['request_id'], 'start', True, 'started', 'waiting', 10.1)
    e.tick(10.4)
    assert io.calls[-1] == ('heartbeat', e.leg['request_id'])
    arrive(e, 15.)
    assert e.state == 'dwell'
    e.tick(17.5)
    assert e.state == 'look' and io.starts()[-1][2] == '4'


def test_curiosity_turns_acquires_approaches_observes_and_cools_down():
    e, io = explorer(); e.start(0.)
    rows = [dict(detected=True, label='plant', id='s:0:7', distance=2., yaw=math.pi/2)]
    e.on_detections(rows, 5.)
    arrive(e, 10.)
    assert e.state == 'turn_to' and float(io.starts()[-1][3]) == pytest.approx(math.pi/2, abs=.01)
    e.pose.yaw = math.pi/2
    arrive(e, 13.)
    assert e.state == 'acquire'
    e.on_detections([dict(rows[0], id='s:0:9', yaw=0.)], 13.1)  # tracker id changed meanwhile
    e.tick(13.2)
    assert e.state == 'approach' and io.starts()[-1][2:] == ('2', 's:0:9')
    target = e.target
    arrive(e, 20.)
    assert e.state == 'observe'
    e.tick(24.5)
    assert e.state == 'look' and target.visits == 1 and target.cooldown_until == pytest.approx(324.5)


def test_object_lost_during_acquire_goes_into_cooldown_and_reselects():
    e, io = explorer(); e.start(0.)
    e.on_detections([dict(detected=True, label='plant', id='s:0:7', distance=1.5, yaw=0.)], 5.)
    open_sectors(e, 10.)
    arrive(e, 10.)
    assert e.state == 'acquire'
    e.on_detections([], 10.5); e.tick(12.6)
    assert e.state == 'wander' and e.objects.instances[0].cooldown_until > 100.


def test_person_coming_close_cancels_for_good():
    e, io = explorer(); e.start(0.); rid = e.leg['request_id']
    person = [dict(detected=True, label='person', id='s:0:1', distance=.8, yaw=0.)]
    e.on_detections(person, 1.); e.on_detections(person, 1.5)
    assert e.state == 'look'
    e.on_detections(person, 2.1)
    assert e.state == 'done' and e.reason == 'yielded_to_person' and io.calls[-1] == ('stop', rid)
    assert e.resume(3.)[0] is False


def test_pause_then_resume_reselects_from_memory():
    e, io = explorer(); e.start(0.)
    assert e.pause(1.) == (True, 'paused') and io.calls[-1][0] == 'stop' and e.leg is None
    open_sectors(e, 2.); e.on_control(AUTO, 2.)
    assert e.resume(2.) == (True, 'resumed') and e.state == 'wander'


def test_estop_and_manual_takeover_end_the_session():
    e, io = explorer(); e.start(0.)
    e.on_control(dict(AUTO, estop_latched=True), 1.)
    assert e.state == 'done' and e.reason == 'estop'
    e.on_control(AUTO, 2.); assert e.start(2.)[0]
    e.on_control(dict(AUTO, mode='MANUAL'), 3.)
    assert e.reason == 'manual_takeover'


def test_authority_loss_in_backend_finishes_and_ordinary_failures_retry():
    e, io = explorer(); e.start(0.); open_sectors(e, 1.)
    rid = e.leg['request_id']
    e.on_leg_reply(rid, 'start', True, 'started', 'waiting', 1.)
    e.on_follow_status(dict(request_id=rid, state='cancelled', reason='motion_authority_lost'), 2.)
    assert e.state == 'done' and e.reason == 'motion_authority_lost'
    e, io = explorer(max_consecutive_failures=3); e.start(0.); open_sectors(e, 1.)
    for k in range(3):
        e.on_leg_reply(e.leg['request_id'], 'start', False, 'observations_stale', 'rejected', 1.)
    assert e.state == 'done' and e.reason.startswith('repeated_leg_failures')


def test_leg_timeout_stops_the_leg_and_blocks_that_sector():
    e, io = explorer(); e.start(0.); open_sectors(e, 1.)
    arrive(e, 1.)
    sector = e.leg['sector']; rid = e.leg['request_id']
    e.tick(40.)
    assert ('stop', rid) in io.calls and sector in e.blocked


def test_foreign_follow_task_preempts():
    e, io = explorer(); e.start(0.)
    e.on_follow_status(dict(request_id='agent-follow-1', state='following', enabled=True), 1.)
    assert e.state == 'done' and e.reason.startswith('preempted_by_agent-follow-1')


def test_budget_ends_the_session_and_pause_does_not_count():
    e, io = explorer(session_budget_s=10.); e.start(0.)
    e.tick(4.); e.pause(4.); e.tick(100.); e.on_control(AUTO, 100.); e.resume(100.)
    e.tick(105.)
    assert e.state != 'done'
    e.tick(107.)
    assert e.reason == 'budget_exhausted'
