"""Wander / curiosity loop. ROS-free: the node supplies inputs and sends the legs.

Started once, it keeps wandering until another task takes the follow backend,
it is stopped, or motion permission goes. One full look-around at the start;
after that it walks 1.2-2.8 m legs through the free region connected to the
robot, rests a few seconds between legs and now and then glances to one side.
An object that scores high enough interrupts the walk: turn to it, approach,
observe. A full turn happens again only when no reachable goal is found.
Every motion is a leg of the person-follow backend (mode 2 object approach,
mode 3 point goal, mode 4 orient); this module never publishes velocity.
"""
from dataclasses import dataclass, field
import math
import uuid
import numpy as np
from .memory import ObjectMemory, SectorMemory, VisitHeatmap, WheelPose, wrap
from .policy import Weights, choose_object, connected_goal, grid_ray_measure, object_scores, wander_goal

RUNNING = ('look', 'glance', 'turn_to', 'acquire', 'approach', 'observe', 'wander', 'rest', 'yield')
# Follow terminal reasons that mean the motion permission itself is gone.
AUTHORITY_REASONS = ('motion_authority_lost', 'motion_authority_revoked', 'local_estop',
                     'estop_or_direct_publisher_conflict', 'transport_request_revoked_or_invalid',
                     'detection_disabled', 'detector_generation_changed', 'direct_publisher_conflict')


@dataclass
class Params:
    session_budget_s: float = 0.          # 0: wander until stopped or preempted
    boundary_radius_m: float = 4.
    initial_look: bool = True
    look_turn_rad: float = 2*math.pi+.2
    look_timeout_s: float = 35.
    turn_timeout_s: float = 12.
    approach_timeout_s: float = 45.
    wander_timeout_s: float = 30.
    acquire_timeout_s: float = 2.5
    acquire_radius_m: float = .8
    observe_s: float = 4.
    rest_min_s: float = 1.
    rest_max_s: float = 4.
    glance_probability: float = .3
    glance_min_rad: float = .5
    glance_max_rad: float = 1.2
    no_goal_glances: int = 2              # side glances before a full look
    no_goal_rest_s: float = 10.
    failure_rest_s: float = 15.
    grid_max_age_s: float = 1.5
    visit_cooldown_s: float = 300.
    failure_cooldown_s: float = 120.
    turn_skip_rad: float = .2
    interaction_radius_m: float = 1.0
    interaction_hold_s: float = 1.0
    resume_after_person_s: float = 5.     # < 0: a close person ends the session
    heartbeat_period_s: float = .3
    lease_s: float = 1.
    control_max_age_s: float = .6
    detection_max_age_s: float = .5
    max_consecutive_failures: int = 4
    person_center_offset_m: float = .28
    weights: Weights = field(default_factory=Weights)


class Explorer:
    def __init__(self, io, params=None, seed=None):
        self.io, self.p = io, params or Params()
        self.rng = np.random.default_rng(seed)
        self.pose = WheelPose()
        self.visits = VisitHeatmap()
        self.objects = ObjectMemory()
        self.sectors = SectorMemory()
        self.grid = None; self.grid_at = -math.inf
        self.state, self.reason = 'idle', 'waiting_for_start'
        self.session = None; self.leg_count = 0; self.leg = None
        self.start_xy = np.zeros(2); self.active_s = 0.; self.last_tick = None
        self.control = None; self.control_at = -math.inf
        self.visible = []; self.visible_at = -math.inf
        self.close_since = None; self.person_seen_at = -math.inf
        self.target = None; self.deadline = None
        self.no_goal = 0; self.failures = 0; self.blocked = set()
        self.travel_heading = None; self.legs_walked = 0; self.walked_m = 0.
        self.follow_status = None; self.last_visit_at = None
        self.last_choice = None; self.history = []

    # ---------- inputs ----------
    def on_wheel(self, stamp, v, w, now):
        if not self.pose.update(stamp, v, w):
            return
        if self.last_visit_at is not None and self.state in RUNNING:
            self.visits.add(self.pose.xy(), now, min(.5, now-self.last_visit_at))
        self.last_visit_at = now

    def on_detections(self, rows, now):
        visible = []; close = False
        for row in rows:
            if not row.get('detected') or row.get('distance') is None or row.get('yaw') is None:
                continue
            person = row['label'] == 'person'
            xy = self.pose.to_odom(row['distance']+(self.p.person_center_offset_m if person else 0.), row['yaw'])
            if person:
                self.objects.touch(xy, now)
                close |= row['distance'] <= self.p.interaction_radius_m
            else:
                self.objects.observe(row['label'], xy, now)
            visible.append((row, xy))
        self.visible, self.visible_at = visible, now
        if close:
            self.person_seen_at = now
        self.close_since = (self.close_since or now) if close else None
        if (self.close_since is not None and now-self.close_since >= self.p.interaction_hold_s
                and self.state in RUNNING and self.state != 'yield'):
            if self.p.resume_after_person_s < 0:
                self.finish(now, 'yielded_to_person')
            else:
                self._stop_leg(); self.target = None
                self.state, self.reason = 'yield', 'person_close'
                self._log(now, 'yield')

    def on_grid(self, snapshot, now):
        self.grid, self.grid_at = snapshot, now
        self.sectors.update(self.pose, now, grid_ray_measure(snapshot.grid, snapshot.resolution,
                                                             snapshot.origin_xy, snapshot.origin_yaw))

    def on_control(self, data, now):
        self.control, self.control_at = data, now
        if self.state in RUNNING:
            if data.get('estop_latched'):
                self.finish(now, 'estop')
            elif data.get('mode') != 'AUTO':
                self.finish(now, 'manual_takeover')

    def on_follow_status(self, status, now):
        self.follow_status = status
        leg = self.leg
        rid = status.get('request_id', '')
        if leg is not None and rid == leg['request_id']:
            if status.get('state') == 'arrived':
                self._leg_done(now)
            elif status.get('state') in ('cancelled', 'expired', 'stopped') and leg['accepted']:
                self._leg_failed(now, status.get('reason') or status.get('state'))
        elif (status.get('enabled') and rid and self.session is not None
                and not rid.startswith('explore-') and self.state in RUNNING):
            # Another task took the backend: that is the end of wandering.
            self.finish(now, 'preempted_by_'+rid[:40], stop_leg=False)

    def on_leg_reply(self, request_id, command, accepted, reason, state, now):
        leg = self.leg
        if leg is None or request_id != leg['request_id']:
            return
        if accepted:
            leg['accepted'] = True
            if state == 'arrived':
                self._leg_done(now)
        elif state == 'arrived':
            self._leg_done(now)
        else:
            self._leg_failed(now, reason or 'rejected')

    # ---------- commands ----------
    def authority(self, now):
        c = self.control
        if c is None or now-self.control_at > self.p.control_max_age_s:
            return False, 'control_state_unavailable'
        if c.get('estop_latched'):
            return False, 'estop_latched'
        if c.get('mode') != 'AUTO':
            return False, 'not_in_auto_mode'
        if not (c.get('automatic_source') == 'PERSON' or c.get('velocity_passthrough') is True):
            return False, 'person_source_not_selected'
        return True, 'ok'

    def start(self, now):
        if self.state in RUNNING:
            return False, 'already_running'
        ok, reason = self.authority(now)
        if not ok:
            return False, reason
        self.session = uuid.uuid4().hex[:8]; self.leg_count = 0
        self.start_xy = self.pose.xy(); self.active_s = 0.; self.last_tick = now
        self.failures = 0; self.no_goal = 0; self.blocked = set(); self.travel_heading = None
        if self.p.initial_look:
            self._look(now)
        else:
            self._select(now)
        return True, 'started'

    def pause(self, now, reason='paused_by_request'):
        if self.state not in RUNNING:
            return False, 'not_running'
        self._stop_leg()
        self.state, self.reason, self.target = 'paused', reason, None
        return True, 'paused'

    def resume(self, now):
        """Re-select from memory; wandering has no original target to insist on."""
        if self.state != 'paused':
            return False, 'not_paused'
        ok, reason = self.authority(now)
        if not ok:
            return False, reason
        self.last_tick = now; self.failures = 0; self.blocked = set()
        self._select(now)
        return True, 'resumed'

    def stop(self, now):
        if self.state not in RUNNING and self.state != 'paused':
            return False, 'not_running'
        self.finish(now, 'stopped_by_request')
        return True, 'stopped'

    def finish(self, now, reason, stop_leg=True):
        if stop_leg:
            self._stop_leg()
        else:
            self.leg = None
        self.state, self.reason, self.target, self.deadline = 'done', reason, None, None
        self._log(now, 'finish', reason=reason)

    # ---------- loop ----------
    def tick(self, now):
        if self.state in RUNNING:
            if self.last_tick is not None:
                self.active_s += max(0., now-self.last_tick)
            if self.p.session_budget_s > 0 and self.active_s >= self.p.session_budget_s:
                self.finish(now, 'budget_exhausted')
        self.last_tick = now
        if self.state not in RUNNING:
            return
        leg = self.leg
        if leg is not None:
            if now >= leg['deadline']:
                self._stop_leg(); self._leg_failed(now, 'leg_timeout', leg=leg)
            elif leg['accepted'] and now-leg['beat_at'] >= self.p.heartbeat_period_s:
                leg['beat_at'] = now; self.io.heartbeat(leg['request_id'], self.p.lease_s)
            return
        if self.state == 'acquire':
            self._acquire(now)
        elif self.state == 'yield':
            if self.close_since is None and now-self.person_seen_at >= self.p.resume_after_person_s:
                self._log(now, 'person_left'); self._select(now)
        elif self.state in ('observe', 'rest') and now >= self.deadline:
            if self.state == 'observe' and self.target is not None:
                self.target.visits += 1; self.target.cooldown_until = now+self.p.visit_cooldown_s
                self._log(now, 'visited', object=self.target.ident, label=self.target.label)
            self.target = None
            self._select(now)

    # ---------- steps ----------
    def _look(self, now):
        self.state = 'look'; self.reason = 'looking_around'
        self._begin_leg(now, '4', f'{self.p.look_turn_rad:.3f}', self.p.look_timeout_s)

    def _glance(self, now, angle, reason):
        self.state, self.reason = 'glance', reason
        self._begin_leg(now, '4', f'{angle:.3f}', self.p.turn_timeout_s)

    def _rest(self, now, seconds, reason):
        self.state, self.reason, self.deadline = 'rest', reason, now+seconds

    def _stalest_side(self, now):
        """Glance towards the heading sector seen longest ago (bounded to +-glance_max)."""
        ages = now-self.sectors.stamp
        index = int(np.argmax(np.where(np.isfinite(ages), ages, 1e9)+self.rng.uniform(0., 1., len(ages))))
        bearing = wrap(self.sectors.heading(index)-self.pose.yaw)
        return math.copysign(float(np.clip(abs(bearing), self.p.glance_min_rad, self.p.glance_max_rad)), bearing or 1.)

    def _select(self, now):
        rows = object_scores(self.objects, self.pose, now, self.start_xy, self.p.boundary_radius_m, self.p.weights)
        choice = choose_object(rows, self.rng, self.p.weights)
        if choice is not None:
            score, inst, parts = choice
            self.target = inst
            delta = inst.xy-self.pose.xy()
            dyaw = wrap(math.atan2(delta[1], delta[0])-self.pose.yaw)
            self.last_choice = dict(kind='curiosity', object=inst.ident, label=inst.label, score=round(score, 3), parts=parts)
            self._log(now, 'curiosity', **self.last_choice)
            if abs(dyaw) > self.p.turn_skip_rad:
                self.state, self.reason = 'turn_to', 'turning_to_'+inst.label
                self._begin_leg(now, '4', f'{dyaw:.3f}', self.p.turn_timeout_s)
            else:
                self._begin_acquire(now)
            return
        goal = None
        if self.grid is not None and now-self.grid_at <= self.p.grid_max_age_s:
            found = connected_goal(self.grid, self.visits, self.pose, now, self.start_xy, self.p.boundary_radius_m,
                                   self.rng, self.travel_heading, self.p.weights)
            if found is not None:
                score, goal, parts = found
                self.last_choice = dict(kind='wander', source='connected_region', score=round(float(score), 3), parts=parts)
        if goal is None:
            found = wander_goal(self.sectors, self.visits, self.pose, now, self.start_xy,
                                self.p.boundary_radius_m, self.rng, self.p.weights, blocked=self.blocked)
            if found is not None:
                score, index, goal = found
                self.last_choice = dict(kind='wander', source='sector', sector=index, score=round(float(score), 3))
        if goal is not None:
            self.no_goal = 0
            rel = self.pose.to_robot(goal)
            self.last_choice['goal_odom'] = [round(float(v), 2) for v in goal]
            self._log(now, 'wander', **self.last_choice)
            self.state, self.reason = 'wander', 'wandering'
            self._begin_leg(now, '3', f'{rel[0]:.3f},{rel[1]:.3f}', self.p.wander_timeout_s,
                            sector=self.last_choice.get('sector'), origin=self.pose.xy().tolist(),
                            goal=goal.tolist())
            return
        # Nothing reachable in view: glance aside first, a full turn only after that.
        self.no_goal += 1
        if self.no_goal <= self.p.no_goal_glances:
            self._glance(now, self._stalest_side(now), 'looking_for_open_space')
        elif self.no_goal == self.p.no_goal_glances+1:
            self._look(now)
        else:
            self.no_goal = 0; self.blocked = set()
            self._rest(now, self.p.no_goal_rest_s, 'no_open_space_waiting')

    def _begin_acquire(self, now):
        self.state, self.reason = 'acquire', 'finding_'+self.target.label
        self.deadline = now+self.p.acquire_timeout_s

    def _acquire(self, now):
        inst = self.target
        if now-self.visible_at <= self.p.detection_max_age_s:
            match = [(float(np.linalg.norm(xy-inst.xy)), row) for row, xy in self.visible
                     if row['label'] == inst.label and np.linalg.norm(xy-inst.xy) <= self.p.acquire_radius_m]
            if match:
                row = min(match, key=lambda item: item[0])[1]
                self.state, self.reason = 'approach', 'approaching_'+inst.label
                self._begin_leg(now, '2', row['id'], self.p.approach_timeout_s)
                return
        if now >= self.deadline:
            inst.failures += 1; inst.cooldown_until = now+self.p.failure_cooldown_s
            self._log(now, 'object_not_found', object=inst.ident)
            self.target = None; self._select(now)

    def _begin_leg(self, now, mode, target_id, timeout, **extra):
        self.leg_count += 1
        rid = f'explore-{self.session}-{self.leg_count}'
        self.leg = dict(request_id=rid, mode=mode, target_id=target_id, kind=self.state,
                        started=now, deadline=now+timeout, accepted=False, beat_at=now, **extra)
        self.io.start_leg(rid, mode, target_id, self.p.lease_s)

    def _stop_leg(self):
        if self.leg is not None:
            self.io.stop_leg(self.leg['request_id'])
        self.leg = None

    def _leg_done(self, now):
        leg, self.leg = self.leg, None
        self.failures = 0
        self._log(now, 'leg_done', kind=leg['kind'], seconds=round(now-leg['started'], 1))
        kind = leg['kind']
        if kind == 'look':
            self.blocked = set(); self._select(now)
        elif kind == 'glance':
            if self.reason == 'looking_for_open_space':
                self._select(now)
            else:
                self._rest(now, self.rng.uniform(self.p.rest_min_s, self.p.rest_max_s), 'resting')
        elif kind == 'turn_to':
            self._begin_acquire(now)
        elif kind == 'approach':
            self.state, self.reason, self.deadline = 'observe', 'observing_'+self.target.label, now+self.p.observe_s
        elif kind == 'wander':
            self.blocked = set(); self.legs_walked += 1
            delta = np.asarray(leg['goal'])-np.asarray(leg['origin'])
            self.walked_m += float(np.linalg.norm(self.pose.xy()-np.asarray(leg['origin'])))
            self.travel_heading = math.atan2(delta[1], delta[0])
            if self.rng.uniform() < self.p.glance_probability:
                side = self.rng.choice([-1., 1.])*self.rng.uniform(self.p.glance_min_rad, self.p.glance_max_rad)
                self._glance(now, side, 'glancing')
            else:
                self._rest(now, self.rng.uniform(self.p.rest_min_s, self.p.rest_max_s), 'resting')

    def _leg_failed(self, now, reason, leg=None):
        leg = leg or self.leg
        self.leg = None
        if leg is None:
            return
        self._log(now, 'leg_failed', kind=leg['kind'], reason=reason)
        if any(reason.startswith(r) for r in AUTHORITY_REASONS):
            self.finish(now, reason, stop_leg=False); return
        self.failures += 1
        if leg['kind'] in ('turn_to', 'approach') and self.target is not None:
            self.target.failures += 1; self.target.cooldown_until = now+self.p.failure_cooldown_s
            self.target = None
        elif leg['kind'] == 'wander' and leg.get('sector') is not None:
            self.blocked.add(leg['sector'])
        if self.failures >= self.p.max_consecutive_failures:
            # Keep wandering, but back off instead of hammering the backend.
            self.failures = 0
            self._rest(now, self.p.failure_rest_s, 'backing_off_after_failures:'+reason[:60])
            return
        self._select(now)

    def _log(self, now, event, **data):
        self.history.append(dict(t=round(now, 2), event=event, **data))
        del self.history[:-40]

    def status(self, now):
        objects = sorted(self.objects.instances, key=lambda o: -o.last_seen)[:12]
        return dict(schema='wla-explore-status/v1', state=self.state, reason=self.reason,
            session=self.session, active_s=round(self.active_s, 1), budget_s=self.p.session_budget_s,
            legs_walked=self.legs_walked, walked_m=round(self.walked_m, 1),
            pose_odom=[round(self.pose.x, 2), round(self.pose.y, 2), round(self.pose.yaw, 3)],
            start_odom=[round(float(v), 2) for v in self.start_xy],
            leg=None if self.leg is None else {k: v for k, v in self.leg.items() if k != 'beat_at'},
            target=None if self.target is None else self.target.summary(now),
            rest_remaining_s=round(self.deadline-now, 1) if self.state in ('rest', 'observe') else None,
            grid_age_s=None if self.grid is None else round(now-self.grid_at, 2),
            last_choice=self.last_choice, consecutive_failures=self.failures,
            sectors=[dict(heading=round(math.degrees(self.sectors.heading(i))), reach_m=round(float(self.sectors.reach[i]), 2),
                          age_s=round(now-self.sectors.stamp[i], 1)) for i in self.sectors.fresh(now, 60.)],
            objects=[o.summary(now) for o in objects], object_count=len(self.objects.instances),
            history=self.history[-10:])
