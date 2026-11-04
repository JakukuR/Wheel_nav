"""One geometry owner and a recovery-only appearance sidecar. No ROS I/O."""
from collections import deque
from copy import copy
from dataclasses import dataclass
import math
import numpy as np
from .sim_core.tracking import VisualTrack


def position(row, pose, offset=.28, expected_label='person'):
    if not isinstance(row, dict) or pose is None:
        return None
    p = np.asarray(pose, dtype=float)
    yaw, distance = row.get('yaw'), row.get('distance')
    if (row.get('detected') is not True or
            (expected_label is not None and row.get('label') != expected_label)
            or p.shape != (3,) or not np.isfinite(p).all()
            or not isinstance(yaw, (int, float)) or isinstance(yaw, bool)
            or not isinstance(distance, (int, float)) or isinstance(distance, bool)
            or not math.isfinite(yaw) or abs(yaw) > math.pi
            or not math.isfinite(distance) or not 0 < distance <= 20):
        return None
    angle = p[2] + yaw
    return p[:2] + (distance + offset)*np.array([math.cos(angle), math.sin(angle)])


# Followed-person Kalman noise, from replaying 2,949 recorded detections
# (2026-10-30 J02 15:02/16:02 and normal follow 19:43). Raw points jitter
# ~5 cm; the old 2.5 cm / 0.7 m/s^2 passed it through and the velocity
# direction jumped 0.25 rad per frame (p95). 5 cm / 0.5: 1 s prediction error
# 0.277 -> 0.250 m, direction jumps 0.13 rad, a sharp 90 deg turn is followed
# 0.2 s later (63 % after 0.80 s instead of 0.59 s).
TARGET_MEAS_STD_M = .05
TARGET_ACCEL_STD_MPS2 = .5
# Matches of one recovery candidate may be this far apart; frames without a usable
# feature in between (crop overlap/clipped, low confidence, not detected) neither
# count nor reset. 2026-11-02 16:09: a 0.75-0.78 match at 198.9/199.1/201.0 s was
# reset by such frames and the person was re-acquired only at 234 s.
RECOVERY_CONFIRM_GAP_S = 2.
# While the geometry follows the person with nobody near, a feature joins the
# gallery this often. Frames >= .90 only repeat the enrolled view: 11-03 19:37 the
# 8 slots filled in seconds, the walker's back then scored a median .54, and a
# reappearance at .45-.52 waited 24 s. The floor rejects a track that swapped.
TEMPLATE_REFRESH_S = 1.5
TEMPLATE_REFRESH_MIN_SIMILARITY = .40
# Driving to the last-seen point ends early only on explicit planner signals,
# never on distance alone (turning first, detours and short output gaps make no
# distance progress but are fine). 2026-11-02 16:38: the point was next to an
# obstacle; depth-coverage recovery backed the robot off 0.75 m short, it drove
# in again, for 37 s, and the search sweep never started.
SEARCH_COVERAGE_REASON = 'depth_coverage_recovery'       # nose at the obstacle: arrive now
SEARCH_BLOCKED_REASONS = ('no_feasible_candidate', 'backing_out')
SEARCH_BLOCKED_S = 2.                                     # in total, then arrive

@dataclass(frozen=True)
class TrackingPolicy:
    state: str
    last_seen: float | None
    prediction_s: float
    target_id: str
    revision: int
    search_heading: float | None = None
    search_arrival_m: float = .35
    search_started: float | None = None
    mode: str = '0'
    # +1 left / -1 right: side of the robot the person was last seen on; the
    # search sweep looks there first.
    search_side: float = 1.

    def translation_allowed(self, now):
        # A stale solution cannot extend prediction or start a new search leg.
        if self.mode == '2':
            return self.state == 'tracking' and self.last_seen is not None and \
                0 <= now-self.last_seen <= .3
        if self.state == 'search_last_seen':
            return True
        # Holding = the newest detection jumped away and was rejected; the track
        # itself is still the last trusted one, so keep driving on its prediction
        # like coasting (YOPOv2-Tracker rejects and keeps flying the same way).
        return (self.state in ('tracking', 'coasting', 'holding') and self.last_seen is not None
                and 0 <= now-self.last_seen <= self.prediction_s)

    def active_search(self, now):
        return self.state == 'search_heading'

    def rotation_allowed(self, now):
        return self.active_search(now)

    def limit_command(self, command, now):
        out = np.array(command, dtype=float, copy=True)
        if not self.translation_allowed(now):
            out[0] = 0.
            if not self.rotation_allowed(now):
                out[1] = 0.
        return out


class FollowTracker:
    """Only real, distinct geometry frames update the single Kalman tracker."""
    def __init__(self, target_id, now, *, prediction_s=2., freshness_s=.3,
                 offset=.28, jump_radius=.45, jump_growth=.25, jump_max=1.5,
                 confirmations=3, search_arrival=.35, history_window_s=5.,
                 history_sample_period_s=.4, expected_label='person', mode='0'):
        self.selected_id = self.target_id = target_id
        self.expected_label = expected_label
        self.mode = str(mode)
        self.started = now
        self.prediction_s, self.freshness_s, self.offset = prediction_s, freshness_s, offset
        self.jump_radius, self.jump_growth, self.jump_max = jump_radius, jump_growth, jump_max
        self.confirmations, self.search_arrival = confirmations, search_arrival
        self.history_options = dict(history_window_s=history_window_s,
            history_sample_period_s=history_sample_period_s,
            history_max_gap_s=min(2., max(.1, freshness_s*2)))
        self.track = None
        self.observations = deque(maxlen=32)
        self.last_seen = None
        self.last_frame = -math.inf
        self.last_position = None
        self.last_bearing = None
        self.heading = None
        self.exit_side = 1.
        self.edge_observations = 0
        self.velocity_evidence = 0
        self.hard_break = False
        self.missing_since = None
        self.revision = 0
        self.return_count = 0
        self.return_stamp = None
        self.reason = 'waiting_initial_geometry'
        self.last_error = None
        self.search_started = None
        self.search_arrived_by = None
        self.search_blocked_s, self.search_blocked_at = 0., None
        self.sweep_started = None

    def prediction(self, now):
        if self.track is None:
            return None
        # Without a measurement update() only rebinds x/P/time (the trail is
        # untouched), so a shallow copy leaves the live track unchanged.
        return copy(self.track).update(None, now)[0]

    def radius(self, now):
        age = 0. if self.last_seen is None else max(0., now-self.last_seen)
        return min(self.jump_max, self.jump_radius+self.jump_growth*age)

    def observed_position(self, stamp):
        """Geometry for the mask acquisition, independent of MPC worker delay."""
        nearest = min(self.observations, key=lambda item: abs(item[0]-stamp), default=None)
        return nearest[1].copy() if nearest is not None and abs(nearest[0]-stamp) <= .05 else None

    def extrapolated_position(self, stamp, horizon=.3):
        """Last observation moved by the track velocity, at most `horizon` s ahead.

        For a depth frame newer than the detector's latest result; it only places
        the removal cylinder, never a target measurement.
        """
        exact = self.observed_position(stamp)
        if exact is not None or not self.observations:
            return exact
        t, point = max(self.observations, key=lambda item: item[0])
        if not 0 <= stamp-t <= horizon:
            return None
        velocity = np.zeros(2) if self.track is None else self.track.x[2:]
        return point + velocity*(stamp-t)

    def _accept(self, point, pose, stamp):
        if self.track is None:
            self.track = VisualTrack(timeout=self.prediction_s, meas_std=TARGET_MEAS_STD_M,
                                     accel_std=TARGET_ACCEL_STD_MPS2, **self.history_options)
            self.observations.clear()
        self.track.update(point, stamp)
        velocity = self.track.x[2:].copy()
        dt = None if self.last_seen is None else stamp-self.last_seen
        measured = None if dt is None or dt <= 0 else (point-self.last_position)/dt
        reliable = (measured is not None and dt <= self.freshness_s*2
                    and .15 <= np.linalg.norm(velocity) <= 1.
                    and np.linalg.norm(measured-velocity) <= .5)
        self.velocity_evidence = self.velocity_evidence+1 if reliable else 0
        self.last_bearing = float(math.atan2(point[1]-pose[1], point[0]-pose[0]))
        relative = (self.last_bearing-float(pose[2])+math.pi) % (2*math.pi)-math.pi
        self.exit_side = 1. if relative >= 0 else -1.
        self.heading = float(math.atan2(velocity[1], velocity[0])) if self.velocity_evidence >= 3 else self.last_bearing
        self.last_position, self.last_seen = point.copy(), float(stamp)
        self.observations.append((float(stamp),point.copy()))
        self.hard_break = False
        self.missing_since = None
        self.return_count = 0
        self.search_started = None
        self.search_arrived_by = None
        self.search_blocked_s, self.search_blocked_at = 0., None
        self.sweep_started = None
        self.reason = 'same_id_geometry'

    def observe(self, objects, pose, stamp, now):
        if not math.isfinite(stamp) or not 0 <= now-stamp <= self.freshness_s or stamp <= self.last_frame:
            return False
        # A temporarily unavailable image-time pose may be retried next tick.
        if pose is None:
            return False
        self.last_frame = stamp
        point = position(objects.get(self.target_id), pose, self.offset, self.expected_label)
        if point is None:
            self.return_count = 0
            if self.missing_since is None:
                self.missing_since = stamp
            self.reason = 'target_not_in_current_frame'
            return False
        predicted = self.prediction(stamp)
        row = objects.get(self.target_id)
        if predicted is not None and isinstance(row, dict) and row.get('image_edge') is True:
            # Box cut by the left/right image border: the depth sample is mostly
            # background (J02/normal follow 2026-11-02: 0.7-2.3 m range jumps at
            # +-0.55 rad). Keep the bearing, take the range from the prediction.
            here = np.asarray(pose[:2], dtype=float)
            ray = point-here
            point = here+float(np.linalg.norm(predicted-here))*ray/max(1e-9, float(np.linalg.norm(ray)))
            self.edge_observations += 1
        self.last_error = None if predicted is None else float(np.linalg.norm(point-predicted))
        if predicted is not None and self.last_error > self.radius(stamp):
            self.hard_break = True
            if self.missing_since is None:
                self.missing_since = stamp
            self.return_count = 0
            self.reason = 'geometry_jump_requires_recovery'
            return False
        # First inspect CURRENT evidence; worker scheduling delay is not loss.
        long_gap = self.last_seen is not None and stamp-self.last_seen > self.prediction_s
        if self.hard_break or long_gap:
            other = [position(row, pose, self.offset, self.expected_label)
                     for ident,row in objects.items() if ident != self.target_id]
            ambiguous = any(p is not None and np.linalg.norm(p-point) < .5 for p in other)
            if ambiguous:
                self.return_count = 0
                return False
            self.return_count = self.return_count+1 if self.return_stamp is not None and 0 < stamp-self.return_stamp <= .6 else 1
            self.return_stamp = stamp
            if self.return_count < self.confirmations:
                self.reason = 'confirming_same_id_geometry'
                return False
            self.track = None
            self.velocity_evidence = 0
            self.revision += 1
        self._accept(point, pose, stamp)
        return True

    def snapshot(self, now, robot_pose=None):
        age = None if self.last_seen is None else max(0., now-self.last_seen)
        if age is None:
            state = 'binding'
        elif self.hard_break and age <= self.prediction_s:
            state = 'holding'
        elif age <= self.freshness_s and self.missing_since is None:
            state = 'tracking'
        elif age <= self.prediction_s:
            state = 'coasting'
        else:
            arrived = (robot_pose is not None and np.linalg.norm(np.asarray(robot_pose[:2])-self.last_position) <= self.search_arrival+.10)
            if arrived and self.search_started is None:
                self.search_arrived_by = 'distance'
                # The sweep starts on arrival; it no longer first aligns with the
                # walking direction (that cost up to ~1 s of turning).
                self.search_started = self.sweep_started = now
            state = ('coasting' if self.mode == '2' else
                     ('search_heading' if self.search_started is not None else 'search_last_seen'))
        if self.mode == '2' and state != 'tracking':
            state = 'holding'
        estimate = self.prediction(now)
        velocity = np.zeros(2) if self.track is None else self.track.x[2:].copy()
        if state not in ('tracking','coasting'):
            velocity[:] = 0.
        return dict(state=state, mode=self.mode, expected_label=self.expected_label, surface_offset_m=self.offset,
            reason=self.reason, locked_id=self.selected_id,
            resolved_id=self.target_id, revision=self.revision, last_seen=self.last_seen,
            last_seen_age_s=age, prediction_s=self.prediction_s,
            position=None if self.last_position is None else self.last_position.tolist(),
            prediction=None if estimate is None else estimate.tolist(), velocity_mps=velocity.tolist(),
            history=[] if self.track is None else [p.tolist() for p in self.track.history],
            history_sampling=None if self.track is None else self.track.trail.diagnostic(),
            search_heading=self.heading, search_arrival_m=self.search_arrival,
            search_started=self.sweep_started, search_arrived_at=self.search_started,
            search_arrived_by=self.search_arrived_by, search_blocked_s=round(self.search_blocked_s, 2),
            search_side=self.exit_side, edge_range_observations=self.edge_observations,
            missing_since=self.missing_since, last_position_error_m=self.last_error,
            position_jump_limit_m=self.radius(now), recovery_needed=state!='tracking')

    def report_planner(self, reason, policy_state, now):
        """Planner outcome of a plan made while driving to the last-seen point."""
        lost = self.last_seen is not None and now-self.last_seen > self.prediction_s
        if (self.mode == '2' or policy_state != 'search_last_seen' or not lost
                or self.search_started is not None):
            return
        if reason == SEARCH_COVERAGE_REASON:
            by = 'coverage'
        elif reason in SEARCH_BLOCKED_REASONS:
            if self.search_blocked_at is not None and 0 < now-self.search_blocked_at <= .3:
                self.search_blocked_s += now-self.search_blocked_at
            self.search_blocked_at = now
            if self.search_blocked_s < SEARCH_BLOCKED_S:
                return
            by = 'blocked'
        else:
            self.search_blocked_at = None
            return
        self.search_started = self.sweep_started = now
        self.search_arrived_by = by

    def rebind(self, proposal, objects, pose, stamp, now):
        if (proposal is None or proposal['revision'] != self.revision
                or not 0 <= now-proposal['stamp'] <= .6 or proposal['count'] < self.confirmations
                or not 0 <= now-stamp <= self.freshness_s):
            return False
        # Appearance-confirmed: no position gate (RecoveryIdentity picked the best
        # match among the visible people).
        point = position(objects.get(proposal['id']), pose, self.offset, self.expected_label)
        if point is None:
            return False
        self.target_id = proposal['id']
        self.revision += 1
        self.track = None
        self.velocity_evidence = 0
        self._accept(point, pose, stamp)
        self.reason = 'reid_reacquired'
        self.last_frame = stamp
        return True


class RecoveryIdentity:
    """Collect indefinitely; never alter geometry, command permission or task life."""
    def __init__(self, *, similarity=.55, margin=.08, confirmations=3):
        # No position gate (2026-11-02: the walker kept going past the 1.5 m gate
        # while the robot searched): the best-matching visible person wins if it is
        # above the same-person 5th percentile (.55; the walker scored down to .31,
        # median .75, a bystander .49-.51) and leads the next person by margin.
        self.threshold, self.margin, self.confirmations = similarity, margin, confirmations
        self.last_template_added = -math.inf
        # Enrollment consistency is independent of the recovery acceptance threshold.
        self.enrollment_similarity = .80
        self.gallery = []  # First two seed templates survive later appearance updates.
        self.provisional = None
        self.last_frame_stamp = -math.inf
        self.pending_id = None
        self.pending_count = 0
        self.pending_stamp = None
        self.proposal = None
        self.state, self.reason = 'collecting', 'waiting_for_quality_template'
        self.feature_frames = self.reacquired = 0
        self.last_similarity = None
        self.candidates = []
        self.feature_diagnostics = {}

    @property
    def complete(self):
        return len(self.gallery) >= 2

    def reset_confirmation(self):
        self.pending_id = self.pending_stamp = None
        self.pending_count = 0
        self.proposal = None

    def observe(self, rows, stamp, diagnostics, tracker, snapshot, *, neighbor=False):
        if stamp <= self.last_frame_stamp:
            return
        self.last_frame_stamp = stamp
        self.feature_frames += 1
        self.feature_diagnostics = dict(diagnostics or {})
        valid = []
        seen = set()
        for row in rows:
            try:
                ident = row['id']; f = np.asarray(row['feature'], dtype=float); p = np.asarray(row['position'],dtype=float)
                if (ident in seen or f.shape != (512,) or p.shape != (2,) or not np.isfinite(f).all()
                        or not np.isfinite(p).all() or np.linalg.norm(f) < 1e-6
                        or self.feature_diagnostics.get(ident,'feature_ready') != 'feature_ready'):
                    continue
                seen.add(ident)
                valid.append(dict(id=ident,feature=f/np.linalg.norm(f),position=p))
            except (KeyError,TypeError,ValueError):
                continue
        self.candidates = []
        # Only currently accepted geometry may train a template. Delayed feature
        # timestamps must also lie in the recent continuous track, never its gap.
        visible = (snapshot['state']=='tracking' and tracker.last_seen is not None
                   and 0 <= tracker.last_seen-stamp <= .3)
        target = next((r for r in valid if r['id']==tracker.target_id), None)
        if visible:
            self.reset_confirmation()
            if target is None or neighbor:
                self.state='ready' if self.complete else 'collecting'
                self.reason='template_quality_unavailable' if not neighbor else 'template_neighbor_ambiguity'
                return
            sim = None if not self.gallery else max(float(target['feature']@g) for g in self.gallery)
            self.last_similarity = sim
            if not self.complete:
                prior = self.provisional
                if (prior is not None and prior[0]==tracker.revision and 0 < stamp-prior[1] <= .6
                        and float(target['feature']@prior[2]) >= self.enrollment_similarity):
                    self.gallery.extend([prior[2].copy(),target['feature'].copy()])
                    self.provisional=None
                else:
                    # Retry a new pair indefinitely, including appearance mismatch.
                    self.provisional=(tracker.revision,stamp,target['feature'].copy())
            elif sim >= TEMPLATE_REFRESH_MIN_SIMILARITY and stamp-self.last_template_added >= TEMPLATE_REFRESH_S:
                self.gallery.append(target['feature'].copy())
                self.last_template_added = stamp
                if len(self.gallery)>8:del self.gallery[2]
            self.state='ready' if self.complete else 'collecting'
            self.reason='template_ready' if self.complete else 'collecting_template_pair'
            return
        self.provisional=None
        if not self.complete:
            self.reset_confirmation();self.state='collecting';self.reason='recovery_template_unavailable'
            return
        self.state='searching'
        prediction=tracker.prediction(stamp)
        scored=sorted(((max(float(row['feature']@g) for g in self.gallery),row) for row in valid),
                      key=lambda item:item[0],reverse=True)
        for sim,row in scored:
            error=math.inf if prediction is None else float(np.linalg.norm(row['position']-prediction))
            self.candidates.append(dict(id=row['id'],similarity=sim,position_error_m=error,qualified=False))
        if not scored:
            # Nothing usable this frame: keep any pending confirmation.
            if self.pending_id is None:self.reason='no_usable_recovery_feature'
            return
        sim,row=scored[0]
        if sim<self.threshold or (len(scored)>1 and sim-scored[1][0]<self.margin):
            self.reset_confirmation();self.reason='no_unambiguous_recovery_candidate';return
        self.candidates[0]['qualified']=True
        same=(row['id']==self.pending_id and self.pending_stamp is not None
              and 0<stamp-self.pending_stamp<=RECOVERY_CONFIRM_GAP_S)
        self.pending_count=self.pending_count+1 if same else 1
        self.pending_id=row['id'];self.pending_stamp=stamp;self.last_similarity=sim
        self.reason='confirming_recovery'
        if self.pending_count>=self.confirmations:
            self.proposal=dict(id=row['id'],stamp=stamp,count=self.pending_count,revision=tracker.revision)

    def snapshot(self):
        return dict(state=self.state,reason=self.reason,role='recovery_only',completed_enrollment=self.complete,
            template_slots=len(self.gallery),feature_frames=self.feature_frames,
            similarity=self.last_similarity,pending_id=self.pending_id,pending_confirmations=self.pending_count,
            reacquired=self.reacquired,candidates=self.candidates,feature_diagnostics=self.feature_diagnostics)


def frame_target_ready(packet, *, target_id, revision, request_id, session, epoch,
                       stamp_ns, frame_id, shape, now_ns, mask_applied):
    """A missing target is explicit detector evidence, not a missing mask packet."""
    if (not isinstance(packet,dict) or packet.get('schema')!='wla-target-mask/v1'
            or type(packet.get('stamp_ns')) is not int
            or type(packet.get('target_revision')) is not int
            or packet.get('request_id')!=request_id or packet.get('session')!=session
            or packet.get('epoch')!=epoch or packet.get('requested_target_id')!=target_id
            or packet.get('target_revision')!=revision or packet.get('depth_stamp_ns')!=stamp_ns
            or packet.get('frame_id')!=frame_id or (packet.get('height'),packet.get('width'))!=shape
            or not 0 <= now_ns-packet.get('stamp_ns',0) <= 300_000_000
            or abs(packet.get('stamp_ns',0)-stamp_ns)>35_000_000):
        return False
    return packet.get('target_present') is False or (packet.get('target_present') is True and mask_applied)
