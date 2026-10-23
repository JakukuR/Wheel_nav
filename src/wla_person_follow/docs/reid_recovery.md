> 历史设计记录：注册等待、ReID 超时及身份否决逻辑已删除。当前实现见 [tracker_follow.md](tracker_follow.md)，ReID 仅作为恢复旁路。

# Target-only ReID and bounded identity recovery — 2026-10-20

Requested policy: extract appearance only for the explicitly selected target.
While lost, first restrict candidates by loss age and predicted odom position,
then compare appearance only for that small candidate set. Idle does not extract
person descriptors; model startup runs one synthetic zero-crop warmup.

Reference inspected: https://github.com/6-robot/jie_deamon at
9403185c0cf99a0b415ddd4f3b4215b44dd0809c. Its lidar_tracker.hpp associates
scan points within TARGET_RADIUS, updates a centroid and optionally smooths it
with a Kalman filter. kalman_filter.hpp exposes predictOnly, but processScan
has no call to it, no visual ReID and no complete identity recovery. We reuse
that position-estimation idea, not its laser inputs or velocity controller.

Architecture:
- Pinned upstream OSNet x0.25/MSMT17 from the author's model repository; model
  SHA and revisions in config/reid_model.json. MIT architecture license is
  included under wla_person_follow/vendor. Weights are loaded with
  torch.load(weights_only=True); only classifier weights are omitted.
- Independent read-only CPU sidecar, nice 5, torch threads 1, at most 5 Hz and
  at most 4 temporally/spatially eligible recovery candidates. Normal tracking
  requests only one ID. An empty allowed-ID list does no person model inference.
- RGB, raw YOLO boxes and relative RGB-D geometry must share EXACT source
  timestamps and detector generation. Bounded 16-frame caches, newest job,
  no accumulating work queue. Reject result age >0.6 s and stale requests >0.5 s.
- Follower requests work via /r680/person_follow/reid_request. Sidecar publishes
  /r680/person_follow/reid_features and /reid_status only; no velocity/service
  client. Stop/request changes invalidate in-flight feature responses.
- Only full-size enough person crops are used; small boxes, heavily truncated
  crops and overlapping people are rejected. Unselected boxes are examined
  solely to detect overlap, never encoded during normal tracking.

Association:
- A new follow request enrolls only its explicit target, using two distinct
  descriptor frames. No motion target is supplied until enrollment completes.
- Keep a small RAM-only gallery (up to 8 samples); strong continuing matches
  >=0.90 may update it. No photos or appearance gallery are saved persistently.
- Match cosine >=0.80, maximum loss 8 s, spatial gate min(1.5, 0.45+0.25*age) m.
  Position is transformed with wheel odometry at image acquisition; bounded
  constant-velocity prediction is capped at 1.5 s, speed estimate at 1.5 m/s.
- Reacquisition requires three different consecutive frames, gaps <=0.6 s,
  and similarity lead >=0.08 over any second eligible person. More than four
  spatial candidates are treated as ambiguous rather than silently dropping
  competitors. Different-ID candidates cannot initialize the gallery.
- Keep the original request ID/locked target ID and resolve observations to the
  verified current track ID. Remove a rebound target from the bystander tracks
  to avoid self-collision. Existing map, MPC, mux, lease and freshness checks stay.
- Features never renew current geometry: control still needs <=0.30 s RGB-D and
  recent verified appearance. No depth/position => no reacquisition.
- Epoch/session/reference changes still reset or terminate according to existing
  policy. This version recovers within the current detector generation; it does
  not silently resume a task after detector/camera restart or an operator stop.

Usage:
```
cd ~/ros2_ws
./r680_follow_trial.sh list
./r680_follow_trial.sh follow --target-id 'FULL_PERSON_ID' --continuous --allow-motion
```
The deployed wrapper enables ReID by default and launches the sidecar in the
existing perception virtual environment. First give the target a sufficiently
clear crop for enrollment. Diagnostics expose reid.state, reference_samples,
lost_s, similarity, position_error_m, confirm_count and resolved_target_id.
The CLI prints these fields; Web highlights the resolved ID and shows ReID state.
`enable_reid:=false` on the launch wrapper is an explicit rollback option.

Validation:
- 40 tests: resolver, preview-only ROS rebind/ambiguity, existing adapter and
  mapping/controller regressions. 11 further tests: real model smoke/crop rules,
  isolated sidecar target-only/idle/stamp/expired-request behavior, trial client.
- ROS tests use domain 198 and localhost discovery. No physical motion or live
  follow request is initiated. Model smoke checks output shape, normalization
  and deterministic output, not real-world identity accuracy.
- Initial CPU benchmark on Orin: single crop median ~53 ms, batch four ~117 ms;
  shared-load latency is not a guarantee. No new GPU inference is used.
- Thresholds are initial engineering settings, not measured false-match rates.
  Real multi-person occlusion/reappearance quality remains to be tested with an
  operator. Read-only deployment evidence is in the accompanying local report.
