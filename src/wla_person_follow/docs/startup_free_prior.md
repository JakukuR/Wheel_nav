# Explicit 0.5m startup free-space prior — 2026-10-20

Superseded in the deployed configuration by [continuous_local_free.md](continuous_local_free.md).
The one-shot mode below remains available but is disabled on this vehicle.

User requested the real-vehicle equivalent of the simulation's initial free
patch, with radius0.5m. Deployed `startup_clear_radius_m: 0.5` in sim_follow.yaml;
the node default remains0 (disabled) for other deployments. Allowed range0..0.5m.

Policy:
- The worker creates the patch once, on the first active follow job after node
  startup, centered at that job's control-time wheel-odom pose. Idle mapping does
  not consume it, so it cannot expire before the user starts their first trial.
- This is an operator-authorized starting assumption, not D455 measurement.
- Only unknown/expired cells within the circle are initialized. Existing
  observed free/occupied cells are preserved. Current depth endpoints are applied
  afterward and can mark an obstacle inside the prior.
- Untouched assumed-free cells are held while the robot has not translated.
  Wheel-odometry XY displacement of at least `startup_motion_threshold_m: 0.03`
  from the original center starts the 12 s countdown. Stationary jitter below
  this threshold and rotation alone do not start it. Stopping or returning to
  the starting position after movement does not pause or reset the countdown.
- Holding refreshes timestamps only for untouched prior cells. Any real sensor
  update retires the affected cells from the hold permanently; those cells then
  use normal sensor TTL. New obstacles are never cleared by the hold, including
  after their observations expire. Ring-slot ownership is checked before refresh.
- Task/target changes neither renew nor recenter the prior. The countdown uses
  control-time odometry timestamps, not commanded velocity or webpage timing.
- Task/identity changes reset tracking/controller state but preserve the rolling
  obstacle map in the same wheel reference. A wheel-reference generation change
  discards that map and does not create a new startup prior. A node restart starts
  a fresh one-shot policy.
- Status reports configured radius, consumed flag and startup_prior provenance,
  center, creation/motion-start/expiry timestamps, movement threshold, reference
  generation and seeded cell count. State is `waiting_for_translation` (expiry
  and remaining seconds are null), `counting_down`, `expired`, or
  `reference_changed`.
  Prior status is attached to the last processed map result. The map's green
  free cells may include this explicit assumption during its lifetime.

Chassis radius0.19m, obstacle margin0.10m, sensor freshness, candidate collision
checks, mux protocol and command leases remain unchanged. ToF stays disabled.
No automatic follow request is created by enabling this configuration.

Verification:
- Python package built into existing versioned simulation overlay.
- 18 port tests pass, including long stationary hold, translation threshold,
  rotation/jitter, exact 12 s countdown, stopping/returning without reset,
  sensor evidence retirement, ring-buffer wrapping, request/reference changes,
  disabled mode and invalid parameter rejection.
- 5 isolated ROS tests pass in domain198, including depth/TF/wheel mapping with
  radius0.5 configured but idle: no assumed patch consumed; existing stop/lease/
  mux tests pass. These tests publish only in the isolated test domain.
- Real captured D455 frame + explicitly synthetic target at [2,0]m, offline:
  radius0 -> forward_known0/225, zero translation, forward_space_unobserved;
  radius0.5 -> forward_known225/225 and225 forward feasible candidates.
  This is a geometry/controller snapshot check, not a real follow/motion trial.
- Deployment reloads only the inactive follow launch. A fresh live status check
  verifies radius 0.5 m, threshold 0.03 m, idle, startup_prior null and
  startup_prior_consumed false before first follow. No physical movement trial
  or follow request is performed for this timer change.

Runtime log/PID: ~/ros2_ws/.runtime/prior-motion-follow.{log,pid}.
Evidence: .runtime/prior-motion-{build,unit,ros}.log and
.runtime/prior-motion-live.json. Local copies accompany the deployment report.
