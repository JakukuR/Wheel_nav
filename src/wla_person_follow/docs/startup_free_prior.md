# Explicit 0.5m startup free-space prior — 2026-10-20

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
- Normal map TTL12s applies; no timer, request, target switch or controller loop
  renews/recenters the startup prior. Measured cells have their normal timestamps.
- Task/identity changes reset tracking/controller state but preserve the rolling
  obstacle map in the same wheel reference. A wheel-reference generation change
  discards that map and does not create a new startup prior. A node restart starts
  a fresh one-shot policy.
- Status reports configured radius, consumed flag and startup_prior provenance,
  center, creation/expiry timestamps, reference generation and seeded cell count.
  Prior status is attached to the last processed map result. The map's green
  free cells may include this explicit assumption during its lifetime.

Chassis radius0.19m, obstacle margin0.10m, sensor freshness, candidate collision
checks, mux protocol and command leases remain unchanged. ToF stays disabled.
No automatic follow request is created by enabling this configuration.

Verification:
- Python package built into existing versioned simulation overlay.
- 15 port tests pass, including one-shot application after long idle, exact0.5m
  extent, preservation of old/new obstacle evidence, expiry, request changes,
  reference reset, disabled mode and invalid parameter rejection.
- 5 isolated ROS tests pass in domain198, including depth/TF/wheel mapping with
  radius0.5 configured but idle: no assumed patch consumed; existing stop/lease/
  mux tests pass. These tests publish only in the isolated test domain.
- Real captured D455 frame + explicitly synthetic target at [2,0]m, offline:
  radius0 -> forward_known0/225, zero translation, forward_space_unobserved;
  radius0.5 -> forward_known225/225 and225 forward feasible candidates.
  This is a geometry/controller snapshot check, not a real follow/motion trial.
- Reloaded only inactive follow launch (parent703481); camera, detector and web
  services remain running. Live status verified configured radius0.5, idle,
  startup_prior null and startup_prior_consumed false before first follow.

Runtime log/PID: ~/ros2_ws/.runtime/startup-prior-follow.{log,pid}.
Evidence: .runtime/startup-prior-{build,unit,ros}.log and
.runtime/startup-prior-{before,after}.json. Local copies accompany this report.
