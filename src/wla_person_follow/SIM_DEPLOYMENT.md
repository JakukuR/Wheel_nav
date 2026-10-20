# close-wide-flex vehicle port — 2026-10-20

This backend is the frozen local simulation strategy, not the old proportional
follower. See SIM_CORE_PROVENANCE.json. It preserves WideTurnTuning, the 24-step
sampled differential MPC, CV target/history, rolling 12 m / 0.1 m / 12 s map,
outside reference, flexible 40 cm preferred / 10 cm minimum modeled clearance,
wait/retreat/replan and bounded predicted-bearing search. max_v=.65 m/s,
max_w=.9 rad/s, accel=.65 m/s², yaw_accel=1.5 rad/s² are UNCHANGED simulation
parameters, not measured chassis capabilities. The footprint/person circles
remain .24/.28 m model assumptions. No claim of physical clearance calibration.

`r680_person_follow.sh run` selects backend=sim and enable_motion=false by
default after the deployment overlay is activated. `backend:=legacy` selects
the unchanged legacy node/config; wla_object_follow still imports that node.
The sim backend supports `follow` only; `approach_interact` returns a clear
rejection, rather than silently substituting another strategy. Use legacy for
that mode. Existing PersonCommand start/heartbeat/stop/status and 0.1–2 s lease
semantics remain. There is no auto-selection of a person and no ReID yet.

## Output

Default output_mode=mux uses `/r680/person_follow/command_candidate` and
`/r680/person_follow/status`, with the existing mux as the sole chassis publisher.
The mux now uses fresh-velocity priority without PERSON/AUTO/status gates.
The follower still owns task identity, lease, perception and planner validity.
Preview remains disabled for motion. Optional output_mode=direct
targets `/r680_base/cmd_vel`. In direct mode enable_motion=false
creates NO publisher on that topic, including no periodic zeros. Preview is
`/r680/person_follow/cmd_vel_candidate`. Enabling direct output requires the
existing command mux and all other publishers to relinquish that topic; only
the chassis subscriber should remain. The node waits one second for discovery
and latches a conflict if any second publisher is seen, even with the same node
name. A conflict requires restart after the publisher is removed. Discovery is
not an atomic hardware interlock; verify exclusive ownership before operation.
This implementation does not stop services or enable motion during deployment.

Direct output bypasses the mux's manual takeover/estop policy. The local
`/r680/person_follow/set_estop` service (SetBool) stops and cancels the task;
reset does not resurrect it. It does not replace a physical stop mechanism or
a driver command timeout. Driver timeout and measured chassis braking remain
unverified. output_mode=mux retains the candidate topic and status diagnostics;
velocity_passthrough=true removes PERSON/AUTO switching from this path. Never operate direct and mux output concurrently.

## Real inputs and deliberate hardware adapter boundaries

- Real aligned D455 depth + image-time TF + CameraInfo. Deprojection and SE(3)
  filtering run in the new C++ `_wla_sim_geometry` module (GIL released).
- Motion reference integrates existing wheel twist with timestamp interpolation;
  reset/invalid wheel data resets reference and prediction. No global map/VIO
  dependency, but planar wheel drift/pitch/roll remain limitations.
- Shared acquisition-stamped relative person observations, not simulator actor
  coordinates. The detector already transforms measured depth into base.
  Its surface position receives an explicit .28 m model center offset. This
  is an approximation, not a measured human anatomical center.
- No ideal instance masks: all actual depth hits remain obstacle evidence,
  even when person prediction also represents them. This is conservative and
  can leave transient trails; it does not erase whole bounding boxes.
- No simulated clear-start patch. Unseen near/side/rear space remains unknown.
  Without ToF, this can legitimately prevent starting, turning or reversing.
- ToF input interfaces are PointCloud2 x/y/z, <=256 points/head, timestamped TF,
  disabled by default. No connected sensor means no fabricated free evidence.
  The recovery policy itself is unchanged; reverse requires its existing
  known-clear rear path check, potentially using prior observed history.
- No feasible MPC sample: preserve core output for parity tests but hardware
  adapter vetoes the demo braking fallback and emits zero. This is not proof
  that a moving chassis instantaneously stops.
- Stale depth, wheel, detection stream, TF failure, expired task, stale solver
  output or local estop veto hardware output. Core parameters/costs are unchanged.
- Predictions use actual source time; expired computation is never kept alive
  by periodic republication. One spawned compute process owns map/controller, with at most one in-flight
  job; a separate 20 Hz ROS callback checks health/output. Process isolation
  avoids Python GIL contention with ROS callbacks. Queue wait and copies are
  included in the result deadline. This is not hard realtime.

The numerically identical planner remains Python/NumPy/SciPy in this first
port; C++ rewrites of MPC/map are deferred until this parity baseline is frozen.
SciPy distance transforms already run in native code. Do not call this an
all-C++ deployment. ReID is explicitly deferred at the user's request.

## Verification and rollback

Tests cover original/ported algorithm parity, native projection against NumPy,
real message adapters, request/lease/data expiry and isolated direct publisher
conflicts. ROS tests MUST use domain 198 and mock topics, without a chassis.
Software tests do not establish real D455/identity/braking reliability.

Deployment uses a versioned non-symlink install overlay, activated only by the
person startup script. Rollback restores that script's saved previous version
or removes its single overlay source line, then starts backend=legacy from a
fresh shell with motion disabled. Do not source a new install and then try to
undo it in the same shell. No whole-workspace reset or driver restart is needed.
