# Following latency, 2026-10-20

Runtime: Orin ROS 2 Jazzy / CycloneDDS; production domain 73. Isolated
validation uses domain 198 and never publishes to the production chassis.

Changes:

- Stream clearing-ray samples to a C++ bitmap instead of constructing large
  NumPy ray tensors; C++ distance-field queries retain the conservative cell
  and query-position offsets. No changed map TTL, collision margins, candidate
  horizon, speed limits or unknown-space rules. Standalone NumPy fallback remains.
- Freeze eval-only OSNet x0.25 on CPU in FP32, folding batch normalization.
  Descriptor/model identity and appearance/position thresholds stay unchanged.
  Target-only normal extraction remains 5 Hz; enrollment/recovery is capped at
  10 Hz. Exact-stamp latest-only data wakes inference; idle encodes nobody.
- Publish the initial selected-target interest immediately. Refresh geometry
  and received ReID batches after depth preprocessing, before solver submission.
- Wake the ROS executor on a completed solution. Existing 20 Hz checks still
  enforce expiry, lease, stop and authority. A guard wake uses the same output
  function, not a second unchecked path. Coalesce duplicate map/path publications.
- Status `latency` records acquisition-to-processing/result/candidate ages and
  first-result dispatch delay. These are software timestamps, not motor response.

Measured on the Orin with the camera/detector/viewer still running:

| Workload | Before median / P95 ms | After median / P95 ms |
|---|---:|---:|
| Same captured depth packet + synthetic target/other-person numerical replay, 70 measured steps each, no profiler | 47.11 / 57.66 | 26.72 / 33.80 |
| OSNet single crop, 25 iterations | 50.73 / 53.34 | 27.28 / 28.78 |
| OSNet four-crop recovery batch, 25 iterations | 103.59 / 104.59 | 70.60 / 73.01 |
| Isolated synthetic RGB-D to first follow candidate, warm model, 73/76 samples | 100.89 / 155.64 | 75.51 / 99.42 |
| ReID source acquisition to features in that replay | 173.63 / 200.42 | 145.34 / 161.58 |

Two-frame target enrollment in that replay: 454.54 -> 289.70 ms (one trial
per version, not a percentile/identity-accuracy claim). Replay injects an 80 ms
detection delay, so it does not benchmark actual YOLO or physical motion.
Live pre-change acquisition-to-relative-detection median/P95: 97.56/114.13 ms.
Runs are short and subject to system load; these are not real-time guarantees.

49 distinct relevant tests passed: native/reference occupancy and collision
parity including negative grid coordinates and ring wrap; frozen/eager embeddings;
target-only sidecar, exact stamps, stale requests, bounded recovery rate; identity
ambiguity/rebind; ROS stop, expiry, mux, and immediate-result output checks.
All 80 numerical replay commands, rejection reasons and candidate counts matched.
Maximum tested frozen/eager descriptor error was below 4e-7 (FP32).

The planner remains at 10 Hz with the simulation's 100 ms prediction step;
raising its rate without reviewing per-step acceleration semantics was avoided.
The mux retains its 20 ms timer. Camera, detector and mux were not reconfigured.
No following trial or chassis motion was initiated as part of this optimization.

Deployment overlay: `releases/sim-follow-20261020-latency/install`, loaded after
`sim-follow-20261020-v1` to retain that release's mux package. Rollback: remove
only the latency overlay line from `r680_person_follow.sh`, then restart the
inactive follower. Camera/detector do not need restarting. New native symbols
are verified from the running release; `numeric_backend` reports the selection.
