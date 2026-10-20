# GMC quality failure isolation — 2026-10-20

The nav detector uses RecoverableTrackingRuntime (nav-owned adapter over the
existing deployed YOLOE FP32 runtime). Before Ultralytics' tracking callback
updates any Kalman state or association, it computes bounded ORB GMC using the
same high-score detection mask. A valid transform is cached for that callback;
there is no second ORB pass.

A failed quality check emits the typed GMCQualityRejected exception, publishes
timing outcome `gmc_degraded`, and discards the image's detection/association
output. The last accepted background reference, tracker lists, Kalman state,
frame counter, ID allocator, model warmup and detector epoch are unchanged.
Empty initial reference images are rejected without poisoning the reference.
Recovery compares against the last accepted frame, not an intervening bad image.

This prevents a transient quality failure from cancelling the active follow
task and thereby deleting its ReID gallery. It is not a claim that YOLO detects
blurred people. No stale frame receives a fresh timestamp. Existing 300 ms
observation validity, task heartbeat leases, collision checks and 8 s ReID
search expiry still apply. Long-lasting insufficient features can leave the
robot waiting/searching; this patch adds neither blind motion nor unlimited
identity retention. A real camera gap >750 ms, camera/model/session change or
unexpected inference/OpenCV error retains the existing hard-reset policy.

Target measurement provenance: nav_yolo_detector projects a central detection
box depth patch through camera intrinsics and mounting TF, publishing vehicle
relative distance/yaw on `/r680/perception/relative_objects`. sim_node transforms
the measurement at its acquisition time using wheel odometry, applies the
configured 0.28 m surface-to-center approximation, and supplies the odom-space
observation to VisualTrack's constant-velocity Kalman filter. ReID selects the
identity; it is not a depth estimator. This change does not alter projection,
center-depth selection, partial-body matching, or obstacle filtering.

Validation: real ORB and upstream BoT-SORT with synthetic detections check that
multiple rejected frames mutate no track, covariance, frame counter or ID; a
translated valid frame recovers the original ID. Isolated domain 198 tests
check unchanged epoch/timestamp and no rejected publication, followed by fresh
recovery; generic inference failure still resets. Real YOLOE-11s-seg GPU replay
using the bundled bus image rejects blur/blank/blank and recovers all four IDs,
without replacing the FP32 CUDA graph generation. This is offline fault
injection, not real-vehicle turning/identity accuracy validation.

The detector and its existing ROS fixture already contained uncommitted FP32
runtime integration before this fix. Those directly required integration
changes are retained alongside this repair; unrelated launch/navigation edits
are excluded. Original bytes and baseline hashes are backed up for review.

Final validation: 64 targeted tests passed (runtime/ROS detector/relative task
control/ReID identity). After guarded reload with five fresh inactive follow
status samples, a 12 s live read-only check received 244 publications at
20.28 Hz, RGB 29.96 Hz, median acquisition-to-publication 94.09 ms; epoch stayed
1 and the new policy marker was present. Camera PID 622203 and viewer PID
722360 stayed running. No follow command or chassis velocity was published
by these checks. Existing stopped task was not resumed. The necessary detector
restart created a new session, so the operator must select a current ID.
