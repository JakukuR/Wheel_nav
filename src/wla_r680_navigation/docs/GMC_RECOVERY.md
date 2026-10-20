# Optional GMC — 2026-10-20

This revision supersedes the pre-association discard policy in commit 6f7ebf0.
GMC is an optional visual compensation aid, not admission for fresh detections
or depth geometry. The deployed FP32 YOLOE model, camera timing and projection
thresholds are unchanged.

Before BoT-SORT updates, bounded ORB estimates the warp. A failed quality check
supplies an identity matrix (explicitly labelled uncompensated), while ordinary
Kalman prediction and association continue. The current image becomes the next
background reference because tracking has also advanced on that image. The
detector publishes the current tracking/detection results and projects their
depth normally. It does not replay earlier boxes or timestamps. Empty YOLO
results are still empty; this cannot repair detector motion-blur misses.

On the first transition to degraded association, public track IDs are retired
and new, monotonic candidate IDs are allocated. Session and epoch stay intact,
so the follow task and its ReID gallery survive. The old locked ID cannot
silently bind to a possibly changed track. The existing target-only ReID opens
its spatial candidate gate and requires appearance/position agreement across
three frames before rebinding; without ReID, a selected old ID is not replaced
automatically. Initial enrollment still requires the explicitly selected ID.
This does not guarantee no within-mode ID switches; appearance and position
checks remain necessary.

Ten consecutive valid GMC frames end a degraded episode. Intermittent quality
changes within an episode do not repeatedly retire IDs. Recovery preserves the
candidate IDs that ReID may already have confirmed. A later, separate degraded
episode allocates fresh IDs again. The public ID allocator is bounded to 4096
cached associations; evicted associations get new IDs, never reused IDs.

Diagnostics: `runtime.gmc_policy=optional_visual_compensation`, instantaneous
`runtime.association_mode=visual|uncompensated`, and the original GMC failure
reason/valid flag/fallback matrix. Raw and relative packets have top-level
`association.mode/revision` for the debounced identity episode. Existing raw
and relative object schemas are retained.

Unexpected inference/OpenCV/contract errors and real camera/session changes
retain hard resets. Existing 300 ms source validity, collision checks, command
leases and 8 s ReID search timeout are unchanged. No follower control policy,
depth sampling or ReID similarity threshold is loosened.

Validation: 66 targeted tests pass (real ORB/BoT-SORT, isolated domain 198
detector/geometry, task control and ReID). Tests cover publication during GMC
failure, unchanged epoch and renewed current-source geometry, ID retirement,
no ID churn during quality flapping, no ID reuse on recovery, wrong appearance
rejection and three-frame re-identification. Real GPU YOLOE replay holds the
image fixed and forces absent background features: four detections continue
through four GMC failures, public IDs switch only once and stay stable after
recovery; CUDA graph generation stays unchanged. This is not physical turning
or multi-person accuracy acceptance.

Before this change, a 12 s live read-only sample (287 published frames) measured
GMC median/P95 6.39/11.99 ms and worker median/P95 41.25/51.40 ms. These timings
are scene/load-dependent. Making GMC optional removes failure-induced output
gaps, not its normal compute cost, and does not establish stable 30 Hz.

Deployment read-only check: 278 publications in 12 s, RGB 29.97 Hz, detections 23.08 Hz, acquisition-to-publish median 85.99 ms; epoch stable at 1, new policy marker present. Camera and viewer were not restarted; five fresh inactive follow samples preceded the detector-owner reload. No motion test was initiated. Runtime now requires a current target ID.
