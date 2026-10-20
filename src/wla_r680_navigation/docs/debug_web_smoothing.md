# Debug page smoothing and transport optimization — 2026-10-20

Applied only to wla_r680_navigation's read-only debug viewer and browser assets.
No follower, detector, camera, mux or motion configuration changes.

Frontend:
- Reconcile sidebar and 2D status rows by key; preserve node identity and scroll.
- Draw topdown once per full response; resize RGB canvas only on dimension change.
- Reuse decoded JPEG for the same acquisition stamp.
- Conditional GET skips redundant payload/decode/paint for unchanged snapshots.
- Short network failures retain pixels with a prominent non-live historical-frame
  label. After 1500ms without a successful response, clear geometry/image and
  invalidate cached data/ETag. Explicit sensor-invalid responses still clear
  immediately; their validity thresholds are not relaxed.
- Pause/resume ignores an earlier in-flight response; resume requests fresh data.
- Display web generation time and request time separately.

Backend:
- Gzip level1 compressed once per snapshot; immutable bytes, ETag and creation
  time swapped atomically for HTTP threads. HTTP/1.1 connection reuse.
- 304 only for a still-valid snapshot; expired snapshot returns unavailable data.
- Render only pages requested in the last2s; reuse JPEG encoding when the exact
  frame is reused. Remove redundant JSON decode/encode while adding topdown.
- Lower debug process scheduling priority with nice+5 (verified NI5). The
  follower keeps NI0. This is a shared-resource mitigation, not hard real-time
  isolation. CPU, memory bandwidth and networking remain shared on Orin.

Architecture: HTTP threads read immutable debug snapshots. Debug ROS callbacks
subscribe to follower/sensor output. The follower does not call HTTP or await
browser acknowledgements. Network/browser latency therefore does not enter the
control loop as a synchronous dependency, but heavy debugging can still compete
for machine resources. Browser-held images are diagnostic only, never control
inputs.

Validation:
- Local Node harness: both planning/detections pass stable DOM/canvas, JPEG
  reuse, 304 skip, bounded non-live hold, expiry, recovery, invalid data and pause.
- Isolated domain197 HTTP/ROS integration passes local and odom modes, including
  gzip/304/keep-alive, time skew, invalid/old/zero stamps, empty geometry, no-RGB
  topdown, recovery and rejected HTTP mutation.
- Live browser has no warning/error logs and shows fresh image/obstacles/map.
- 25 sequential requests before/after (same client, 200ms between requests,
  Accept-Encoding:gzip, persistent HTTPConnection): all responses valid.
  Before: mean214947bytes; median115.9ms; P95 357.9ms.
  After: mean79344bytes; median81.1ms; P95 146.6ms.
  These are short-run network observations, not guaranteed latency bounds.

Restarted only standalone viewer: PID700578, NI5.
Follower parent696079/child696163, detector665660 and camera622203 unchanged.
No target-follow command or physical movement test was issued.
Runtime log/PID: ~/ros2_ws/.runtime/debug-smooth-viewer.{log,pid}.
Test logs: .runtime/debug-smooth-test.log and debug-smooth-test-odom.log.
