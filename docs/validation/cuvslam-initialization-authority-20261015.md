# cuVSLAM initialization authority and startup timeout

## Observed failures
- Run nav-20261015-113137-1286498 used --auto-vio-init --enable-motion.
  The guard reported hardware output enabled, but the root cuVSLAM branch did
  not start the base authorization session or call /r680_base/set_automatic.
  The initializer requested motion while wheel travel remained zero.
- Run nav-20261015-112807-1272890 restarted an uninitialized frontend for missing
  output odometry despite live tracking reports. No inertial-ready state had
  been reached at the first fault.

## Changes
- For cuVSLAM with both --auto-vio-init and --enable-motion, start the permission
  session alongside bringup. Wait for the guard, fresh stopped wheels, and
  MANUAL/NAV ownership; request AUTO once and confirm it before publishing a
  fresh bounded-initialization authorization heartbeat.
- Initializer requires this heartbeat in the hardware-enabled cuVSLAM route.
  Existing bounded motion, emergency stop, collision and manual takeover paths
  remain active. Ordinary navigation permission follows initialization success,
  goal cancellation and current healthy/stopped confirmation.
- Failure/exit revokes authorization; manual takeover is not reacquired.
  Root post-initialization handoff is bounded to 30 seconds.
- Before cuVSLAM has ever been ready, watchdog checks frontend/continuity
  heartbeats without requiring valid running odometry. After readiness the
  original odometry timeout applies.

## Evidence and limitations
- Watchdog CTest: frontend_map_watchdog, cuvslam_startup_match and
  cuvslam_startup_timeout passed (107.20, 31.11 and 20.09 seconds).
- First direct authority test passed success, no authority, authority process
  exit, manual takeover, emergency stop and obstacle cases with the real
  initializer, guard and mux, synthetic sensors and no chassis driver.
- Legacy initializer fixture passed all 11 cases plus legacy guard.
- A later combined CTest attempt failed: the existing navigation-motion test
  exceeded its 60-second suite budget; the synthetic authority fixture supplied
  about 80 Hz IMU during concurrent host replay and failed the actual 100 Hz gate.
  No production gate was reduced. Test timeout was changed to 90 seconds, and
  fixture IMU samples now use a 200 Hz synthetic sample timeline. Added delayed
  guard startup coverage.
- Final rerun was stopped at the user's request. Latest test-fixture adjustments
  and delayed-guard case have NOT completed verification. Production targets
  built successfully before interruption. No further tests were run.
- Minimal original-launch initializer-only probe applied the run-specific result
  path correctly. This does not explain the missing result in the older live run.
- No physical wheel motion or production AUTO service call was performed by this
  work. ROS domain 198 fixtures used no driver; legacy preview used domain 74.

## Remaining known gaps
- --enable-motion without --auto-vio-init in the cuVSLAM branch still publishes
  mission permission after readiness without automatically requesting base AUTO.
- The initializer's moving state denotes requested motion, not measured motion.
  There is no dedicated no-motion-response timeout; zero travel may consume the
  motion window and eventually report inertial/localization timeout.
- Failure JSON is periodically rewritten with current inputs, so recovered input
  flags may hide failure-time conditions.
- The baseline 150 ms raw-pose gate remains. Queue latency and featureless-scene
  tracking loss are not repaired by this change.
