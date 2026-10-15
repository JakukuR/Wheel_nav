# cuVSLAM continuity restart removal — 2026-10-15

## Observed failure

Run `/home/orin/nav_run/nav-20261015-160021-2338692` was cuVSLAM with
`--no-degraded-odom`, not the pending official OpenVINS/chassis-IMU comparison.
At 16:02:26, 16:03:28 and 16:04:49, the continuity layer rejected raw source ages
150.040, 154.134 and 153.704 ms. Its raw tracking status remained
`tracking_inertial_ready`; auxiliary IMU ages were approximately 3, 2.3 and 0.43 ms.
The continuity node entered Blocked and required two seconds of healthy stopped
recovery, while watchdog killed the estimator after three 10 ms false reports.
One short expiry therefore generated multiple fault reports and restarted the
frontend before its recovery condition could complete.

## Change

- Remove the continuity-false counter and its `fault("continuous odometry unhealthy")`
  call from vo_watchdog. Availability still feeds its non-latched health decision.
- Wire `r680_nav.sh --no-degraded-odom` to launch `enable_degraded_odom:=false`.
  The continuity node is omitted; cuVSLAM directly publishes `/d455_slam/odom`
  and TF. Watchdog/monitor/initializer use the corresponding existing direct path.
  The intermediate 150 ms rejection and two-second recovery latch are absent
  from this launch path, rather than relaxed or replaced with another threshold.
- With degradation enabled, the bounded fallback node and its budget/freshness
  rules still exist. This change does not remove all navigation health checks.
  Genuine odometry silence is still handled by the existing 1.5 second watchdog.
- No SDK algorithm, motion limit, automatic-mode authorization, or official
  OpenVINS estimator code is changed.

## Verification

- Built `vo_watchdog` successfully in the existing Jazzy workspace. Installed
  executable resolves to `build/wla_r680_navigation/vo_watchdog`.
- `bash -n r680_nav.sh`, launch routing for both degradation modes, existing
  namespaced cuVSLAM parameter rewrite, and `git diff --check` passed.
- Isolated ROS domain 197 regression used synthetic odometry/map/health, actual
  watchdog executable, `lost_limit=3`, and `restart_enabled=false`. A 400 ms
  odometry/continuity interruption closed health and recovered without fault
  latch. Sustained false continuity with live odometry also recovered without
  fault latch. Subsequent genuine odometry silence triggered the existing
  timeout at approximately 1.57 seconds. No command-velocity publishers existed.
- Evidence: `.runtime/continuity_restart_removal/build.log` and `regression.log`.
- No navigation or camera session was launched/restarted and no chassis motion
  command was sent. Next navigation launch loads the new routing. These checks
  do not establish physical navigation accuracy or fix OpenVINS divergence.

Pre-existing OpenVINS camera configuration edits are excluded from this commit.
