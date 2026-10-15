# cuVSLAM startup map-match regression, 2026-10-15

## Scope and baseline

The operator selected bea956fb00af27483b394a5c60999771828af71f as the
known-good behavioral baseline and requested a minimal regression repair.
Compared with pre-change HEAD 8d6d6f0, the tracked cuVSLAM frontend source,
continuous-odometry source and degraded-odometry configuration are unchanged.
The 150 ms raw-odometry freshness gate and wheel/IMU fallback requirements
already existed in that baseline. This comparison does not establish that the
external SDK runtime or sensor-driver binaries are identical.

## Reproduced regression

The later map-evidence policy ignored backend matches received before VIO
readiness and required a fresh match paired within 50 ms of a localization pose.
A synthetic ROS-domain-199 comparison used the baseline watchdog compiled into
/tmp, and the pre-change installed watchdog, with restart disabled:

| Sequence | bea956f | Pre-change HEAD |
| --- | --- | --- |
| Match before VIO readiness; delayed readiness with fresh pose and TF | Healthy | Blocked |
| New match after readiness, with matching pose and TF | Healthy | Healthy |

The real run nav-20261015-105733-1154575 logged RTAB-Map localization before
VIO initialization completed. Its eventual continuous-odometry failure recorded
raw_source_age_s=0.155085, raw_ready=1, visual_observed=1, watchdog_ready=0,
wheel_age_s=0.007746 and imu_age_s=0.007741. This is consistent with the timing
regression, but the text logs do not prove every map-admission prerequisite or
establish this as the only cause of that failed run.

## Change

Only first-time cuVSLAM map admission retains a backend match through inertial
initialization. The candidate must be fresh and newer than the startup epoch
when received. It does not authorize navigation by itself: current visual
tracking, healthy odometry, a fresh validated localization pose, covariance
checks and pose/TF agreement remain required.

After initial confirmation, expiry or loss clears map evidence and requires new
paired evidence. Faults, respawn, manual priors and clock resets also disable
initial-match retention. RGB-D admission is unchanged. No raw-odom, auxiliary
sensor, fallback-budget or motion-authorization thresholds were modified.

## Verification

- Built vo_watchdog and test_map_localization_evidence on Orin/Jazzy.
- frontend_map_watchdog: PASS (RGB-D and both cuVSLAM continuity modes).
- cuvslam_startup_match: PASS for bounded and unbounded watchdog modes; covers
  stale initial matches, delayed VIO, missing/stale pose, inconsistent TF,
  expiry, and the requirement for new paired evidence after loss.
- map_localization_evidence: unit checks include a simulated 60-second warmup,
  missing TF/odom/VIO, stale-at-arrival candidates and explicit resets.
- visual_odom_sensor_independence: synthetic continuous-odometry inputs verify
  normal visual operation survives IMU, wheel or both auxiliary inputs becoming
  stale; fallback blocks on either missing input; blocked visual recovery needs
  fresh stopped wheel evidence but not auxiliary IMU.
- The sensor test supplies a synthetic watchdog; it does not claim other Nav2
  or chassis health gates tolerate missing wheel feedback.

All runtime tests use isolated domains (198/199), synthetic input, no camera or
chassis driver startup and no motor command publishers. These checks do not
validate real motion, actual featureless-area recovery or production startup.
The installed watchdog is a symlink to the rebuilt binary, so the next ordinary
launch will use this repair. No running navigation process was restarted.

The 150 ms freshness gate remains active by the selected baseline scope. A
continued real source-age overrun can still block odometry; this repair does not
claim to fix processing latency or SDK feature loss.
