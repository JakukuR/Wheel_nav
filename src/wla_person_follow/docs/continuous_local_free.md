# Continuous 0.5 m robot-centered free-space assumption — 2026-10-20

Requested policy: default the region within 0.5 m of the robot to known free
unless observed obstacles occupy it, and permit bounded reverse recovery.

Deployment config: `local_assumed_free_radius_m: 0.5`,
`startup_clear_radius_m: 0.0`. The new node parameter defaults to zero elsewhere.
This replaces the deployment's earlier one-shot startup patch and motion-gated
12 s countdown. It applies while idle as well as following, on fresh map jobs.

Map implementation:
- Recompute an ephemeral known-space overlay at the current wheel-odom XY pose
  before planning and publishing the grid; never write observation log/stamps.
- Existing occupied sensor cells always win, including inside the chassis
  footprint. The overlay never clears obstacles or extends their timestamps.
- Sensor evidence still expires using the existing 12 s TTL. Expired cells in
  the current circle revert to the explicit assumption; outside it they become
  unknown. Motion leaves no trail of assumed-free cells.
- Real measurements retain their regular occupancy fusion and clearing logic.
  This is an operator assumption, not a depth/ToF measurement or a guarantee
  about unobserved objects. ToF remains disabled.

Recovery:
- Keep the existing 2 s blockage wait, maximum reverse speed 0.15 m/s and maximum
  retreat distance 0.35 m. Try shorter segments down to 0.06 m when the whole
  original retreat cannot fit. Check 0.05 m beyond the endpoint too.
- Select the longest checked segment, freeze its distance for that attempt,
  then stop/replan. Existing timeouts and lost-target rules remain.
- Both supervisor and MPC retain occupied-cell/body-clearance, known-space and
  predicted-person checks. The MPC checks the swept robot body each cycle and
  rejects a newly obstructed retreat. No command bypass was added.
- Chassis radius is still 0.19 m and obstacle margin 0.10 m. The 0.5 m figure is
  the map assumption radius, not a chassis size or available reverse distance.

Web:
- The published grid includes the same free-space overlay used by the planner.
- Fresh person-grid data and status show a yellow dashed 0.50 m circle and an
  explicit assumption label; occupied cells keep their normal appearance.
- No extra ROS commands or control dependencies are introduced by the viewer.

Validation: 22 algorithm tests and 5 isolated-domain ROS tests passed. Includes
idle/moving overlay, no trail, disabled/invalid configuration, obstacle priority
inside/outside chassis footprint, short reverse command, obstacle/person veto,
and a multi-step retreat that completes without extending its goal. ROS tests
use domain 198, local discovery and mock command topics; no physical motion.
Built into the existing sim-follow-20261020-v1 release. Live checks and restart
process IDs are recorded in the accompanying local deployment report.
