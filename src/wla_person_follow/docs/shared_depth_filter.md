# Algorithm-owned depth filtering — 2026-10-20

Implemented and loaded on orin@192.168.9.118, ~/ros2_ws.

The follower consumes raw aligned D455 depth directly. Native `_wla_sim_geometry`
performs deprojection, SE(3), height selection, local ground estimation and isolated
XY-cluster removal in-process with the GIL released during numerical loops.
It does not subscribe to or wait for external obstacle/clearing clouds.

Policy matches the former local live display: optical depth 0.15–5m, stride4;
clearing-height band [-0.08,0.50]m; 0.20m ground cells with 3x3 neighboring
samples, >=5 samples, 20th percentile ground; >=0.05m ground-relative obstacle
height, fallback >=0.08m; isolated components need >=3 connected 0.05m XY cells.
The cluster helper is copied from the existing navigation implementation;
`test_native_filter_matches_display_ground_height_and_clusters` independently
checks the complete default policy. These are a fixed native policy, not runtime
navigation-node YAML parameters.

Marking endpoints and measured clearing rays remain separate. The map no longer
reclassifies ground/cluster-filtered points using its legacy 0.08–1.6m band.
No return is not fabricated free space; unknown-space, freshness, identity,
estop, command leases and mux predicates are unchanged. Radius .19m, margin .10m.
ToF remains disabled with its existing interface.

Accepted native marking points are published once per frame on
`/r680/person_follow/depth_obstacles` with the acquisition timestamp. Both local
viewer point/voxel layers default to this topic. Idle mapping can be viewed
without an active motion task; inactive paths remain empty. Viewer rendering is
still 5Hz; follower processing is scheduled at 10Hz and output checking at 20Hz.
The grid retains 12s local history, whereas the point layer is the current frame;
that distinction and time skew can still produce visual differences. RGB
projection retains <=30ms skew rejection; the 2D top view remains independent of RGB.
The local viewer now requires the simulation follow node, depth and wheel input.
The separate external depth_to_points process is retained for its other
consumers; this change removes duplicate filtering between the follower and its
viewer, not all depth processing across the entire ROS graph.

Verification:
- Built native and Python packages into the existing versioned release overlay.
- 15 unit tests pass (12 simulation-port + 3 standalone-debug).
- 5 ROS adapter/mux tests pass in isolated domain198; no hardware topic publisher.
- Domain197 debug HTTP/ROS fixture passes projection, stale/invalid data, 2D map,
  empty geometry, camera loss and read-only HTTP checks.
- JavaScript syntax check passes after updating the source description.
- Same real frame: 1320 native points / 1320 original display points;
  bidirectional maximum positional difference 5.34e-7m (float arithmetic).
- Recomputed native output is bit-for-bit equal to the live follower-published
  float32 cloud for an identical captured depth stamp; one publisher observed.
- Recorded-frame frontend replay median3.32ms, p95 4.17ms. Live sampled frontend
  approximately3.3–13.0ms; available idle map/planner samples46.3–55.2ms.
  Offline core median49.3ms uses an explicitly synthetic target, not a drive trial.
- Viewer API confirms both point/voxel topics; browser shows live points and map.
- Task inactive before restart and afterward. Reloaded only follow launch and
  standalone viewer. Camera/detector session and epoch stayed unchanged.
- No new follow request or physical motion trial was issued.

Runtime parents: follow696079 (child696163), viewer696080. Logs/PIDs are under
`~/ros2_ws/.runtime/shared-filter-{follow,viewer}.{log,pid}`.
Evidence: `.runtime/shared-filter-{build,unit,ros,debug}.log`,
`.runtime/shared-filter-{before,after}.json`. Local copies are beside this report.
