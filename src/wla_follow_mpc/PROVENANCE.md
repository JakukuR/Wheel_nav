# MPC core provenance

As of the 2026-10-12 ros2_ws integration, this package links the exported
`wla_diff_mpc::wla_diff_mpc_core` library from the existing `wla_diff_mpc` package.
The Nav2 controller plugin links that same library. There is one source and
installed implementation of reference sampling, QP construction, OSQP solving,
and nonlinear prediction; the service no longer carries copied core sources.

The earlier isolated 2026-10-10 delivery copied the core from Wheel_nav commit
`bea956fb00af27483b394a5c60999771828af71f` under Apache-2.0. The three core source
files were compared with the live production package (normalizing line endings) before
switching to linkage. Core algorithm and Nav2 plugin identifier are unchanged.

`src/server.cpp` and `srv/ComputeMpc.srv` supply the person-follow request adapter.
The service does not publish velocity or instantiate the Nav2 controller plugin.
It tracks a planar reference with velocity/rate bounds; the client separately
checks nonlinear prediction, measured-velocity braking, costmap and voxel data.
It is not an ESDF-constrained optimization or a calibrated chassis delay model.

Newly lowered speed limits have priority over requested-command slew, matching
the existing Nav2 core. First-step slew is checked again after a second solve,
relative to the measured input clamped to the new command limits. This is not a
physical-deceleration promise: the caller must retain the original measured
velocity and check the full stopping envelope with its reaction allowance.
Invalid or late responses contain zero command and no prediction.
