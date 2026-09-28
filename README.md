# Differential-drive MPC (experimental)

This Nav2 controller plugin uses the unicycle model from
[ego-planner-for-ground-robot](https://github.com/Dangko/ego-planner-for-ground-robot)
and the sparse OSQP formulation style of
[LMPC_OSQP_EIGEN](https://github.com/USE-jx/LMPC_OSQP_EIGEN).
Both sources target ROS 1; this package implements the controller for ROS 2
Nav2 and does not copy their vehicle-specific controller logic.

The state is `x = [x, y, yaw]` and the control is `u = [v, wz]`. For each
reference step of length `dt`, the nominal motion is
`f(x,u) = x + dt * [v*cos(yaw), v*sin(yaw), wz]`. Linearized tracking errors
obey `e[k+1] = A[k] e[k] + B[k] du[k] + d[k]`, where

```
A = [1 0 -dt*v_ref*sin(yaw_ref); 0 1 dt*v_ref*cos(yaw_ref); 0 0 1]
B = [dt*cos(yaw_ref) 0; dt*sin(yaw_ref) 0; 0 dt]
d = f(x_ref[k], u_ref[k]) - x_ref[k+1]
```

The sparse QP decision vector is `[e[0..N], du[0..N-1]]`. OSQP minimizes
`sum(e' Q e + du' R du + (u[k]-u[k-1])' S (u[k]-u[k-1]))`, with a separate
terminal weight. `H` therefore has diagonal `2Q`, `2R`, and the diagonal and
off-diagonal terms from `2S`. Equality constraints enforce the current pose
and linearized dynamics; inequality constraints bound `v`, `wz` and command
changes per model step. The solver checks convergence and all constraint
residuals before returning a command.

A newly reduced Nav2 speed limit has priority over the first-step command
slew bound. Otherwise a limit of 0.18 m/s after a 0.30 m/s command, with a
0.06 m/s per-step slew bound, makes the QP infeasible. On Jazzy, a failed
solve or blocked predicted footprint raises `NoValidControl`: Nav2 sends a
zero command and retries for its configured `failure_tolerance` before
aborting. Error messages include the OSQP status or the blocked pose.

`config/mpc_example.yaml` is an optional controller parameter block. The
vehicle's active `nav2.yaml` continues to select MPPI. This controller also
checks the predicted footprint against the local costmap, but obstacle
avoidance itself remains the responsibility of the global path and Nav2
costmap; it is not a collision-constrained optimizer. It refuses a blocked
prediction instead of producing a command into it. The predicted path is
published at `FollowPath/predicted_path` when the plugin name is `FollowPath`.

The path-following reference keeps the last path tangent through its terminal
point. Within `final_align_enter_distance`, the controller latches into a
separate, low-speed yaw alignment using the goal orientation. It leaves that
mode only beyond `final_align_exit_distance`, avoiding mode chatter from pose
noise. `final_align_wz_max` and `turn_time_constant` limit and taper the final
angular command; the reference keeps position fixed during alignment.

The current formulation assumes the commanded `(v,wz)` is achieved within a
model step. R680 measurements have shown substantial chassis response delay,
especially for yaw. Do not enable this plugin for unattended real-car motion
until the drive latency is resolved and the controller is validated at low
speed. The Nav2 collision monitor and final command guard must remain active.

Build dependencies: Eigen3, OSQP, OsqpEigen, and the ROS 2 packages listed in
`package.xml`. On the Orin, OSQP/OsqpEigen can be installed into an isolated
prefix, then that prefix supplied to CMake via `CMAKE_PREFIX_PATH` and to the
runtime linker via `LD_LIBRARY_PATH`. `qp_regression` is an offline test that
does not publish any robot motion command.
