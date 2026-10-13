# Native relative-follow kernels

C++17 / pybind11 calculation library. Owns local measured-ray grids, obstacle inflation, bounded Dijkstra search and curved-command collision checks. No ROS node, map, Nav2 or actuator publisher. Used by wla_person_follow; that package owns contracts, leases and motion authorization.

Build with colcon build --packages-select wla_follow_local wla_person_follow --cmake-args -DCMAKE_BUILD_TYPE=Release. Tests and limits are documented in wla_person_follow/README.md. A native library does not establish end-to-end hard real-time scheduling or hardware acceptance.

`target_kalman.hpp` adds fixed-size Eigen constant-velocity prediction with innovation rejection, bounded uncertainty and a maximum two-second accepted-measurement age. Identity and last-seen behavior are owned by wla_person_follow.
