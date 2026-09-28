#pragma once

#include "wla_diff_mpc/qp.hpp"

#include <nav_msgs/msg/path.hpp>

namespace wla_diff_mpc {

bool finalAlignmentMode(bool was_aligning, double distance_to_goal,
                        double enter_distance, double exit_distance);

// Rotate at a fixed position with a bounded, decaying yaw-rate reference.
Reference makeRotationReference(const State & current, double target_yaw,
                                int horizon, double dt, double w_max, double time_constant);

// Track the path tangent until the controller explicitly enters final yaw alignment.
Reference samplePath(const nav_msgs::msg::Path & path, const State & current,
                     int horizon, double dt, double v_max, double w_max,
                     double turn_time_constant);

}  // namespace wla_diff_mpc
