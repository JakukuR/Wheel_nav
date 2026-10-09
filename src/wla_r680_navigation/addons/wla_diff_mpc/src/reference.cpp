#include "wla_diff_mpc/reference.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace wla_diff_mpc {
namespace {

double unwrap(double angle, double previous) {
  return previous + std::remainder(angle - previous, 2.0 * M_PI);
}

double pathHeading(const nav_msgs::msg::Path & path, size_t index, double fallback) {
  for (size_t i = index; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    if (std::hypot(b.x - a.x, b.y - a.y) > 1e-5) {
      return std::atan2(b.y - a.y, b.x - a.x);
    }
  }
  for (size_t i = std::min(index, path.poses.size() - 1); i > 0; --i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    if (std::hypot(b.x - a.x, b.y - a.y) > 1e-5) {
      return std::atan2(b.y - a.y, b.x - a.x);
    }
  }
  return fallback;
}

}  // namespace

bool finalAlignmentMode(bool was_aligning, double distance_to_goal,
                        double enter_distance, double exit_distance) {
  if (distance_to_goal <= enter_distance) return true;
  if (distance_to_goal >= exit_distance) return false;
  return was_aligning;
}

Reference makeRotationReference(const State & current, double target_yaw,
                                int horizon, double dt, double w_max, double time_constant) {
  if (horizon < 1 || dt <= 0.0 || w_max <= 0.0 || time_constant <= 0.0) {
    throw std::invalid_argument("invalid MPC rotation reference settings");
  }
  Reference ref;
  ref.states.reserve(horizon + 1);
  ref.inputs.reserve(horizon);
  ref.states.push_back(current);
  double remaining = std::remainder(target_yaw - current[2], 2.0 * M_PI);
  const double gain = 1.0 - std::exp(-dt / time_constant);
  for (int k = 0; k < horizon; ++k) {
    const double step = std::clamp(remaining * gain, -w_max * dt, w_max * dt);
    State next = ref.states.back();
    next[2] += step;
    ref.states.push_back(next);
    ref.inputs.emplace_back(0.0, step / dt);
    remaining -= step;
  }
  return ref;
}

Reference samplePath(const nav_msgs::msg::Path & path, const State & current,
                     int horizon, double dt, double v_max, double w_max,
                     double turn_time_constant) {
  if (path.poses.empty() || horizon < 1 || dt <= 0.0 || v_max <= 0.0 || w_max <= 0.0) {
    throw std::invalid_argument("invalid MPC path reference");
  }
  size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    const double d = std::hypot(p.x - current[0], p.y - current[1]);
    if (d < best) { best = d; nearest = i; }
  }
  if (best > 1.5) throw std::invalid_argument("MPC path is too far from robot");

  const double goal_yaw = tf2::getYaw(path.poses.back().pose.orientation);
  const double first_heading = pathHeading(path, nearest, goal_yaw);
  const double heading_error = std::remainder(first_heading - current[2], 2.0 * M_PI);
  if (std::abs(heading_error) > 0.7) {
    return makeRotationReference(current, first_heading, horizon, dt, w_max,
                                 turn_time_constant);
  }

  Reference ref;
  ref.states.reserve(horizon + 1);
  ref.inputs.reserve(horizon);
  const double step_length = v_max * dt;
  size_t segment = nearest;
  double segment_offset = 0.0;
  for (int k = 0; k <= horizon; ++k) {
    double advance = k == 0 ? 0.0 : step_length;
    while (advance > 0.0 && segment + 1 < path.poses.size()) {
      const auto & a = path.poses[segment].pose.position;
      const auto & b = path.poses[segment + 1].pose.position;
      const double length = std::hypot(b.x - a.x, b.y - a.y);
      if (length < 1e-6) { ++segment; segment_offset = 0.0; continue; }
      const double available = length - segment_offset;
      if (advance < available) { segment_offset += advance; advance = 0.0; }
      else { advance -= available; ++segment; segment_offset = 0.0; }
    }
    State x;
    if (segment + 1 < path.poses.size()) {
      const auto & a = path.poses[segment].pose.position;
      const auto & b = path.poses[segment + 1].pose.position;
      const double length = std::hypot(b.x - a.x, b.y - a.y);
      const double t = length > 1e-6 ? segment_offset / length : 0.0;
      x << a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), pathHeading(path, segment, goal_yaw);
    } else {
      const auto & goal = path.poses.back().pose;
      // Do not inject goal yaw as a discontinuity into the last path step.
      x << goal.position.x, goal.position.y,
        pathHeading(path, path.poses.size() - 1, goal_yaw);
    }
    x[2] = unwrap(x[2], k == 0 ? current[2] : ref.states.back()[2]);
    ref.states.push_back(x);
  }
  for (int k = 0; k < horizon; ++k) {
    const auto delta = ref.states[k + 1] - ref.states[k];
    ref.inputs.emplace_back(std::min(v_max, std::hypot(delta[0], delta[1]) / dt),
      std::clamp(delta[2] / dt, -w_max, w_max));
  }
  return ref;
}

}  // namespace wla_diff_mpc
