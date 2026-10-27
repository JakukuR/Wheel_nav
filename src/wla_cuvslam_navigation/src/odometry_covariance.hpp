#pragma once
#include <array>
#include <cmath>
#include <stdexcept>

namespace wla_vio {
// Uncertainty published with each odometry message. PoseCache/VelocityCache
// return the spread of recent samples: near zero (often singular) while the
// robot stands still, and growing with speed. RTAB-Map inverts the pose matrix
// into link information, so a singular one aborts it (Link::setInfMatrix).
// These nominal diagonals keep the matrix invertible; inertial-only poses get
// a larger variance so they are weighted as weak evidence.
struct NominalCovariance {
  double linear_variance{1e-3}, angular_variance{1e-3};          // m^2, rad^2
  double linear_velocity_variance{1e-2}, angular_velocity_variance{1e-2};  // (m/s)^2, (rad/s)^2
  double inertial_only_scale{100.0};

  void validate() const {
    for (const double v : {linear_variance, angular_variance, linear_velocity_variance, angular_velocity_variance}) {
      if (!std::isfinite(v) || v <= 0.0) throw std::invalid_argument("odometry variance must be finite and > 0");
    }
    if (!std::isfinite(inertial_only_scale) || inertial_only_scale < 1.0)
      throw std::invalid_argument("inertial_only_variance_scale must be finite and >= 1");
  }
  static std::array<double, 36> diagonal(double linear, double angular, double scale) {
    std::array<double, 36> c{};
    for (int i = 0; i < 3; ++i) { c[i * 7] = linear * scale; c[(i + 3) * 7] = angular * scale; }
    return c;
  }
  std::array<double, 36> pose(bool inertial_only) const {
    return diagonal(linear_variance, angular_variance, inertial_only ? inertial_only_scale : 1.0);
  }
  std::array<double, 36> twist(bool inertial_only) const {
    return diagonal(linear_velocity_variance, angular_velocity_variance, inertial_only ? inertial_only_scale : 1.0);
  }
};
}
