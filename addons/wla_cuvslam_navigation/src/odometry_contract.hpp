#pragma once
#include <cmath>
#include <Eigen/Geometry>

namespace wla_vio {
struct BodyVelocity { Eigen::Vector3d linear, angular; };
inline BodyVelocity bodyVelocity(const Eigen::Vector3d & previous,
  const Eigen::Quaterniond & previous_q, const Eigen::Vector3d & current,
  const Eigen::Quaterniond & current_q, double dt)
{
  if (!std::isfinite(dt) || dt <= 0 || dt > 0.2) {
    throw std::invalid_argument("invalid velocity interval");
  }
  const Eigen::AngleAxisd delta(current_q * previous_q.conjugate());
  return {current_q.conjugate() * ((current - previous) / dt),
    current_q.conjugate() * (delta.axis() * delta.angle() / dt)};
}
// Require a continuous post-initialization window. An SDK IMU-state object
// alone is available before gravity initialization and cannot open motion.
class InitializationGate {
public:
  bool update(bool gravity_and_bias_valid, double stamp, double duration)
  {
    if (!gravity_and_bias_valid || !std::isfinite(stamp) ||
      (last_ >= 0 && (stamp <= last_ || stamp - last_ > 0.2))) {
      since_ = -1;
    }
    last_ = stamp;
    if (!gravity_and_bias_valid) return false;
    if (since_ < 0) since_ = stamp;
    return stamp - since_ >= duration;
  }
private:
  double since_{-1}, last_{-1};
};
}
