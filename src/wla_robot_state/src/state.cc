#include "wla_robot_state/state.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace wla::state {
std::array<double, 8> ParseJointFeedback(const std::string& json) {
  const auto value = nlohmann::json::parse(json);
  if (!value.at("ready").get<bool>() || !value.at("gate").get<std::string>().empty())
    throw std::invalid_argument("controller_feedback_not_ready");
  const auto& q = value.at("q_rad");
  if (!q.is_array() || q.size() != JOINT_NAMES.size()) throw std::invalid_argument("joint_feedback_size_mismatch");
  std::array<double, 8> out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = q.at(i).get<double>();
    if (!std::isfinite(out[i])) throw std::invalid_argument("nonfinite_joint_feedback");
  }
  return out;
}
Eigen::Quaterniond CheckOrientation(const Eigen::Quaterniond& value) {
  if (!value.coeffs().allFinite() || std::abs(value.norm()-1.0) > .05)
    throw std::invalid_argument("invalid_imu_orientation");
  return value.normalized();
}
Eigen::Vector3d ToRpy(const Eigen::Quaterniond& value) {
  const auto r = CheckOrientation(value).toRotationMatrix();
  return {std::atan2(r(2,1), r(2,2)), std::asin(std::clamp(-r(2,0), -1.0, 1.0)), std::atan2(r(1,0), r(0,0))};
}
}  // namespace wla::state
