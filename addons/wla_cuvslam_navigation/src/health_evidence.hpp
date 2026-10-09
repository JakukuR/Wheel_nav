#pragma once
#include <string>
#include <vector>

namespace wla_vio {
struct HealthEvidence {
  bool gravity_available{false}, gravity_valid{false};
  bool imu_available{false}, gyro_bias_valid{false}, accel_bias_valid{false};
  bool velocity_valid{false};
  bool valid() const {
    return gravity_available && gravity_valid && imu_available &&
      gyro_bias_valid && accel_bias_valid && velocity_valid;
  }
  std::string reason(bool stable_ready) const {
    std::vector<std::string> reasons;
    if (!gravity_available) reasons.emplace_back("gravity_unavailable");
    else if (!gravity_valid) reasons.emplace_back("gravity_out_of_bounds");
    if (!imu_available) reasons.emplace_back("imu_state_unavailable");
    else {
      if (!gyro_bias_valid) reasons.emplace_back("gyro_bias_out_of_bounds");
      if (!accel_bias_valid) reasons.emplace_back("accel_bias_out_of_bounds");
    }
    if (!velocity_valid) reasons.emplace_back("velocity_interval_invalid");
    if (reasons.empty()) return stable_ready ? "tracking_inertial_ready" : "inertial_stability_window";
    std::string result;
    for (const auto & reason:reasons) {if (!result.empty()) result+='|';result+=reason;}
    return result;
  }
};
}
