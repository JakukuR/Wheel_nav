#ifndef WLA_ROBOT_STATE_INCLUDE_STATE_H
#define WLA_ROBOT_STATE_INCLUDE_STATE_H
#include <Eigen/Geometry>
#include <array>
#include <string>
namespace wla::state {
inline const std::array<std::string, 8> JOINT_NAMES = {"abad_L_Joint", "abad_R_Joint",
    "hip_L_Joint", "hip_R_Joint", "knee_L_Joint", "knee_R_Joint", "wheel_L_Joint", "wheel_R_Joint"};
[[nodiscard]] std::array<double, 8> ParseJointFeedback(const std::string& json);
[[nodiscard]] Eigen::Quaterniond CheckOrientation(const Eigen::Quaterniond& value);
[[nodiscard]] Eigen::Vector3d ToRpy(const Eigen::Quaterniond& value);
}  // namespace wla::state
#endif
