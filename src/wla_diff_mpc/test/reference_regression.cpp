#include "wla_diff_mpc/reference.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <stdexcept>

using namespace wla_diff_mpc;

void check(bool value, const char * message) {
  if (!value) throw std::runtime_error(message);
}

int main() {
  nav_msgs::msg::Path path;
  path.poses.resize(3);
  path.poses[0].pose.position.x = 0.0;
  path.poses[1].pose.position.x = 0.2;
  path.poses[2].pose.position.x = 0.4;
  // The requested final orientation opposes travel. The path-following
  // reference must not jump to it within one model step.
  tf2::Quaternion goal_yaw;
  goal_yaw.setRPY(0.0, 0.0, M_PI);
  path.poses[2].pose.orientation = tf2::toMsg(goal_yaw);
  const auto approach = samplePath(path, State(0.25, 0.0, 0.0),
    20, 0.1, 0.3, 0.5, 0.65);
  check(approach.states.size() == 21 && approach.inputs.size() == 20,
        "wrong path reference horizon");
  for (size_t k = 0; k < approach.inputs.size(); ++k) {
    check(std::abs(approach.inputs[k][1]) < 1e-3,
      "goal orientation discontinuity entered path-following reference");
  }
  const auto at_end = samplePath(path, State(0.4, 0.0, 0.0),
    20, 0.1, 0.3, 0.5, 0.65);
  check(std::abs(at_end.states.back()[2]) < 1e-3,
        "terminal path tangent changed to final goal orientation");

  // Final yaw alignment should keep position fixed, be rate-limited, and
  // monotonically reduce a small angular error instead of changing sign.
  const auto alignment = makeRotationReference(State(0.4, 0.0, 0.0),
    0.25, 20, 0.1, 0.22, 0.65);
  double last_rate = 1.0;
  for (size_t k = 0; k < alignment.inputs.size(); ++k) {
    const double rate = alignment.inputs[k][1];
    check(rate >= -1e-9 && rate <= 0.22 + 1e-9,
      "final yaw rate changed sign or exceeded cap");
    check(rate <= last_rate + 1e-9, "final yaw rate did not decay");
    check(std::abs(alignment.states[k][0] - 0.4) < 1e-9 &&
          std::abs(alignment.states[k][1]) < 1e-9,
          "final alignment translated the robot");
    last_rate = rate;
  }
  check(alignment.states.back()[2] > 0.0 && alignment.states.back()[2] < 0.25,
        "final yaw reference overshot target");

  check(finalAlignmentMode(false, 0.19, 0.20, 0.28),
        "did not enter alignment within goal checker xy tolerance");
  check(finalAlignmentMode(true, 0.24, 0.20, 0.28),
        "alignment mode chattered inside hysteresis band");
  check(!finalAlignmentMode(true, 0.30, 0.20, 0.28),
        "alignment did not exit after moving away from goal");
}
