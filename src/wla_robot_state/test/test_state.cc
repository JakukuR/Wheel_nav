#include "wla_robot_state/state.h"
#include <nlohmann/json.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace wla::state;
namespace {
void Check(bool value) { if (!value) throw std::runtime_error("state_semantics_failed"); }
template<class F> void Reject(F function) {
  bool rejected = false; try { function(); } catch (const std::exception&) { rejected = true; } Check(rejected);
}
}
int main() {
  try {
    nlohmann::json source{{"ready",true},{"gate",""},{"q_rad",{0.,1.,2.,3.,4.,5.,6.,7.}},
        {"reference_q_rad",{99.,99.,99.,99.,99.,99.}}};
    const auto q = ParseJointFeedback(source.dump());
    Check(q[0] == 0 && q[7] == 7 && JOINT_NAMES[0] == "abad_L_Joint" && JOINT_NAMES[7] == "wheel_R_Joint");
    source["ready"] = false; Reject([&]() { (void)ParseJointFeedback(source.dump()); });
    source["ready"] = true; source["gate"] = "stale"; Reject([&]() { (void)ParseJointFeedback(source.dump()); });
    source["gate"] = ""; source["q_rad"] = {1,2}; Reject([&]() { (void)ParseJointFeedback(source.dump()); });
    Reject([&]() { (void)CheckOrientation(Eigen::Quaterniond(0,0,0,0)); });
    const auto orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.8,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(-.2,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitX()));
    const auto initial = ToRpy(orientation);
    Check(std::abs(initial.x()-.1) < 1e-12 && std::abs(initial.y()+.2) < 1e-12);
    const Eigen::Quaterniond zero_yaw(Eigen::AngleAxisd(-initial.z(),Eigen::Vector3d::UnitZ()));
    const auto relative = ToRpy(zero_yaw*orientation);
    Check(std::abs(relative.z()) < 1e-12 && std::abs(relative.x()-.1) < 1e-12 && std::abs(relative.y()+.2) < 1e-12);
    std::cout << "state semantics: feedback names, target exclusion, readiness, input validation and yaw gauge passed\n";
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
