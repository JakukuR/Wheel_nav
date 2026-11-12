#include "wla_ground/geometry.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace wla::ground;
namespace {
void Check(bool value) { if (!value) throw std::runtime_error("time_check_failed"); }
template<class F> void Reject(F function) { bool rejected = false; try { function(); } catch (const std::invalid_argument&) { rejected = true; } Check(rejected); }
}
int main() {
  try {
    AttitudeBuffer buffer(.02, 3); double span = 0.0;
    Reject([&]() { (void)buffer.sample(1, span); });
    const auto q1 = Eigen::Quaterniond(FromRpy(Eigen::Vector3d(0,0,-.02)));
    const auto q2 = Eigen::Quaterniond(FromRpy(Eigen::Vector3d(0,0,.02)));
    buffer.push(1, q1); buffer.push(1.01, q2);
    Check(std::abs(buffer.sample(1.005, span).angularDistance(Eigen::Quaterniond::Identity())) < 1e-10);
    Check(std::abs(span-.01) < 1e-12);
    Reject([&]() { (void)buffer.sample(1.011, span); });
    Reject([&]() { (void)buffer.sample(.99, span); });
    Reject([&]() { buffer.push(1.01, q2); });
    Reject([&]() { buffer.push(.99, q2); });
    buffer.push(1.05, q1); Reject([&]() { (void)buffer.sample(1.03, span); });
    buffer.push(1.06, Eigen::Quaterniond(-q1.w(),-q1.x(),-q1.y(),-q1.z()));
    Check(buffer.sample(1.055, span).angularDistance(q1) < 1e-10);
    Reject([&]() { (void)buffer.sample(1, span); });
    buffer.reset(); Reject([&]() { (void)buffer.sample(1.05, span); });
    std::cout << "time: bounded SLERP, sign flip, gaps, no extrapolation, repeated/reversed time and reset checks passed\n";
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
