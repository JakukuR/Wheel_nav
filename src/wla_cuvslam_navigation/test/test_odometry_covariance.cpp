#include "odometry_covariance.hpp"
#include <Eigen/Dense>
#include <stdexcept>
using namespace wla_vio;
void check(bool v, const char *what) {if(!v)throw std::runtime_error(what);}
// RTAB-Map takes the inverse of the pose covariance as link information and
// aborts unless every diagonal entry of that inverse is finite and > 0.
bool rtabmap_accepts(const std::array<double, 36> &c) {
  const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> m(c.data());
  const Eigen::FullPivLU<Eigen::Matrix<double, 6, 6>> lu(m);
  if (!lu.isInvertible()) return false;
  const Eigen::Matrix<double, 6, 6> info = lu.inverse();
  for (int i = 0; i < 6; ++i) if (!std::isfinite(info(i, i)) || info(i, i) <= 0) return false;
  return true;
}
int main() {
  NominalCovariance n;n.validate();
  for (const bool inertial : {false, true}) {
    check(rtabmap_accepts(n.pose(inertial)), "pose covariance not invertible");
    check(rtabmap_accepts(n.twist(inertial)), "twist covariance not invertible");
  }
  // Inertial-only poses are weaker evidence, never stronger, and stay below the
  // watchdog's 1.0 m^2 / rad^2 rejection limit at the default scale.
  check(n.pose(true)[0] > n.pose(false)[0] && n.pose(true)[35] > n.pose(false)[35], "inertial-only not down-weighted");
  check(n.pose(true)[0] < 1.0 && n.pose(true)[35] < 1.0, "default inertial-only variance above watchdog limit");
  // The matrix that crashed RTAB-Map: stationary history spread with constant
  // orientation has zero rotational variance.
  std::array<double, 36> history{};history[0]=1.44e-8;history[7]=1.07e-8;history[14]=3.69e-10;
  check(!rtabmap_accepts(history), "regression fixture no longer singular");
  for (const double bad : {0.0, -1e-3, std::nan("")}) {
    bool rejected = false;
    NominalCovariance b;b.linear_variance=bad;
    try {b.validate();} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected, "invalid variance accepted");
  }
  bool rejected = false;
  NominalCovariance s;s.inertial_only_scale=0.5;
  try {s.validate();} catch (const std::invalid_argument &) {rejected = true;}
  check(rejected, "scale below 1 accepted");
}
