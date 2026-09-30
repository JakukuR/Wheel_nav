#include <cstdlib>
#include <iostream>
#include "odometry_contract.hpp"
void require(bool ok) { if (!ok) { std::cerr << "odometry contract failed\n"; std::exit(1); } }
int main()
{
  const Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  auto v = wla_vio::bodyVelocity(Eigen::Vector3d::Zero(), q,
    Eigen::Vector3d(0, 0.01, 0), q, 0.1);
  require(std::abs(v.linear.x() - 0.1) < 1e-9 && std::abs(v.linear.y()) < 1e-9);
  const Eigen::Quaterniond rotated(Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()));
  v = wla_vio::bodyVelocity(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
    Eigen::Vector3d::Zero(), rotated, 0.1);
  require(std::abs(v.angular.z() - 0.2) < 1e-9);
  bool rejected = false;
  try {wla_vio::bodyVelocity(Eigen::Vector3d::Zero(), q, Eigen::Vector3d::Zero(), q, 0);}
  catch (const std::invalid_argument &) {rejected = true;}
  require(rejected);
  wla_vio::InitializationGate gate;
  for (int i = 0; i < 60; ++i) require(!gate.update(false, i * 0.05, 2.0));
  for (int i = 60; i < 100; ++i) require(!gate.update(true, i * 0.05, 2.0));
  require(gate.update(true, 5.0, 2.0));
  require(!gate.update(false, 5.05, 2.0));
  require(!gate.update(true, 5.1, 2.0));
  require(!gate.update(true, 8.0, 2.0)); // gap resets the window
  require(!gate.update(true, 7.0, 2.0)); // backward stamp resets
  std::cout << "body-frame twist, invalid dt, initialization and gap gates passed\n";
}
