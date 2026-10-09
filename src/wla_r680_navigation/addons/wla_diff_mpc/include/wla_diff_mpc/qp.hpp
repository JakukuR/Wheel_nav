#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include <vector>

namespace wla_diff_mpc {

using State = Eigen::Vector3d;  // x [m], y [m], unwrapped yaw [rad]
using Input = Eigen::Vector2d;  // forward speed [m/s], yaw rate [rad/s]

struct Settings {
  int horizon = 20;
  double dt = 0.1;
  State q = (State() << 8.0, 8.0, 3.0).finished();
  State q_terminal = (State() << 16.0, 16.0, 6.0).finished();
  Input r = (Input() << 0.4, 0.3).finished();
  Input r_rate = (Input() << 0.8, 0.5).finished();
  Input u_min = (Input() << -0.08, -0.5).finished();
  Input u_max = (Input() << 0.3, 0.5).finished();
  Input acceleration_max = (Input() << 0.6, 1.2).finished();
};

struct Reference {
  std::vector<State> states;  // horizon + 1
  std::vector<Input> inputs;  // horizon
};

struct LinearStep {
  Eigen::Matrix3d a;
  Eigen::Matrix<double, 3, 2> b;
  State defect;
};

// Linearize the differential-drive unicycle around one reference step.
LinearStep linearize(const State & state, const Input & input,
                     const State & next, double dt);

struct Problem {
  Eigen::SparseMatrix<double> hessian;  // upper triangle of 1/2 z' H z
  Eigen::VectorXd gradient;
  Eigen::SparseMatrix<double> constraints;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  int horizon = 0;

  int stateIndex(int k) const { return 3 * k; }
  int inputIndex(int k) const { return 3 * (horizon + 1) + 2 * k; }
};

Problem makeProblem(const Settings & settings, const State & current,
                    const Input & previous_command, const Reference & reference);

}  // namespace wla_diff_mpc
