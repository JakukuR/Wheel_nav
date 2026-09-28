#include "wla_diff_mpc/qp.hpp"
#include "wla_diff_mpc/solver.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace wla_diff_mpc;

void check(bool condition, const char * message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  const State x(1.0, 2.0, 0.4);
  const Input u(0.2, -0.3);
  const State next(1.05, 2.04, 0.37);
  const auto step = linearize(x, u, next, 0.1);
  for (int i = 0; i < 3; ++i) {
    State xp = x;
    xp[i] += 1e-6;
    const auto fp = linearize(xp, u, next, 0.1).defect;
    check(((fp - step.defect) / 1e-6 - step.a.col(i)).norm() < 1e-5,
          "state Jacobian differs from finite difference");
  }
  for (int i = 0; i < 2; ++i) {
    Input up = u;
    up[i] += 1e-6;
    const auto fp = linearize(x, up, next, 0.1).defect;
    check(((fp - step.defect) / 1e-6 - step.b.col(i)).norm() < 1e-5,
          "input Jacobian differs from finite difference");
  }

  Settings s;
  Reference straight;
  for (int k = 0; k <= s.horizon; ++k) {
    straight.states.emplace_back(0.02 * k, 0.0, 0.0);
    if (k < s.horizon) straight.inputs.emplace_back(0.2, 0.0);
  }
  auto qp = makeProblem(s, State(-0.15, 0.0, 0.0), Input::Zero(), straight);
  auto answer = solve(qp, straight, 0.1);
  check(answer.valid, "straight QP did not solve");
  check(answer.command[0] > 0.0 && answer.command[0] <= s.acceleration_max[0] * s.dt + 2e-3,
        "first forward command violates acceleration bound");
  check(std::abs(answer.command[1]) < 0.02, "straight QP asks for rotation");
  const Eigen::VectorXd residual = qp.constraints * answer.decision;
  check((residual.array() >= qp.lower.array() - 2e-3).all() &&
        (residual.array() <= qp.upper.array() + 2e-3).all(), "QP constraints violated");

  Reference rotation;
  for (int k = 0; k <= s.horizon; ++k) {
    rotation.states.emplace_back(0.0, 0.0, 0.0);
    if (k < s.horizon) rotation.inputs.emplace_back(0.0, 0.0);
  }
  qp = makeProblem(s, State(0.0, 0.0, 0.4), Input::Zero(), rotation);
  answer = solve(qp, rotation, 0.1);
  check(answer.valid, "rotation QP did not solve");
  check(answer.command[1] < 0.0 &&
        answer.command[1] >= -s.acceleration_max[1] * s.dt - 2e-3,
        "yaw correction violates direction or acceleration bound");
  check(std::abs(answer.command[0]) < 0.02, "rotation QP asks for translation");

  // A curved reference has nonzero yaw rate. The first command must still
  // satisfy the previous-command slew limit and the QP equalities.
  Reference curve;
  const double curvature = 0.4;
  for (int k = 0; k <= s.horizon; ++k) {
    const double t = s.dt * k;
    curve.states.emplace_back(0.2 / curvature * std::sin(curvature * t),
      0.2 / curvature * (1.0 - std::cos(curvature * t)), curvature * t);
    if (k < s.horizon) curve.inputs.emplace_back(0.2, curvature);
  }
  qp = makeProblem(s, State(-0.03, 0.01, 0.02), Input::Zero(), curve);
  answer = solve(qp, curve, 0.1);
  check(answer.valid, "curved QP did not solve");
  check(answer.command[0] >= s.u_min[0] - 2e-3 &&
        answer.command[0] <= s.acceleration_max[0] * s.dt + 2e-3,
        "curved QP exceeds first longitudinal command limit");
  check(std::abs(answer.command[1]) <= s.acceleration_max[1] * s.dt + 2e-3,
        "curved QP exceeds first yaw command limit");
  const Eigen::VectorXd curved_residual = qp.constraints * answer.decision;
  check((curved_residual.array() >= qp.lower.array() - 2e-3).all() &&
        (curved_residual.array() <= qp.upper.array() + 2e-3).all(),
        "curved QP constraints violated");

  // The Hessian exposed to OSQP is upper triangular. Its symmetric form
  // should remain positive semidefinite, including command-rate cross terms.
  Eigen::MatrixXd dense = Eigen::MatrixXd(qp.hessian);
  dense = dense.selfadjointView<Eigen::Upper>();
  check((dense.diagonal().array() >= 0.0).all(), "negative Hessian diagonal");
  for (int k = 0; k < 10; ++k) {
    const Eigen::VectorXd test = Eigen::VectorXd::LinSpaced(dense.rows(), -1.0 + k * 0.1, 1.0);
    check(test.dot(dense * test) >= -1e-8, "Hessian is not positive semidefinite");
  }

  try {
    Settings bad = s;
    bad.dt = 0.0;
    (void)makeProblem(bad, State::Zero(), Input::Zero(), rotation);
    throw std::runtime_error("invalid time step accepted");
  } catch (const std::invalid_argument &) {}
  std::cout << "differential MPC QP regression passed\n";
}
