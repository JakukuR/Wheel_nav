#include "wla_diff_mpc/solver.hpp"

#include <OsqpEigen/OsqpEigen.h>

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace wla_diff_mpc {

Solution solve(const Problem & qp, const Reference & reference, double time_limit_seconds) {
  if (qp.horizon < 2 || reference.inputs.size() != static_cast<size_t>(qp.horizon) ||
      !std::isfinite(time_limit_seconds) || time_limit_seconds <= 0.0) {
    throw std::invalid_argument("invalid OSQP problem or time limit");
  }
  Solution result;
  // OsqpEigen v0.8 keeps these buffers until the solver is destroyed.
  Eigen::VectorXd gradient = qp.gradient;
  Eigen::VectorXd lower = qp.lower;
  Eigen::VectorXd upper = qp.upper;
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(false);
  solver.settings()->setMaxIteration(400);
  solver.settings()->setAbsoluteTolerance(1e-3);
  solver.settings()->setRelativeTolerance(1e-3);
  solver.settings()->setTimeLimit(time_limit_seconds);
  solver.data()->setNumberOfVariables(qp.gradient.size());
  solver.data()->setNumberOfConstraints(qp.lower.size());
  if (!solver.data()->setHessianMatrix(qp.hessian) ||
      !solver.data()->setGradient(gradient) ||
      !solver.data()->setLinearConstraintsMatrix(qp.constraints) ||
      !solver.data()->setLowerBound(lower) ||
      !solver.data()->setUpperBound(upper) || !solver.initSolver()) {
    return result;
  }
  const auto start = std::chrono::steady_clock::now();
  const auto exit = solver.solveProblem();
  result.solve_seconds = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start).count();
  if (exit != OsqpEigen::ErrorExitFlag::NoError ||
      solver.getStatus() != OsqpEigen::Status::Solved) {
    return result;
  }
  result.decision = solver.getSolution().cast<double>();
  if (result.decision.size() != qp.gradient.size() || !result.decision.allFinite()) {
    result.decision.resize(0);
    return result;
  }
  const Eigen::VectorXd residual = qp.constraints * result.decision;
  if ((residual.array() < qp.lower.array() - 2e-3).any() ||
      (residual.array() > qp.upper.array() + 2e-3).any()) {
    result.decision.resize(0);
    return result;
  }
  result.command = reference.inputs[0] +
    result.decision.segment<2>(qp.inputIndex(0));
  result.valid = result.command.allFinite();
  return result;
}

}  // namespace wla_diff_mpc
