#include "wla_diff_mpc/solver.hpp"

#include <OsqpEigen/OsqpEigen.h>

#include <algorithm>
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
  solver.settings()->setMaxIteration(800);
  solver.settings()->setAbsoluteTolerance(5e-4);
  solver.settings()->setRelativeTolerance(5e-4);
  solver.settings()->setTimeLimit(time_limit_seconds);
  solver.data()->setNumberOfVariables(qp.gradient.size());
  solver.data()->setNumberOfConstraints(qp.lower.size());
  if (!solver.data()->setHessianMatrix(qp.hessian) ||
      !solver.data()->setGradient(gradient) ||
      !solver.data()->setLinearConstraintsMatrix(qp.constraints) ||
      !solver.data()->setLowerBound(lower) ||
      !solver.data()->setUpperBound(upper) || !solver.initSolver()) {
    result.failure_reason = "OSQP initialization failed";
    return result;
  }
  const auto start = std::chrono::steady_clock::now();
  const auto exit = solver.solveProblem();
  result.solve_seconds = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start).count();
  if (exit != OsqpEigen::ErrorExitFlag::NoError ||
      solver.getStatus() != OsqpEigen::Status::Solved) {
    result.failure_reason = "OSQP exit=" + std::to_string(static_cast<int>(exit)) +
      " status=" + std::to_string(static_cast<int>(solver.getStatus()));
    return result;
  }
  result.decision = solver.getSolution().cast<double>();
  if (result.decision.size() != qp.gradient.size() || !result.decision.allFinite()) {
    result.failure_reason = "nonfinite or wrong-sized OSQP solution";
    result.decision.resize(0);
    return result;
  }
  const Eigen::VectorXd residual = qp.constraints * result.decision;
  const double max_violation = std::max((qp.lower - residual).maxCoeff(),
    (residual - qp.upper).maxCoeff());
  if (max_violation > 2e-3) {
    result.failure_reason = "OSQP constraint violation=" + std::to_string(max_violation);
    result.decision.resize(0);
    return result;
  }
  result.command = reference.inputs[0] +
    result.decision.segment<2>(qp.inputIndex(0));
  result.valid = result.command.allFinite();
  if (!result.valid) result.failure_reason = "nonfinite command";
  return result;
}

}  // namespace wla_diff_mpc
