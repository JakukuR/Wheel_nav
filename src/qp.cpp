#include "wla_diff_mpc/qp.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace wla_diff_mpc {
namespace {

void validate(const Settings & s, const State & current, const Input & previous,
              const Reference & reference) {
  if (s.horizon < 2 || s.horizon > 80 || !std::isfinite(s.dt) || s.dt <= 0.0 || s.dt > 0.5 ||
      !current.allFinite() || !previous.allFinite() ||
      reference.states.size() != static_cast<size_t>(s.horizon + 1) ||
      reference.inputs.size() != static_cast<size_t>(s.horizon)) {
    throw std::invalid_argument("invalid differential MPC size, timing or state");
  }
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(s.q[i]) || !std::isfinite(s.q_terminal[i]) ||
        s.q[i] < 0.0 || s.q_terminal[i] < 0.0) {
      throw std::invalid_argument("MPC state weights must be finite and nonnegative");
    }
  }
  for (int i = 0; i < 2; ++i) {
    if (!std::isfinite(s.r[i]) || !std::isfinite(s.r_rate[i]) ||
        !std::isfinite(s.u_min[i]) || !std::isfinite(s.u_max[i]) ||
        !std::isfinite(s.acceleration_max[i]) || s.r[i] <= 0.0 ||
        s.r_rate[i] < 0.0 || s.u_min[i] >= s.u_max[i] ||
        s.acceleration_max[i] <= 0.0) {
      throw std::invalid_argument("invalid differential MPC input bounds or weights");
    }
  }
  for (const auto & x : reference.states) {
    if (!x.allFinite()) throw std::invalid_argument("nonfinite MPC reference state");
  }
  for (const auto & u : reference.inputs) {
    if (!u.allFinite()) throw std::invalid_argument("nonfinite MPC reference input");
  }
}

}  // namespace

LinearStep linearize(const State & x, const Input & u, const State & next, double dt) {
  LinearStep step{Eigen::Matrix3d::Identity(), Eigen::Matrix<double, 3, 2>::Zero(),
                  State::Zero()};
  step.a(0, 2) = -dt * u[0] * std::sin(x[2]);
  step.a(1, 2) = dt * u[0] * std::cos(x[2]);
  step.b(0, 0) = dt * std::cos(x[2]);
  step.b(1, 0) = dt * std::sin(x[2]);
  step.b(2, 1) = dt;
  step.defect = x - next;
  step.defect[0] += dt * u[0] * std::cos(x[2]);
  step.defect[1] += dt * u[0] * std::sin(x[2]);
  step.defect[2] += dt * u[1];
  return step;
}

Problem makeProblem(const Settings & s, const State & current,
                    const Input & previous, const Reference & ref) {
  validate(s, current, previous, ref);
  Problem qp;
  qp.horizon = s.horizon;
  const int n = 3 * (s.horizon + 1) + 2 * s.horizon;
  const int dynamics_rows = 3 * (s.horizon + 1);
  const int input_rows = dynamics_rows + 2 * s.horizon;
  const int m = input_rows + 2 * s.horizon;
  qp.hessian.resize(n, n);
  qp.constraints.resize(m, n);
  qp.gradient = Eigen::VectorXd::Zero(n);
  qp.lower = Eigen::VectorXd::Zero(m);
  qp.upper = Eigen::VectorXd::Zero(m);
  std::vector<Eigen::Triplet<double>> h;
  std::vector<Eigen::Triplet<double>> a;

  // OSQP minimizes 1/2 z' H z + g' z. Decision vector is
  // z = [e_0 ... e_N, delta_u_0 ... delta_u_(N-1)].
  for (int k = 1; k <= s.horizon; ++k) {
    const State & q = k == s.horizon ? s.q_terminal : s.q;
    for (int j = 0; j < 3; ++j) h.emplace_back(qp.stateIndex(k) + j,
                                               qp.stateIndex(k) + j, 2.0 * q[j]);
  }
  for (int k = 0; k < s.horizon; ++k) {
    for (int j = 0; j < 2; ++j) h.emplace_back(qp.inputIndex(k) + j,
                                               qp.inputIndex(k) + j, 2.0 * s.r[j]);
  }

  // Penalize actual command changes, including the first change from the last
  // command issued. The off-diagonal entries belong only to the upper triangle.
  for (int k = 0; k < s.horizon; ++k) {
    const Input ref_difference = k == 0 ? ref.inputs[0] - previous :
      ref.inputs[k] - ref.inputs[k - 1];
    for (int j = 0; j < 2; ++j) {
      const int now = qp.inputIndex(k) + j;
      h.emplace_back(now, now, 2.0 * s.r_rate[j]);
      qp.gradient[now] += 2.0 * s.r_rate[j] * ref_difference[j];
      if (k > 0) {
        const int before = qp.inputIndex(k - 1) + j;
        h.emplace_back(before, before, 2.0 * s.r_rate[j]);
        h.emplace_back(before, now, -2.0 * s.r_rate[j]);
        qp.gradient[before] -= 2.0 * s.r_rate[j] * ref_difference[j];
      }
    }
  }

  // e_0 = measured pose - reference pose; wrap yaw to the closest branch.
  State initial_error = current - ref.states[0];
  initial_error[2] = std::remainder(initial_error[2], 2.0 * M_PI);
  for (int j = 0; j < 3; ++j) {
    a.emplace_back(j, qp.stateIndex(0) + j, 1.0);
    qp.lower[j] = qp.upper[j] = initial_error[j];
  }
  for (int k = 0; k < s.horizon; ++k) {
    const auto step = linearize(ref.states[k], ref.inputs[k], ref.states[k + 1], s.dt);
    const int row = qp.stateIndex(k + 1);
    for (int j = 0; j < 3; ++j) {
      a.emplace_back(row + j, qp.stateIndex(k + 1) + j, -1.0);
      for (int col = 0; col < 3; ++col) {
        a.emplace_back(row + j, qp.stateIndex(k) + col, step.a(j, col));
      }
      for (int col = 0; col < 2; ++col) {
        a.emplace_back(row + j, qp.inputIndex(k) + col, step.b(j, col));
      }
      qp.lower[row + j] = qp.upper[row + j] = -step.defect[j];
    }
    for (int j = 0; j < 2; ++j) {
      const int command_row = dynamics_rows + 2 * k + j;
      a.emplace_back(command_row, qp.inputIndex(k) + j, 1.0);
      qp.lower[command_row] = s.u_min[j] - ref.inputs[k][j];
      qp.upper[command_row] = s.u_max[j] - ref.inputs[k][j];

      const int rate_row = input_rows + 2 * k + j;
      a.emplace_back(rate_row, qp.inputIndex(k) + j, 1.0);
      const double reference_change = k == 0 ? ref.inputs[k][j] - previous[j] :
        ref.inputs[k][j] - ref.inputs[k - 1][j];
      if (k > 0) a.emplace_back(rate_row, qp.inputIndex(k - 1) + j, -1.0);
      qp.lower[rate_row] = -s.acceleration_max[j] * s.dt - reference_change;
      qp.upper[rate_row] = s.acceleration_max[j] * s.dt - reference_change;
    }
  }
  qp.hessian.setFromTriplets(h.begin(), h.end());
  qp.constraints.setFromTriplets(a.begin(), a.end());
  qp.hessian.makeCompressed();
  qp.constraints.makeCompressed();
  return qp;
}

}  // namespace wla_diff_mpc
