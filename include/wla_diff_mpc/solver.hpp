#pragma once

#include "wla_diff_mpc/qp.hpp"

namespace wla_diff_mpc {

struct Solution {
  bool valid = false;
  Input command = Input::Zero();
  Eigen::VectorXd decision;
  double solve_seconds = 0.0;
};

// One bounded OSQP solve. An invalid or non-converged QP never yields motion.
Solution solve(const Problem & problem, const Reference & reference,
               double time_limit_seconds = 0.025);

}  // namespace wla_diff_mpc
