#pragma once
#include "wla_diff_mpc/qp.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace wla_diff_mpc {
inline double brakingSpeed(double distance, double deceleration, double reaction, double margin) {
  if (!std::isfinite(distance+deceleration+reaction+margin) || deceleration<=0 || reaction<0 || margin<0)
    return 0;
  const double available=std::max(0.0,distance-margin);
  return std::max(0.0,std::sqrt(std::pow(deceleration*reaction,2)+2*deceleration*available)-deceleration*reaction);
}
// Sample between states so a thin obstacle cannot fall between horizon points.
inline double collisionDistance(const std::vector<State> & states, double resolution,
                               const std::function<bool(const State &)> & blocked) {
  if (states.empty() || resolution<=0) return 0;
  if (!states.front().allFinite() || blocked(states.front())) return 0;
  double distance=0;
  for (size_t k=1;k<states.size();++k) {
    if (!states[k].allFinite()) return 0;
    State delta=states[k]-states[k-1];delta[2]=std::remainder(delta[2],2*M_PI);
    const double length=delta.head<2>().norm();
    const int steps=std::max(1,static_cast<int>(std::ceil(std::max(length/(resolution*0.5),std::abs(delta[2])/0.05))));
    if (steps>4096) return 0;
    for (int i=1;i<=steps;++i) {
      const double t=static_cast<double>(i)/steps;
      if (blocked(states[k-1]+delta*t)) return distance+length*t;
    }
    distance+=length;
  }
  return std::numeric_limits<double>::infinity();
}
inline std::vector<State> nonlinearPrediction(const State & current,const Reference & ref,
                                             const Problem & qp,const Eigen::VectorXd & decision,double dt) {
  std::vector<State> states{current};
  for (int k=0;k<qp.horizon;++k) {
    const Input u=ref.inputs[k]+decision.segment<2>(qp.inputIndex(k));
    const auto & p=states.back(); const double mid=p[2]+u[1]*dt/2;
    states.emplace_back(p[0]+u[0]*dt*std::cos(mid),p[1]+u[0]*dt*std::sin(mid),p[2]+u[1]*dt);
  }
  return states;
}
}
