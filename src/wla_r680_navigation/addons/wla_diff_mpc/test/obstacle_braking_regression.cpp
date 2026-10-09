#include "wla_diff_mpc/obstacle_braking.hpp"
#include "wla_diff_mpc/solver.hpp"
#include <stdexcept>
using namespace wla_diff_mpc;
void check(bool x){if(!x)throw std::runtime_error("obstacle braking regression");}
int main(){
  const double v=brakingSpeed(0.45,0.4,0.7,0.1);
  check(v>0 && v<0.4 && v*0.7+v*v/(2*0.4)<=0.35+1e-9);
  check(brakingSpeed(0.05,0.4,0.7,0.1)==0);
  std::vector<State> states{{0,0,0},{0.1,0,0}};
  const double d=collisionDistance(states,0.02,[](const State & p){return p[0]>=0.045 && p[0]<=0.055;});
  check(d>=0.045 && d<=0.055); // Thin obstacle between prediction knots.
  check(std::isinf(collisionDistance(states,0.02,[](const State &){return false;})));
  check(collisionDistance(states,0.02,[](const State &){return true;})==0);
  states={{0,0,3.13},{0,0,-3.13}};
  check(std::isinf(collisionDistance(states,0.02,[](const State & p){return std::abs(p[2])<3;})));
  Settings s; Reference ref;
  for(int k=0;k<=s.horizon;++k){ref.states.emplace_back(k*0.02,0,0);if(k<s.horizon)ref.inputs.emplace_back(0.2,0);}
  auto qp=makeProblem(s,State::Zero(),Input::Zero(),ref);
  auto result=solve(qp,ref,0.1);check(result.valid);
  auto prediction=nonlinearPrediction(State::Zero(),ref,qp,result.decision,s.dt);
  check(prediction.size()==21 && prediction[1][0]>0 && std::abs(prediction.back()[1])<1e-5);
  check(std::isfinite(collisionDistance(prediction,0.02,[](const State & p){return p[0]>0.25;})));
  s.u_max[0]=0.08;
  for(int k=0;k<=s.horizon;++k){ref.states[k]=State(k*0.008,0,0);if(k<s.horizon)ref.inputs[k]=Input(0.08,0);}
  qp=makeProblem(s,State::Zero(),Input(0.3,0),ref);result=solve(qp,ref,0.1);check(result.valid);
  prediction=nonlinearPrediction(State::Zero(),ref,qp,result.decision,s.dt);
  check(result.command[0]<=0.08+0.002);
  check(std::isinf(collisionDistance(prediction,0.02,[](const State & p){return p[0]>0.25;})));
}
