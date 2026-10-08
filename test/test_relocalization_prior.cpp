#include "wla_r680_navigation/relocalization_prior.hpp"
#include <limits>
#include <stdexcept>
using namespace wla_r680_navigation;
void check(bool x) {if (!x) throw std::runtime_error("relocalization prior regression");}
int main() {
  auto p=stoppedRelocalizationPrior({3,4,M_PI/2},{10,20,0},{10.1,20,0.1},0.5,0.8);
  check(p && std::abs(p->x-3)<1e-9 && std::abs(p->y-4.1)<1e-9);
  check(std::abs(p->yaw-M_PI/2-0.1)<1e-9);
  check(!stoppedRelocalizationPrior({3,4,0},{0,0,0},{0.6,0,0},0.5,0.8));
  check(!stoppedRelocalizationPrior({3,4,0},{0,0,0},{0,0,0.9},0.5,0.8));
  check(!stoppedRelocalizationPrior({3,4,0},{0,0,0},{std::numeric_limits<double>::quiet_NaN(),0,0},0.5,0.8));
  p=stoppedRelocalizationPrior({0,0,3.13},{0,0,3.13},{0,0,-3.13},0.5,0.8);
  check(p && std::abs(p->yaw+3.13)<1e-9);
}
