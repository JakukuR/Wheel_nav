#include <cstdlib>
#include <iostream>
#include <limits>
#include "wla_r680_navigation/vio_init_policy.hpp"
namespace wla = wla_r680_navigation;
void require(bool value) {if (!value) {std::cerr << "init policy regression failed\n"; std::exit(1);}}
wla::InitInput healthy() {wla::InitInput i; i.sensors = i.services = i.clear = i.stopped = true; return i;}
void start(wla::InitPolicy & p, wla::InitInput & i) {
  p.tick(0, i); p.tick(0.1, i); p.tick(1, i);
  require(p.state() == wla::InitPolicy::State::Moving);
}
int main() {
  auto i = healthy();
  wla::InitPolicy p;
  start(p, i);
  for (int n = 0; n < 30; ++n) {
    auto c = p.tick(1.1 + n*0.1, i);
    require(c.linear > 0 && c.linear <= 0.06 && std::abs(c.angular) <= 0.20);
  }
  i.inertial = true;
  require(p.tick(4.2, i).linear == 0);
  require(p.state() == wla::InitPolicy::State::Stopping);
  p.tick(4.3, i); p.tick(4.9, i);
  require(p.state() == wla::InitPolicy::State::Localization);
  p.tick(5, i); require(p.state() != wla::InitPolicy::State::Succeeded);
  i.localized = true; p.tick(5.1, i);
  require(p.state() == wla::InitPolicy::State::Succeeded);
  for (int fault = 0; fault < 7; ++fault) {
    wla::InitPolicy f; auto input = healthy(); start(f, input);
    switch (fault) {
      case 0: input.sensors = false; break;
      case 1: input.services = false; break;
      case 2: input.clear = false; break;
      case 3: input.radius = 0.25; break;
      case 4: input.travel = 0.25; break;
      case 5: input.yaw = 0.35; break;
      default: input.speed = 0.13;
    }
    require(f.tick(1.1, input).linear == 0);
    require(f.state() == wla::InitPolicy::State::Failed);
    input = healthy(); f.tick(2, input);
    require(f.state() == wla::InitPolicy::State::Failed);
  }
  wla::InitPolicy timeout; i = healthy(); start(timeout, i); timeout.tick(22, i);
  require(timeout.state() == wla::InitPolicy::State::Failed);
  require(timeout.reason() == "inertial_initialization_timeout");
  wla::InitPolicy localization_timeout; i = healthy(); start(localization_timeout, i);
  i.inertial = true; localization_timeout.tick(22, i);
  require(localization_timeout.reason() == "mapping_or_localization_timeout");
  wla::InitPolicy budget; i = healthy(); start(budget, i); i.travel = 0.16;
  require(budget.tick(1.2, i).linear == 0);
  require(budget.state() == wla::InitPolicy::State::Stopping);
  wla::InitPolicy initialized; i = healthy(); i.inertial = true;
  require(initialized.tick(0, i).linear == 0);
  require(initialized.state() == wla::InitPolicy::State::Stopping);
  require(wla::bound_init_component(1, 0.06, 0.08) == 0.06);
  require(wla::bound_init_component(0.06, 0, 0.08) == 0); // bypass residual smoother velocity on stop
  require(wla::bound_init_component(0.2, -0.1, 0.2) == 0); // sign reversal
  require(wla::bound_init_component(std::numeric_limits<double>::quiet_NaN(), 0.06, 0.08) == 0);
  std::cout << "success, stop/localization handoff, seven latched faults, timeout, budget and residual bounds passed\n";
}
