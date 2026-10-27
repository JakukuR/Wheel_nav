#include <cstdlib>
#include <string>
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
  using State = wla::InitPolicy::State; using Leg = wla::InitPolicy::Leg;
  auto i = healthy();
  wla::InitPolicy p;
  start(p, i);
  // Forward leg: straight, no turn, no reverse.
  for (int n = 0; n < 10; ++n) {
    auto c = p.tick(1.1 + n*0.1, i);
    require(c.linear > 0 && c.linear <= 0.06 && c.angular == 0);
  }
  // Turning in place stays under the collision monitor; the forward box is not a fault.
  i.travel = 0.16;
  auto c = p.tick(2.2, i);
  require(p.leg() == Leg::TurnOut && c.linear == 0 && c.angular > 0 && c.angular <= 0.20);
  require(p.state() == State::Moving);
  i.heading = 0.26; i.clear = false; c = p.tick(2.3, i);
  require(p.leg() == Leg::TurnBack && c.linear == 0 && c.angular < 0);
  i.heading = 0.04; i.clear = true;
  require(p.tick(2.4, i).angular == 0 && p.state() == State::Stopping);
  p.tick(2.5, i); p.tick(3.1, i);
  require(p.state() == State::Settling && std::string(p.name()) == "waiting_inertial");
  p.tick(3.2, i); require(p.state() == State::Settling);
  // Success needs inertial initialization only; map admission is gated elsewhere.
  i.inertial = true; i.localized = false; p.tick(3.3, i);
  require(p.state() == State::Succeeded);
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
  wla::InitPolicy blocked; i = healthy(); start(blocked, i); i.clear = false;
  blocked.tick(1.1, i); require(blocked.state() == State::Failed && blocked.reason() == "near_field_blocked");
  // Hardware output off: no measured motion, every leg ends on its own time bound.
  wla::InitPolicy preview; i = healthy(); start(preview, i);
  for (double t = 1.1; t < 12 && preview.state() == State::Moving; t += 0.1) preview.tick(t, i);
  require(preview.state() == State::Stopping);
  wla::InitPolicy cap; i = healthy(); start(cap, i); cap.tick(9.5, i);
  require(cap.state() == State::Stopping);  // motion_duration bounds all legs together
  // Inertial ready but not yet localized: still drive the legs for map viewpoints.
  wla::InitPolicy views; i = healthy(); i.inertial = true; start(views, i);
  // Static wait: hold still until inertial or the wait expires, then drive the legs.
  wla::InitConfig waiting_config; waiting_config.static_wait = 5;
  wla::InitPolicy hold(waiting_config); i = healthy();
  hold.tick(0, i); hold.tick(0.1, i); hold.tick(1, i);
  require(hold.state() == State::Stationary);
  i.inertial = true; hold.tick(2, i);
  require(hold.state() == State::Moving && hold.leg() == Leg::Forward);
  wla::InitPolicy expire(waiting_config); i = healthy();
  expire.tick(0, i); expire.tick(0.1, i); expire.tick(5.8, i);
  require(expire.state() == State::Stationary);
  expire.tick(6, i); require(expire.state() == State::Moving);
  wla::InitPolicy initialized; i = healthy(); i.inertial = i.localized = true;
  require(initialized.tick(0, i).linear == 0);
  require(initialized.state() == State::Stopping);
  require(wla::bound_init_component(1, 0.06, 0.08) == 0.06);
  require(wla::bound_init_component(0.06, 0, 0.08) == 0); // bypass residual smoother velocity on stop
  require(wla::bound_init_component(0.2, -0.1, 0.2) == 0); // sign reversal
  require(wla::bound_init_component(std::numeric_limits<double>::quiet_NaN(), 0.06, 0.08) == 0);
  std::cout << "forward/turn/return legs, inertial-only success, seven latched faults, timeouts and residual bounds passed\n";
}
