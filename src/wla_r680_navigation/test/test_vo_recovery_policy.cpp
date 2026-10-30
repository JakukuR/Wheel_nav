#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "wla_r680_navigation/vo_recovery_policy.hpp"

using wla_r680_navigation::RecoveryConfig;
using wla_r680_navigation::RecoveryInput;
using wla_r680_navigation::RecoveryPolicy;
using State = RecoveryPolicy::State;

namespace {
// Integrate the commanded turn as the heading, like a robot that obeys.
struct Rig {
  RecoveryPolicy p{RecoveryConfig{}};
  RecoveryInput in{0, true, true, true, true, 0};
  int restarts{0};
  double max_turn{0}, min_turn{0};
  void run(double seconds, bool follow = true)
  {
    for (const double end = in.now + seconds; in.now < end; in.now += 0.05) {
      const auto out = p.tick(in);
      restarts += out.restart;
      if (follow) {in.heading += out.angular * 0.05;}
      max_turn = std::max(max_turn, in.heading);
      min_turn = std::min(min_turn, in.heading);
      assert(std::abs(out.angular) <= 0.3 + 1e-12);
    }
  }
};
}  // namespace

int main()
{
  {
    Rig r;
    r.run(2.9);
    assert(r.p.state() == State::Waiting && !r.p.moving());
    r.run(1.0);
    assert(r.p.moving());
    r.run(20);
    assert(r.max_turn > 1.0 && r.min_turn < -1.0 && std::abs(r.in.heading) < 0.06);
    assert(r.restarts == 1 && r.p.state() == State::Restart);
    std::puts("PASS wait, sweep +-60 deg, return, then one frontend restart");
    r.in.lost = false;
    r.p.tick(r.in);
    assert(r.p.state() == State::Idle && r.p.attempts() == 0);
    std::puts("PASS recovered odometry stands down");
  }
  {
    Rig r;
    r.in.stopped = false;
    r.run(10);
    assert(r.p.state() == State::Waiting && !r.p.moving());
    r.in.stopped = true;
    r.in.authorized = false;
    r.run(5);
    assert(!r.p.moving());
    std::puts("PASS never starts while moving or unauthorized");
  }
  {
    Rig r;
    r.run(4);
    assert(r.p.moving());
    r.in.clear = false;
    r.run(0.1);
    assert(r.p.state() == State::Cooldown && !r.p.moving() && r.p.attempts() == 1);
    std::puts("PASS blocked near field aborts the sweep");
  }
  {
    Rig r;
    // A robot that cannot turn: legs time out, attempts run out, then it waits.
    r.run(300, false);
    assert(r.p.state() == State::Exhausted && r.restarts == 3);
    std::puts("PASS bounded attempts, then waits for an operator");
    r.in.lost = false; r.p.tick(r.in); r.in.lost = true; r.p.tick(r.in);
    assert(r.p.state() == State::Waiting);
    std::puts("PASS a later loss starts fresh");
  }
  {
    bool threw = false;
    try {RecoveryConfig c; c.angular = 0.8; RecoveryPolicy p(c);} catch (const std::invalid_argument &) {threw = true;}
    assert(threw);
    std::puts("PASS envelope limits enforced");
  }
  return 0;
}
