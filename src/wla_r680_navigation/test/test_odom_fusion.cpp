// Synthetic checks of the sliding-window odometry core (no ROS).
#include <cassert>
#include <cmath>
#include <cstdio>
#include <functional>

#include "wla_r680_navigation/odom_fusion.hpp"

using namespace wla_r680_navigation::fusion;
using gtsam::Pose2;

namespace {
struct Truth {double v{}, w{};};

struct Sim {
  Estimator est{Config{}};
  Pose2 truth;
  double t{0}, bias{0.01}, last_v{0};
  Pose2 visual_offset;                 // cuVSLAM frame jump
  std::function<Truth(double)> motion = [](double) {return Truth{};};
  std::function<bool(double)> visual = [](double) {return true;};
  std::function<double(double)> wheel_v;  // measured wheel speed (slip), default truth
  double next_visual{0}, next_key{0};
  double max_output_step{0};
  bool ever_bridging{false};
  std::optional<Pose2> last_output;

  void run(double seconds)
  {
    const double end = t + seconds;
    while (t < end - 1e-9) {
      const double dt = 0.005;
      const Truth m = motion(t);
      const double measured = wheel_v ? wheel_v(t) : m.v;
      est.addSample({t, measured, m.w, m.w + bias, (m.v - last_v) / dt});
      last_v = m.v;
      if (t >= next_visual - 1e-9) {
        if (visual(t)) {est.addVisual({t, visual_offset.compose(truth), 150, 0.5});}
        next_visual += 1.0 / 30;
      }
      // Keyframes lag the inputs like the node does.
      const double lagged = t - 0.15;
      if (auto k = est.nextKeyframe()) {
        while (*k <= lagged) {est.step(*k); k = est.nextKeyframe();}
      } else if (lagged > 0) {
        est.step(lagged);
      }
      ever_bridging |= est.state() == State::Bridging;
      if (const auto out = est.predict()) {
        if (last_output) {
          const double d = std::hypot(out->pose.x() - last_output->x(), out->pose.y() - last_output->y());
          max_output_step = std::max(max_output_step, d);
        }
        last_output = out->pose;
      }
      truth = truth.compose(Pose2(m.v * dt, 0, m.w * dt));
      t += dt;
    }
  }

  double error() const
  {
    const auto out = est.predict();
    assert(out);
    // Prediction runs to the latest sample, i.e. the pose before the last truth step.
    return std::hypot(out->pose.x() - truth.x(), out->pose.y() - truth.y());
  }
};

void check(bool ok, const char * what)
{
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
  std::fflush(stdout);
  if (!ok) {std::abort();}
}
}  // namespace

int main()
{
  {
    // Vision present: bias is estimated and the output follows the truth.
    Sim s;
    s.motion = [](double t) {return t < 2 ? Truth{} : Truth{0.3, t < 8 ? 0.0 : 0.4};};
    s.run(12);
    check(s.est.state() == State::Good, "good with vision");
    check(std::abs(s.est.bias() - 0.01) < 0.003, "gyro bias estimated from vision and ZUPT");
    check(s.error() < 0.03, "tracks the truth with vision");
    check(!s.est.slipping(), "no slip on consistent motion");
  }
  {
    // A pose arriving too late for a keyframe or two (robot: up to ~150 ms)
    // is not loss of vision; the next factor spans the gap.
    Sim s;
    s.motion = [](double t) {return t < 2 ? Truth{} : Truth{0.2, 0.1};};
    s.visual = [](double t) {return !(t > 5.0 && t < 5.2) && !(t > 7.0 && t < 7.15);};
    s.run(10);
    check(!s.ever_bridging && s.est.state() == State::Good, "short visual gaps do not drop to bridging");
    check(s.error() < 0.03, "spanning factors keep tracking the truth");
  }
  {
    // Vision drops for 3 s while driving slowly: bridging, continuous, recovers.
    Sim s;
    s.motion = [](double t) {return t < 2 ? Truth{} : Truth{0.15, 0.1};};
    s.visual = [](double t) {return t < 6 || t > 9;};
    s.run(8);
    check(s.est.state() == State::Bridging, "bridging without vision");
    check(s.est.sigmaXy() > 0 && s.est.sigmaXy() < 0.25, "bridging uncertainty grows within limits");
    check(s.error() < 0.05, "wheel/gyro bridge stays close to the truth");
    s.run(3);
    check(s.est.state() == State::Good && s.est.epoch() == 0, "vision returns without a new epoch");
    check(s.max_output_step < 0.01, "output continuous across the gap");
  }
  {
    // Long gap while moving exceeds the bridge: LOST, then a new epoch.
    Sim s;
    s.motion = [](double t) {return t < 2 ? Truth{} : Truth{0.3, 0.3};};
    s.visual = [](double t) {return t < 4 || t > 40;};
    s.run(30);
    check(s.est.state() == State::Lost, "lost after the uncertainty limit");
    s.run(12);
    check(s.est.state() == State::Good && s.est.epoch() == 1, "re-anchored with a new epoch");
  }
  {
    // Wheels spin while the robot is held: slip detected, pose follows vision.
    Sim s;
    s.motion = [](double) {return Truth{};};
    s.wheel_v = [](double t) {return t > 4 && t < 7 ? 0.4 : 0.0;};
    s.run(6);
    check(s.est.slipping(), "slip detected against vision and IMU");
    check(s.error() < 0.05, "slipping wheels do not move the pose");
    s.run(3);
    check(!s.est.slipping(), "slip clears with hysteresis");
  }
  {
    // cuVSLAM jumps 1 m: a new visual segment, no jump in the odometry.
    Sim s;
    s.motion = [](double t) {return t < 2 ? Truth{} : Truth{0.2, 0.0};};
    s.run(5);
    s.visual_offset = Pose2(1.0, -0.5, 0.3);
    s.run(4);
    check(s.est.visualJumps() >= 1, "visual jump detected");
    check(s.error() < 0.05 && s.max_output_step < 0.01, "jump kept out of the odometry");
    check(s.est.state() == State::Good, "tracking continues in the new segment");
  }
  {
    // Stationary without vision: ZUPT holds the pose and keeps the bridge open.
    Sim s;
    s.visual = [](double t) {return t < 3;};
    s.run(20);
    check(s.est.state() == State::Bridging, "stationary blind robot stays bridging");
    check(s.error() < 0.005, "zero-velocity updates hold the pose");
  }
  {
    check(visualScore(Config{}, 10, 0.5) == 1.0, "too few observations is unusable");
    check(visualScore(Config{}, 150, 0.5) < 0.1, "many spread observations score well");
    check(visualScore(Config{}, 150, 0.05) > visualScore(Config{}, 150, 0.5), "clustered features score worse");
    const std::vector<Sample> line{{0, 1, 0, 0, 0}, {0.5, 1, 0, 0, 0}};
    const auto d = integrate(line, 0, 1, 0);
    check(std::abs(d.x() - 1) < 1e-9 && std::abs(d.y()) < 1e-9, "integration holds samples");
  }
  return 0;
}
