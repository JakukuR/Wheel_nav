#pragma once
#include <algorithm>
#include <cmath>
#include <string>

namespace wla_r680_navigation {
struct InitConfig {
  double startup_timeout{45}, init_timeout{20}, stationary_time{0.8}, stop_time{0.5};
  double motion_duration{8}, target_travel{0.16}, max_travel{0.25}, max_radius{0.25};
  double max_yaw{0.35}, linear{0.06}, angular{0.20}, max_measured_linear{0.12}, max_measured_angular{0.35};
};
struct InitInput {
  bool sensors{false}, services{false}, clear{false}, stopped{false};
  bool inertial{false}, localized{false};
  double radius{0}, travel{0}, yaw{0}, speed{0}, angular_speed{0};
};
struct InitCommand {double linear{0}, angular{0};};
class InitPolicy {
public:
  enum class State {Waiting, Stationary, Moving, Stopping, Localization, Succeeded, Failed};
  explicit InitPolicy(InitConfig config = {}) : config_(config) {}
  State state() const {return state_;}
  const std::string & reason() const {return reason_;}
  const char * name() const {
    switch (state_) {
      case State::Waiting: return "waiting_inputs";
      case State::Stationary: return "stationary";
      case State::Moving: return "moving";
      case State::Stopping: return "stopping";
      case State::Localization: return "waiting_localization";
      case State::Succeeded: return "succeeded";
      default: return "failed";
    }
  }
  void fail(const std::string & reason) {
    if (state_ == State::Failed) return;
    state_ = State::Failed; reason_ = reason;
  }
  InitCommand tick(double now, const InitInput & in) {
    if (!std::isfinite(now) || (last_time_ >= 0 && now < last_time_)) fail("invalid_clock");
    last_time_ = now;
    if (started_ < 0) started_ = now;
    if (state_ == State::Failed || state_ == State::Succeeded) return {};
    if (activation_ < 0 && now - started_ >= config_.startup_timeout) fail("startup_inputs_timeout");
    if (activation_ >= 0 && now - activation_ >= config_.init_timeout) {
      fail(in.inertial ? "mapping_or_localization_timeout" : "inertial_initialization_timeout");
    }
    if (activation_ >= 0) {
      if (!in.sensors || !in.services) fail("input_or_safety_lifecycle_lost");
      if (!std::isfinite(in.radius + in.travel + in.yaw + in.speed + in.angular_speed)) fail("nonfinite_motion");
      if (in.radius >= config_.max_radius || in.travel >= config_.max_travel ||
        std::abs(in.yaw) >= config_.max_yaw) fail("motion_envelope_exceeded");
      if (std::abs(in.speed) > config_.max_measured_linear ||
        std::abs(in.angular_speed) > config_.max_measured_angular) fail("chassis_overspeed");
    }
    switch (state_) {
      case State::Waiting:
        if (in.sensors && in.services) {
          if (in.inertial) {activation_ = now; state_ = State::Stopping; stable_ = -1;}
          else if (in.clear) {state_ = State::Stationary; stable_ = -1;}
        }
        break;
      case State::Stationary:
        if (!in.sensors || !in.services || !in.clear) {state_ = State::Waiting; stable_ = -1; break;}
        if (!in.stopped) {stable_ = -1; break;}
        if (stable_ < 0) stable_ = now;
        if (now - stable_ >= config_.stationary_time) {
          activation_ = now; motion_start_ = now; stable_ = -1;
          state_ = in.inertial ? State::Stopping : State::Moving;
        }
        break;
      case State::Moving:
        if (!in.clear) {fail("near_field_blocked"); break;}
        if (in.inertial || now - motion_start_ >= config_.motion_duration ||
          in.travel >= config_.target_travel) {state_ = State::Stopping; stable_ = -1; break;}
        break;
      case State::Stopping:
        if (!in.stopped) {stable_ = -1; break;}
        if (stable_ < 0) stable_ = now;
        if (now - stable_ >= config_.stop_time) state_ = State::Localization;
        break;
      case State::Localization:
        if (!in.stopped) {fail("unexpected_motion_after_stop"); break;}
        if (in.inertial && in.localized) state_ = State::Succeeded;
        break;
      default: break;
    }
    if (state_ != State::Moving) return {};
    const double t = now - motion_start_;
    // Gentle forward excitation with changes in velocity and yaw. No reverse.
    return {config_.linear * (0.75 + 0.25 * std::sin(M_PI * t)),
      config_.angular * std::sin(M_PI * t / 2.0)};
  }
private:
  InitConfig config_;
  State state_{State::Waiting};
  double started_{-1}, activation_{-1}, motion_start_{-1}, stable_{-1}, last_time_{-1};
  std::string reason_;
};

// The final gate never forwards smoother residuals beyond the current bootstrap
// request, and zero request stops immediately even if smoothing retains velocity.
inline double bound_init_component(double checked, double requested, double cap) {
  if (!std::isfinite(checked) || !std::isfinite(requested) ||
    requested == 0 || checked * requested <= 0) return 0;
  return std::copysign(std::min({std::abs(checked), std::abs(requested), cap}), requested);
}
}
