#pragma once
// LOST recovery after initialization (z-localization style): wait for vision,
// then sweep in place to find features (+a, -a, back to 0; never translate,
// the camera cannot see behind), then restart the visual frontend once and
// wait for it. Odometry continuity is the fusion backend's business; this only
// decides motion. Any return of usable odometry ends the attempt.
#include <cmath>
#include <stdexcept>

namespace wla_r680_navigation {

struct RecoveryConfig {
  double wait_s{3.0};          // stopped, waiting for vision to come back by itself
  double turn_angle{1.05};     // +-60 deg sweep
  double angular{0.3};         // rad/s
  double leg_timeout_s{8.0};
  double restart_wait_s{25.0}; // frontend restart incl. static inertial init
  double cooldown_s{30.0};
  int max_attempts{3};
  void validate() const {
    for (double v : {wait_s, turn_angle, angular, leg_timeout_s, restart_wait_s, cooldown_s}) {
      if (!std::isfinite(v) || v <= 0) {throw std::invalid_argument("invalid recovery parameter");}
    }
    if (turn_angle > 1.6 || angular > 0.5 || max_attempts < 1) {
      throw std::invalid_argument("recovery envelope exceeded");
    }
  }
};

struct RecoveryInput {
  double now{};
  bool lost{false};        // odometry LOST (fresh)
  bool stopped{false};     // wheels stopped
  bool clear{false};       // near field clear, chassis/depth/safety cloud live
  bool authorized{false};  // bounded base motion authorized, safety chain active
  double heading{0};       // wheel/gyro yaw, unwrapped by the caller
};

struct RecoveryOutput {double angular{0}; bool restart{false};};

class RecoveryPolicy {
public:
  enum class State {Idle, Waiting, SweepOut, SweepBack, Return, Restart, Cooldown, Exhausted};

  explicit RecoveryPolicy(RecoveryConfig c) : c_(c) {c_.validate();}

  RecoveryOutput tick(const RecoveryInput & in)
  {
    RecoveryOutput out;
    if (!in.lost) {
      // Odometry usable again (vision back or bridging): stand down.
      if (state_ != State::Idle) {reason_ = "odometry recovered";}
      state_ = State::Idle; attempts_ = 0;
      return out;
    }
    switch (state_) {
      case State::Idle:
        enter(State::Waiting, in.now, "odometry lost; waiting for vision");
        break;
      case State::Waiting:
        if (in.now - since_ >= c_.wait_s && in.stopped && in.clear && in.authorized) {
          origin_ = in.heading;
          enter(State::SweepOut, in.now, "sweeping for features");
        }
        break;
      case State::SweepOut:
      case State::SweepBack:
      case State::Return: {
        if (!in.clear || !in.authorized) {abort(in.now, !in.clear ? "near field blocked" : "authorization lost"); break;}
        const double target = state_ == State::SweepOut ? c_.turn_angle :
          state_ == State::SweepBack ? -c_.turn_angle : 0.0;
        const double error = target - (in.heading - origin_);
        if (std::abs(error) <= 0.05 || in.now - since_ > c_.leg_timeout_s) {
          if (state_ == State::SweepOut) {enter(State::SweepBack, in.now, "sweeping back");}
          else if (state_ == State::SweepBack) {enter(State::Return, in.now, "returning to heading");}
          else {
            enter(State::Restart, in.now, "sweep found no usable vision; restarting frontend");
            out.restart = true;
          }
        } else {
          out.angular = std::copysign(c_.angular, error);
        }
        break;
      }
      case State::Restart:
        if (in.now - since_ >= c_.restart_wait_s) {abort(in.now, "frontend restart did not recover");}
        break;
      case State::Cooldown:
        if (in.now - since_ >= c_.cooldown_s) {
          if (attempts_ >= c_.max_attempts) {state_ = State::Exhausted; reason_ = "attempts exhausted; operator needed";}
          else {enter(State::Waiting, in.now, "retrying");}
        }
        break;
      case State::Exhausted:
        break;
    }
    return out;
  }

  State state() const {return state_;}
  bool moving() const {return state_ == State::SweepOut || state_ == State::SweepBack || state_ == State::Return;}
  int attempts() const {return attempts_;}
  const char * reason() const {return reason_;}
  const char * name() const
  {
    switch (state_) {
      case State::Idle: return "idle";
      case State::Waiting: return "waiting_for_vision";
      case State::SweepOut: case State::SweepBack: case State::Return: return "sweeping";
      case State::Restart: return "restarting_frontend";
      case State::Cooldown: return "cooldown";
      case State::Exhausted: return "exhausted";
    }
    return "unknown";
  }

private:
  void enter(State s, double now, const char * why) {state_ = s; since_ = now; reason_ = why;}
  void abort(double now, const char * why) {++attempts_; enter(State::Cooldown, now, why);}

  RecoveryConfig c_;
  State state_{State::Idle};
  double since_{0}, origin_{0};
  int attempts_{0};
  const char * reason_{""};
};

}  // namespace wla_r680_navigation
