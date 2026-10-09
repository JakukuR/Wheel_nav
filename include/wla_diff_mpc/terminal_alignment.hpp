#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace wla_diff_mpc {

struct TerminalSettings {
  double enter_distance{0.20}, exit_distance{0.28}, max_w{0.22};
  double stopped_v{0.025}, stopped_w{0.04}, stopped_duration{0.35};
  double angular_deceleration{0.4}, reaction_time{0.7}, yaw_gain{1.0};
};

// Pure state machine: no ROS, QP or actuator access. A zero command is a request
// to stop; only fresh measured velocity can confirm that stopping succeeded.
class TerminalAlignment {
public:
  enum class Phase {Track, Stopping, Align, Brake, Hold};
  explicit TerminalAlignment(TerminalSettings settings = {}) : settings_(settings) {
    const auto & s=settings_;
    if (!std::isfinite(s.enter_distance+s.exit_distance+s.max_w+s.stopped_v+
        s.stopped_w+s.stopped_duration+s.angular_deceleration+s.reaction_time+s.yaw_gain) ||
        s.enter_distance<=0 || s.exit_distance<=s.enter_distance || s.max_w<=0 ||
        s.stopped_v<=0 || s.stopped_w<=0 || s.stopped_duration<=0 ||
        s.angular_deceleration<=0 || s.reaction_time<0 || s.yaw_gain<=0)
      throw std::invalid_argument("invalid terminal alignment settings");
  }
  void reset() {phase_=Phase::Track; stopped_since_=nan();direction_=0;alignment_started_=false;}
  Phase phase() const {return phase_;}
  static const char * name(Phase p) {
    switch(p) {
      case Phase::Track:return "track"; case Phase::Stopping:return "stopping";
      case Phase::Align:return "align"; case Phase::Brake:return "brake";
      case Phase::Hold:return "hold";
    }
    return "invalid";
  }
  // Returns true when this state machine owns the command. Linear speed is
  // always exactly zero in every terminal phase; yaw_rate is the only output.
  bool update(double distance, double error, double measured_v, double measured_w,
              bool feedback_valid, double now, double xy_tolerance,
              double yaw_tolerance, double & yaw_rate) {
    yaw_rate=0;
    error=std::remainder(error,2*3.14159265358979323846);
    const double enter=std::min(settings_.enter_distance,xy_tolerance);
    const double exit=settings_.exit_distance;
    if (!std::isfinite(distance+error+now+xy_tolerance+yaw_tolerance) ||
        distance<0 || xy_tolerance<=0 || yaw_tolerance<=0) {
      phase_=Phase::Stopping; stopped_since_=nan(); return true;
    }
    if (phase_==Phase::Track) {
      if (distance>enter) return false;
      phase_=Phase::Stopping; stopped_since_=nan();
    }
    if (!feedback_valid || !std::isfinite(measured_v+measured_w)) {
      if (phase_==Phase::Align) phase_=Phase::Stopping;
      stopped_since_=nan(); return true;
    }
    const bool stopped=std::abs(measured_v)<=settings_.stopped_v &&
      std::abs(measured_w)<=settings_.stopped_w;
    if (!stopped) stopped_since_=nan();
    else if (!std::isfinite(stopped_since_)) stopped_since_=now;
    const bool settled=stopped && now-stopped_since_>=settings_.stopped_duration;
    if (distance>exit && phase_!=Phase::Stopping) {
      phase_=Phase::Stopping; stopped_since_=nan(); return true;
    }
    if (phase_==Phase::Stopping) {
      if (!settled) return true;
      // Before alignment starts, braking must finish inside the actual XY
      // tolerance. Once aligning, retain the existing exit-distance hysteresis
      // through intermediate stops instead of switching back to path tracking
      // for small pose/map corrections across the entry tolerance.
      if (distance>(alignment_started_ ? exit : xy_tolerance)) {reset();return true;}
      direction_=0;
      alignment_started_=true;
      phase_=std::abs(error)<=yaw_tolerance ? Phase::Hold : Phase::Align;
      return true; // Keep zero on the transition tick, after confirmed stopping.
    }
    if (phase_==Phase::Hold) {
      // Never chase pose noise while coasting. Reacquire only after settling
      // outside the actual goal tolerance; inside tolerance stay latched at zero.
      if (settled && distance>xy_tolerance) {reset();return true;}
      if (settled && std::abs(error)>yaw_tolerance) phase_=Phase::Align;
      return true;
    }
    if (phase_==Phase::Brake) {
      if (!settled) return true;
      direction_=0;
      phase_=std::abs(error)<=yaw_tolerance ? Phase::Hold : Phase::Align;
      return true;
    }
    const double available=std::max(0.0,std::abs(error)-0.75*yaw_tolerance);
    if (std::abs(error)<=yaw_tolerance) {phase_=Phase::Hold;return true;}
    if (std::abs(measured_v)>settings_.stopped_v) {
      phase_=Phase::Stopping;stopped_since_=nan();return true;
    }
    // No reverse command while the body is still rotating the other way.
    if ((measured_w*error<0 && std::abs(measured_w)>settings_.stopped_w) || direction_*error<0) {
      phase_=Phase::Brake;stopped_since_=nan();return true;
    }
    const double speed=std::abs(measured_w);
    const double stopping_angle=speed*settings_.reaction_time+
      speed*speed/(2*settings_.angular_deceleration);
    if (speed>settings_.stopped_w && stopping_angle>=available) {
      phase_=Phase::Brake;stopped_since_=nan();return true;
    }
    const double a=settings_.angular_deceleration, t=settings_.reaction_time;
    const double cap=std::max(0.0,std::sqrt(a*a*t*t+2*a*available)-a*t);
    yaw_rate=std::copysign(std::min({settings_.max_w,settings_.yaw_gain*available,cap}),error);
    if (yaw_rate!=0) direction_=error>0 ? 1 : -1;
    return true;
  }
private:
  static double nan() {return std::numeric_limits<double>::quiet_NaN();}
  TerminalSettings settings_;
  Phase phase_{Phase::Track};
  int direction_{0};
  bool alignment_started_{false};
  double stopped_since_{nan()};
};
} // namespace wla_diff_mpc
