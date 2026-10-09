#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wla_r680_navigation {
inline double wrapYaw(double a) {return std::atan2(std::sin(a), std::cos(a));}
struct PlanarPose {double x{}, y{}, yaw{};};
inline PlanarPose integrateWheelGyro(PlanarPose p, double v, double w, double dt) {
  if (!std::isfinite(v+w+dt) || dt<=0 || dt>0.1)
    throw std::invalid_argument("invalid dead reckoning interval/input");
  const double turn=w*dt, half=turn*0.5;
  const double sinc=std::abs(half)<1e-8 ? 1.0 : std::sin(half)/half;
  p.x+=v*dt*sinc*std::cos(p.yaw+half);
  p.y+=v*dt*sinc*std::sin(p.yaw+half);
  p.yaw=wrapYaw(p.yaw+turn);
  return p;
}
struct DegradedBudget {
  double elapsed{}, travel{}, turn{};
  bool advance(double v,double w,double dt,double seconds,double distance,double angle) {
    elapsed+=dt; travel+=std::abs(v)*dt; turn+=std::abs(w)*dt;
    return std::isfinite(elapsed+travel+turn) && elapsed<seconds && travel<distance && turn<angle;
  }
  double positionSigma() const {return 0.02+0.02*elapsed+0.05*travel;}
  double yawSigma() const {return 0.02+0.02*elapsed+0.05*turn;}
};
// Time-aligned incoming visual poses must remain near the independent wheel/gyro prediction.
inline bool consistentVisual(PlanarPose visual,PlanarPose predicted,double xy,double yaw) {
  return std::isfinite(visual.x+visual.y+visual.yaw) &&
    std::hypot(visual.x-predicted.x,visual.y-predicted.y)<=xy &&
    std::abs(wrapYaw(visual.yaw-predicted.yaw))<=yaw;
}
inline double degradedCommandScale(double v,double w,double max_v,double max_w) {
  if(!std::isfinite(v+w)) return 0;
  return std::min({1.0,max_v/std::max(std::abs(v),1e-12),max_w/std::max(std::abs(w),1e-12)});
}
}  // namespace wla_r680_navigation
