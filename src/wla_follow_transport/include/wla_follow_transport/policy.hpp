#pragma once
#include <nlohmann/json.hpp>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>

namespace wla_follow {
using Json = nlohmann::json;
using Vec3 = std::array<double, 3>;
inline Vec3 advance(Vec3 p, Vec3 v, double dt) {
  const double mid = p[2] + v[2] * dt * .5;
  return {p[0] + (v[0]*std::cos(mid)-v[1]*std::sin(mid))*dt,
          p[1] + (v[0]*std::sin(mid)+v[1]*std::cos(mid))*dt, p[2]+v[2]*dt};
}
struct MotionSample { double stamp; Vec3 pose, twist; };
struct Motion {
  std::deque<MotionSample> samples;
  uint64_t generation = 0;
  void reset() { samples.clear(); ++generation; }
  bool update(double stamp, Vec3 v, double now) {
    if (!std::isfinite(stamp) || !std::isfinite(now) ||
        !std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2]) ||
        now-stamp < 0 || now-stamp > .20 || std::abs(v[0])>1.5 ||
        std::abs(v[1])>.3 || std::abs(v[2])>2.) { reset(); return false; }
    Vec3 pose{};
    if (!samples.empty()) {
      const auto &old = samples.back(); const double dt=stamp-old.stamp;
      if (dt<=0) return false;
      if (dt>.20) reset();
      else { Vec3 mean; for (size_t i=0;i<3;++i) mean[i]=(v[i]+old.twist[i])*.5;
        pose=advance(old.pose,mean,dt); }
    }
    samples.push_back({stamp,pose,v}); if(samples.size()>128) samples.pop_front();
    return true;
  }
  bool healthy(double now) const {
    return !samples.empty() && now>=samples.back().stamp && now-samples.back().stamp<=.20;
  }
  bool pose_at(double stamp, Vec3 &pose) const {
    if (samples.empty() || stamp<samples.front().stamp || stamp-samples.back().stamp>.05) return false;
    for(auto it=samples.rbegin();it!=samples.rend();++it)
      if(it->stamp<=stamp) { pose=advance(it->pose,it->twist,stamp-it->stamp); return true; }
    return false;
  }
  Json snapshot() const {
    Json rows=Json::array();
    for(const auto &s:samples) rows.push_back({s.stamp,s.pose,s.twist});
    return rows;
  }
};

// Clock values are supplied by the owner, so gates can be tested without ROS or sleeps.
struct OutputGate {
  // generation: any input/reference change. cancel_generation: only task-ending
  // events (stop, estop, revoked authority, expiry); in-flight start/heartbeat
  // replies are rejected by this one, not by a wheel reference reset.
  uint64_t generation=0, sequence=0, motion_generation=0, cancel_generation=0;
  bool enabled=false, estop=false, active=false, authority=false, observations=false;
  std::string request, detector_session;
  int64_t detector_epoch=-1, observation_stamp=0;
  double lease_until=0, authority_until=0, wheel_received=0, last_worker=0;
  // max_age bounds detector observations; map_max_age bounds the depth frame
  // behind a result. Results replanned at control rate on the newest map may
  // use an older frame than observations may be (never older than map_max_age).
  double max_age=.30, map_max_age=.30, result_ttl=.15;
  // How long an accepted result may drive the base without a newer one (>= result_ttl,
  // <= map_max_age). 2026-10-29 17:47 J02: 70-100 ms plans against a 0.2 s hold
  // zeroed 10-20 % of commands while following and 53 % while detouring.
  double result_hold=.15;
  std::deque<int64_t> delivered_frames;
  Json result;
  void invalidate() { ++generation; result=Json(); }
  void revoke() { active=false; lease_until=0; ++cancel_generation; invalidate(); }
  void delivered(int64_t ns) {
    delivered_frames.push_back(ns); if(delivered_frames.size()>16) delivered_frames.pop_front();
  }
  bool accept(const Json &r, double mono, int64_t ros_ns) {
    try {
      const auto seq=r.at("sequence").get<uint64_t>();
      if(seq<=sequence) return false;
      sequence=seq;
      if(r.at("generation").get<uint64_t>()!=generation ||
         r.at("motion_generation").get<uint64_t>()!=motion_generation ||
         r.at("request_id").get<std::string>()!=request ||
         r.at("detector_session").get<std::string>()!=detector_session ||
         r.at("detector_epoch").get<int64_t>()!=detector_epoch) return false;
      const double generated=r.at("generated_at").get<double>();
      const double until=r.at("valid_until").get<double>();
      const auto frame=r.at("frame_ns").get<int64_t>();
      const auto obs=r.at("observation_ns").get<int64_t>();
      const double v=r.at("linear").get<double>(), w=r.at("angular").get<double>();
      bool known=false; for(auto ns:delivered_frames) known|=ns==frame;
      if(!known || !std::isfinite(generated) || !std::isfinite(until) ||
         generated>mono || mono-generated>result_ttl || until<=mono || until>generated+result_hold+1e-6 ||
         frame<=0 || ros_ns<frame || (ros_ns-frame)*1e-9>map_max_age ||
         obs<=0 || obs>observation_stamp || ros_ns<obs || (ros_ns-obs)*1e-9>max_age ||
         !std::isfinite(v) || !std::isfinite(w) || std::abs(v)>1.5 || std::abs(w)>2. ||
         !r.at("permitted").is_boolean() || !r.at("permitted").get<bool>()) return false;
      result=r; last_worker=mono; return true;
    } catch(const Json::exception &) { return false; }
  }
  std::string blocker(double mono,int64_t ros_ns,bool wheel_healthy) const {
    if(!enabled) return "motion_disabled";
    if(estop) return "local_estop";
    if(!active || mono>=lease_until) return "task_inactive_or_expired";
    if(!authority || mono>=authority_until) return "motion_permission_unavailable";
    if(!wheel_healthy || mono-wheel_received>.20) return "wheel_odom_stale";
    if(!observations || ros_ns<observation_stamp || (ros_ns-observation_stamp)*1e-9>max_age) return "target_observations_stale";
    if(result.is_null()) return "no_valid_transport_result";
    if(result.at("generation").get<uint64_t>()!=generation ||
       result.at("motion_generation").get<uint64_t>()!=motion_generation) return "transport_reference_changed";
    if(mono>=result.at("valid_until").get<double>() || mono-last_worker>result_hold ||
       ros_ns<result.at("frame_ns").get<int64_t>() ||
       (ros_ns-result.at("frame_ns").get<int64_t>())*1e-9>map_max_age ||
       ros_ns<result.at("observation_ns").get<int64_t>() ||
       (ros_ns-result.at("observation_ns").get<int64_t>())*1e-9>max_age) return "transport_result_expired";
    return "";
  }
};
}  // namespace wla_follow
