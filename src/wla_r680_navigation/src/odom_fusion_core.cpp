#include "wla_r680_navigation/odom_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "wla_r680_navigation/degraded_odometry.hpp"

namespace wla_r680_navigation::fusion {
namespace {
using gtsam::Pose2;
using gtsam::Vector3;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::X;
namespace noise = gtsam::noiseModel;

// Relative pose X_i^-1 X_j against the wheel/gyro increment integrated with
// bias b_i. Turn rate comes from the gyro, so wheel slip only affects the
// translation part, whose sigma the scheduler inflates.
class WheelGyroFactor final : public gtsam::NoiseModelFactor3<Pose2, Pose2, double> {
public:
  WheelGyroFactor(gtsam::Key i, gtsam::Key j, gtsam::Key b, std::vector<Sample> samples,
    double t0, double t1, const gtsam::SharedNoiseModel & model)
  : NoiseModelFactor3(model, i, j, b), samples_(std::move(samples)), t0_(t0), t1_(t1) {}

  gtsam::Vector evaluateError(const Pose2 & a, const Pose2 & c, const double & bias,
    boost::optional<gtsam::Matrix &> H1 = boost::none, boost::optional<gtsam::Matrix &> H2 = boost::none,
    boost::optional<gtsam::Matrix &> H3 = boost::none) const override
  {
    // Analytic pose Jacobians; only the bias derivative of the integrated
    // increment is numerical (one extra integration per linearization).
    const Pose2 predicted = integrate(samples_, t0_, t1_, bias);
    gtsam::Matrix3 Ha, Hc, Hp, Hr, Hl;
    const Pose2 relative = a.between(c, Ha, Hc);
    const Vector3 error = Pose2::Logmap(predicted.between(relative, Hp, Hr), Hl);
    if (H1) {*H1 = Hl * Hr * Ha;}
    if (H2) {*H2 = Hl * Hr * Hc;}
    if (H3) {
      constexpr double eps = 1e-5;
      const Vector3 dpred = Pose2::Logmap(predicted.between(integrate(samples_, t0_, t1_, bias + eps))) / eps;
      *H3 = Hl * Hp * dpred;
    }
    return error;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::make_shared<WheelGyroFactor>(*this);
  }

private:
  std::vector<Sample> samples_;
  double t0_, t1_;
};

noise::Diagonal::shared_ptr sigmas(double xy, double yaw)
{
  return noise::Diagonal::Sigmas((gtsam::Vector(3) << xy, xy, yaw).finished());
}
}  // namespace

const char * name(State s)
{
  switch (s) {
    case State::Waiting: return "WAITING";
    case State::Good: return "GOOD";
    case State::Bridging: return "BRIDGING";
    case State::Lost: return "LOST";
  }
  return "UNKNOWN";
}

Pose2 integrate(const std::vector<Sample> & samples, double t0, double t1, double b)
{
  PlanarPose p;
  if (samples.empty() || t1 <= t0) {return Pose2();}
  // Zero-order hold: each sample holds until the next one (the first and last
  // also hold before/after themselves; the caller bounds those gaps).
  size_t i = 0;
  while (i + 1 < samples.size() && samples[i + 1].t <= t0) {++i;}
  double t = t0;
  while (t1 - t > 1e-9) {
    const double end = i + 1 < samples.size() ? std::min(t1, samples[i + 1].t) : t1;
    while (end - t > 1e-9) {
      const double dt = std::min(0.05, end - t);
      p = integrateWheelGyro(p, samples[i].v, samples[i].gyro - b, dt);
      t += dt;
    }
    t = end;
    if (i + 1 < samples.size()) {++i;}
  }
  return Pose2(p.x, p.y, p.yaw);
}

double visualScore(const Config & c, int observations, double coverage)
{
  if (observations >= 0 && observations < c.min_observations) {return 1.0;}
  const double features = observations < 0 ? 0.0 :
    1.0 - std::min(1.0, static_cast<double>(observations) / c.ref_observations);
  const double spread = coverage < 0 ? 0.0 : 1.0 - std::min(1.0, coverage / c.ref_coverage);
  return std::clamp(0.6 * features + 0.4 * spread, 0.0, 1.0);
}

Estimator::Estimator(Config config)
: c_(config)
{
  const double values[] = {c_.keyframe_s, c_.window_s, c_.wheel_sigma_min, c_.wheel_sigma_per_m,
    c_.gyro_noise, c_.bias_walk, c_.bias_prior, c_.vo_sigma_xy, c_.vo_sigma_yaw, c_.zupt_sigma,
    c_.slip_w, c_.slip_a, c_.slip_vo_m, c_.max_sigma_xy, c_.max_sigma_yaw, c_.visual_max_gap_s,
    c_.wheel_scale_sigma, c_.bias_sigma, c_.max_sample_gap_s};
  if (c_.visual_off_frames < 1 || c_.visual_off_frames * c_.keyframe_s >= c_.window_s / 2) {
    throw std::invalid_argument("visual_off_frames must keep spanning factors inside the window");
  }
  for (const double v : values) {
    if (!std::isfinite(v) || v <= 0) {throw std::invalid_argument("invalid odom fusion parameter");}
  }
  if (c_.keyframe_s > 0.5 || c_.window_s < 4 * c_.keyframe_s || c_.visual_on >= c_.visual_off ||
    c_.slip_off >= c_.slip_on || c_.min_observations < 1 || c_.ref_observations < c_.min_observations)
  {
    throw std::invalid_argument("inconsistent odom fusion parameters");
  }
}

Estimator::~Estimator() = default;

void Estimator::addSample(const Sample & s)
{
  if (!samples_.empty() && s.t <= samples_.back().t) {return;}
  samples_.push_back(s);
  stalled_ = false;
}

void Estimator::addVisual(const Visual & v)
{
  if (!visuals_.empty() && v.t <= visuals_.back().t) {return;}
  visuals_.push_back(v);
  while (visuals_.size() > 2 && visuals_.front().t < v.t - 2.0) {visuals_.pop_front();}
}

void Estimator::markStale(const std::string & why)
{
  stalled_ = true;
  stall_reason_ = why;
}

std::optional<double> Estimator::nextKeyframe() const
{
  if (state_ == State::Waiting) {return std::nullopt;}
  return t_ + c_.keyframe_s;
}

std::optional<Visual> Estimator::visualAt(double t) const
{
  for (size_t i = 1; i < visuals_.size(); ++i) {
    const auto & a = visuals_[i - 1];
    const auto & b = visuals_[i];
    if (a.t <= t && t <= b.t) {
      if (b.t - a.t > c_.visual_max_gap_s) {return std::nullopt;}
      const double alpha = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0.0;
      Visual v = alpha < 0.5 ? a : b;
      v.t = t;
      v.pose = a.pose.compose(Pose2::Expmap(alpha * Pose2::Logmap(a.pose.between(b.pose))));
      return v;
    }
  }
  return std::nullopt;
}

std::vector<Sample> Estimator::samplesBetween(double t0, double t1) const
{
  std::vector<Sample> out;
  for (size_t i = 0; i < samples_.size(); ++i) {
    const bool holds_t0 = samples_[i].t <= t0 && (i + 1 == samples_.size() || samples_[i + 1].t > t0);
    if (holds_t0 || (samples_[i].t > t0 && samples_[i].t <= t1)) {out.push_back(samples_[i]);}
  }
  return out;
}

void Estimator::anchor(double t, const Visual & v)
{
  // iSAM2 updates only the affected part of the window each keyframe; the
  // batch smoother re-solved and re-marginalized all of it (~40% of a core).
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(c_.window_s, params);
  key_ = 0;
  t_ = t;
  pose_ = v.pose;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  graph.add(gtsam::PriorFactor<Pose2>(X(0), pose_, sigmas(1e-3, 1e-3)));
  graph.add(gtsam::PriorFactor<double>(B(0), bias_, noise::Isotropic::Sigma(1, c_.bias_prior)));
  values.insert(X(0), pose_);
  values.insert(B(0), bias_);
  smoother_->update(graph, values, {{X(0), t}, {B(0), t}});
  last_visual_ = v;
  last_visual_key_ = 0;
  span_ = Pose2();
  visual_on_ = true;
  visual_miss_ = 0;
  sigma_xy_ = sigma_yaw_ = 0;
  blind_distance_ = blind_moving_s_ = blind_random_xy2_ = blind_random_yaw2_ = 0;
  state_ = State::Good;
  reason_.clear();
}

void Estimator::bridge(double travelled, double trans_sigma, double dt, bool stationary)
{
  // Random parts add in quadrature; wheel scale and gyro bias are correlated
  // over the blind stretch, so they grow linearly. A stationary robot (ZUPT)
  // observes its bias and does not accumulate either.
  blind_distance_ += travelled;
  if (!stationary) {
    blind_moving_s_ += dt;
    blind_random_yaw2_ += c_.gyro_noise * c_.gyro_noise * dt;
  }
  blind_random_xy2_ += trans_sigma * trans_sigma;
  sigma_yaw_ = std::hypot(std::sqrt(blind_random_yaw2_), c_.bias_sigma * blind_moving_s_);
  sigma_xy_ = std::sqrt(blind_random_xy2_ + std::pow(c_.wheel_scale_sigma * blind_distance_, 2) +
    std::pow(blind_distance_ * sigma_yaw_, 2));
}

void Estimator::step(double t)
{
  if (state_ == State::Waiting) {
    const auto v = visualAt(t);
    if (v && visualScore(c_, v->observations, v->coverage) < c_.visual_on) {anchor(t, *v);}
    return;
  }
  if (t <= t_) {return;}
  const double dt = t - t_;
  const auto window = samplesBetween(t_, t);
  double gap = window.empty() ? dt : std::max(window.front().t > t_ ? window.front().t - t_ : 0.0,
    t - window.back().t);
  for (size_t i = 1; i < window.size(); ++i) {gap = std::max(gap, window[i].t - window[i - 1].t);}
  if (gap > c_.max_sample_gap_s) {
    // Chassis inputs stopped: restart the window at the current pose; the
    // odometry is no longer continuous, so vision must re-anchor (new epoch).
    Visual here{t, pose_, -1, -1};
    const State before = state_;
    anchor(t, here);
    visual_on_ = false;
    last_visual_.reset();
    visual_good_ = 0;
    state_ = State::Lost;
    reason_ = before == State::Lost ? reason_ : "chassis wheel/IMU gap";
    return;
  }

  const Pose2 delta = integrate(window, t_, t, bias_);
  const double travelled = std::hypot(delta.x(), delta.y());
  double wheel_w = 0, gyro = 0, accel = 0;
  bool stationary = true;
  int n = 0;
  for (const auto & s : window) {
    if (s.t <= t_) {continue;}
    wheel_w += s.wheel_w; gyro += s.gyro; accel += s.accel_x; ++n;
    stationary = stationary && std::abs(s.v) < c_.stationary_v && std::abs(s.wheel_w) < c_.stationary_w;
  }
  if (n == 0) {n = 1; stationary = false;}
  wheel_w /= n; gyro /= n; accel /= n;
  const double wheel_accel = (window.back().v - window.front().v) / dt;
  // Slow mean removes gravity leakage from pitch and accelerometer bias.
  const double imu_accel = accel_ready_ ? accel - accel_mean_ : 0.0;
  accel_mean_ = accel_ready_ ? accel_mean_ + std::min(1.0, dt / 2.0) * (accel - accel_mean_) : accel;
  accel_ready_ = true;

  // Visual quality with hysteresis; a re-enabled segment starts without a factor.
  // A late or missing pose for a keyframe or two (cuVSLAM output lags up to
  // ~150 ms) is not loss of vision: the next factor spans the gap instead.
  const auto v = visualAt(t);
  visual_score_ = v ? visualScore(c_, v->observations, v->coverage) : 1.0;
  const bool usable = v && visual_score_ <= c_.visual_off;
  span_ = span_.compose(delta);
  bool new_segment = false;
  if (visual_on_) {
    if (usable) {visual_miss_ = 0;}
    else if (++visual_miss_ >= c_.visual_off_frames) {
      visual_on_ = false; last_visual_.reset(); visual_good_ = 0; visual_miss_ = 0;
    }
  } else if (v && visual_score_ < c_.visual_on) {
    if (++visual_good_ >= c_.visual_on_frames) {visual_on_ = true; new_segment = true;}
  } else {
    visual_good_ = 0;
  }
  std::optional<Pose2> vo;
  if (visual_on_ && !new_segment && last_visual_ && usable) {vo = last_visual_->pose.between(v->pose);}
  const uint64_t vo_from = last_visual_key_;
  const double span_travelled = std::hypot(span_.x(), span_.y());
  const int span_frames = static_cast<int>(key_ + 1 - vo_from);
  if (vo && (std::hypot(vo->x() - span_.x(), vo->y() - span_.y()) > c_.vo_jump_m + span_travelled ||
    std::abs(Pose2::Logmap(span_.between(*vo))(2)) > c_.vo_jump_rad))
  {
    // cuVSLAM relocalized internally or jumped: start a new segment instead
    // of passing the discontinuity into odometry.
    ++visual_jumps_;
    vo.reset();
  }

  const double s_w = std::abs(wheel_w - (gyro - bias_)) / c_.slip_w;
  const double s_a = std::abs(wheel_accel - imu_accel) / c_.slip_a;
  const double s_vo = vo ? std::hypot(vo->x() - span_.x(), vo->y() - span_.y()) / (c_.slip_vo_m * span_frames) : 0.0;
  slip_score_ = std::min(1.0, std::max({s_w, s_a, s_vo}));
  if (slip_score_ > c_.slip_on) {++slip_high_; slip_low_ = 0;}
  else if (slip_score_ < c_.slip_off) {++slip_low_; slip_high_ = 0;}
  if (!slipping_ && slip_high_ >= c_.slip_on_frames) {slipping_ = true;}
  if (slipping_ && slip_low_ >= c_.slip_off_frames) {slipping_ = false;}

  double trans_sigma = (c_.wheel_sigma_min + c_.wheel_sigma_per_m * travelled) *
    (1.0 + c_.slip_inflation * slip_score_);
  if (slipping_) {trans_sigma = std::max(trans_sigma * 100.0, travelled);}
  const double yaw_sigma = c_.gyro_noise * std::sqrt(dt) + 1e-4;

  const auto xi = X(key_), xj = X(key_ + 1), bi = B(key_), bj = B(key_ + 1);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  graph.add(WheelGyroFactor(xi, xj, bi, window, t_, t, sigmas(trans_sigma, yaw_sigma)));
  graph.add(gtsam::BetweenFactor<double>(bi, bj, 0.0,
    noise::Isotropic::Sigma(1, c_.bias_walk * std::sqrt(dt))));
  if (stationary && !slipping_) {
    graph.add(gtsam::BetweenFactor<Pose2>(xi, xj, Pose2(), sigmas(c_.zupt_sigma, c_.zupt_sigma)));
  }
  if (vo) {
    const double scale = (1.0 + 2.0 * visual_score_) * std::sqrt(static_cast<double>(span_frames));
    graph.add(gtsam::BetweenFactor<Pose2>(X(vo_from), xj, *vo, noise::Robust::Create(
      noise::mEstimator::Huber::Create(1.345), sigmas(c_.vo_sigma_xy * scale, c_.vo_sigma_yaw * scale))));
  }
  values.insert(xj, pose_.compose(delta));
  values.insert(bj, bias_);
  smoother_->update(graph, values, {{xj, t}, {bj, t}});
  const auto estimate = smoother_->calculateEstimate();
  fused_v_ = pose_.between(estimate.at<Pose2>(xj)).x() / dt;
  pose_ = estimate.at<Pose2>(xj);
  bias_ = estimate.at<double>(bj);
  ++key_;
  t_ = t;
  if (visual_on_ && usable) {last_visual_ = v; last_visual_key_ = key_; span_ = Pose2();}

  if (vo) {
    ++visual_factors_;
    sigma_xy_ = sigma_yaw_ = 0;
    blind_distance_ = blind_moving_s_ = blind_random_xy2_ = blind_random_yaw2_ = 0;
    if (state_ == State::Lost) {
      // Continuity was lost in between: downstream map corrections are void.
      ++epoch_;
    }
    state_ = State::Good;
    reason_.clear();
  } else {
    const double bridge_sigma = slipping_ ? std::max(travelled, c_.wheel_sigma_min) :
      (stationary ? 0.0 : c_.wheel_sigma_min + c_.wheel_sigma_per_m * travelled);
    bridge(travelled, bridge_sigma, dt, stationary && !slipping_);
    if (state_ != State::Lost) {
      if (sigma_xy_ > c_.max_sigma_xy || sigma_yaw_ > c_.max_sigma_yaw) {
        state_ = State::Lost;
        reason_ = "bridging uncertainty exceeded";
      } else if (!visual_on_) {
        state_ = State::Bridging;
      }
    }
  }
  while (samples_.size() > 2 && samples_[1].t <= t_ - 0.5) {samples_.pop_front();}
}

std::optional<Output> Estimator::predict() const
{
  if (state_ == State::Waiting || samples_.empty()) {return std::nullopt;}
  const double t = std::max(t_, samples_.back().t);
  // Slipping wheels must not move the prediction either: use the speed the
  // smoother estimated over the last keyframe instead.
  const bool slip = slipping_ || slip_score_ > c_.slip_on;
  // Extend the cached increment since the last keyframe with new samples
  // only (piecewise zero-order hold integrates identically in pieces).
  if (cache_base_ != t_ || cache_slip_ != slip || cache_end_ > t) {
    cache_base_ = cache_end_ = t_; cache_delta_ = Pose2(); cache_slip_ = slip;
  }
  auto window = samplesBetween(cache_end_, t);
  if (slip) {
    for (auto & s : window) {s.v = fused_v_;}
  }
  cache_delta_ = cache_delta_.compose(integrate(window, cache_end_, t, bias_));
  cache_end_ = t;
  Output out;
  out.t = t;
  out.pose = pose_.compose(cache_delta_);
  out.v = slip ? fused_v_ : samples_.back().v;
  out.w = samples_.back().gyro - bias_;
  out.sigma_xy = sigma_xy_;
  out.sigma_yaw = sigma_yaw_;
  return out;
}

}  // namespace wla_r680_navigation::fusion
