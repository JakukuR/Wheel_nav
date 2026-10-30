#pragma once
// Planar sliding-window odometry: wheel speed + gyro preintegration, cuVSLAM
// relative poses, gyro-bias random walk and zero-velocity constraints in one
// GTSAM fixed-lag smoother. Vision drops out without a mode switch; wheel slip
// and visual quality only change factor weights (with hysteresis).
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose2.h>

namespace gtsam {class IncrementalFixedLagSmoother;}

namespace wla_r680_navigation::fusion {

// One chassis sample in the base frame (IMU rate, latest wheel speed held).
struct Sample {double t{}, v{}, wheel_w{}, gyro{}, accel_x{};};
// cuVSLAM pose projected to the floor plane, in cuVSLAM's own odometry frame.
struct Visual {double t{}; gtsam::Pose2 pose; int observations{-1}; double coverage{-1};};

struct Config {
  double keyframe_s{0.1}, window_s{3.0};
  // Wheel factor translation sigma per interval: min + per_m * travelled.
  double wheel_sigma_min{0.002}, wheel_sigma_per_m{0.05};
  double gyro_noise{0.005};      // rad/s/sqrt(Hz)
  double bias_walk{0.0005};      // rad/s/sqrt(s)
  double bias_prior{0.02};       // rad/s
  double vo_sigma_xy{0.01}, vo_sigma_yaw{0.005};
  double zupt_sigma{0.001};
  double stationary_v{0.005}, stationary_w{0.01};
  // Visual quality: score 0 (good) .. 1 (unusable), hysteresis on/off.
  int min_observations{20}, ref_observations{120};
  double ref_coverage{0.35}, visual_off{0.7}, visual_on{0.4};
  int visual_on_frames{3}, visual_off_frames{5};
  double visual_max_gap_s{0.12};
  double max_sample_gap_s{0.25};
  double vo_jump_m{0.15}, vo_jump_rad{0.15};
  // Slip: wheel yaw rate vs gyro, wheel accel vs IMU accel, wheel vs VO motion.
  double slip_w{0.2}, slip_a{1.5}, slip_vo_m{0.04};
  double slip_on{0.8}, slip_off{0.4};
  int slip_on_frames{2}, slip_off_frames{5};
  double slip_inflation{10.0};   // continuous: sigma *= 1 + slip_inflation * score
  // Bridging (no vision): correlated wheel-scale and gyro-bias errors grow
  // linearly with distance and moving time; LOST past the limits.
  double wheel_scale_sigma{0.03}, bias_sigma{0.003};
  double max_sigma_xy{0.25}, max_sigma_yaw{0.25};
};

enum class State {Waiting, Good, Bridging, Lost};
const char * name(State s);

struct Output {
  double t{};
  gtsam::Pose2 pose;
  double v{}, w{}, sigma_xy{}, sigma_yaw{};
};

// Pose increment over [t0, t1] from zero-order-held samples with gyro bias b.
gtsam::Pose2 integrate(const std::vector<Sample> & samples, double t0, double t1, double b);
// Visual quality score in [0, 1]; negative inputs mean "not reported".
double visualScore(const Config & c, int observations, double coverage);

class Estimator {
public:
  explicit Estimator(Config config);
  ~Estimator();
  void addSample(const Sample & s);
  void addVisual(const Visual & v);
  // Inputs stopped (wheel or IMU stale): reported LOST until samples resume.
  // Whether continuity survived is decided from the data: a gap longer than
  // max_sample_gap_s restarts the window and needs vision (new epoch).
  void markStale(const std::string & why);
  // Next keyframe time, once the estimator is anchored (else nullopt).
  std::optional<double> nextKeyframe() const;
  // Anchor on the first good visual pose, or add a keyframe at t.
  void step(double t);
  std::optional<Output> predict() const;

  State state() const {return stalled_ && state_ != State::Waiting ? State::Lost : state_;}
  uint32_t epoch() const {return epoch_;}
  double bias() const {return bias_;}
  double slipScore() const {return slip_score_;}
  bool slipping() const {return slipping_;}
  double visualQuality() const {return visual_score_;}
  bool visualActive() const {return visual_on_;}
  double sigmaXy() const {return sigma_xy_;}
  double sigmaYaw() const {return sigma_yaw_;}
  uint64_t visualFactors() const {return visual_factors_;}
  uint64_t visualJumps() const {return visual_jumps_;}
  const std::string & reason() const {return stalled_ ? stall_reason_ : reason_;}

private:
  std::optional<Visual> visualAt(double t) const;
  std::vector<Sample> samplesBetween(double t0, double t1) const;
  void anchor(double t, const Visual & v);
  void bridge(double travelled, double trans_sigma, double dt, bool stationary);

  Config c_;
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  std::deque<Sample> samples_;
  std::deque<Visual> visuals_;
  State state_{State::Waiting};
  uint32_t epoch_{0};
  uint64_t key_{0}, visual_factors_{0}, visual_jumps_{0};
  double t_{0}, bias_{0}, accel_mean_{0}, fused_v_{0};
  bool accel_ready_{false};
  gtsam::Pose2 pose_;
  std::optional<Visual> last_visual_;   // latest usable visual pose in this segment
  uint64_t last_visual_key_{0};         // its keyframe
  gtsam::Pose2 span_;                   // wheel/gyro motion since that keyframe
  double visual_score_{1}, slip_score_{0}, sigma_xy_{0}, sigma_yaw_{0};
  double blind_distance_{0}, blind_moving_s_{0}, blind_random_xy2_{0}, blind_random_yaw2_{0};
  bool visual_on_{false}, slipping_{false}, stalled_{false};
  std::string stall_reason_;
  int visual_good_{0}, visual_miss_{0}, slip_high_{0}, slip_low_{0};
  std::string reason_{"waiting for a good visual pose"};
  // predict() cache: increment from keyframe time cache_base_ to cache_end_.
  mutable double cache_base_{-1}, cache_end_{0};
  mutable gtsam::Pose2 cache_delta_;
  mutable bool cache_slip_{false};
};

}  // namespace wla_r680_navigation::fusion
