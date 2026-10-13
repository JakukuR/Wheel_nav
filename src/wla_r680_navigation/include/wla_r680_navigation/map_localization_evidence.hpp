#pragma once
#include <cmath>
#include <cstdint>

namespace wla_r680_navigation {
// A match is only a candidate. Call pose() only after covariance/frame checks
// and agreement with TF at the pose's source timestamp. Times are ROS seconds.
class MapLocalizationEvidence {
public:
  void reset(double epoch) {
    epoch_ = epoch; match_ = pose_ = 0.; confirmed_ = false;
  }
  void match(double stamp, double now, double stale) {
    if (fresh(stamp, now, stale) && stamp > epoch_ && stamp > match_) match_ = stamp;
  }
  void pose(double stamp, double now, double stale) {
    if (fresh(stamp, now, stale) && stamp > epoch_ && stamp > pose_) pose_ = stamp;
  }
  bool update(double now, double stale, bool tf_fresh, bool odometry_ok,
              bool visual_tracking, double degraded_grace = 0.) {
    // Invalidate an established localization on loss. Starting a new epoch
    // rejects late/pre-loss packets; subsequent pose-only input cannot reopen.
    if (confirmed_ && (!tf_fresh || !odometry_ok || !fresh(pose_, now, stale + degraded_grace))) {
      reset(now); return false;
    }
    if (!confirmed_ && visual_tracking && odometry_ok && tf_fresh &&
        fresh(match_, now, stale) && fresh(pose_, now, stale) &&
        std::abs(match_ - pose_) <= 0.05) confirmed_ = true;
    return confirmed_;
  }
  bool confirmed() const {return confirmed_;}
  double epoch() const {return epoch_;}
private:
  static bool fresh(double stamp, double now, double stale) {
    return stamp > 0. && std::isfinite(stamp) && now-stamp >= -0.2 && now-stamp <= stale;
  }
  double epoch_{0.}, match_{0.}, pose_{0.};
  bool confirmed_{false};
};
}  // namespace wla_r680_navigation
