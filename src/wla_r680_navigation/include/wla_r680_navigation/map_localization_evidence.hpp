#pragma once
#include <cmath>
#include <cstdint>

namespace wla_r680_navigation {
// A match is only a candidate. Call pose() only after covariance/frame checks
// and agreement with TF at the pose's source timestamp. Times are ROS seconds.
class MapLocalizationEvidence {
public:
  void reset(double epoch, bool retain_initial_match = false) {
    epoch_ = epoch; match_ = pose_ = 0.; confirmed_ = false;
    retain_initial_match_ = retain_initial_match;
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
    // cuVSLAM can finish inertial initialization after the backend's first
    // map match. During initial admission only, retain a match that was fresh
    // when received; a fresh verified pose, TF and odometry are still required.
    const bool matched = retain_initial_match_ ? match_ > epoch_ :
      fresh(match_, now, stale) && std::abs(match_ - pose_) <= 0.05;
    if (!confirmed_ && visual_tracking && odometry_ok && tf_fresh &&
        matched && fresh(pose_, now, stale)) {
      confirmed_ = true; retain_initial_match_ = false;
    }
    return confirmed_;
  }
  bool retainsInitialMatch() const {return retain_initial_match_;}
  bool confirmed() const {return confirmed_;}
  double epoch() const {return epoch_;}
private:
  static bool fresh(double stamp, double now, double stale) {
    return stamp > 0. && std::isfinite(stamp) && now-stamp >= -0.2 && now-stamp <= stale;
  }
  double epoch_{0.}, match_{0.}, pose_{0.};
  bool confirmed_{false}, retain_initial_match_{false};
};
}  // namespace wla_r680_navigation
