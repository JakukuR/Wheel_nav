#pragma once
#include <cmath>
#include <cstdint>
#include <optional>
#include "wla_r680_navigation/relocalization_prior.hpp"

namespace wla_r680_navigation {
// Evidence is tied to new RTAB frames, not repeated watchdog timer ticks.
class RecoveryEvidence {
public:
  void reset() {last_stamp_=0; count_=0; previous_.reset();}
  bool add(int64_t stamp, PlanarPose pose, double max_translation, double max_yaw) {
    if (stamp<=last_stamp_ || !std::isfinite(pose.x+pose.y+pose.yaw)) return false;
    if (previous_ && (std::hypot(pose.x-previous_->x,pose.y-previous_->y)>max_translation ||
        std::abs(wrapYaw(pose.yaw-previous_->yaw))>max_yaw || stamp-last_stamp_>3000000000LL))
      count_=0;
    previous_=pose;last_stamp_=stamp;++count_;return true;
  }
  int count() const {return count_;}
private:
  int64_t last_stamp_{0};int count_{0};std::optional<PlanarPose> previous_;
};
}
