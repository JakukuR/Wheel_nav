#pragma once
#include <cmath>
#include <cstdint>
#include "wla_r680_navigation/relocalization_prior.hpp"

namespace wla_r680_navigation {
// A single verified RTAB match establishes recovery evidence for this search.
// The caller checks its generation, freshness and pose/TF agreement first.
class RecoveryEvidence {
public:
  void reset() {last_stamp_=0;}
  bool accept(int64_t stamp, PlanarPose pose) {
    if (stamp<=last_stamp_ || !std::isfinite(pose.x+pose.y+pose.yaw)) return false;
    last_stamp_=stamp;return true;
  }
  bool verified() const {return last_stamp_>0;}
private:
  int64_t last_stamp_{0};
};
}
