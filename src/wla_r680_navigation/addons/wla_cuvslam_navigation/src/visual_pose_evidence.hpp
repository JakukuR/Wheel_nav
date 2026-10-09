#pragma once
#include <cstdint>

namespace wla_vio {
// Feature observations do not certify a pose solution: the SDK may still be propagating IMU only.
inline bool visualPoseObserved(bool inertial_only, bool warming_up, int64_t state_stamp,
                               int64_t pose_stamp, int64_t observations, int minimum_observations) {
  return !inertial_only && !warming_up && state_stamp == pose_stamp &&
    observations >= minimum_observations;
}
}
