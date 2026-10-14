#pragma once
#include <cmath>
#include <cstdint>

namespace wla_openvins {
// The patched OpenVINS sequence advances only after a real MSCKF EKFUpdate.
// Feature count is diagnostic; it is not a second acceptance test on that update.
inline bool record_visual_correction(uint64_t before, uint64_t after, unsigned accepted,
                                     double stamp, double timeout, double &last,
                                     unsigned &recovery) {
  if (after <= before || accepted == 0 || !std::isfinite(stamp) || stamp <= last)
    return false;
  recovery = last >= 0 && stamp - last <= timeout ? recovery + 1 : 1;
  last = stamp;
  return true;
}
}  // namespace wla_openvins
