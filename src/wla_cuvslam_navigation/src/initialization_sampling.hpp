#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace wla_vio {
// Request genuine image keyframes at a bounded cadence during initialization.
// All other frames retain SDK automatic selection. No inertial threshold changes.
class InitializationSampling {
public:
  explicit InitializationSampling(double period) : period_ns_(0) {
    if (!std::isfinite(period) || (period != 0 && (period < 0.2 || period > 1.0))) {
      throw std::invalid_argument("initialization keyframe period must be 0 or 0.2..1.0s");
    }
    period_ns_ = static_cast<int64_t>(period * 1e9);
  }
  bool request(int64_t stamp, bool gravity_available) {
    if (gravity_available) finished_ = true;
    if (finished_ || period_ns_ == 0 || stamp <= last_request_) return false;
    if (last_request_ < 0 || stamp - last_request_ >= period_ns_) {
      last_request_ = stamp; return true;
    }
    return false;
  }
private:
  int64_t period_ns_, last_request_{-1};
  bool finished_{false};
};
}
