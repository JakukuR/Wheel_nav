#include <cassert>
#include <limits>
#include "initialization_sampling.hpp"
int main() {
  wla_vio::InitializationSampling disabled(0), sampling(0.25);
  assert(!disabled.request(1000000000, false));
  assert(sampling.request(1000000000, false));
  assert(!sampling.request(1000000000, false));
  assert(!sampling.request(1100000000, false));
  assert(sampling.request(1250000000, false));
  assert(!sampling.request(1300000000, true));
  assert(!sampling.request(2000000000, false)); // Never resume special cadence after initialization.
  for (double value : {-0.1, 0.1, 1.1, std::numeric_limits<double>::infinity()}) {
    bool rejected = false;
    try {wla_vio::InitializationSampling invalid(value);} catch (const std::invalid_argument &) {rejected = true;}
    assert(rejected);
  }
}
