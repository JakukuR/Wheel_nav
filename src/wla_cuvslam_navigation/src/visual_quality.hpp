#pragma once
#include <algorithm>
#include <vector>

namespace wla_vio {
// Fraction of a cells x cells grid over the left image that holds at least one
// tracked observation: a few clustered features score low even if numerous.
template<class Observations>
double gridCoverage(const Observations & observations, int width, int height, int cells = 8)
{
  if (width <= 0 || height <= 0 || cells <= 0) {return 0.0;}
  std::vector<char> hit(static_cast<size_t>(cells * cells), 0);
  for (const auto & o : observations) {
    if (o.camera_index != 0 || !(o.u >= 0) || !(o.v >= 0) || o.u >= width || o.v >= height) {continue;}
    const int x = std::min(cells - 1, static_cast<int>(o.u * cells / width));
    const int y = std::min(cells - 1, static_cast<int>(o.v * cells / height));
    hit[static_cast<size_t>(y * cells + x)] = 1;
  }
  return static_cast<double>(std::count(hit.begin(), hit.end(), 1)) / (cells * cells);
}
}  // namespace wla_vio
