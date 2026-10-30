#include <cassert>
#include <cmath>
#include <vector>
#include "visual_quality.hpp"

struct Obs {float u, v; int camera_index;};

int main()
{
  std::vector<Obs> clustered(200, Obs{10, 10, 0});
  assert(std::abs(wla_vio::gridCoverage(clustered, 640, 480) - 1.0 / 64) < 1e-12);
  std::vector<Obs> spread;
  for (int y = 0; y < 8; ++y) {for (int x = 0; x < 8; ++x) {spread.push_back({x * 80.f + 5, y * 60.f + 5, 0});}}
  assert(wla_vio::gridCoverage(spread, 640, 480) == 1.0);
  spread.push_back({700, 10, 0});   // outside the image
  spread.push_back({5, 5, 1});      // right camera ignored
  assert(wla_vio::gridCoverage(std::vector<Obs>{{5, 5, 1}, {-1, 3, 0}}, 640, 480) == 0.0);
  assert(wla_vio::gridCoverage(spread, 0, 480) == 0.0);
  return 0;
}
