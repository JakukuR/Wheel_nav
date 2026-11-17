#include "wla_uvc_depth/ground_map.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace wla::uvc;
void require(bool v, const char *m) {
  if (!v)
    throw std::runtime_error(m);
}
std::vector<GroundPoint> scene(double tilt = 0) {
  std::vector<GroundPoint> p;
  for (int y = -50; y <= 50; ++y)
    for (int x = 20; x < 180; ++x)
      p.push_back({{x * .01, y * .01, -.15 + std::tan(tilt) * x * .01}, true});
  return p;
}
int cell(const GroundMap &g, double x, double y) {
  return int(std::floor((y - g.y0) / g.resolution)) * g.width +
         int(std::floor((x - g.x0) / g.resolution));
}
int main() {
  try {
    const cv::Vec3d n(0, 0, 1);
    auto p = scene();
    auto g = MakeGroundMap(p, n, .15);
    require(g.available, "flat plane accepted");
    require(g.obstacles.empty() && g.uncertain.empty(),
            "flat plane not obstacles");
    require(g.occupancy[cell(g, .8, 0)] == 0, "observed ground cell");
    require(g.occupancy[cell(g, 2.5, 0)] == -1, "unobserved is unknown");
    for (int y = 0; y < 10; ++y)
      for (int x = 0; x < 10; ++x)
        p.push_back({{.8 + x * .005, .1 + y * .005, -.11}, true});
    g = MakeGroundMap(p, n, .15);
    require(g.available && g.obstacles.size() >= 100, "4 cm step retained");
    require(g.occupancy[cell(g, .82, .12)] == 100,
            "ground never erases a step in same column");
    p.push_back({{1.21, -.21, -.13}, false});
    p.push_back({{1.31, -.21, -.21}, false});
    g = MakeGroundMap(p, n, .15);
    require(g.occupancy[cell(g, 1.21, -.21)] == 100,
            "2 cm uncertain band retained");
    require(g.occupancy[cell(g, 1.31, -.21)] == 100, "below plane not cleared");
    auto competing = scene();
    for (int repeat = 0; repeat < 2; ++repeat)
      for (int y = 40; y < 120; ++y)
        for (int x = 20; x < 180; ++x)
          competing.push_back(
              {{x * .01, y * .01, -.15 + std::tan(10 * CV_PI / 180) * y * .01},
               true});
    g = MakeGroundMap(competing, n, .15);
    require(g.available && g.angleDeg <= 5,
            "competing slanted plane cannot hide supported floor");
    g = MakeGroundMap(scene(8 * CV_PI / 180), n, .15);
    require(!g.available,
            "slope inconsistent with flat support rejects filtering");
    for (auto x : g.occupancy)
      require(x == -1, "bad plane gives unknown map");
    g = MakeGroundMap({}, n, .15);
    require(!g.available, "missing depth cannot reuse history");
    p = scene();
    for (auto &q : p)
      q.body[2] -= .08;
    g = MakeGroundMap(p, n, .15);
    require(!g.available, "wrong height rejected");
    std::cout << "ground, low steps, below-plane, unknown and mismatch "
                 "contracts passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
