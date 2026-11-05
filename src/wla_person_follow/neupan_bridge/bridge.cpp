// Runs unmodified neupan_cpp in its own process, the way its neupan_node calls it.
// in:  x y yaw reset npath nobs \n path xy... \n obstacle xy... \n
// out: RESULT solved stop arrive min_distance ms v w
#include <chrono>
#include <iostream>
#include <vector>

#include "neupan/neupan_planner.hpp"

using namespace neupan;

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: neupan_bridge <planner.yaml> <dune.bin>" << std::endl;
    return 2;
  }
  auto planner = NeuPANPlanner::fromYaml(argv[1], argv[2]);
  std::cout << "READY" << std::endl;
  double x, y, yaw;
  int reset, npath, nobs;
  while (std::cin >> x >> y >> yaw >> reset >> npath >> nobs) {
    std::vector<InitialPath::PathPoint> path(npath);
    for (auto& q : path) {
      double a, b;
      std::cin >> a >> b;
      q << a, b, 0.0, 1.0;
    }
    Mat2X obstacles(2, nobs);
    for (int i = 0; i < nobs; ++i) std::cin >> obstacles(0, i) >> obstacles(1, i);
    const Vec3 state(x, y, yaw);
    if (reset) {
      planner.setInitialPath(path);
      planner.reset();
    } else {
      planner.replaceInitialPath(path, state);
    }
    NeuPANPlanner::Info info;
    const auto begun = std::chrono::steady_clock::now();
    const Vec2 u = planner.forward(state, obstacles, info);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begun).count();
    std::cout << "RESULT " << info.solved << ' ' << info.stop << ' ' << info.arrive << ' ' << info.min_distance << ' '
              << ms << ' ' << u(0) << ' ' << u(1) << std::endl;
  }
}
