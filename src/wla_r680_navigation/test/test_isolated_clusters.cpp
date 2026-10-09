#include <iostream>
#include <vector>

#include "wla_r680_navigation/isolated_clusters.hpp"

struct Point {double x; double y; double z;};

int main()
{
  // A noisy cell can contain many depth samples and still cover only one pixel.
  const std::vector<Point> points{
    {0.01, 0.01, 0.10}, {0.02, 0.02, 0.11}, {0.03, 0.01, 0.12},
    {0.31, 0.01, 0.20}, {0.36, 0.01, 0.20},
    {1.01, 0.01, 0.20}, {1.06, 0.01, 0.20}, {1.11, 0.01, 0.20},
    {-1.01, -1.01, 0.20}, {-1.06, -1.01, 0.20}, {-1.11, -1.01, 0.20},
  };
  const auto filtered = wla_r680_navigation::remove_isolated_clusters(points, 0.05, 3);
  if (filtered.size() != 6 || filtered.front().x != 1.01 || filtered.back().x != -1.11) {
    std::cerr << "isolated cells were retained or connected obstacles were removed\n";
    return 1;
  }
  if (wla_r680_navigation::remove_isolated_clusters(points, 0.05, 1).size() != points.size()) {
    std::cerr << "disabled filtering did not preserve input\n";
    return 1;
  }
  return 0;
}
