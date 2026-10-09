#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "wla_r680_navigation/scan_projection.hpp"

struct Point {double x; double y; double z;};

int main()
{
  using wla_r680_navigation::add_scan_point;
  const wla_r680_navigation::ScanProjectionConfig config{
    -M_PI_2, M_PI_2, M_PI / 180.0, 0.15, 4.5, -0.08, 0.50};
  std::vector<float> ranges(
    wla_r680_navigation::scan_ray_count(config),
    std::numeric_limits<float>::infinity());
  if (!add_scan_point(Point{2.0, 0.0, 0.2}, config, ranges) ||
    !add_scan_point(Point{1.0, 0.0, 0.3}, config, ranges) ||
    add_scan_point(Point{0.8, 0.0, -0.1}, config, ranges) ||
    add_scan_point(Point{-1.0, 0.0, 0.2}, config, ranges) ||
    add_scan_point(Point{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.2},
      config, ranges))
  {
    std::cerr << "point acceptance failed\n";
    return 1;
  }
  const auto middle = static_cast<std::size_t>(M_PI_2 / config.angle_increment);
  if (std::abs(ranges[middle] - 1.0F) > 0.001F ||
    !std::isinf(ranges[middle + 10]))
  {
    std::cerr << "nearest point or unknown angle was projected incorrectly\n";
    return 1;
  }
  return 0;
}
