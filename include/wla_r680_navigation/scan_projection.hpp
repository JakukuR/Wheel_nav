#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace wla_r680_navigation
{

struct ScanProjectionConfig
{
  double angle_min;
  double angle_max;
  double angle_increment;
  double range_min;
  double range_max;
  double min_height;
  double max_height;
};

inline std::size_t scan_ray_count(const ScanProjectionConfig & config)
{
  return static_cast<std::size_t>(
    std::ceil((config.angle_max - config.angle_min) / config.angle_increment)) + 1;
}

template<typename Point>
bool add_scan_point(
  const Point & point, const ScanProjectionConfig & config, std::vector<float> & ranges)
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
    point.z < config.min_height || point.z > config.max_height)
  {
    return false;
  }
  const double range = std::hypot(point.x, point.y);
  if (range < config.range_min || range > config.range_max) {
    return false;
  }
  const double angle = std::atan2(point.y, point.x);
  if (angle < config.angle_min || angle > config.angle_max) {
    return false;
  }
  const auto index = static_cast<std::size_t>(
    std::llround((angle - config.angle_min) / config.angle_increment));
  if (index >= ranges.size()) {
    return false;
  }
  ranges[index] = std::min(ranges[index], static_cast<float>(range));
  return true;
}

}  // namespace wla_r680_navigation
