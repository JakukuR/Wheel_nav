#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace wla_r680_navigation
{
namespace detail
{
inline uint64_t xy_key(int32_t x, int32_t y)
{
  return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32U) |
         static_cast<uint32_t>(y);
}
}  // namespace detail

// Keep only obstacle components occupying at least min_cells connected XY cells.
// This counts occupied map cells, not points: several noisy depth returns at one
// location must not turn a 1-2-pixel artifact into an accepted obstacle.
template<typename Point>
std::vector<Point> remove_isolated_clusters(
  const std::vector<Point> & points, double cell_size, size_t min_cells)
{
  if (points.empty() || min_cells <= 1 || !std::isfinite(cell_size) || cell_size <= 0.0) {
    return points;
  }
  struct Cell {int32_t x; int32_t y;};
  std::vector<Cell> cells;
  cells.reserve(points.size());
  std::unordered_map<uint64_t, size_t> lookup;
  lookup.reserve(points.size());
  for (const auto & point : points) {
    const auto x = static_cast<int32_t>(std::floor(point.x / cell_size));
    const auto y = static_cast<int32_t>(std::floor(point.y / cell_size));
    const auto key = detail::xy_key(x, y);
    if (lookup.emplace(key, cells.size()).second) {cells.push_back({x, y});}
  }

  std::vector<uint8_t> keep(cells.size(), 0U);
  std::vector<uint8_t> visited(cells.size(), 0U);
  std::vector<size_t> component;
  component.reserve(cells.size());
  for (size_t start = 0; start < cells.size(); ++start) {
    if (visited[start]) {continue;}
    component.clear();
    component.push_back(start);
    visited[start] = 1U;
    for (size_t head = 0; head < component.size(); ++head) {
      const auto cell = cells[component[head]];
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {continue;}
          const auto neighbor = lookup.find(detail::xy_key(cell.x + dx, cell.y + dy));
          if (neighbor == lookup.end() || visited[neighbor->second]) {continue;}
          visited[neighbor->second] = 1U;
          component.push_back(neighbor->second);
        }
      }
    }
    if (component.size() >= min_cells) {
      for (const auto index : component) {keep[index] = 1U;}
    }
  }

  std::vector<Point> result;
  result.reserve(points.size());
  for (const auto & point : points) {
    const auto x = static_cast<int32_t>(std::floor(point.x / cell_size));
    const auto y = static_cast<int32_t>(std::floor(point.y / cell_size));
    if (keep[lookup.at(detail::xy_key(x, y))]) {result.push_back(point);}
  }
  return result;
}
}  // namespace wla_r680_navigation
