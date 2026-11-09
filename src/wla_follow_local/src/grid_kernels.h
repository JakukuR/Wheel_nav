#ifndef WLA_FOLLOW_SRC_GRID_KERNELS_H
#define WLA_FOLLOW_SRC_GRID_KERNELS_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace wla::follow::detail {

// Marks the cells crossed by sampled rays (every 0.45 cell, up to 7 m, ending
// 0.12 m short of the endpoint) in a size*size bitmap over the window starting
// at `lower`; bitmap index is the window cell i*size+j. Shared by
// _wla_sim_geometry.ray_slots and RollingMap so the two cannot diverge.
inline void MarkRayCells(const double* endpoints, std::size_t count, double origin_x, double origin_y,
                         double lower_x, double lower_y, double res, int size, std::vector<unsigned char>& seen)
{
    const double step = res * .45;
    const int samples = static_cast<int>(std::ceil(7. / step));
    for (std::size_t i = 0; i < count; ++i)
    {
        const double dx = endpoints[2 * i] - origin_x, dy = endpoints[2 * i + 1] - origin_y;
        const double length = std::sqrt(dx * dx + dy * dy);
        if (!std::isfinite(length))
        {
            throw std::invalid_argument("invalid_ray_endpoint");
        }
        const double ux = dx / std::max(length, 1e-9), uy = dy / std::max(length, 1e-9);
        for (int j = 0; j < samples; ++j)
        {
            const double distance = j * step;
            if (!(distance < length - .12))
            {
                break;
            }
            const double x = std::floor((origin_x + ux * distance) / res) - lower_x;
            const double y = std::floor((origin_y + uy * distance) / res) - lower_y;
            if (x >= 0 && x < size && y >= 0 && y < size)
            {
                seen[static_cast<std::size_t>(x) * static_cast<std::size_t>(size) + static_cast<std::size_t>(y)] = 1;
            }
        }
    }
}

// Clearance query of a window field at (x, y): the field value of the cell
// minus the cell half diagonal and the offset of the point from the cell
// centre; 0 outside the window. Shared by _wla_sim_geometry.grid_query (the
// Python map's query) and the C++ planner.
inline double GridQuery(const double* field, int n, double lower_x, double lower_y, double res, double x, double y)
{
    const double area = res / std::sqrt(2.);
    const double cx = std::floor(x / res), cy = std::floor(y / res);
    const double gx = cx - lower_x, gy = cy - lower_y;
    if (!(gx >= 0 && gx < n && gy >= 0 && gy < n))
    {
        return 0.;
    }
    const double dx = x - (cx + .5) * res, dy = y - (cy + .5) * res;
    return field[static_cast<int>(gx) * n + static_cast<int>(gy)] - area - std::sqrt(dx * dx + dy * dy);
}

inline bool GridLineFree(const unsigned char* ok, int n, int a, int b)
{
    const int ax = a / n, ay = a % n, bx = b / n, by = b % n, steps = std::max(std::abs(bx - ax), std::abs(by - ay)) * 2;
    for (int k = 0; k <= steps; ++k)
    {
        const double u = steps ? static_cast<double>(k) / steps : 0.;
        const int x = static_cast<int>(std::lround(ax + (bx - ax) * u)), y = static_cast<int>(std::lround(ay + (by - ay) * u));
        if (!ok[x * n + y])
        {
            return false;
        }
    }
    return true;
}

// Douglas-Peucker on cells; a chord through a blocked cell is always split.
inline void GridSimplify(const std::vector<int>& cells, int first, int last, int n, double tolerance,
                         const unsigned char* ok, std::vector<int>& kept)
{
    if (last - first < 2)
    {
        return;
    }
    const double ax = cells[first] / n, ay = cells[first] % n, dx = cells[last] / n - ax, dy = cells[last] % n - ay,
                 length = std::hypot(dx, dy);
    double worst = -1.;
    int split = first + 1;
    for (int i = first + 1; i < last; ++i)
    {
        const double px = cells[i] / n - ax, py = cells[i] % n - ay;
        const double distance = length > 0 ? std::abs(dx * py - dy * px) / length : std::hypot(px, py);
        if (distance > worst)
        {
            worst = distance;
            split = i;
        }
    }
    if (worst <= tolerance && GridLineFree(ok, n, cells[first], cells[last]))
    {
        return;
    }
    GridSimplify(cells, first, split, n, tolerance, ok, kept);
    kept.push_back(cells[split]);
    GridSimplify(cells, split, last, n, tolerance, ok, kept);
}

// 8-connected A* from cell `from` to cell `to` over usable cells of an n x n
// mask, no diagonal corner cutting; the route simplified with `tolerance`
// cells, or empty when unreachable. Shared by _wla_sim_geometry.grid_astar
// and the C++ detour planner.
inline std::vector<int> GridAstarCells(const unsigned char* ok, int n, int from, int to, double tolerance)
{
    std::vector<int> route;
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> cost(static_cast<std::size_t>(n) * n, inf);
    std::vector<int> parent(static_cast<std::size_t>(n) * n, -1);
    std::vector<unsigned char> closed(static_cast<std::size_t>(n) * n, 0);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    const int gx = to / n, gy = to % n;
    const auto estimate = [&](int x, int y) {
        return std::hypot(static_cast<double>(x - gx), static_cast<double>(y - gy));
    };
    cost[from] = 0.;
    open.emplace(estimate(from / n, from % n), from);
    static const int moves[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
    while (!open.empty())
    {
        const int index = open.top().second;
        open.pop();
        if (closed[index])
        {
            continue;
        }
        closed[index] = 1;
        if (index == to)
        {
            for (int at = to; at != -1; at = parent[at])
            {
                route.push_back(at);
            }
            std::reverse(route.begin(), route.end());
            break;
        }
        const int x = index / n, y = index % n;
        for (int m = 0; m < 8; ++m)
        {
            const int nx = x + moves[m][0], ny = y + moves[m][1];
            if (nx < 0 || nx >= n || ny < 0 || ny >= n)
            {
                continue;
            }
            const int next = nx * n + ny;
            if (!ok[next] || closed[next])
            {
                continue;
            }
            if (m >= 4 && (!ok[nx * n + y] || !ok[x * n + ny]))
            {
                continue;
            }
            const double candidate = cost[index] + (m < 4 ? 1. : M_SQRT2);
            if (candidate < cost[next])
            {
                cost[next] = candidate;
                parent[next] = index;
                open.emplace(candidate + estimate(nx, ny), next);
            }
        }
    }
    if (route.size() >= 2)
    {
        std::vector<int> kept{route.front()};
        GridSimplify(route, 0, static_cast<int>(route.size()) - 1, n, tolerance, ok, kept);
        kept.push_back(route.back());
        route.swap(kept);
    }
    return route;
}

} // namespace wla::follow::detail

#endif // WLA_FOLLOW_SRC_GRID_KERNELS_H
