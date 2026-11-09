#ifndef WLA_FOLLOW_SRC_GRID_KERNELS_H
#define WLA_FOLLOW_SRC_GRID_KERNELS_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
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

} // namespace wla::follow::detail

#endif // WLA_FOLLOW_SRC_GRID_KERNELS_H
