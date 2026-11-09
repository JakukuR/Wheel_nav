#ifndef WLA_FOLLOW_SRC_MAP_VIEW_H
#define WLA_FOLLOW_SRC_MAP_VIEW_H

#include <cstdint>
#include <stdexcept>

#include <wla_follow/rolling_map.h>

#include "grid_kernels.h"

namespace wla::follow::detail {

// Read-only access of the planners to a built RollingMap, with the query of
// the Python map (RollingDepthMap.query through _wla_sim_geometry.grid_query).
struct MapView
{
    explicit MapView(const RollingMap& map)
    {
        if (!map.hasWindow() || !map.hasFields())
        {
            throw std::logic_error("rolling map fields are not built yet");
        }
        m_n = map.size();
        m_res = map.resolution();
        const auto lower = map.lower();
        m_lower = {lower[0], lower[1]};
        m_lowerX = static_cast<double>(lower[0]);
        m_lowerY = static_cast<double>(lower[1]);
        m_dist = map.distData();
        m_freeDist = map.freeDistData();
        m_known = map.knownData();
    }

    // local_map.query(xy, unknown=...)
    [[nodiscard]] double query(double x, double y, bool unknown = false) const noexcept
    {
        return GridQuery(unknown ? m_freeDist : m_dist, m_n, m_lowerX, m_lowerY, m_res, x, y);
    }

    // (local_map.cells[i, j] + .5) * local_map.res
    [[nodiscard]] double centerX(int i) const noexcept { return (static_cast<double>(i + m_lower[0]) + .5) * m_res; }
    [[nodiscard]] double centerY(int j) const noexcept { return (static_cast<double>(j + m_lower[1]) + .5) * m_res; }

    int m_n{0};
    double m_res{0.};
    std::array<std::int64_t, 2> m_lower{};
    double m_lowerX{0.};
    double m_lowerY{0.};
    const double* m_dist{nullptr};
    const double* m_freeDist{nullptr};
    const std::uint8_t* m_known{nullptr};
};

} // namespace wla::follow::detail

#endif // WLA_FOLLOW_SRC_MAP_VIEW_H
