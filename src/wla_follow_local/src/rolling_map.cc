// C++ owner of the rolling depth map. Every method mirrors the Python
// reference sim_core/rolling_depth_map.py statement by statement; comments
// there explain the policy. Element-wise equality with the reference is
// checked by test/test_native_rolling_map.py and the replay comparison.
#include <wla_follow/rolling_map.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "grid_kernels.h"

namespace wla::follow {

namespace {

constexpr double INF = std::numeric_limits<double>::infinity();

[[nodiscard]] std::int64_t Mod(std::int64_t value, std::int64_t n) noexcept
{
    const std::int64_t r = value % n;
    return r < 0 ? r + n : r;
}

// numpy: floor(v / res).astype(int64); out-of-range values are rejected.
[[nodiscard]] std::int64_t CellOf(double value, double res)
{
    const double cell = std::floor(value / res);
    if (!std::isfinite(cell) || std::abs(cell) > 4e18)
    {
        throw std::invalid_argument("nonfinite_map_point");
    }
    return static_cast<std::int64_t>(cell);
}

[[nodiscard]] double Norm(double dx, double dy) noexcept
{
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

struct RollingMap::Impl
{
    double m_res;
    std::int64_t m_n;
    double m_ttl;
    double m_robotRadius;
    double m_targetClearTtl;
    double m_freeRadius{0.};
    bool m_hasCenter{false};
    std::array<double, 2> m_center{0., 0.};

    std::vector<double> m_log, m_stamp, m_clearStamp, m_since;
    std::vector<std::int64_t> m_tags;
    std::array<std::int64_t, 2> m_lower{0, 0};
    bool m_hasWindow{false};
    std::int64_t m_version{0};
    std::uint64_t m_windowId{0}, m_buildId{0};

    bool m_hasHeldSlots{false};
    std::vector<std::uint8_t> m_heldSlots;
    std::vector<std::array<std::int64_t, 2>> m_heldCells;
    double m_heldStamp{0.};

    bool m_hasBuilt{false};
    std::int64_t m_builtVersion{0};
    double m_builtRadius{0.}, m_builtTargetClearTtl{0.}, m_builtTime{0.};
    std::array<double, 2> m_builtCenter{0., 0.};

    bool m_hasFields{false};
    std::vector<std::uint8_t> m_sensorKnown, m_occupied, m_known, m_assumedFree;
    std::vector<double> m_dist, m_freeDist;

    explicit Impl(const Options& o)
        : m_res(o.m_resolution), m_n(o.m_size), m_ttl(o.m_ttl), m_robotRadius(o.m_robotRadius),
          m_targetClearTtl(o.m_targetClearTtl)
    {
        if (!std::isfinite(m_res) || m_res <= 0. || m_n < 2 || m_n > 4096 || !std::isfinite(m_ttl) ||
            !std::isfinite(m_robotRadius) || !std::isfinite(m_targetClearTtl))
        {
            throw std::invalid_argument("invalid_rolling_map_options");
        }
        const auto cells = static_cast<std::size_t>(m_n * m_n);
        m_log.assign(cells, 0.);
        m_stamp.assign(cells, -INF);
        m_clearStamp.assign(cells, -INF);
        m_since.assign(cells, -INF);
        m_tags.assign(2 * cells, -1000000000);
    }

    [[nodiscard]] std::size_t slot(std::int64_t cx, std::int64_t cy) const noexcept
    {
        return static_cast<std::size_t>(Mod(cx, m_n) * m_n + Mod(cy, m_n));
    }

    void requireWindow() const
    {
        if (!m_hasWindow)
        {
            throw std::logic_error("rolling_map_window_not_initialised");
        }
    }

    void retireStartup(std::size_t s) noexcept
    {
        if (m_hasHeldSlots)
        {
            m_heldSlots[s] = 0;
        }
    }

    void slots(const double* xy, std::size_t count, std::size_t stride, std::vector<std::size_t>& out) const
    {
        requireWindow();
        std::vector<std::uint8_t> seen(static_cast<std::size_t>(m_n * m_n), 0);
        for (std::size_t i = 0; i < count; ++i)
        {
            const std::int64_t cx = CellOf(xy[i * stride], m_res), cy = CellOf(xy[i * stride + 1], m_res);
            if (cx >= m_lower[0] && cx < m_lower[0] + m_n && cy >= m_lower[1] && cy < m_lower[1] + m_n)
            {
                seen[static_cast<std::size_t>((cx - m_lower[0]) * m_n + (cy - m_lower[1]))] = 1;
            }
        }
        out.clear();
        for (std::size_t u = 0; u < seen.size(); ++u)
        {
            if (seen[u])
            {
                const auto i = static_cast<std::int64_t>(u) / m_n, j = static_cast<std::int64_t>(u) % m_n;
                out.push_back(slot(i + m_lower[0], j + m_lower[1]));
            }
        }
    }

    void slide(double x, double y)
    {
        const std::array<std::int64_t, 2> lower{CellOf(x, m_res) - m_n / 2, CellOf(y, m_res) - m_n / 2};
        if (m_hasWindow && lower == m_lower)
        {
            return;
        }
        m_lower = lower;
        ++m_version;
        m_hasWindow = true;
        ++m_windowId;
        for (std::int64_t i = 0; i < m_n; ++i)
        {
            for (std::int64_t j = 0; j < m_n; ++j)
            {
                const std::int64_t cx = i + m_lower[0], cy = j + m_lower[1];
                const std::size_t s = slot(cx, cy);
                if (m_tags[2 * s] != cx || m_tags[2 * s + 1] != cy)
                {
                    m_log[s] = 0.;
                    m_stamp[s] = -INF;
                    m_clearStamp[s] = -INF;
                    m_since[s] = -INF;
                    m_tags[2 * s] = cx;
                    m_tags[2 * s + 1] = cy;
                }
            }
        }
    }

    [[nodiscard]] double center(std::int64_t cell) const noexcept
    {
        return (static_cast<double>(cell) + .5) * m_res;
    }

    [[nodiscard]] bool expiryBetween(double t0, double t1) const noexcept
    {
        for (const double s : m_stamp)
        {
            if (s >= t0 - m_ttl && s < t1 - m_ttl)
            {
                return true;
            }
        }
        for (const double s : m_clearStamp)
        {
            if (s >= t0 - m_targetClearTtl && s < t1 - m_targetClearTtl)
            {
                return true;
            }
        }
        return false;
    }

    // EDT of a padded window mask, scaled by the resolution (scipy layout).
    void distanceField(const std::vector<std::uint8_t>& mask, std::vector<double>& out) const
    {
        const int n = static_cast<int>(m_n), padded = n + 2;
        std::vector<std::uint8_t> foreground(static_cast<std::size_t>(padded * padded), 0);
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                foreground[static_cast<std::size_t>((i + 1) * padded + j + 1)] = mask[static_cast<std::size_t>(i * n + j)];
            }
        }
        std::vector<double> squared;
        SquaredDistanceTransform(foreground, padded, padded, squared);
        out.resize(static_cast<std::size_t>(n * n));
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                out[static_cast<std::size_t>(i * n + j)] = std::sqrt(squared[static_cast<std::size_t>((i + 1) * padded + j + 1)]) * m_res;
            }
        }
    }
};

RollingMap::RollingMap(const Options& options)
    : m_impl(std::make_unique<Impl>(options))
{}

RollingMap::~RollingMap() = default;

void RollingMap::slide(double x, double y)
{
    m_impl->slide(x, y);
}

int RollingMap::initialClearPatch(double x, double y, double t, double radius, bool only_unknown, bool hold_until_motion)
{
    Impl& m = *m_impl;
    ++m.m_version;
    m.requireWindow();
    if (hold_until_motion)
    {
        m.m_heldSlots.assign(static_cast<std::size_t>(m.m_n * m.m_n), 0);
        m.m_hasHeldSlots = true;
        m.m_heldCells.clear();
    }
    int count = 0;
    for (std::int64_t i = 0; i < m.m_n; ++i)
    {
        for (std::int64_t j = 0; j < m.m_n; ++j)
        {
            const std::int64_t cx = i + m.m_lower[0], cy = j + m.m_lower[1];
            const std::size_t s = m.slot(cx, cy);
            bool inside = Norm(m.center(cx) - x, m.center(cy) - y) < radius;
            if (only_unknown)
            {
                inside = inside && (t - m.m_stamp[s] > m.m_ttl);
            }
            if (!inside)
            {
                continue;
            }
            m.m_log[s] = -2.;
            m.m_stamp[s] = t;
            if (hold_until_motion)
            {
                m.m_heldSlots[s] = 1;
                m.m_heldCells.push_back({cx, cy});
            }
            ++count;
        }
    }
    if (hold_until_motion)
    {
        m.m_heldStamp = t;
    }
    return count;
}

void RollingMap::advanceStartupPrior(double t, bool release)
{
    Impl& m = *m_impl;
    ++m.m_version;
    std::vector<std::array<std::int64_t, 2>> kept;
    if (!m.m_heldCells.empty())
    {
        if (!m.m_hasHeldSlots)
        {
            throw std::logic_error("startup_prior_without_slots");
        }
        for (const auto& cell : m.m_heldCells)
        {
            const std::size_t s = m.slot(cell[0], cell[1]);
            if (m.m_heldSlots[s] && m.m_tags[2 * s] == cell[0] && m.m_tags[2 * s + 1] == cell[1] &&
                m.m_stamp[s] == m.m_heldStamp && m.m_log[s] == -2.)
            {
                kept.push_back(cell);
            }
        }
        for (const auto& cell : kept)
        {
            m.m_stamp[m.slot(cell[0], cell[1])] = t;
        }
    }
    m.m_heldCells = release ? std::vector<std::array<std::int64_t, 2>>{} : std::move(kept);
    m.m_heldStamp = t;
    if (release)
    {
        m.m_hasHeldSlots = false;
        m.m_heldSlots.clear();
    }
}

void RollingMap::update(std::span<const CloudView> clouds, const std::array<double, 3>& pose, double t, bool rebuild)
{
    Impl& m = *m_impl;
    ++m.m_version;
    m.slide(pose[0], pose[1]);
    std::vector<double> hits;
    std::vector<std::size_t> touched;
    std::vector<double> xy;
    for (const CloudView& cloud : clouds)
    {
        const std::size_t count = cloud.m_points.size() / 3;
        if (cloud.m_pointsOdom.size() != 3 * count || cloud.m_dynamic.size() != count ||
            cloud.m_targetPointsOdom.size() % 3 != 0 || cloud.m_markingPointsOdom.size() % 3 != 0 ||
            cloud.m_points.size() % 3 != 0)
        {
            throw std::invalid_argument("invalid_map_cloud");
        }
        const double* p = cloud.m_points.data();
        const double* world = cloud.m_pointsOdom.data();
        if (const std::size_t targets = cloud.m_targetPointsOdom.size() / 3; targets)
        {
            m.slots(cloud.m_targetPointsOdom.data(), targets, 3, touched);
            for (const std::size_t s : touched)
            {
                m.m_log[s] = -2.;
                m.m_stamp[s] = t;
                m.m_clearStamp[s] = t;
                m.retireStartup(s);
            }
        }
        // Rays only from points no higher than the sensor (+2 cm).
        xy.clear();
        for (std::size_t i = 0; i < count; ++i)
        {
            if (p[3 * i + 2] <= cloud.m_origin[2] + .02)
            {
                xy.push_back(world[3 * i]);
                xy.push_back(world[3 * i + 1]);
            }
        }
        if (const std::size_t endpoints = xy.size() / 2; endpoints)
        {
            // First endpoint per endpoint cell (np.unique return_index).
            std::vector<std::int64_t> cx(endpoints), cy(endpoints);
            std::int64_t min_x = std::numeric_limits<std::int64_t>::max(), min_y = min_x;
            for (std::size_t i = 0; i < endpoints; ++i)
            {
                cx[i] = CellOf(xy[2 * i], m.m_res);
                cy[i] = CellOf(xy[2 * i + 1], m.m_res);
                min_x = std::min(min_x, cx[i]);
                min_y = std::min(min_y, cy[i]);
            }
            std::int64_t width = 0;
            for (std::size_t i = 0; i < endpoints; ++i)
            {
                width = std::max(width, cy[i] - min_y);
            }
            width += 1;
            std::unordered_set<std::int64_t> keys;
            keys.reserve(endpoints);
            std::vector<double> unique;
            unique.reserve(xy.size());
            for (std::size_t i = 0; i < endpoints; ++i)
            {
                if (keys.insert((cx[i] - min_x) * width + (cy[i] - min_y)).second)
                {
                    unique.push_back(xy[2 * i]);
                    unique.push_back(xy[2 * i + 1]);
                }
            }
            std::vector<unsigned char> seen(static_cast<std::size_t>(m.m_n * m.m_n), 0);
            detail::MarkRayCells(unique.data(), unique.size() / 2, cloud.m_originOdom[0], cloud.m_originOdom[1],
                                 static_cast<double>(m.m_lower[0]), static_cast<double>(m.m_lower[1]),
                                 m.m_res, static_cast<int>(m.m_n), seen);
            for (std::size_t u = 0; u < seen.size(); ++u)
            {
                if (!seen[u])
                {
                    continue;
                }
                const std::size_t s = m.slot(m.m_lower[0] + static_cast<std::int64_t>(u) / m.m_n,
                                             m.m_lower[1] + static_cast<std::int64_t>(u) % m.m_n);
                m.m_log[s] = std::max(-2., m.m_log[s] - .8);
                m.m_stamp[s] = t;
                m.m_clearStamp[s] = -INF;
                m.retireStartup(s);
            }
        }
        // Floor endpoints certify traversable ground.
        xy.clear();
        for (std::size_t i = 0; i < count; ++i)
        {
            if (p[3 * i + 2] < .04)
            {
                xy.push_back(world[3 * i]);
                xy.push_back(world[3 * i + 1]);
            }
        }
        m.slots(xy.data(), xy.size() / 2, 2, touched);
        for (const std::size_t s : touched)
        {
            m.m_log[s] = std::max(-2., m.m_log[s] - .8);
            m.m_stamp[s] = t;
            m.m_clearStamp[s] = -INF;
            m.retireStartup(s);
        }
        if (cloud.m_hasMarking)
        {
            const double* marking = cloud.m_markingPointsOdom.data();
            for (std::size_t i = 0; i < cloud.m_markingPointsOdom.size() / 3; ++i)
            {
                hits.push_back(marking[3 * i]);
                hits.push_back(marking[3 * i + 1]);
            }
        }
        else
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                const double z = p[3 * i + 2];
                if (z >= .08 && z <= 1.6 && !cloud.m_dynamic[i])
                {
                    hits.push_back(world[3 * i]);
                    hits.push_back(world[3 * i + 1]);
                }
            }
        }
    }
    m.slots(hits.data(), hits.size() / 2, 2, touched);
    for (const std::size_t s : touched)
    {
        const bool was = (t - m.m_stamp[s] <= m.m_ttl) && (m.m_log[s] >= 0);
        m.m_since[s] = was ? m.m_since[s] : t;
        m.m_log[s] = std::min(3., m.m_log[s] + 2.);
        m.m_stamp[s] = t;
        m.m_clearStamp[s] = -INF;
        m.retireStartup(s);
    }
    // The robot's own footprint stays known unless an obstacle is current there.
    for (std::int64_t i = 0; i < m.m_n; ++i)
    {
        for (std::int64_t j = 0; j < m.m_n; ++j)
        {
            const std::int64_t cx = i + m.m_lower[0], cy = j + m.m_lower[1];
            const std::size_t s = m.slot(cx, cy);
            if (!(Norm(m.center(cx) - pose[0], m.center(cy) - pose[1]) < m.m_robotRadius))
            {
                continue;
            }
            if ((t - m.m_stamp[s] <= m.m_ttl) && (m.m_log[s] >= 0))
            {
                continue;
            }
            m.m_log[s] = std::min(m.m_log[s], -1.);
            m.m_stamp[s] = t;
            m.m_clearStamp[s] = -INF;
            m.retireStartup(s);
        }
    }
    if (rebuild)
    {
        this->rebuild(t, false);
    }
}

std::vector<double> RollingMap::dropEstablished(std::span<const double> points, std::span<const double> trail,
                                                double t, double radius) const
{
    const Impl& m = *m_impl;
    if (points.size() % 3 != 0 || trail.size() % 3 != 0)
    {
        throw std::invalid_argument("invalid_drop_established_input");
    }
    const std::size_t count = points.size() / 3, length = trail.size() / 3;
    if (!count || !length)
    {
        return {};
    }
    std::vector<std::size_t> touched;
    m.slots(points.data(), count, 3, touched);
    std::vector<std::array<std::int64_t, 2>> real;
    for (const std::size_t s : touched)
    {
        const double since = m.m_since[s];
        const bool occupied = (m.m_log[s] >= 0) && (t - m.m_stamp[s] <= m.m_ttl) && std::isfinite(since);
        const double then_x = NumpyInterp(since, trail.data(), trail.data() + 1, length, 3);
        const double then_y = NumpyInterp(since, trail.data(), trail.data() + 2, length, 3);
        const std::int64_t tx = m.m_tags[2 * s], ty = m.m_tags[2 * s + 1];
        if (occupied && ((since < trail[0]) || (Norm(m.center(tx) - then_x, m.center(ty) - then_y) > radius)))
        {
            real.push_back({tx, ty});
        }
    }
    if (real.empty())
    {
        return std::vector<double>(points.begin(), points.end());
    }
    std::vector<std::int64_t> cx(count), cy(count);
    std::int64_t max_y = std::numeric_limits<std::int64_t>::min(), min_y = std::numeric_limits<std::int64_t>::max();
    for (std::size_t i = 0; i < count; ++i)
    {
        cx[i] = CellOf(points[3 * i], m.m_res);
        cy[i] = CellOf(points[3 * i + 1], m.m_res);
        max_y = std::max(max_y, cy[i]);
        min_y = std::min(min_y, cy[i]);
    }
    for (const auto& cell : real)
    {
        min_y = std::min(min_y, cell[1]);
    }
    const std::int64_t base = max_y - min_y + 1;
    std::unordered_set<std::int64_t> blocked;
    for (const auto& cell : real)
    {
        blocked.insert(cell[0] * base + cell[1]);
    }
    std::vector<double> kept;
    kept.reserve(points.size());
    for (std::size_t i = 0; i < count; ++i)
    {
        if (!blocked.count(cx[i] * base + cy[i]))
        {
            kept.insert(kept.end(), points.begin() + static_cast<std::ptrdiff_t>(3 * i),
                        points.begin() + static_cast<std::ptrdiff_t>(3 * i + 3));
        }
    }
    return kept;
}

int RollingMap::retirePersonTrail(std::span<const double> trail, double t, double radius, double window, double max_gap)
{
    Impl& m = *m_impl;
    if (trail.size() % 3 != 0)
    {
        throw std::invalid_argument("invalid_person_trail");
    }
    const std::size_t length = trail.size() / 3;
    if (radius <= 0 || !length)
    {
        return 0;
    }
    const auto stamp_at = [&](std::size_t k) { return trail[3 * k]; };
    const double first = stamp_at(0) - window, last = stamp_at(length - 1) + window;
    std::vector<std::size_t> ghosts;
    for (std::size_t s = 0; s < m.m_since.size(); ++s)
    {
        const double since = m.m_since[s];
        if (!(since >= first && since <= last && m.m_log[s] >= 0 && t - m.m_stamp[s] <= m.m_ttl))
        {
            continue;
        }
        // np.searchsorted (left), clipped.
        std::size_t lo = 0, hi = length;
        while (lo < hi)
        {
            const std::size_t mid = (lo + hi) / 2;
            if (stamp_at(mid) < since)
            {
                lo = mid + 1;
            }
            else
            {
                hi = mid;
            }
        }
        const std::size_t after = std::min(lo, length - 1);
        const std::size_t before = after == 0 ? 0 : after - 1;
        const double t0 = stamp_at(before), t1 = stamp_at(after);
        const bool bracketed = (t0 <= since) && (since <= t1) && (t1 - t0 <= max_gap);
        const double weight = t1 > t0 ? (since - t0) / std::max(t1 - t0, 1e-9) : 0.;
        const bool near_before = std::abs(since - t0) <= std::abs(since - t1);
        const bool close = std::min(std::abs(since - t0), std::abs(since - t1)) <= window;
        double px, py;
        if (bracketed)
        {
            px = trail[3 * before + 1] + weight * (trail[3 * after + 1] - trail[3 * before + 1]);
            py = trail[3 * before + 2] + weight * (trail[3 * after + 2] - trail[3 * before + 2]);
        }
        else
        {
            const std::size_t k = near_before ? before : after;
            px = trail[3 * k + 1];
            py = trail[3 * k + 2];
        }
        if ((bracketed || close) && Norm(m.center(m.m_tags[2 * s]) - px, m.center(m.m_tags[2 * s + 1]) - py) <= radius)
        {
            ghosts.push_back(s);
        }
    }
    if (ghosts.empty())
    {
        return 0;
    }
    for (const std::size_t s : ghosts)
    {
        m.m_log[s] = 0.;
        m.m_stamp[s] = -INF;
        m.m_clearStamp[s] = -INF;
        m.m_since[s] = -INF;
    }
    ++m.m_version;
    return static_cast<int>(ghosts.size());
}

int RollingMap::retireDisc(double x, double y, double radius, double t)
{
    Impl& m = *m_impl;
    if (radius <= 0)
    {
        return 0;
    }
    int count = 0;
    for (std::size_t s = 0; s < m.m_log.size(); ++s)
    {
        if (!(m.m_log[s] >= 0 && t - m.m_stamp[s] <= m.m_ttl))
        {
            continue;
        }
        if (!(std::hypot(m.center(m.m_tags[2 * s]) - x, m.center(m.m_tags[2 * s + 1]) - y) <= radius))
        {
            continue;
        }
        m.m_log[s] = 0.;
        m.m_stamp[s] = -INF;
        m.m_clearStamp[s] = -INF;
        m.m_since[s] = -INF;
        ++count;
    }
    if (count)
    {
        ++m.m_version;
    }
    return count;
}

void RollingMap::rebuild(double t, bool reuse)
{
    Impl& m = *m_impl;
    m.requireWindow();
    const std::array<double, 2> center = m.m_hasCenter ? m.m_center : std::array<double, 2>{0., 0.};
    const double radius = m.m_freeRadius;
    const bool same = reuse && m.m_hasBuilt && m.m_builtVersion == m.m_version && m.m_builtRadius == radius &&
                      m.m_builtTargetClearTtl == m.m_targetClearTtl && t >= m.m_builtTime &&
                      !m.expiryBetween(m.m_builtTime, t);
    if (same && (radius <= 0 || center == m.m_builtCenter))
    {
        m.m_builtTime = t;
        return;
    }
    const auto n = static_cast<std::size_t>(m.m_n);
    if (!same)
    {
        m.m_sensorKnown.assign(n * n, 0);
        m.m_occupied.assign(n * n, 0);
        std::vector<std::uint8_t> free(n * n, 0);
        for (std::int64_t i = 0; i < m.m_n; ++i)
        {
            for (std::int64_t j = 0; j < m.m_n; ++j)
            {
                const std::size_t s = m.slot(i + m.m_lower[0], j + m.m_lower[1]);
                const auto w = static_cast<std::size_t>(i * m.m_n + j);
                bool known = t - m.m_stamp[s] <= m.m_ttl;
                const double target_age = t - m.m_clearStamp[s];
                known = known && !(std::isfinite(target_age) && target_age > m.m_targetClearTtl);
                m.m_sensorKnown[w] = known;
                m.m_occupied[w] = known && (m.m_log[s] >= 0);
                free[w] = !m.m_occupied[w];
            }
        }
        m.distanceField(free, m.m_dist);
    }
    m.m_known = m.m_sensorKnown;
    m.m_assumedFree.assign(n * n, 0);
    if (radius > 0)
    {
        if (!m.m_hasCenter)
        {
            throw std::logic_error("local_assumed_free_center_unset");
        }
        for (std::int64_t i = 0; i < m.m_n; ++i)
        {
            for (std::int64_t j = 0; j < m.m_n; ++j)
            {
                const auto w = static_cast<std::size_t>(i * m.m_n + j);
                const bool inside = Norm(m.center(i + m.m_lower[0]) - m.m_center[0], m.center(j + m.m_lower[1]) - m.m_center[1]) < radius;
                m.m_assumedFree[w] = inside && !m.m_known[w];
                m.m_known[w] = m.m_known[w] || m.m_assumedFree[w];
            }
        }
    }
    std::vector<std::uint8_t> free_known(n * n, 0);
    for (std::size_t w = 0; w < n * n; ++w)
    {
        free_known[w] = m.m_known[w] && !m.m_occupied[w];
    }
    m.distanceField(free_known, m.m_freeDist);
    m.m_hasBuilt = true;
    m.m_builtVersion = m.m_version;
    m.m_builtRadius = radius;
    m.m_builtTargetClearTtl = m.m_targetClearTtl;
    m.m_builtTime = t;
    m.m_builtCenter = center;
    m.m_hasFields = true;
    ++m.m_buildId;
}

void RollingMap::slots(std::span<const double> xy, std::vector<std::int64_t>& a, std::vector<std::int64_t>& b) const
{
    if (xy.size() % 2 != 0)
    {
        throw std::invalid_argument("invalid_slots_input");
    }
    std::vector<std::size_t> touched;
    m_impl->slots(xy.data(), xy.size() / 2, 2, touched);
    a.clear();
    b.clear();
    for (const std::size_t s : touched)
    {
        a.push_back(static_cast<std::int64_t>(s) / m_impl->m_n);
        b.push_back(static_cast<std::int64_t>(s) % m_impl->m_n);
    }
}

double RollingMap::resolution() const noexcept { return m_impl->m_res; }
int RollingMap::size() const noexcept { return static_cast<int>(m_impl->m_n); }
double RollingMap::ttl() const noexcept { return m_impl->m_ttl; }
double RollingMap::robotRadius() const noexcept { return m_impl->m_robotRadius; }
double RollingMap::targetClearTtl() const noexcept { return m_impl->m_targetClearTtl; }

void RollingMap::setTargetClearTtl(double value)
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument("invalid_target_clear_ttl");
    }
    m_impl->m_targetClearTtl = value;
}

double RollingMap::localFreeRadius() const noexcept { return m_impl->m_freeRadius; }

void RollingMap::setLocalFreeRadius(double value)
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument("invalid_local_assumed_free_radius");
    }
    m_impl->m_freeRadius = value;
}

bool RollingMap::hasLocalFreeCenter() const noexcept { return m_impl->m_hasCenter; }
std::array<double, 2> RollingMap::localFreeCenter() const noexcept { return m_impl->m_center; }

void RollingMap::setLocalFreeCenter(double x, double y)
{
    if (!std::isfinite(x) || !std::isfinite(y))
    {
        throw std::invalid_argument("invalid_local_assumed_free_center");
    }
    m_impl->m_center = {x, y};
    m_impl->m_hasCenter = true;
}

std::int64_t RollingMap::version() const noexcept { return m_impl->m_version; }
bool RollingMap::hasWindow() const noexcept { return m_impl->m_hasWindow; }
std::array<std::int64_t, 2> RollingMap::lower() const noexcept { return m_impl->m_lower; }
std::uint64_t RollingMap::windowId() const noexcept { return m_impl->m_windowId; }
std::uint64_t RollingMap::buildId() const noexcept { return m_impl->m_buildId; }
bool RollingMap::hasFields() const noexcept { return m_impl->m_hasFields; }
const double* RollingMap::logData() const noexcept { return m_impl->m_log.data(); }
const double* RollingMap::stampData() const noexcept { return m_impl->m_stamp.data(); }
const double* RollingMap::targetClearStampData() const noexcept { return m_impl->m_clearStamp.data(); }
const double* RollingMap::occupiedSinceData() const noexcept { return m_impl->m_since.data(); }
const std::int64_t* RollingMap::tagsData() const noexcept { return m_impl->m_tags.data(); }
const std::uint8_t* RollingMap::sensorKnownData() const noexcept { return m_impl->m_sensorKnown.data(); }
const std::uint8_t* RollingMap::occupiedData() const noexcept { return m_impl->m_occupied.data(); }
const std::uint8_t* RollingMap::knownData() const noexcept { return m_impl->m_known.data(); }
const std::uint8_t* RollingMap::assumedFreeData() const noexcept { return m_impl->m_assumedFree.data(); }
const double* RollingMap::distData() const noexcept { return m_impl->m_dist.data(); }
const double* RollingMap::freeDistData() const noexcept { return m_impl->m_freeDist.data(); }

void SquaredDistanceTransform(const std::vector<std::uint8_t>& foreground, int rows, int cols, std::vector<double>& out)
{
    if (rows < 1 || cols < 1 || foreground.size() != static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols))
    {
        throw std::invalid_argument("invalid_distance_transform_input");
    }
    // Pass 1: distance along each column to the nearest zero cell.
    std::vector<double> g(foreground.size());
    for (int j = 0; j < cols; ++j)
    {
        int last = -1;
        for (int i = 0; i < rows; ++i)
        {
            const auto k = static_cast<std::size_t>(i * cols + j);
            if (!foreground[k])
            {
                last = i;
            }
            g[k] = last < 0 ? INF : static_cast<double>(i - last);
        }
        last = -1;
        for (int i = rows - 1; i >= 0; --i)
        {
            const auto k = static_cast<std::size_t>(i * cols + j);
            if (!foreground[k])
            {
                last = i;
            }
            if (last >= 0)
            {
                g[k] = std::min(g[k], static_cast<double>(last - i));
            }
        }
        for (int i = 0; i < rows; ++i)
        {
            const auto k = static_cast<std::size_t>(i * cols + j);
            if (!std::isfinite(g[k]))
            {
                throw std::invalid_argument("distance_transform_column_without_zero");
            }
            g[k] *= g[k];
        }
    }
    // Pass 2: lower envelope of parabolas along each row (Felzenszwalb).
    // Inputs are small exact integers, so every parabola choice is exact.
    out.resize(foreground.size());
    std::vector<int> v(static_cast<std::size_t>(cols));
    std::vector<double> z(static_cast<std::size_t>(cols) + 1);
    for (int i = 0; i < rows; ++i)
    {
        const double* f = g.data() + static_cast<std::size_t>(i) * cols;
        int k = 0;
        v[0] = 0;
        z[0] = -INF;
        z[1] = INF;
        for (int q = 1; q < cols; ++q)
        {
            double s;
            while (true)
            {
                const int p = v[static_cast<std::size_t>(k)];
                s = ((f[q] + static_cast<double>(q) * q) - (f[p] + static_cast<double>(p) * p)) / (2. * q - 2. * p);
                if (s <= z[static_cast<std::size_t>(k)])
                {
                    --k;
                    continue;
                }
                break;
            }
            ++k;
            v[static_cast<std::size_t>(k)] = q;
            z[static_cast<std::size_t>(k)] = s;
            z[static_cast<std::size_t>(k) + 1] = INF;
        }
        k = 0;
        for (int q = 0; q < cols; ++q)
        {
            while (z[static_cast<std::size_t>(k) + 1] < q)
            {
                ++k;
            }
            const int p = v[static_cast<std::size_t>(k)];
            out[static_cast<std::size_t>(i) * cols + q] = static_cast<double>(q - p) * (q - p) + f[p];
        }
    }
}

double NumpyInterp(double x, const double* xp, const double* fp, std::size_t count, std::size_t stride)
{
    if (!count)
    {
        throw std::invalid_argument("interp_empty_xp");
    }
    const auto at = [stride](const double* base, std::size_t k) { return base[k * stride]; };
    if (std::isnan(x))
    {
        return x;
    }
    if (count == 1)
    {
        return at(fp, 0);
    }
    if (x > at(xp, count - 1))
    {
        return at(fp, count - 1);
    }
    if (x < at(xp, 0))
    {
        return at(fp, 0);
    }
    // Largest j with xp[j] <= x.
    std::size_t lo = 0, hi = count;
    while (hi - lo > 1)
    {
        const std::size_t mid = (lo + hi) / 2;
        if (at(xp, mid) <= x)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    const std::size_t j = lo;
    if (j == count - 1 || at(xp, j) == x)
    {
        return at(fp, j);
    }
    const double slope = (at(fp, j + 1) - at(fp, j)) / (at(xp, j + 1) - at(xp, j));
    double result = slope * (x - at(xp, j)) + at(fp, j);
    if (std::isnan(result))
    {
        result = slope * (x - at(xp, j + 1)) + at(fp, j + 1);
        if (std::isnan(result) && at(fp, j) == at(fp, j + 1))
        {
            result = at(fp, j);
        }
    }
    return result;
}

} // namespace wla::follow
