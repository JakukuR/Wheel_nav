// Port of sim_core/approach_goal.py ApproachGoal.select. The cKDTree queries
// are only ever compared with a threshold, so an exact brute-force nearest
// distance (scipy's sqrt(fma(dy, dy, dx * dx))) gives the same decisions.
#include <wla_follow/planning.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "map_view.h"
#include "numpy_blas.h"
#include "numpy_ops.h"

namespace wla::follow {

namespace {

using detail::BlasNorm2;
using detail::MapView;
using detail::RowNorm2;

constexpr double HALF_LENGTH = .24;
constexpr double HALF_WIDTH = .24;
constexpr double DESIRED_GAP = .15;
constexpr double PLAN_MARGIN_M = 1.2;
constexpr double ROUTE_CELL_M = .1;
constexpr double INF = std::numeric_limits<double>::infinity();

[[nodiscard]] double SweepRadius() noexcept
{
    return std::hypot(HALF_LENGTH, HALF_WIDTH);
}

// clear_segments: mapping.query(points) >= radius + margin + .014
[[nodiscard]] bool ClearPoint(const MapView& map, const Point2& q) noexcept
{
    return map.query(q[0], q[1]) >= SweepRadius() + .10 + .014;
}

// cKDTree(surface).query(q)[0] >= threshold
[[nodiscard]] bool SurfaceClear(std::span<const Point2> surface, const Point2& q, double threshold) noexcept
{
    double best = INF;
    for (const Point2& s : surface)
    {
        const double dx = q[0] - s[0], dy = q[1] - s[1];
        const double d2 = std::fma(dy, dy, dx * dx);
        best = d2 < best ? d2 : best;
    }
    return std::sqrt(best) >= threshold;
}

[[nodiscard]] bool PathClear(const MapView& map, std::span<const Point2> surface, std::span<const Point2> samples,
                             double threshold)
{
    for (const Point2& q : samples)
    {
        if (!ClearPoint(map, q))
        {
            return false;
        }
    }
    for (const Point2& q : samples)
    {
        if (!SurfaceClear(surface, q, threshold))
        {
            return false;
        }
    }
    return true;
}

// surface_gap: (surface - pose) @ rotation(yaw) (BLAS, fused), box distance.
[[nodiscard]] double SurfaceGap(const std::array<double, 3>& pose, std::span<const Point2> surface)
{
    const double c = std::cos(pose[2]), s = std::sin(pose[2]);
    double best = INF;
    bool first = true;
    for (const Point2& q : surface)
    {
        const double dx = q[0] - pose[0], dy = q[1] - pose[1];
        const double lx = std::fma(dy, s, dx * c);
        const double ly = std::fma(dy, c, dx * -s);
        const double ex = std::max(std::fabs(lx) - HALF_LENGTH, 0.), ey = std::max(std::fabs(ly) - HALF_WIDTH, 0.);
        const double d = RowNorm2(ex, ey);
        best = first || d < best || std::isnan(d) ? (std::isnan(best) && !first ? best : d) : best;
        first = false;
    }
    return best;
}

} // namespace

std::vector<Point2> ApproachSamples(std::span<const Point2> path)
{
    if (path.size() < 2)
    {
        return {path.begin(), path.end()};
    }
    std::vector<Point2> out;
    for (std::size_t k = 0; k + 1 < path.size(); ++k)
    {
        const Point2& a = path[k];
        const Point2& b = path[k + 1];
        const auto n = std::max<std::size_t>(
            2, static_cast<std::size_t>(std::ceil(BlasNorm2(b[0] - a[0], b[1] - a[1]) / .025)) + 1);
        const auto part = detail::Linspace2(a, b, n);
        out.insert(out.end(), part.begin(), part.end());
    }
    return out;
}

struct ApproachGoal::Impl
{
    std::optional<ApproachPlan> m_plan;
    double m_plannedAt{-INF};
    std::optional<Point2> m_center;
    std::int64_t m_generation{0};
    std::optional<double> m_stableSince;

    [[nodiscard]] ApproachReport diagnostic(const std::array<double, 3>& pose, std::span<const Point2> surface,
                                            double now) const
    {
        ApproachReport report;
        report.m_plan = *m_plan;
        report.m_currentClearanceM = SurfaceGap(pose, surface);
        report.m_endpointDistanceM = BlasNorm2(m_plan->m_point[0] - pose[0], m_plan->m_point[1] - pose[1]);
        report.m_stableS = now - *m_stableSince;
        return report;
    }

    std::optional<ApproachReport> select(const std::array<double, 3>& pose, std::span<const Point2> raw,
                                         const MapView& map, double now);
};

std::optional<ApproachReport> ApproachGoal::Impl::select(const std::array<double, 3>& pose,
                                                         std::span<const Point2> raw, const MapView& map,
                                                         double now)
{
    // np.unique(np.round(surface / .025) * .025, axis=0): rows sorted lexicographically.
    std::vector<Point2> surface;
    surface.reserve(raw.size());
    bool finite = true;
    for (const Point2& q : raw)
    {
        const Point2 r{std::nearbyint(q[0] / .025) * .025, std::nearbyint(q[1] / .025) * .025};
        finite = finite && std::isfinite(r[0]) && std::isfinite(r[1]);
        surface.push_back(r);
    }
    if (!finite)
    {
        m_plan.reset();
        return std::nullopt;
    }
    std::sort(surface.begin(), surface.end(),
              [](const Point2& a, const Point2& b) { return a[0] != b[0] ? a[0] < b[0] : a[1] < b[1]; });
    surface.erase(std::unique(surface.begin(), surface.end(),
                              [](const Point2& a, const Point2& b) { return a[0] == b[0] && a[1] == b[1]; }),
                  surface.end());
    if (surface.size() < 3)
    {
        m_plan.reset();
        return std::nullopt;
    }
    Point2 center{};
    for (std::size_t d = 0; d < 2; ++d)
    {
        std::vector<double> column;
        column.reserve(surface.size());
        for (const Point2& q : surface)
        {
            column.push_back(q[d]);
        }
        std::sort(column.begin(), column.end());
        const std::size_t half = column.size() / 2;
        center[d] = column.size() % 2 ? column[half] : (column[half - 1] + column[half]) / 2;
    }
    const Point2 robot{pose[0], pose[1]};
    const double sweep = SweepRadius();
    if (m_plan && now - m_plannedAt < .4 &&
        BlasNorm2(center[0] - (*m_center)[0], center[1] - (*m_center)[1]) < .06)
    {
        const auto& path = m_plan->m_path;
        std::size_t near = 0;
        double best = INF;
        for (std::size_t k = 0; k < path.size(); ++k)
        {
            const double d = RowNorm2(path[k][0] - robot[0], path[k][1] - robot[1]);
            if (k == 0 || d < best)
            {
                best = d;
                near = k;
            }
        }
        std::vector<Point2> suffix{robot};
        suffix.insert(suffix.end(), path.begin() + static_cast<std::ptrdiff_t>(near), path.end());
        const auto samples = ApproachSamples(suffix);
        if (PathClear(map, surface, samples, sweep + DESIRED_GAP))
        {
            m_plan->m_path = std::move(suffix);
            return diagnostic(pose, surface, now);
        }
    }
    const std::optional<ApproachPlan> old = m_plan;
    m_plannedAt = now;
    m_center = center;
    m_plan.reset();
    const int stride = std::max(1, static_cast<int>(std::nearbyint(ROUTE_CELL_M / map.m_res)));
    const double res = map.m_res * stride;
    const int n = (map.m_n + stride - 1) / stride;
    const auto nodes = static_cast<std::size_t>(n) * n;
    std::vector<Point2> xy(nodes);
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            xy[static_cast<std::size_t>(i) * n + j] = {map.centerX(i * stride), map.centerY(j * stride)};
        }
    }
    const Point2 low{std::min(robot[0], center[0]) - PLAN_MARGIN_M, std::min(robot[1], center[1]) - PLAN_MARGIN_M};
    const Point2 high{std::max(robot[0], center[0]) + PLAN_MARGIN_M, std::max(robot[1], center[1]) + PLAN_MARGIN_M};
    const double threshold = sweep + DESIRED_GAP + .014;
    std::vector<std::uint8_t> valid(nodes, 0);
    for (std::size_t k = 0; k < nodes; ++k)
    {
        const Point2& q = xy[k];
        if (q[0] >= low[0] && q[1] >= low[1] && q[0] <= high[0] && q[1] <= high[1])
        {
            valid[k] = ClearPoint(map, q) && SurfaceClear(surface, q, threshold);
        }
    }
    std::vector<int> srcs, dsts;
    std::vector<double> costs;
    const auto fraction = detail::Linspace(0., 1., 7);
    constexpr int MOVES[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};
    for (const auto& move : MOVES)
    {
        const int dx = move[0], dy = move[1];
        std::vector<int> a_list, b_list;
        for (int i = 0; i + dx < n; ++i)
        {
            for (int j = std::max(0, -dy); j < n - std::max(0, dy); ++j)
            {
                const int a = i * n + j, b = (i + dx) * n + (j + dy);
                bool ok = valid[static_cast<std::size_t>(a)] && valid[static_cast<std::size_t>(b)];
                if (dx && dy)
                {
                    ok = ok && valid[static_cast<std::size_t>((i + dx) * n + j)] &&
                         valid[static_cast<std::size_t>(i * n + (j + dy))];
                }
                if (!ok)
                {
                    continue;
                }
                const Point2& pa = xy[static_cast<std::size_t>(a)];
                const Point2& pb = xy[static_cast<std::size_t>(b)];
                const Point2 d{pb[0] - pa[0], pb[1] - pa[1]};
                bool good = true;
                for (const double f : fraction)
                {
                    good = good && ClearPoint(map, {pa[0] + f * d[0], pa[1] + f * d[1]});
                }
                for (const double f : fraction)
                {
                    good = good && SurfaceClear(surface, {pa[0] + f * d[0], pa[1] + f * d[1]}, threshold);
                }
                if (good)
                {
                    a_list.push_back(a);
                    b_list.push_back(b);
                }
            }
        }
        if (a_list.empty())
        {
            continue;
        }
        const double cost = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * res;
        srcs.insert(srcs.end(), a_list.begin(), a_list.end());
        srcs.insert(srcs.end(), b_list.begin(), b_list.end());
        dsts.insert(dsts.end(), b_list.begin(), b_list.end());
        dsts.insert(dsts.end(), a_list.begin(), a_list.end());
        costs.insert(costs.end(), 2 * a_list.size(), cost);
    }
    const int virtual_node = static_cast<int>(nodes);
    for (std::size_t dest = 0; dest < nodes; ++dest)
    {
        if (!valid[dest] || !(RowNorm2(xy[dest][0] - robot[0], xy[dest][1] - robot[1]) <= res * 1.6))
        {
            continue;
        }
        const Point2 ends[2] = {robot, xy[dest]};
        const auto segment = ApproachSamples(ends);
        if (PathClear(map, surface, segment, threshold))
        {
            srcs.push_back(virtual_node);
            dsts.push_back(static_cast<int>(dest));
            costs.push_back(std::max(1e-6, BlasNorm2(xy[dest][0] - robot[0], xy[dest][1] - robot[1])));
        }
    }
    if (srcs.empty())
    {
        return std::nullopt;
    }
    const auto graph = detail::BuildCsr(virtual_node + 1, srcs, dsts, costs);
    std::vector<double> distances;
    std::vector<int> predecessor;
    detail::Dijkstra(graph, virtual_node, distances, predecessor);
    std::vector<std::size_t> reachable;
    for (std::size_t k = 0; k < nodes; ++k)
    {
        if (std::isfinite(distances[k]))
        {
            reachable.push_back(k);
        }
    }
    if (reachable.empty())
    {
        return std::nullopt;
    }
    std::vector<std::size_t> kept;
    std::vector<double> yaw, gaps;
    for (const std::size_t k : reachable)
    {
        const Point2& q = xy[k];
        const double angle = std::atan2(center[1] - q[1], center[0] - q[0]);
        const double c = std::cos(angle), s = std::sin(angle);
        double gap = INF;
        bool first = true;
        for (const Point2& p : surface)
        {
            const double dx = p[0] - q[0], dy = p[1] - q[1];
            const double x = dx * c + dy * s;
            const double y = -dx * s + dy * c;
            const double g = std::hypot(std::max(std::fabs(x) - HALF_LENGTH, 0.), std::max(std::fabs(y) - HALF_WIDTH, 0.));
            gap = first || g < gap || std::isnan(g) ? (std::isnan(gap) && !first ? gap : g) : gap;
            first = false;
        }
        if (gap >= DESIRED_GAP)
        {
            kept.push_back(k);
            yaw.push_back(angle);
            gaps.push_back(gap);
        }
    }
    if (kept.empty())
    {
        return std::nullopt;
    }
    double minimum = gaps[0];
    for (const double g : gaps)
    {
        minimum = g < minimum ? g : minimum;
    }
    const double band = std::max(DESIRED_GAP + .05, minimum + .025);
    std::size_t choice = kept.size();
    for (std::size_t k = 0; k < kept.size(); ++k)
    {
        if (gaps[k] <= band && (choice == kept.size() || distances[kept[k]] < distances[kept[choice]]))
        {
            choice = k;
        }
    }
    const auto dest = static_cast<int>(kept[choice]);
    std::vector<Point2> route;
    int current = dest;
    while (current != virtual_node)
    {
        if (current < 0 || route.size() > nodes)
        {
            return std::nullopt;
        }
        route.push_back(xy[static_cast<std::size_t>(current)]);
        current = predecessor[static_cast<std::size_t>(current)];
    }
    std::vector<Point2> path{robot};
    path.insert(path.end(), route.rbegin(), route.rend());
    const Point2 endpoint = xy[static_cast<std::size_t>(dest)];
    if (!old || BlasNorm2(old->m_point[0] - endpoint[0], old->m_point[1] - endpoint[1]) > .12)
    {
        m_generation += 1;
        m_stableSince = now;
    }
    ApproachPlan plan;
    plan.m_point = endpoint;
    plan.m_path = std::move(path);
    plan.m_yaw = yaw[choice];
    plan.m_goalClearanceM = gaps[choice];
    plan.m_fallback = gaps[choice] > DESIRED_GAP + .05;
    plan.m_reachableCells = static_cast<std::int64_t>(kept.size());
    plan.m_generation = m_generation;
    m_plan = std::move(plan);
    return diagnostic(pose, surface, now);
}

ApproachGoal::ApproachGoal() : m_impl(std::make_unique<Impl>()) {}
ApproachGoal::~ApproachGoal() = default;

std::optional<ApproachReport> ApproachGoal::select(const std::array<double, 3>& pose, std::span<const Point2> surface,
                                                   const RollingMap& map, double now)
{
    return m_impl->select(pose, surface, MapView(map), now);
}

const std::optional<ApproachPlan>& ApproachGoal::plan() const noexcept
{
    return m_impl->m_plan;
}

double ApproachGoal::plannedAt() const noexcept
{
    return m_impl->m_plannedAt;
}

const std::optional<Point2>& ApproachGoal::center() const noexcept
{
    return m_impl->m_center;
}

std::int64_t ApproachGoal::generation() const noexcept
{
    return m_impl->m_generation;
}

std::optional<double> ApproachGoal::stableSince() const noexcept
{
    return m_impl->m_stableSince;
}

} // namespace wla::follow
