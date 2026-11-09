// Statement-by-statement port of sim_core/detour.py and the HybridFollower of
// sim_core/hybrid_controller.py. Every numpy call keeps its rounding:
// 1-D norm/dot fused (BlasNorm2/BlasDot2), row norms and elementwise
// arithmetic unfused, Python min/max on scalars, argsort and Dijkstra ported
// (numpy_ops.h). Comments quote the reference where the order matters.
#include <wla_follow/planning.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include "map_view.h"
#include "numpy_blas.h"
#include "numpy_ops.h"

namespace wla::follow {

namespace {

using detail::BlasDot2;
using detail::BlasNorm2;
using detail::Clip;
using detail::Linspace;
using detail::MapView;
using detail::RowNorm2;
using detail::Sign;
using detail::Wrap;

constexpr double INF = std::numeric_limits<double>::infinity();
constexpr double GRID_ROUTE_CELL_M = .1;
constexpr double ASTAR_SIMPLIFY_M = .1;
constexpr double ASTAR_START_CLEAR_M = .2;
// hybrid_controller.ESCAPE_SLACK_M
constexpr double ESCAPE_SLACK_M = .01;

// Python builtin min(a, b) / max(a, b): the first unless the second is
// strictly smaller / larger.
[[nodiscard]] double PyMin(double a, double b) noexcept { return b < a ? b : a; }
[[nodiscard]] double PyMax(double a, double b) noexcept { return b > a ? b : a; }
// np.minimum / np.maximum for non-NaN inputs.
[[nodiscard]] double NpMin(double a, double b) noexcept { return (a <= b || std::isnan(a)) ? a : b; }
[[nodiscard]] double NpMax(double a, double b) noexcept { return (a >= b || std::isnan(a)) ? a : b; }

[[nodiscard]] Point2 Sub(const Point2& a, const Point2& b) noexcept { return {a[0] - b[0], a[1] - b[1]}; }

// np.argmin: first minimum, a NaN wins at its first occurrence.
[[nodiscard]] std::size_t ArgMin(const std::vector<double>& v)
{
    std::size_t best = 0;
    for (std::size_t k = 0; k < v.size(); ++k)
    {
        if (std::isnan(v[k]))
        {
            return k;
        }
        if (v[k] < v[best])
        {
            best = k;
        }
    }
    return best;
}

// One side's best edge point: (score, point), side +1 (left) / -1 (right).
struct SideChoice
{
    double m_side;
    double m_score;
    Point2 m_point;
};

[[nodiscard]] const SideChoice* Find(const std::vector<SideChoice>& best, std::optional<double> side)
{
    if (!side)
    {
        return nullptr;
    }
    for (const SideChoice& entry : best)
    {
        if (entry.m_side == *side)
        {
            return &entry;
        }
    }
    return nullptr;
}

} // namespace

// ---------------------------------------------------------------- free functions

double ChassisYaw(double w, const Tuning& c) noexcept
{
    const double a = std::fabs(w), lo = c.m_chassisDeadbandLo, hi = c.m_chassisDeadbandHi;
    const double floor = .2 * lo;
    const double eff = a < lo ? .2 * a : (a < hi ? floor + (a - lo) / PyMax(hi - lo, 1e-9) * (hi - floor) : a);
    return Sign(w) * eff;
}

double ApproachTangent(const std::array<double, 3>& pose, const Point2& command, std::span<const Obstacle> tracks,
                       const Point2& target, double radius)
{
    double result = 0.;
    // robot_v = rotation(yaw) @ [v, 0] (BLAS gemv, fused)
    const double cy = std::cos(pose[2]), sy = std::sin(pose[2]);
    const Point2 robot_v{std::fma(-sy, 0., cy * command[0]), std::fma(cy, 0., sy * command[0])};
    const Point2 direction{target[0] - pose[0], target[1] - pose[1]};
    for (const Obstacle& o : tracks)
    {
        const Point2 delta{pose[0] - o.m_position[0], pose[1] - o.m_position[1]};
        const double distance = BlasNorm2(delta[0], delta[1]);
        const double gap = PyMax(.08, distance - o.m_radius - radius);
        if (gap >= 1.2)
        {
            continue;
        }
        const double scale = PyMax(distance, 1e-9);
        const Point2 normal{delta[0] / scale, delta[1] / scale};
        const double approaching = PyMax(0., -BlasDot2(normal[0], normal[1], robot_v[0] - o.m_velocity[0],
                                                        robot_v[1] - o.m_velocity[1]));
        Point2 tangent{-normal[1], normal[0]};
        if (BlasDot2(tangent[0], tangent[1], direction[0], direction[1]) < 0)
        {
            tangent = {-tangent[0], -tangent[1]};
        }
        const double cross = std::cos(pose[2]) * tangent[1] - std::sin(pose[2]) * tangent[0];
        result += .08 * approaching * (1 / gap - 1 / 1.2) * cross;
    }
    return Clip(result, -.18, .18);
}

// ---------------------------------------------------------------- DetourPlanner

struct DetourPlanner::Impl
{
    double m_probeM{3.}, m_nearM{.4}, m_farM{3.}, m_maxBearing{1.9}, m_sampleM{.05};
    std::size_t m_candidatesPerSide{40};
    double m_switchRatio{1.3}, m_onwardM{.8}, m_standoffM{.15};
    std::optional<double> m_side;
    DetourReport m_last;
    int m_lookHold{0};
    bool m_triggerPath{false};
    int m_exitCycles{12};
    int m_clearCount{0};
    std::optional<Point2> m_held;

    [[nodiscard]] bool clear(const MapView& map, const Point2& a, const Point2& b, double need, double body) const
    {
        const Point2 d = Sub(b, a);
        const double length = BlasNorm2(d[0], d[1]);
        const auto u = Linspace(0., 1., std::max<std::size_t>(2, static_cast<std::size_t>(length / m_sampleM) + 1));
        bool ok_need = true, ok_body = true;
        for (const double t : u)
        {
            const double x = a[0] + t * d[0], y = a[1] + t * d[1];
            ok_need = ok_need && map.query(x, y) >= need;
            ok_body = ok_body && map.query(x, y, true) >= body;
        }
        return ok_need && ok_body;
    }

    // _clear_many: one sample count for the whole batch.
    [[nodiscard]] std::vector<bool> clearMany(const MapView& map, const std::vector<Point2>& a,
                                              const std::vector<Point2>& b, double need, double body) const
    {
        std::vector<bool> ok(a.size(), true);
        if (a.empty())
        {
            return ok;
        }
        double longest = -INF;
        for (std::size_t k = 0; k < a.size(); ++k)
        {
            const double l = RowNorm2(b[k][0] - a[k][0], b[k][1] - a[k][1]);
            longest = (l > longest || std::isnan(l)) ? l : longest;
        }
        const auto u = Linspace(0., 1., std::max<std::size_t>(2, static_cast<std::size_t>(longest / m_sampleM) + 1));
        for (std::size_t k = 0; k < a.size(); ++k)
        {
            const Point2 d = Sub(b[k], a[k]);
            for (const double t : u)
            {
                const double x = a[k][0] + t * d[0], y = a[k][1] + t * d[1];
                if (!(map.query(x, y) >= need && map.query(x, y, true) >= body))
                {
                    ok[k] = false;
                    break;
                }
            }
        }
        return ok;
    }

    [[nodiscard]] bool pathClear(const MapView& map, const Point2& p, const std::vector<Point2>& path,
                                 double need) const
    {
        std::vector<Point2> pts{p};
        pts.insert(pts.end(), path.begin(), path.end());
        double travelled = 0.;
        for (std::size_t k = 0; k + 1 < pts.size(); ++k)
        {
            const Point2& a = pts[k];
            const Point2 d = Sub(pts[k + 1], a);
            const double l = RowNorm2(d[0], d[1]);
            if (l < 1e-6)
            {
                continue;
            }
            const double take = PyMin(l, m_probeM - travelled);
            const Point2 b{a[0] + d[0] * (take / l), a[1] + d[1] * (take / l)};
            if (!clear(map, a, b, need, 0.))
            {
                return false;
            }
            travelled += take;
            if (travelled >= m_probeM - 1e-6)
            {
                break;
            }
        }
        return true;
    }

    [[nodiscard]] std::vector<SideChoice> candidates(const MapView& map, const Point2& p, const Point2& target,
                                                     const Point2& delta, double distance, const Tuning& c,
                                                     double need, double standoff, bool reachable) const
    {
        const int n = map.m_n;
        const double offset = map.m_res / std::sqrt(2.);
        std::vector<std::uint8_t> safe(static_cast<std::size_t>(n) * n);
        for (std::size_t k = 0; k < safe.size(); ++k)
        {
            safe[k] = (map.m_dist[k] - offset >= need + standoff) && (map.m_freeDist[k] - offset >= c.m_radius + standoff);
        }
        std::vector<Point2> points;
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                if (!safe[static_cast<std::size_t>(i) * n + j])
                {
                    continue;
                }
                bool unsafe_near = false;
                for (int dx = -1; dx <= 1 && !unsafe_near; ++dx)
                {
                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        if (!dx && !dy)
                        {
                            continue;
                        }
                        const int x = i + dx, y = j + dy;
                        if (x < 0 || x >= n || y < 0 || y >= n || !safe[static_cast<std::size_t>(x) * n + y])
                        {
                            unsafe_near = true;
                            break;
                        }
                    }
                }
                if (unsafe_near)
                {
                    points.push_back({map.centerX(i), map.centerY(j)});
                }
            }
        }
        if (points.empty())
        {
            return {};
        }
        const double heading = std::atan2(delta[1], delta[0]);
        std::vector<Point2> kept;
        std::vector<double> side, score;
        for (const Point2& point : points)
        {
            const Point2 rel = Sub(point, p);
            const double r = RowNorm2(rel[0], rel[1]);
            const double bearing = Wrap(std::atan2(rel[1], rel[0]) - heading);
            if (!(r > m_nearM && r < m_farM && std::fabs(bearing) < m_maxBearing))
            {
                continue;
            }
            kept.push_back(point);
            side.push_back(Sign(delta[0] * rel[1] - delta[1] * rel[0]));
            score.push_back(r + RowNorm2(target[0] - point[0], target[1] - point[1]) + .5 * std::fabs(bearing));
        }
        std::vector<SideChoice> best;
        for (const double s : {1., -1.})
        {
            std::vector<std::size_t> idx;
            std::vector<double> sub;
            for (std::size_t k = 0; k < kept.size(); ++k)
            {
                if (side[k] == s)
                {
                    idx.push_back(k);
                    sub.push_back(score[k]);
                }
            }
            const auto sorted = detail::ArgSort(sub);
            std::vector<std::size_t> order;
            for (std::size_t k = 0; k < sorted.size() && k < m_candidatesPerSide; ++k)
            {
                order.push_back(idx[static_cast<std::size_t>(sorted[k])]);
            }
            if (order.empty())
            {
                continue;
            }
            if (reachable)
            {
                std::vector<Point2> w, ahead, from(order.size(), p);
                const Point2 unit{delta[0] / distance, delta[1] / distance};
                for (const std::size_t k : order)
                {
                    const Point2& q = kept[k];
                    const double onward = NpMin(m_onwardM, RowNorm2(target[0] - q[0], target[1] - q[1]));
                    w.push_back(q);
                    ahead.push_back({q[0] + unit[0] * onward, q[1] + unit[1] * onward});
                }
                const auto reach = clearMany(map, from, w, need, c.m_radius);
                const auto onward_ok = clearMany(map, w, ahead, need, c.m_radius);
                std::size_t pass = order.size();
                for (std::size_t k = 0; k < order.size(); ++k)
                {
                    if (reach[k] && onward_ok[k])
                    {
                        pass = k;
                        break;
                    }
                }
                if (pass == order.size())
                {
                    continue;
                }
                order = {order[pass]};
            }
            best.push_back({s, score[order[0]], kept[order[0]]});
        }
        return best;
    }

    std::optional<std::vector<Point2>> look(const MapView& map, const Point2& p, const Point2& delta,
                                            const std::vector<SideChoice>& seen)
    {
        const int n = map.m_n;
        const double heading = std::atan2(delta[1], delta[0]);
        long left = 0, right = 0;
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                if (map.m_known[static_cast<std::size_t>(i) * n + j])
                {
                    continue;
                }
                const double rx = map.centerX(i) - p[0], ry = map.centerY(j) - p[1];
                const double r = RowNorm2(rx, ry);
                const double bearing = Wrap(std::atan2(ry, rx) - heading);
                if (r > .3 && r < 2. && std::fabs(bearing) > .4 && std::fabs(bearing) < 1.9)
                {
                    const double s = Sign(bearing);
                    left += s == 1.;
                    right += s == -1.;
                }
            }
        }
        const auto count = [&](double s) { return s > 0 ? left : right; };
        if (left == 0 && right == 0 && seen.empty())
        {
            m_last = {};
            m_last.m_kind = DetourReport::Kind::Nothing;
            return std::nullopt;
        }
        double preferred;
        if (!seen.empty())
        {
            const SideChoice* first = &seen.front();
            for (const SideChoice& entry : seen)
            {
                if (entry.m_score < first->m_score)
                {
                    first = &entry;
                }
            }
            preferred = first->m_side;
        }
        else
        {
            preferred = right > left ? -1. : 1.;
        }
        double side = (m_side && (*m_side == 1. || *m_side == -1.) && count(*m_side)) ? *m_side : preferred;
        if (m_lookHold >= 10)
        {
            side = (count(-side) || Find(seen, -side)) ? -side : side;
            m_lookHold = 0;
        }
        m_side = side;
        const double angle = heading + side * 1.2;
        const Point2 look{p[0] + .6 * std::cos(angle), p[1] + .6 * std::sin(angle)};
        m_last = {};
        m_last.m_kind = DetourReport::Kind::Looking;
        m_last.m_left = side > 0;
        m_last.m_waypoint = look;
        return std::vector<Point2>{p, look};
    }

    [[nodiscard]] std::optional<std::vector<Point2>> astarRoute(const MapView& map, const Point2& p,
                                                                const std::vector<Point2>& route,
                                                                const Tuning& c) const
    {
        if (route.size() < 3)
        {
            return std::nullopt;
        }
        const int n = map.m_n;
        const double offset = map.m_res / std::sqrt(2.);
        std::vector<unsigned char> usable(static_cast<std::size_t>(n) * n);
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                const std::size_t k = static_cast<std::size_t>(i) * n + j;
                const bool ok = (map.m_dist[k] - offset - c.m_radius >= 0.) && (map.m_freeDist[k] - offset >= c.m_radius);
                usable[k] = ok || RowNorm2(map.centerX(i) - p[0], map.centerY(j) - p[1]) <= ASTAR_START_CLEAR_M;
            }
        }
        const double res = map.m_res;
        const auto cell = [&](const Point2& q, int& out) {
            const double x = std::floor(q[0] / res) - map.m_lowerX, y = std::floor(q[1] / res) - map.m_lowerY;
            if (!(x >= 0 && x < n && y >= 0 && y < n))
            {
                return false;
            }
            out = static_cast<int>(x) * n + static_cast<int>(y);
            return usable[static_cast<std::size_t>(out)] != 0;
        };
        int from = 0, to = 0;
        std::vector<int> cells;
        if (cell(p, from) && cell(route[route.size() - 2], to))
        {
            cells = detail::GridAstarCells(usable.data(), n, from, to, ASTAR_SIMPLIFY_M / res);
        }
        if (cells.size() < 2)
        {
            return std::nullopt;
        }
        std::vector<Point2> found;
        for (const int k : cells)
        {
            found.push_back({(k / n + map.m_lowerX + .5) * res, (k % n + map.m_lowerY + .5) * res});
        }
        found[0] = p;
        std::vector<Point2> kept{found[0]};
        for (std::size_t k = 1; k < found.size(); ++k)
        {
            if (RowNorm2(found[k][0] - p[0], found[k][1] - p[1]) >= c.m_gridRouteLookaheadM)
            {
                kept.push_back(found[k]);
            }
        }
        if (kept.size() >= 2)
        {
            found = kept;
        }
        found.push_back(route.back());
        return found;
    }

    [[nodiscard]] std::optional<std::vector<Point2>> gridRoute(const MapView& map, const Point2& p,
                                                               const Point2& target, const Tuning& c,
                                                               double window_m, double clearance_weight) const
    {
        double res = map.m_res;
        const double offset = res / std::sqrt(2.);
        const int fine = map.m_n;
        std::vector<std::uint8_t> ok(static_cast<std::size_t>(fine) * fine);
        std::vector<double> clear(ok.size());
        std::vector<Point2> centers(ok.size());
        for (int i = 0; i < fine; ++i)
        {
            for (int j = 0; j < fine; ++j)
            {
                const std::size_t k = static_cast<std::size_t>(i) * fine + j;
                centers[k] = {map.centerX(i), map.centerY(j)};
                const bool inside = std::fabs(centers[k][0] - p[0]) <= window_m && std::fabs(centers[k][1] - p[1]) <= window_m;
                clear[k] = map.m_dist[k] - offset - c.m_radius;
                ok[k] = inside && clear[k] >= 0. && (map.m_freeDist[k] - offset >= c.m_radius);
            }
        }
        // Search on a ~0.1 m lattice: blocks usable only if all fine cells are.
        const int s = std::max(1, static_cast<int>(std::nearbyint(GRID_ROUTE_CELL_M / res)));
        int n = fine;
        if (s > 1)
        {
            const int m = fine / s;
            std::vector<std::uint8_t> block_ok(static_cast<std::size_t>(m) * m);
            std::vector<double> block_clear(block_ok.size());
            std::vector<Point2> block_centers(block_ok.size());
            for (int bi = 0; bi < m; ++bi)
            {
                for (int bj = 0; bj < m; ++bj)
                {
                    bool all = true;
                    double low = INF;
                    Point2 sum{0., 0.};
                    for (int a = 0; a < s; ++a)
                    {
                        for (int b = 0; b < s; ++b)
                        {
                            const std::size_t k = static_cast<std::size_t>(bi * s + a) * fine + (bj * s + b);
                            all = all && ok[k];
                            low = std::isnan(clear[k]) || clear[k] < low ? clear[k] : low;
                            sum[0] += centers[k][0];
                            sum[1] += centers[k][1];
                        }
                    }
                    const std::size_t q = static_cast<std::size_t>(bi) * m + bj;
                    block_ok[q] = all;
                    block_clear[q] = low;
                    block_centers[q] = {sum[0] / static_cast<double>(s * s), sum[1] / static_cast<double>(s * s)};
                }
            }
            ok.swap(block_ok);
            clear.swap(block_clear);
            centers.swap(block_centers);
            res = res * s;
            n = m;
        }
        std::vector<int> idx;
        std::vector<int> pos(ok.size(), -1);
        for (std::size_t k = 0; k < ok.size(); ++k)
        {
            if (ok[k])
            {
                pos[k] = static_cast<int>(idx.size());
                idx.push_back(static_cast<int>(k));
            }
        }
        if (idx.size() < 4)
        {
            return std::nullopt;
        }
        std::vector<double> penalty(idx.size());
        for (std::size_t a = 0; a < idx.size(); ++a)
        {
            penalty[a] = clearance_weight * NpMax(c.m_margin + .15 - clear[static_cast<std::size_t>(idx[a])], 0.);
        }
        std::vector<int> src, dst;
        std::vector<double> cost;
        constexpr int MOVES[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};
        for (const auto& move : MOVES)
        {
            const int di = move[0], dj = move[1];
            const double step = std::hypot(static_cast<double>(di), static_cast<double>(dj)) * res;
            std::vector<int> ea, eb;
            std::vector<double> ew;
            for (std::size_t a = 0; a < idx.size(); ++a)
            {
                const int ii = idx[a] / n, jj = idx[a] % n, ti = ii + di, tj = jj + dj;
                if (!(ti >= 0 && ti < n && tj >= 0 && tj < n))
                {
                    continue;
                }
                const std::size_t b = static_cast<std::size_t>(ti) * n + tj;
                bool good = ok[b];
                if (di && dj)
                {
                    good = good && ok[static_cast<std::size_t>(ii) * n + tj] && ok[static_cast<std::size_t>(ti) * n + jj];
                }
                if (!good)
                {
                    continue;
                }
                const int pb = pos[b];
                ea.push_back(static_cast<int>(a));
                eb.push_back(pb);
                ew.push_back(step + .5 * step * (penalty[a] + penalty[static_cast<std::size_t>(pb)]));
            }
            src.insert(src.end(), ea.begin(), ea.end());
            src.insert(src.end(), eb.begin(), eb.end());
            dst.insert(dst.end(), eb.begin(), eb.end());
            dst.insert(dst.end(), ea.begin(), ea.end());
            cost.insert(cost.end(), ew.begin(), ew.end());
            cost.insert(cost.end(), ew.begin(), ew.end());
        }
        // (The reference's `if not src` never fires: src is a list of arrays.)
        const auto graph = detail::BuildCsr(static_cast<int>(idx.size()), src, dst, cost);
        std::vector<Point2> cells_xy(idx.size());
        std::vector<double> to_robot(idx.size());
        for (std::size_t a = 0; a < idx.size(); ++a)
        {
            cells_xy[a] = centers[static_cast<std::size_t>(idx[a])];
            to_robot[a] = RowNorm2(cells_xy[a][0] - p[0], cells_xy[a][1] - p[1]);
        }
        const std::size_t start = ArgMin(to_robot);
        if (BlasNorm2(cells_xy[start][0] - p[0], cells_xy[start][1] - p[1]) > 2 * res)
        {
            return std::nullopt;
        }
        std::vector<double> distance;
        std::vector<int> pred;
        detail::Dijkstra(graph, static_cast<int>(start), distance, pred);
        std::vector<double> key(idx.size()), to_target(idx.size());
        for (std::size_t a = 0; a < idx.size(); ++a)
        {
            to_target[a] = RowNorm2(cells_xy[a][0] - target[0], cells_xy[a][1] - target[1]);
            key[a] = std::isfinite(distance[a]) ? to_target[a] + .05 * distance[a] : INF;
        }
        const auto order = detail::ArgSort(key);
        const double direct = BlasNorm2(target[0] - p[0], target[1] - p[1]) - .3;
        std::optional<std::size_t> goal;
        for (std::size_t k = 0; k < order.size() && k < 300; ++k)
        {
            const auto g = static_cast<std::size_t>(order[k]);
            if (!std::isfinite(distance[g]) || to_target[g] > direct)
            {
                break;
            }
            const Point2& w = cells_xy[g];
            const Point2 onward = Sub(target, w);
            const double length = BlasNorm2(onward[0], onward[1]);
            const double scale = PyMin(1., 1. / PyMax(length, 1e-9));
            const Point2 ahead{w[0] + onward[0] * scale, w[1] + onward[1] * scale};
            if (this->clear(map, w, ahead, c.m_radius, c.m_radius))
            {
                goal = g;
                break;
            }
        }
        if (!goal)
        {
            return std::nullopt;
        }
        std::vector<std::size_t> chain{*goal};
        while (chain.back() != start)
        {
            const int previous = pred[chain.back()];
            if (previous < 0 || chain.size() > idx.size())
            {
                throw std::logic_error("grid_route_broken_predecessors");
            }
            chain.push_back(static_cast<std::size_t>(previous));
        }
        std::reverse(chain.begin(), chain.end());
        const std::size_t thin = std::max(2, 4 / s);
        std::vector<std::size_t> keep{0};
        for (std::size_t k = thin; k + 1 < chain.size(); k += thin)
        {
            keep.push_back(k);
        }
        keep.push_back(chain.size() - 1);
        std::vector<Point2> route{p};
        for (std::size_t k = 1; k < keep.size(); ++k)
        {
            route.push_back(cells_xy[chain[keep[k]]]);
        }
        route.push_back(target);
        return route;
    }

    std::optional<std::vector<Point2>> plan(const MapView& map, const std::array<double, 3>& pose,
                                            const Point2& target, const Tuning& c,
                                            const std::optional<std::vector<Point2>>& reference)
    {
        const Point2 p{pose[0], pose[1]};
        const Point2 delta = Sub(target, p);
        const double distance = BlasNorm2(delta[0], delta[1]);
        const double need = c.m_radius + c.m_margin;
        m_last = {};
        m_last.m_kind = DetourReport::Kind::Inactive;
        if (distance < .5)
        {
            m_side.reset();
            return std::nullopt;
        }
        if (m_triggerPath && reference && reference->size() >= 2 && pathClear(map, p, *reference, c.m_radius))
        {
            m_side.reset();
            return std::nullopt;
        }
        const double reach = PyMin(1., m_probeM / distance);
        const Point2 probe{p[0] + delta[0] * reach, p[1] + delta[1] * reach};
        if (clear(map, p, probe, c.m_radius, 0.))
        {
            m_clearCount += 1;
            if (m_held && m_clearCount < m_exitCycles &&
                BlasNorm2((*m_held)[0] - p[0], (*m_held)[1] - p[1]) > .25 && clear(map, p, *m_held, c.m_radius, 0.))
            {
                m_last = {};
                m_last.m_kind = DetourReport::Kind::Holding;
                m_last.m_left = m_side && *m_side != 0. && *m_side > 0;
                m_last.m_waypoint = *m_held;
                return std::vector<Point2>{p, *m_held, target};
            }
            m_side.reset();
            m_held.reset();
            return std::nullopt;
        }
        m_clearCount = 0;
        std::vector<SideChoice> best;
        for (const double standoff : {m_standoffM, 0.})
        {
            best = candidates(map, p, target, delta, distance, c, need, standoff, true);
            if (!best.empty())
            {
                break;
            }
        }
        if (best.empty())
        {
            auto route = gridRoute(map, p, target, c, 3.5, 2.);
            if (route)
            {
                bool astar = false;
                if (c.m_gridRouteAstar)
                {
                    auto refined = astarRoute(map, p, *route, c);
                    if (refined)
                    {
                        route = std::move(refined);
                        astar = true;
                    }
                }
                m_lookHold = 0;
                m_held = (*route)[1];
                m_last = {};
                m_last.m_kind = DetourReport::Kind::Grid;
                m_last.m_waypoint = (*route)[1];
                m_last.m_route = *route;
                m_last.m_astar = astar;
                return route;
            }
            const auto seen = candidates(map, p, target, delta, distance, c, need, 0., false);
            return look(map, p, delta, seen);
        }
        m_lookHold = 0;
        const SideChoice* choice = &best.front();
        for (const SideChoice& entry : best)
        {
            if (entry.m_score < choice->m_score)
            {
                choice = &entry;
            }
        }
        const SideChoice* held = Find(best, m_side);
        if (held && held->m_score <= m_switchRatio * choice->m_score)
        {
            choice = held;
        }
        m_side = choice->m_side;
        m_held = choice->m_point;
        m_last = {};
        m_last.m_kind = DetourReport::Kind::Edge;
        m_last.m_left = choice->m_side > 0;
        m_last.m_waypoint = choice->m_point;
        m_last.m_score = choice->m_score;
        return std::vector<Point2>{p, choice->m_point, target};
    }
};

DetourPlanner::DetourPlanner() : m_impl(std::make_unique<Impl>()) {}
DetourPlanner::~DetourPlanner() = default;

void DetourPlanner::reset()
{
    m_impl = std::make_unique<Impl>();
}

std::optional<std::vector<Point2>> DetourPlanner::plan(const RollingMap& map, const std::array<double, 3>& pose,
                                                       const Point2& target, const Tuning& c,
                                                       const std::optional<std::vector<Point2>>& reference)
{
    return m_impl->plan(MapView(map), pose, target, c, reference);
}

std::optional<std::vector<Point2>> DetourPlanner::gridRoute(const RollingMap& map, const Point2& p,
                                                            const Point2& target, const Tuning& c, double window_m,
                                                            double clearance_weight) const
{
    return m_impl->gridRoute(MapView(map), p, target, c, window_m, clearance_weight);
}

bool DetourPlanner::lookReached(bool reached) noexcept
{
    m_impl->m_lookHold = reached ? m_impl->m_lookHold + 1 : 0;
    return reached;
}

const DetourReport& DetourPlanner::last() const noexcept
{
    return m_impl->m_last;
}

void DetourPlanner::setTrigger(bool path) noexcept
{
    m_impl->m_triggerPath = path;
}

// ---------------------------------------------------------------- HybridFollower

struct HybridFollower::Impl
{
    DetourPlanner m_detour;
    int m_stuckCycles{0};
    std::vector<Point2> m_trail;
    double m_backed{0.};
    std::optional<Point2> m_lastPose;
    bool m_reversing{false};
    std::optional<std::pair<double, Point2>> m_backCheck;
    bool m_lastInfeasible{false};
    CandidateReport m_lastCandidates;
    std::vector<Point2> m_lastReferencePath;

    void remember(const std::array<double, 3>& pose)
    {
        const Point2 p{pose[0], pose[1]};
        if (m_reversing && m_lastPose)
        {
            m_backed += BlasNorm2(p[0] - (*m_lastPose)[0], p[1] - (*m_lastPose)[1]);
        }
        m_lastPose = p;
        if (m_reversing)
        {
            m_reversing = false;
            return;
        }
        if (m_trail.empty() || BlasNorm2(p[0] - m_trail.back()[0], p[1] - m_trail.back()[1]) >= .05)
        {
            m_trail.push_back(p);
            while (m_trail.size() > 60)
            {
                m_trail.erase(m_trail.begin());
            }
        }
    }

    std::optional<Point2> backup(const MapView& map, const std::array<double, 3>& pose, const Point2& previous,
                                 const Tuning& c, std::optional<double> now)
    {
        const Point2 p{pose[0], pose[1]};
        if (m_backed >= c.m_backupMaxM || m_trail.size() < 3)
        {
            return std::nullopt;
        }
        std::optional<Point2> goal;
        for (std::size_t k = m_trail.size() - 1; k-- > 0;)
        {
            if (BlasNorm2(m_trail[k][0] - p[0], m_trail[k][1] - p[1]) >= .3)
            {
                goal = m_trail[k];
                break;
            }
        }
        if (!goal)
        {
            return std::nullopt;
        }
        const Point2 d = Sub(*goal, p);
        std::vector<double> clear;
        for (const double u : Linspace(0., 1., 8))
        {
            clear.push_back(map.query(p[0] + u * d[0], p[1] + u * d[1]));
        }
        const double rear = pose[2] + std::numbers::pi;
        const Point2 back_dir{std::cos(rear), std::sin(rear)};
        const auto behind = Linspace(0., .3, 7);
        for (std::size_t k = 1; k < behind.size(); ++k)
        {
            clear.push_back(map.query(p[0] + behind[k] * back_dir[0], p[1] + behind[k] * back_dir[1]));
        }
        double lowest = clear[1];
        for (std::size_t k = 1; k < clear.size(); ++k)
        {
            lowest = (clear[k] < lowest || std::isnan(clear[k])) ? clear[k] : lowest;
        }
        if (lowest < PyMin(c.m_radius + .02, clear[0]))
        {
            return std::nullopt;
        }
        const double err = Wrap(std::atan2((*goal)[1] - p[1], (*goal)[0] - p[0]) - rear);
        if (now)
        {
            if (!m_backCheck || BlasNorm2(p[0] - m_backCheck->second[0], p[1] - m_backCheck->second[1]) >= .05)
            {
                m_backCheck.emplace(*now, p);
            }
            else if (*now - m_backCheck->first >= c.m_backupStallS)
            {
                m_backed = c.m_backupMaxM;
                m_backCheck.reset();
                return std::nullopt;
            }
        }
        double w = Clip(1.5 * err, -.5, .5);
        const double v = -PyMax(c.m_backupMinSpeed, c.m_backupSpeed * std::cos(err));
        w = Clip(w, previous[1] - c.m_yawAccel * c.m_step, previous[1] + c.m_yawAccel * c.m_step);
        m_reversing = true;
        m_lastCandidates = {};
        m_lastCandidates.m_kind = CandidateReport::Kind::Backing;
        m_lastCandidates.m_backed = m_backed;
        m_lastCandidates.m_hasDetour = true;
        m_lastCandidates.m_detour = m_detour.last();
        m_lastReferencePath = {p, *goal};
        return Point2{v, w};
    }

    [[nodiscard]] std::vector<Point2> roundCorners(const MapView& map, const std::vector<Point2>& path,
                                                   const Tuning& c) const
    {
        if (c.m_referenceCornerM <= 0 || path.size() < 3)
        {
            return path;
        }
        std::vector<double> heading;
        for (std::size_t k = 0; k + 1 < path.size(); ++k)
        {
            heading.push_back(std::atan2(path[k + 1][1] - path[k][1], path[k + 1][0] - path[k][0]));
        }
        double turn = -INF;
        for (std::size_t k = 0; k + 1 < heading.size(); ++k)
        {
            const double a = std::fabs(Wrap(heading[k + 1] - heading[k]));
            turn = (a > turn || std::isnan(a)) ? a : turn;
        }
        if (turn < 10. * (std::numbers::pi / 180.))
        {
            return path;
        }
        double lowest = INF;
        bool first = true;
        const auto take = [&](double x, double y) {
            const double q = map.query(x, y);
            lowest = first || q < lowest || std::isnan(q) ? q : lowest;
            first = false;
        };
        for (std::size_t i = 0; i + 1 < path.size(); ++i)
        {
            const Point2 d = Sub(path[i + 1], path[i]);
            const double length = RowNorm2(d[0], d[1]);
            const auto u = Linspace(0., 1., std::max<std::size_t>(2, static_cast<std::size_t>(std::ceil(length / .05)) + 1));
            for (std::size_t k = 0; k + 1 < u.size(); ++k)
            {
                take(path[i][0] + u[k] * d[0], path[i][1] + u[k] * d[1]);
            }
        }
        take(path.back()[0], path.back()[1]);
        const double need = PyMin(c.m_radius + c.m_margin, lowest);
        for (const double scale : {1., .6, .3})
        {
            const auto curve = BsplineCorners(path, c.m_referenceCornerM * scale, .05);
            if (curve.size() >= 2)
            {
                double low = INF;
                bool start = true;
                for (const Point2& q : curve)
                {
                    const double v = map.query(q[0], q[1]);
                    low = start || v < low || std::isnan(v) ? v : low;
                    start = false;
                }
                if (low >= need - 1e-3)
                {
                    return curve;
                }
            }
        }
        return path;
    }

    [[nodiscard]] std::vector<Point2> shortcut(const MapView& map, const std::array<double, 3>& pose,
                                               const std::vector<Point2>& path, const Tuning& c) const
    {
        const Point2 p{pose[0], pose[1]};
        for (std::size_t j = path.size() - 1; j > 0; --j)
        {
            const Point2 d = Sub(path[j], p);
            const auto n = std::max<std::size_t>(2, static_cast<std::size_t>(std::ceil(BlasNorm2(d[0], d[1]) / .05)) + 1);
            const auto u = Linspace(0., 1., n);
            double low = INF;
            for (std::size_t k = 1; k < u.size(); ++k)
            {
                const double v = map.query(p[0] + u[k] * d[0], p[1] + u[k] * d[1], true);
                low = k == 1 || v < low || std::isnan(v) ? v : low;
            }
            if (low >= c.m_radius + .05)
            {
                std::vector<Point2> out{p};
                out.insert(out.end(), path.begin() + static_cast<std::ptrdiff_t>(j), path.end());
                return out;
            }
        }
        return path;
    }

    Point2 recover(const MapView& map, const std::array<double, 3>& pose, const Point2& previous, const Point2& target,
                   const Tuning& c, std::optional<double> now)
    {
        if (const auto back = backup(map, pose, previous, c, now))
        {
            return *back;
        }
        m_lastCandidates = {};
        m_lastCandidates.m_kind = CandidateReport::Kind::Recovery;
        m_lastCandidates.m_backed = m_backed;
        const double err = Wrap(std::atan2(target[1] - pose[1], target[0] - pose[0]) - pose[2]);
        if (std::fabs(err) <= .15)
        {
            return {0., 0.};
        }
        const double w = Clip(1.5 * err, -.6, .6);
        return {0., Clip(w, previous[1] - c.m_yawAccel * c.m_step, previous[1] + c.m_yawAccel * c.m_step)};
    }

    Point2 command(const RollingMap& raw, const MapView& map, const Tuning& tuning, const CommandInput& in);
};


namespace {

// np.interp(x, xp, fp) (numpy 1.26 arr_interp, unfused, measured on the Orin).
[[nodiscard]] double Interp(double x, const std::vector<double>& xp, const std::vector<double>& fp)
{
    if (std::isnan(x))
    {
        return x;
    }
    const std::size_t len = xp.size();
    if (x > xp[len - 1])
    {
        return fp[len - 1];
    }
    if (x < xp[0])
    {
        return fp[0];
    }
    // binary_search_with_guess: the largest j with xp[j] <= x.
    const auto j = static_cast<std::size_t>(std::upper_bound(xp.begin(), xp.end(), x) - xp.begin()) - 1;
    if (j == len - 1 || xp[j] == x)
    {
        return fp[j];
    }
    const double slope = (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j]);
    double result = slope * (x - xp[j]) + fp[j];
    if (std::isnan(result))
    {
        result = slope * (x - xp[j + 1]) + fp[j + 1];
        if (std::isnan(result) && fp[j] == fp[j + 1])
        {
            result = fp[j];
        }
    }
    return result;
}

// np.min reduction step: NaN propagates.
[[nodiscard]] double NanMin(double acc, double v) noexcept
{
    return (std::isnan(acc) || v < acc || std::isnan(v)) ? (std::isnan(acc) ? acc : v) : acc;
}

constexpr std::size_t SPEED_COUNT = 13;
constexpr std::size_t OFFSET_COUNT = 25;
constexpr std::size_t ROLLOUTS = SPEED_COUNT * OFFSET_COUNT;

} // namespace

Point2 HybridFollower::Impl::command(const RollingMap& raw, const MapView& map, const Tuning& tuning,
                                     const CommandInput& in)
{
    Tuning c = tuning;
    const auto& pose = in.m_pose;
    const Point2 robot{pose[0], pose[1]};
    const Point2& previous = in.m_previous;
    remember(pose);
    const bool searching = in.m_status == "search_last_seen" || in.m_status == "search_heading";
    if (searching)
    {
        const double arrival = in.m_policy ? in.m_policy->m_searchArrivalM : .35;
        c.m_desiredGap = arrival;
        c.m_targetStopGap = arrival;
        c.m_targetHardGap = 0.;
        c.m_targetClearanceMax = 0.;
        c.m_maxV = PyMin(c.m_maxV, c.m_searchSpeed);
    }
    m_lastInfeasible = false;
    m_lastCandidates = {};
    m_lastReferencePath.clear();
    if (!in.m_target || in.m_status == "lost" || in.m_status == "binding" || in.m_status == "holding")
    {
        return {0., 0.};
    }
    const Point2 target = *in.m_target;
    if (in.m_recovery)
    {
        return recover(map, pose, previous, target, c, in.m_policyTime);
    }
    const bool has_reference = in.m_referencePath.has_value();
    std::vector<Point2> path = has_reference ? *in.m_referencePath : BezierReference(in.m_history);
    bool detouring = false;
    if (!has_reference && !searching)
    {
        const auto detour = m_detour.plan(raw, pose, target, c, path);
        if (detour)
        {
            path = *detour;
            detouring = true;
        }
        const bool looking = detour && m_detour.last().looking() && m_stuckCycles >= 3;
        if (looking)
        {
            if (const auto back = backup(map, pose, previous, c, in.m_policyTime))
            {
                return *back;
            }
            const Point2& aim = (*detour)[1];
            const double err = Wrap(std::atan2(aim[1] - pose[1], aim[0] - pose[0]) - pose[2]);
            if (!m_detour.lookReached(std::fabs(err) <= .15))
            {
                double w = Clip(1.5 * err, -.6, .6);
                w = Clip(w, previous[1] - c.m_yawAccel * c.m_step, previous[1] + c.m_yawAccel * c.m_step);
                m_lastCandidates = {};
                m_lastCandidates.m_kind = CandidateReport::Kind::Looking;
                m_lastCandidates.m_hasDetour = true;
                m_lastCandidates.m_detour = m_detour.last();
                m_lastReferencePath = *detour;
                return {0., w};
            }
        }
    }
    const bool detour_looking = m_detour.last().looking();
    if ((!detouring || !detour_looking) && m_stuckCycles == 0)
    {
        m_backed = 0.;
        m_backCheck.reset();
    }
    if (path.empty())
    {
        path = {target};
    }
    if (BlasNorm2(path.back()[0] - target[0], path.back()[1] - target[1]) > .01)
    {
        path.push_back(target);
    }
    if (!has_reference && !detouring && !searching && path.size() >= 2)
    {
        path = shortcut(map, pose, path, c);
    }
    if (path.size() < 2)
    {
        path.insert(path.begin(), robot);
    }
    const std::vector<Point2> corners = path;
    path = roundCorners(map, path, c);
    {
        // fraction / nearest: prepend the robot when it is behind the first point.
        std::vector<double> fraction(path.size() - 1), norms(path.size() - 1);
        for (std::size_t i = 0; i + 1 < path.size(); ++i)
        {
            const Point2 e = Sub(path[i + 1], path[i]);
            const Point2 rel = Sub(robot, path[i]);
            fraction[i] = Clip((rel[0] * e[0] + rel[1] * e[1]) / NpMax(e[0] * e[0] + e[1] * e[1], 1e-12), 0., 1.);
            norms[i] = RowNorm2(path[i][0] + fraction[i] * e[0] - robot[0], path[i][1] + fraction[i] * e[1] - robot[1]);
        }
        if (ArgMin(norms) == 0 && fraction[0] <= 1e-6 && BlasNorm2(path[0][0] - robot[0], path[0][1] - robot[1]) > .03)
        {
            path.insert(path.begin(), robot);
        }
    }
    m_lastReferencePath = path;
    const std::size_t points = path.size();
    std::vector<double> lengths(points - 1), arcs(points, 0.), px(points), py(points);
    for (std::size_t i = 0; i < points; ++i)
    {
        px[i] = path[i][0];
        py[i] = path[i][1];
    }
    for (std::size_t i = 0; i + 1 < points; ++i)
    {
        lengths[i] = RowNorm2(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]);
        arcs[i + 1] = arcs[i] + lengths[i];
    }
    std::vector<double> alpha(points - 1), norms(points - 1);
    for (std::size_t i = 0; i + 1 < points; ++i)
    {
        const Point2 seg = Sub(path[i + 1], path[i]);
        const Point2 rel = Sub(robot, path[i]);
        alpha[i] = Clip((rel[0] * seg[0] + rel[1] * seg[1]) / NpMax(lengths[i] * lengths[i], 1e-12), 0., 1.);
        norms[i] = RowNorm2(path[i][0] + alpha[i] * seg[0] - robot[0], path[i][1] + alpha[i] * seg[1] - robot[1]);
    }
    const std::size_t idx = ArgMin(norms);
    const double s0 = arcs[idx] + alpha[idx] * lengths[idx];
    const Point2& velocity = in.m_velocity;
    const double target_speed = BlasNorm2(velocity[0], velocity[1]);
    double lateral = 0.;
    if (!has_reference && !searching && !detouring)
    {
        const double bearing = std::fabs(Wrap(std::atan2(target[1] - pose[1], target[0] - pose[0]) - pose[2]));
        lateral = Clip((bearing - c.m_lateralBearingStart) / PyMax(c.m_fovSoftAngle - c.m_lateralBearingStart, 1e-6), 0., 1.);
    }
    double cap = c.m_maxV;
    if (!has_reference && !searching)
    {
        const Point2 aim = detouring && corners.size() > 2 && !m_detour.last().looking() ? corners[1] : target;
        const double off = std::fabs(Wrap(std::atan2(aim[1] - pose[1], aim[0] - pose[0]) - pose[2]));
        cap = c.m_maxV * Clip((c.m_turnFirstAngle - off) / PyMax(c.m_turnFirstAngle - c.m_turnSlowAngle, 1e-6), 0., 1.);
    }
    const double to_target = BlasNorm2(target[0] - robot[0], target[1] - robot[1]);
    const double remaining = PyMin(arcs.back() - s0, to_target);
    const double cruise = Clip(target_speed + .65 * (remaining - c.m_desiredGap), 0.,
                               PyMin(c.m_maxV * (1. - c.m_lateralSlowdown * lateral), cap));
    const auto horizon = static_cast<std::size_t>(c.m_horizon);
    std::vector<double> times(horizon);
    std::vector<Point2> refs(horizon);
    const double speed_scale = PyMax(target_speed, 1e-9);
    for (std::size_t k = 0; k < horizon; ++k)
    {
        times[k] = static_cast<double>(k + 1) * c.m_step;
        const double ref_s = NpMax(s0, NpMin(s0 + cruise * times[k], arcs.back() + target_speed * times[k] - c.m_desiredGap));
        refs[k] = {Interp(ref_s, arcs, px), Interp(ref_s, arcs, py)};
        const double beyond = NpMax(ref_s - arcs.back(), 0.);
        refs[k][0] = refs[k][0] + beyond * velocity[0] / speed_scale;
        refs[k][1] = refs[k][1] + beyond * velocity[1] / speed_scale;
    }
    // meshgrid(speed_list, offsets).ravel(): rollout i * 13 + j.
    std::array<double, SPEED_COUNT> speed_list{-.25, -.15, -.075};
    {
        const auto ramp = Linspace(0., c.m_maxV, 10);
        std::copy(ramp.begin(), ramp.end(), speed_list.begin() + 3);
    }
    const auto offset_list = Linspace(-1.2, 1.2, OFFSET_COUNT);
    std::array<double, ROLLOUTS> speeds{}, offsets{};
    for (std::size_t i = 0; i < OFFSET_COUNT; ++i)
    {
        for (std::size_t j = 0; j < SPEED_COUNT; ++j)
        {
            speeds[i * SPEED_COUNT + j] = speed_list[j];
            offsets[i * SPEED_COUNT + j] = offset_list[i];
        }
    }
    if (lateral > 0.)
    {
        const double limit = c.m_maxV * (1. - c.m_lateralSlowdown * lateral);
        for (double& v : speeds)
        {
            v = NpMin(v, limit);
        }
    }
    for (double& v : speeds)
    {
        v = NpMin(v, cap);
    }
    if (cap <= 1e-6 && remaining > c.m_desiredGap)
    {
        for (double& v : speeds)
        {
            v = NpMax(v, 0.);
        }
    }
    if (!searching && c.m_targetClearanceMax > 0 && BlasNorm2(velocity[0], velocity[1]) < .05 &&
        c.m_targetHardGap + .01 <= to_target && to_target <= c.m_radius + c.m_targetRadius + c.m_targetClearanceMax)
    {
        speeds.fill(0.);
    }
    std::optional<double> search_heading;
    const PolicyView* policy = in.m_policy ? &*in.m_policy : nullptr;
    if (in.m_status == "search_heading" && policy && policy->m_searchHeading)
    {
        if (!in.m_policyTime && policy->m_searchStarted)
        {
            throw std::invalid_argument("search_heading_needs_policy_time");
        }
        const double phase = policy->m_searchStarted ? PyMax(0., *in.m_policyTime - *policy->m_searchStarted) : 0.;
        const double onward = PyMax(0., phase - c.m_searchExitTurn / c.m_searchTurnRate) * c.m_searchTurnRate;
        search_heading = *policy->m_searchHeading + policy->m_searchSide * (c.m_searchExitTurn + onward);
    }
    const bool policy_stop = policy && !policy->m_translationAllowed;
    const bool hold_turn = policy_stop && !policy->m_activeSearch;
    if (policy_stop)
    {
        speeds.fill(0.);
    }
    std::array<double, ROLLOUTS> sx, sy, sth, cv, cw, av, aw, score{}, first_v{}, first_w{};
    std::array<bool, ROLLOUTS> safe, known_ok, static_ok, people_ok, near_fov;
    const Point2 seed = in.m_measured ? *in.m_measured : previous;
    sx.fill(pose[0]);
    sy.fill(pose[1]);
    sth.fill(pose[2]);
    cv.fill(previous[0]);
    cw.fill(previous[1]);
    av.fill(seed[0]);
    aw.fill(seed[1]);
    safe.fill(true);
    known_ok.fill(true);
    static_ok.fill(true);
    people_ok.fill(true);
    near_fov.fill(true);
    const double apf = ApproachTangent(pose, previous, in.m_obstacles, target, c.m_radius);
    // Inside the obstacle band a rollout may not get closer than the best it has
    // reached, and one that moves must leave the band (and unknown space) within
    // the horizon; beyond ESCAPE_SLACK_M outside the band nothing changes.
    const double start_clear = map.query(pose[0], pose[1]) - c.m_radius - .001;
    const double start_unknown = map.query(pose[0], pose[1], true) - c.m_radius - .001;
    std::array<double, ROLLOUTS> floor_static, floor_unknown;
    floor_static.fill(PyMin(0., start_clear));
    floor_unknown.fill(PyMin(0., start_unknown));
    const double slack = start_clear < ESCAPE_SLACK_M ? ESCAPE_SLACK_M : 0.;
    const double slack_unknown = start_unknown < ESCAPE_SLACK_M ? ESCAPE_SLACK_M : 0.;
    std::array<bool, ROLLOUTS> left, left_unknown;
    left.fill(start_clear >= 0.);
    left_unknown.fill(start_unknown >= 0.);
    const double accel_step = c.m_accel * c.m_step, yaw_step = c.m_yawAccel * c.m_step;
    const double tau_w_gain = PyMin(1., c.m_step / c.m_chassisTauW);
    const double tau_v_gain = PyMin(1., c.m_step / PyMax(c.m_chassisTauV, 1e-6));
    const double known_until = PyMax(.5, std::fabs(previous[0]) / c.m_accel + c.m_step);
    const double visible = c.m_cameraHalfAngle - c.m_fovEdgeMargin;
    const double track_weight = 6 * (1. - .75 * lateral);
    const double blend = .25 + .75 * lateral;
    const double two_pi = 2 * std::numbers::pi;
    const auto& obstacles = in.m_obstacles;
    for (std::size_t k = 0; k < horizon; ++k)
    {
        const double tau = times[k];
        const Point2 goal{target[0] + velocity[0] * tau, target[1] + velocity[1] * tau};
        const double look_s = PyMin(arcs.back(), s0 + cruise * tau + .7);
        const Point2 look{Interp(look_s, arcs, px), Interp(look_s, arcs, py)};
        const Point2 bearing_goal = in.m_facePoint ? *in.m_facePoint : goal;
        const double decay = apf * std::exp(-tau);
        for (std::size_t r = 0; r < ROLLOUTS; ++r)
        {
            const double route_heading = std::atan2(look[1] - sy[r], look[0] - sx[r]);
            const double target_heading = std::atan2(goal[1] - sy[r], goal[0] - sx[r]);
            double heading = route_heading + blend * Wrap(target_heading - route_heading);
            if (has_reference)
            {
                heading = route_heading;
                if (in.m_facePoint)
                {
                    const double face = std::atan2((*in.m_facePoint)[1] - sy[r], (*in.m_facePoint)[0] - sx[r]);
                    heading = RowNorm2(goal[0] - sx[r], goal[1] - sy[r]) < .12 ? face : heading;
                }
            }
            heading = heading + decay;
            double want_w = Clip(2. * Wrap(heading - sth[r]) + offsets[r], -c.m_maxW, c.m_maxW);
            if (search_heading)
            {
                want_w = Clip(2. * Wrap(*search_heading - sth[r]) + .25 * offsets[r], -c.m_searchTurnRate, c.m_searchTurnRate);
            }
            if (hold_turn)
            {
                want_w = 0.;
            }
            const double desired_v = NpMin(speeds[r], NpMax(0., .8 * (RowNorm2(goal[0] - sx[r], goal[1] - sy[r]) - c.m_targetStopGap)));
            const double old_v = cv[r], old_w = cw[r];
            cv[r] = cv[r] + Clip(desired_v - cv[r], -accel_step, accel_step);
            cw[r] = cw[r] + Clip(want_w - cw[r], -yaw_step, yaw_step);
            if (policy_stop)
            {
                cv[r] = 0.;
                if (hold_turn)
                {
                    cw[r] = 0.;
                }
            }
            if (k == 0)
            {
                first_v[r] = cv[r];
                first_w[r] = cw[r];
            }
            if (c.m_chassisTauW > 0)
            {
                aw[r] = aw[r] + (ChassisYaw(cw[r], c) - aw[r]) * tau_w_gain;
                av[r] = av[r] + (cv[r] - av[r]) * tau_v_gain;
            }
            else
            {
                av[r] = cv[r];
                aw[r] = cw[r];
            }
            const double ox = sx[r], oy = sy[r];
            const double turn = aw[r] * c.m_step;
            const double distance = av[r] * c.m_step * detail::Sinc(turn / two_pi);
            sx[r] = sx[r] + distance * std::cos(sth[r] + turn / 2);
            sy[r] = sy[r] + distance * std::sin(sth[r] + turn / 2);
            sth[r] = sth[r] + turn;
            const double mx = (ox + sx[r]) * .5, my = (oy + sy[r]) * .5;
            const double q0 = map.query(ox, oy), q1 = map.query(mx, my), q2 = map.query(sx[r], sy[r]);
            const double u0 = map.query(ox, oy, true), u1 = map.query(mx, my, true), u2 = map.query(sx[r], sy[r], true);
            double clearance = NanMin(NanMin(q0, q1), q2) - c.m_radius - .001;
            const double unknown = NanMin(NanMin(u0, u1), u2) - c.m_radius - .001;
            const bool stationary = RowNorm2(sx[r] - pose[0], sy[r] - pose[1]) < 1e-8;
            if (tau <= known_until)
            {
                const bool allowed = unknown >= floor_unknown[r] - slack_unknown || stationary;
                safe[r] = safe[r] && allowed;
                known_ok[r] = known_ok[r] && allowed;
                floor_unknown[r] = NpMin(0., NpMax(floor_unknown[r], unknown));
            }
            left_unknown[r] = left_unknown[r] || unknown >= 0.;
            const bool static_safe = clearance >= floor_static[r] - slack;
            safe[r] = safe[r] && static_safe;
            static_ok[r] = static_ok[r] && static_safe;
            floor_static[r] = NpMin(0., NpMax(floor_static[r], clearance));
            left[r] = left[r] || clearance >= 0.;
            const double m0 = NpMax(c.m_margin - clearance, 0.);
            score[r] = score[r] + c.m_marginWeight * (m0 * m0);
            const double m1 = NpMax(.15 - unknown, 0.);
            score[r] = score[r] + 15 * (m1 * m1);
            if (!obstacles.empty())
            {
                double person_clear = 0.;
                for (std::size_t o = 0; o < obstacles.size(); ++o)
                {
                    const Obstacle& ob = obstacles[o];
                    const double before = tau - c.m_step;
                    const Point2 d0{ox - (ob.m_position[0] + before * ob.m_velocity[0]),
                                    oy - (ob.m_position[1] + before * ob.m_velocity[1])};
                    const Point2 d1{sx[r] - (ob.m_position[0] + tau * ob.m_velocity[0]),
                                    sy[r] - (ob.m_position[1] + tau * ob.m_velocity[1])};
                    const Point2 delta{d1[0] - d0[0], d1[1] - d0[1]};
                    const double u = Clip(-(d0[0] * delta[0] + d0[1] * delta[1]) /
                                              NpMax(delta[0] * delta[0] + delta[1] * delta[1], 1e-12),
                                          0., 1.);
                    const double value = RowNorm2(d0[0] + u * delta[0], d0[1] + u * delta[1]) - ob.m_radius -
                                         c.m_radius - .025 * tau;
                    person_clear = o == 0 ? value : NanMin(person_clear, value);
                }
                const bool person_safe = person_clear >= c.m_margin;
                safe[r] = safe[r] && person_safe;
                people_ok[r] = people_ok[r] && person_safe;
                clearance = NpMin(clearance, person_clear);
            }
            double angle = Wrap(std::atan2(bearing_goal[1] - sy[r], bearing_goal[0] - sx[r]) - sth[r]);
            if (search_heading)
            {
                angle = Wrap(*search_heading - sth[r]);
            }
            if (tau <= .6 && !detouring)
            {
                near_fov[r] = near_fov[r] && std::fabs(angle) <= visible;
            }
            const double gap = RowNorm2(goal[0] - sx[r], goal[1] - sy[r]);
            safe[r] = safe[r] && gap >= c.m_targetHardGap;
            const double ex = sx[r] - refs[k][0], ey = sy[r] - refs[k][1];
            const double g = gap - c.m_desiredGap;
            score[r] = score[r] + (track_weight * (ex * ex + ey * ey) + 2 * (g * g));
            const double soft = NpMax(std::fabs(angle) - c.m_fovSoftAngle, 0.);
            score[r] = score[r] + (c.m_centeringWeight * (angle * angle) + 60 * (soft * soft));
            const double close = NpMax(.35 - clearance, 0.);
            score[r] = score[r] + (2 * (close * close) + .15 * (cw[r] * cw[r]));
            const double dv = cv[r] - old_v, dw = cw[r] - old_w, back = NpMax(-cv[r], 0.);
            score[r] = score[r] + (.25 * (dv * dv + dw * dw) + 4 * (back * back));
        }
    }
    std::array<bool, ROLLOUTS> turning;
    for (std::size_t r = 0; r < ROLLOUTS; ++r)
    {
        turning[r] = RowNorm2(sx[r] - pose[0], sy[r] - pose[1]) < 1e-8;
        known_ok[r] = known_ok[r] && (left_unknown[r] || turning[r]);
        static_ok[r] = static_ok[r] && (left[r] || turning[r]);
        safe[r] = safe[r] && (left[r] || turning[r]) && (left_unknown[r] || turning[r]);
    }
    std::array<bool, ROLLOUTS> feasible;
    bool any = false;
    for (std::size_t r = 0; r < ROLLOUTS; ++r)
    {
        feasible[r] = safe[r] && near_fov[r];
        any = any || feasible[r];
    }
    if (!any)
    {
        feasible = safe;
    }
    CandidateReport report;
    report.m_kind = CandidateReport::Kind::Sampled;
    any = false;
    for (std::size_t r = 0; r < ROLLOUTS; ++r)
    {
        const bool forward = first_v[r] > .01;
        report.m_safe += safe[r];
        report.m_feasible += feasible[r];
        report.m_forward += feasible[r] && forward;
        report.m_reasons[0] += known_ok[r];
        report.m_reasons[1] += static_ok[r];
        report.m_reasons[2] += people_ok[r];
        report.m_forwardTotal += forward;
        report.m_forwardKnown += forward && known_ok[r];
        report.m_forwardStatic += forward && static_ok[r];
        report.m_forwardPeople += forward && people_ok[r];
        any = any || feasible[r];
    }
    m_stuckCycles = report.m_forward == 0 ? m_stuckCycles + 1 : 0;
    if (m_detour.last().active())
    {
        report.m_hasDetour = true;
        report.m_detour = m_detour.last();
    }
    m_lastCandidates = report;
    if (!any)
    {
        m_lastInfeasible = true;
        if (m_stuckCycles >= 3 && !has_reference && !searching)
        {
            if (const auto back = backup(map, pose, previous, c, in.m_policyTime))
            {
                m_lastInfeasible = false;
                return *back;
            }
        }
        return {0., 0.};
    }
    bool moving = false;
    for (std::size_t r = 0; r < ROLLOUTS; ++r)
    {
        moving = moving || (feasible[r] && !turning[r]);
    }
    if (start_clear < 0. && !moving && m_stuckCycles >= 3 && !has_reference && !searching)
    {
        // Inside the band and only turning in place is left: back out along the trail.
        if (const auto back = backup(map, pose, previous, c, in.m_policyTime))
        {
            return *back;
        }
    }
    std::vector<double> masked(ROLLOUTS);
    for (std::size_t r = 0; r < ROLLOUTS; ++r)
    {
        masked[r] = feasible[r] ? score[r] : INF;
    }
    const std::size_t best = ArgMin(masked);
    return {first_v[best], first_w[best]};
}

HybridFollower::HybridFollower() : m_impl(std::make_unique<Impl>()) {}
HybridFollower::~HybridFollower() = default;

void HybridFollower::reset()
{
    // Like the reference: detour, stuck, trail and backing state; the last_*
    // diagnostics stay until the next command.
    m_impl->m_detour.reset();
    m_impl->m_stuckCycles = 0;
    m_impl->m_trail.clear();
    m_impl->m_backed = 0.;
    m_impl->m_lastPose.reset();
    m_impl->m_reversing = false;
    m_impl->m_backCheck.reset();
}

Point2 HybridFollower::command(const RollingMap& map, const Tuning& c, const CommandInput& input)
{
    return m_impl->command(map, MapView(map), c, input);
}

bool HybridFollower::lastInfeasible() const noexcept
{
    return m_impl->m_lastInfeasible;
}

const CandidateReport& HybridFollower::lastCandidates() const noexcept
{
    return m_impl->m_lastCandidates;
}

const std::vector<Point2>& HybridFollower::lastReferencePath() const noexcept
{
    return m_impl->m_lastReferencePath;
}

int HybridFollower::stuckCycles() const noexcept
{
    return m_impl->m_stuckCycles;
}

double HybridFollower::backed() const noexcept
{
    return m_impl->m_backed;
}

const std::vector<Point2>& HybridFollower::trail() const noexcept
{
    return m_impl->m_trail;
}

DetourPlanner& HybridFollower::detour() noexcept
{
    return m_impl->m_detour;
}

} // namespace wla::follow
