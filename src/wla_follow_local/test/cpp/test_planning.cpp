// Native checks of the planning pieces that need no Python reference.
// Bit equality with the Python reference (numpy argsort, scipy dijkstra, the
// MPC, detour and approach goal) is tested in test_native_planning.py.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include <wla_follow/planning.h>

#include "../../src/numpy_ops.h"

namespace {

using wla::follow::Point2;
namespace detail = wla::follow::detail;

int g_failures = 0;

void Check(bool condition, const char* what, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAILED line %d: %s\n", line, what);
        ++g_failures;
    }
}

#define CHECK(x) Check((x), #x, __LINE__)

template <typename F>
bool Throws(F&& f)
{
    try
    {
        f();
    }
    catch (const std::exception&)
    {
        return true;
    }
    return false;
}

void TestArgSortOrders()
{
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> small(0, 6);
    for (int trial = 0; trial < 300; ++trial)
    {
        std::vector<double> v(static_cast<std::size_t>(1 + trial % 200));
        for (double& x : v)
        {
            x = trial % 3 ? small(rng) : std::uniform_real_distribution<double>(-1, 1)(rng);
        }
        if (trial % 7 == 0)
        {
            v[0] = std::numeric_limits<double>::quiet_NaN();
        }
        const auto order = detail::ArgSort(v);
        std::vector<int> seen(v.size(), 0);
        bool sorted = true;
        for (std::size_t k = 0; k < order.size(); ++k)
        {
            seen[static_cast<std::size_t>(order[k])] += 1;
            if (k && detail::sort::Less(v[static_cast<std::size_t>(order[k])], v[static_cast<std::size_t>(order[k - 1])]))
            {
                sorted = false;
            }
        }
        CHECK(sorted);
        CHECK(std::all_of(seen.begin(), seen.end(), [](int s) { return s == 1; }));
        if (trial % 7 == 0 && v.size() > 1)
        {
            CHECK(order.back() == 0);  // NaN sorts last
        }
    }
    // Below 16 elements numpy uses insertion sort: ties keep index order.
    const auto ties = detail::ArgSort({2., 1., 2., 1., 0.});
    CHECK((ties == std::vector<std::int64_t>{4, 1, 3, 0, 2}));
}

void TestDijkstraOnSmallGraphs()
{
    // 0 -> 1 -> 2 (1 + 1) beats 0 -> 2 (3); duplicate 1 -> 2 edges are summed.
    const auto graph = detail::BuildCsr(4, {0, 1, 0, 1}, {1, 2, 2, 2}, {1., .5, 3., .5});
    CHECK((graph.m_indptr == std::vector<int>{0, 2, 3, 3, 3}));
    CHECK((graph.m_indices == std::vector<int>{1, 2, 2}));
    std::vector<double> distance;
    std::vector<int> pred;
    detail::Dijkstra(graph, 0, distance, pred);
    CHECK(distance[0] == 0. && distance[1] == 1. && distance[2] == 2.);
    CHECK(std::isinf(distance[3]) && pred[3] == -9999 && pred[0] == -9999);
    CHECK(pred[1] == 0 && pred[2] == 1);
    CHECK(Throws([&] { detail::Dijkstra(graph, 4, distance, pred); }));
    CHECK(Throws([] { (void)detail::BuildCsr(2, {0}, {2}, {1.}); }));
}

void TestLinspaceKeepsEnds()
{
    const auto u = detail::Linspace(.1, .7, 7);
    CHECK(u.front() == .1 && u.back() == .7);
    const auto p = detail::Linspace2({0., 1.}, {.3, 1.}, 4);  // zero y step: divide first
    CHECK(p.back()[0] == .3 && p[1][1] == 1. && p[1][0] == 1. / 3. * .3);
}

void TestChassisYawDeadband()
{
    const wla::follow::Tuning c;
    for (double w = -1.; w <= 1.; w += .01)
    {
        CHECK(wla::follow::ChassisYaw(w, c) == -wla::follow::ChassisYaw(-w, c));
        CHECK(std::fabs(wla::follow::ChassisYaw(w, c)) <= std::fabs(w) + 1e-12);
    }
    CHECK(wla::follow::ChassisYaw(.5, c) == .5);
    CHECK(wla::follow::ChassisYaw(.1, c) == .2 * .1);
}

void TestPlannersFailClosed()
{
    wla::follow::RollingMap::Options options;
    options.m_resolution = .05;
    options.m_size = 160;
    wla::follow::RollingMap map(options);
    wla::follow::HybridFollower follower;
    wla::follow::CommandInput in;
    in.m_target = Point2{2., 0.};
    in.m_status = "visible";
    const wla::follow::Tuning c;
    CHECK(Throws([&] { (void)follower.command(map, c, in); }));  // no fields yet
    wla::follow::ApproachGoal goal;
    const std::vector<Point2> surface{{1., 0.}, {1., .1}, {1.1, 0.}};
    CHECK(Throws([&] { (void)goal.select({0., 0., 0.}, surface, map, 0.); }));
    map.slide(0., 0.);
    CHECK(map.initialClearPatch(0., 0., 0., .5, true, true) > 0);
    map.rebuild(0., false);
    in.m_status = "lost";
    const auto stop = follower.command(map, c, in);
    CHECK(stop[0] == 0. && stop[1] == 0.);
    in.m_status = "visible";
    in.m_history = {{1., 0.}, {1.5, 0.}, {2., 0.}};
    const auto out = follower.command(map, c, in);
    CHECK(std::isfinite(out[0]) && std::isfinite(out[1]));
    CHECK(std::fabs(out[0]) <= c.m_maxV && std::fabs(out[1]) <= c.m_maxW);
    const auto& report = follower.lastCandidates();
    CHECK(report.m_kind == wla::follow::CandidateReport::Kind::Sampled);
    CHECK(report.m_feasible <= report.m_safe && report.m_safe <= 325);
    // Recovery never drives forward; it turns towards a target behind.
    in.m_recovery = true;
    in.m_target = Point2{-2., .5};
    const auto turn = follower.command(map, c, in);
    CHECK(turn[0] <= 0.);
    CHECK(std::fabs(turn[1]) <= c.m_yawAccel * c.m_step + 1e-12);
    follower.reset();
    CHECK(follower.trail().empty() && follower.stuckCycles() == 0);
}

} // namespace

int main()
{
    TestArgSortOrders();
    TestDijkstraOnSmallGraphs();
    TestLinspaceKeepsEnds();
    TestChassisYawDeadband();
    TestPlannersFailClosed();
    if (g_failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", g_failures);
        return EXIT_FAILURE;
    }
    std::puts("test_planning: all checks passed");
    return EXIT_SUCCESS;
}
