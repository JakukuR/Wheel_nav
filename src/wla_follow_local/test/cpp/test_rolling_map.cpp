// Native checks of the rolling map pieces that need no Python reference.
// Element-wise equality with the Python map is tested in test_native_rolling_map.py.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>

#include <wla_follow/rolling_map.h>

namespace {

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

void TestDistanceTransformMatchesBruteForce()
{
    std::mt19937 rng(7);
    for (int trial = 0; trial < 40; ++trial)
    {
        const int rows = 3 + static_cast<int>(rng() % 30), cols = 3 + static_cast<int>(rng() % 30);
        std::vector<std::uint8_t> fg(static_cast<std::size_t>(rows * cols));
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                const bool border = i == 0 || j == 0 || i == rows - 1 || j == cols - 1;
                fg[static_cast<std::size_t>(i * cols + j)] = !border && (rng() % 100) < 85;
            }
        }
        std::vector<double> out;
        wla::follow::SquaredDistanceTransform(fg, rows, cols, out);
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                double best = std::numeric_limits<double>::infinity();
                for (int a = 0; a < rows; ++a)
                {
                    for (int b = 0; b < cols; ++b)
                    {
                        if (!fg[static_cast<std::size_t>(a * cols + b)])
                        {
                            best = std::min(best, static_cast<double>((a - i) * (a - i) + (b - j) * (b - j)));
                        }
                    }
                }
                CHECK(out[static_cast<std::size_t>(i * cols + j)] == best);
            }
        }
    }
}

void TestInterp()
{
    const double xp[] = {0., 1., 3.};
    const double fp[] = {10., 20., 0.};
    CHECK(wla::follow::NumpyInterp(-1., xp, fp, 3, 1) == 10.);
    CHECK(wla::follow::NumpyInterp(-std::numeric_limits<double>::infinity(), xp, fp, 3, 1) == 10.);
    CHECK(wla::follow::NumpyInterp(5., xp, fp, 3, 1) == 0.);
    CHECK(wla::follow::NumpyInterp(1., xp, fp, 3, 1) == 20.);
    CHECK(wla::follow::NumpyInterp(.5, xp, fp, 3, 1) == 15.);
    CHECK(wla::follow::NumpyInterp(2., xp, fp, 3, 1) == 10.);
    CHECK(std::isnan(wla::follow::NumpyInterp(std::nan(""), xp, fp, 3, 1)));
}

void TestSlideKeepsOverlapAndForgetsTheRest()
{
    wla::follow::RollingMap::Options options;
    options.m_resolution = .1;
    options.m_size = 20;
    wla::follow::RollingMap map(options);
    map.slide(0., 0.);
    const std::vector<double> cloud{.5, .0, .3};
    const std::vector<std::uint8_t> dynamic{0};
    wla::follow::CloudView view;
    view.m_points = cloud;
    view.m_pointsOdom = cloud;
    view.m_dynamic = dynamic;
    view.m_origin = {0., 0., .5};
    view.m_originOdom = {0., 0., .5};
    const std::array<double, 3> pose{0., 0., 0.};
    map.update(std::span<const wla::follow::CloudView>(&view, 1), pose, 1., true);
    std::vector<std::int64_t> a, b;
    const std::vector<double> hit{.5, .0};
    map.slots(hit, a, b);
    CHECK(a.size() == 1);
    const std::size_t s = static_cast<std::size_t>(a[0] * 20 + b[0]);
    CHECK(map.logData()[s] == 2.);
    CHECK(map.occupiedSinceData()[s] == 1.);
    const std::int64_t version = map.version();
    map.slide(.05, .05);  // same window
    CHECK(map.version() == version);
    map.slide(.35, 0.);   // overlap keeps the hit
    CHECK(map.logData()[s] == 2.);
    map.slide(5., 5.);    // window moved past it
    CHECK(map.logData()[s] == 0.);
    CHECK(std::isinf(map.stampData()[s]));
}

} // namespace

int main()
{
    TestDistanceTransformMatchesBruteForce();
    TestInterp();
    TestSlideKeepsOverlapAndForgetsTheRest();
    if (g_failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", g_failures);
        return EXIT_FAILURE;
    }
    std::puts("test_rolling_map: all checks passed");
    return EXIT_SUCCESS;
}
