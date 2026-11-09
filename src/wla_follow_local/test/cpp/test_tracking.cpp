// Native checks of the tracking pieces that need no Python reference.
// Bit equality with the Python reference is tested in test_native_tracking.py.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <stdexcept>
#include <vector>

#include <wla_follow/tracking.h>

#include "../../src/numpy_blas.h"

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

void TestSolveMatchesInverse()
{
    std::mt19937 rng(3);
    std::normal_distribution<double> normal;
    for (int trial = 0; trial < 1000; ++trial)
    {
        const double a = normal(rng), b = normal(rng), c = normal(rng), d = normal(rng);
        const double s00 = a * a + b * b + .01, s01 = a * c + b * d, s11 = c * c + d * d + .01;
        const double r0 = normal(rng), r1 = normal(rng);
        const auto x = wla::follow::detail::Solve2(wla::follow::detail::Factor2(s00, s01, s01, s11), r0, r1);
        const double det = s00 * s11 - s01 * s01;
        CHECK(std::fabs(x[0] - (s11 * r0 - s01 * r1) / det) < 1e-8 * (1. + std::fabs(x[0])));
        CHECK(std::fabs(x[1] - (s00 * r1 - s01 * r0) / det) < 1e-8 * (1. + std::fabs(x[1])));
    }
    CHECK(Throws([] { (void)wla::follow::detail::Factor2(0., 0., 0., 1.); }));
    CHECK(Throws([] { (void)wla::follow::detail::Factor2(1., 2., 2., 4.); }));
}

void TestHistoryResamplesOnTheGrid()
{
    wla::follow::TargetHistory history({5., .4, .6});
    history.update({0., 0.}, 10.);
    history.update({1., 0.}, 10.5);       // sample at 10.4 interpolated
    history.update({9., 9.}, 10.5);       // same stamp: ignored
    const auto points = history.points();
    CHECK(points.size() == 3);
    CHECK(std::fabs(points[1][0] - .8) < 1e-12);
    CHECK(points[2][0] == 1.);
    CHECK(history.sampleCount() == 2);
    history.update({2., 0.}, 11.2);       // gap .7 > .6: new sequence
    CHECK(history.sampleCount() == 1);
    CHECK(*history.oldestStamp() == 11.2);
    CHECK(Throws([&] { history.update({NAN, 0.}, 12.); }));
    CHECK(Throws([] { wla::follow::TargetHistory bad({1., .4, .6}); }));
    CHECK(Throws([] { wla::follow::TargetHistory bad({20., .2, .6}); }));  // 100 samples > 64
}

void TestTrackPredictionsLeaveTheState()
{
    wla::follow::VisualTrack::Options options;
    options.m_timeout = 2.;
    options.m_gate = 13.8;
    options.m_coastDecay = std::make_pair(.7, .4);
    const auto walk = [](wla::follow::VisualTrack& track) {
        for (int k = 0; k < 30; ++k)
        {
            const double t = .1 * k;
            track.update(wla::follow::Point2{1. + .8 * t, .2 * t}, t);
        }
    };
    wla::follow::VisualTrack a(options);
    CHECK(!a.predictedPosition(0.));
    CHECK(!a.update(std::nullopt, 0.).m_position);
    walk(a);
    const auto before = a.state();
    for (const double ahead : {0., .3, .9, 1.5, 2.5})
    {
        wla::follow::VisualTrack b(options);  // the reference's copy(track).update(None, t)
        b.update(std::nullopt, 0.);
        walk(b);
        const auto predicted = a.predict(2.9 + ahead);
        const auto stepped = b.update(std::nullopt, 2.9 + ahead);
        CHECK(predicted.m_position == stepped.m_position);
        CHECK(predicted.m_velocity == stepped.m_velocity);
        CHECK(predicted.m_state == stepped.m_state);
        const auto position = a.predictedPosition(2.9 + ahead);
        CHECK(std::fabs((*position)[0] - (*predicted.m_position)[0]) < 1e-12);
    }
    CHECK(a.state().m_x == before.m_x && a.state().m_covariance == before.m_covariance);
    CHECK(a.update(std::nullopt, 6.).m_state == wla::follow::TrackState::Lost);
    CHECK(wla::follow::TrackStateName(wla::follow::TrackState::Coast) == "coast");
}

void TestGateSkipsAtMostTwice()
{
    wla::follow::VisualTrack::Options options;
    options.m_timeout = 2.;
    options.m_gate = 13.8;
    wla::follow::VisualTrack track(options);
    for (int k = 0; k < 20; ++k)
    {
        track.update(wla::follow::Point2{2., 0.}, .1 * k);
    }
    track.update(wla::follow::Point2{5., 0.}, 2.0);
    track.update(wla::follow::Point2{5., 0.}, 2.1);
    CHECK(track.state().m_rejected == 2);
    track.update(wla::follow::Point2{5., 0.}, 2.2);  // third in a row lands
    CHECK(track.state().m_skips == 0);
    CHECK(track.state().m_x[0] > 2.5);
}

void TestPeopleExpireAndDrop()
{
    wla::follow::PeopleTracks people;
    const std::vector<wla::follow::Detection> both{{7, {1., 0.}}, {3, {2., 1.}}};
    const std::vector<wla::follow::Detection> only_three{{3, {2., 1.}}};
    CHECK(people.update(both, 0.).size() == 2);
    CHECK(people.ids() == (std::vector<std::int64_t>{7, 3}));
    CHECK(people.update(only_three, 1.).size() == 2);
    CHECK(people.predict(1.6).size() == 1);         // 7 is older than 1.5 s
    CHECK(people.ids().size() == 2);                // predict keeps it
    CHECK(people.update(only_three, 1.7).size() == 1);
    CHECK(people.drop(3) && !people.drop(3) && people.ids().empty());
    const std::vector<wla::follow::Detection> twice{{1, {0., 0.}}, {1, {1., 0.}}};
    CHECK(Throws([&] { (void)people.update(twice, 2.); }));
}

void TestReferenceCurvesKeepEndpoints()
{
    const std::vector<wla::follow::Point2> history{{0., 0.}, {.4, 0.}, {.8, .1}, {1.2, .4}, {1.5, .9}};
    const auto bezier = wla::follow::BezierReference(history);
    CHECK(bezier.front() == history.front() && bezier.back() == history.back());
    CHECK(bezier.size() == 8 * 4 + 1);
    const std::vector<wla::follow::Point2> bad{{0., 0.}, {NAN, 1.}};
    CHECK(Throws([&] { (void)wla::follow::BezierReference(bad); }));
    const std::vector<wla::follow::Point2> corner{{0., 0.}, {1., 0.}, {1., 1.}};
    const auto curve = wla::follow::BsplineCorners(corner, .35, .05);
    CHECK(curve.front() == corner.front() && curve.back() == corner.back());
    for (std::size_t i = 1; i < curve.size(); ++i)
    {
        CHECK(std::hypot(curve[i][0] - curve[i - 1][0], curve[i][1] - curve[i - 1][1]) < .06);
    }
    CHECK(wla::follow::BsplineCorners(corner, 0., .05).size() == 3);
}

} // namespace

int main()
{
    TestSolveMatchesInverse();
    TestHistoryResamplesOnTheGrid();
    TestTrackPredictionsLeaveTheState();
    TestGateSkipsAtMostTwice();
    TestPeopleExpireAndDrop();
    TestReferenceCurvesKeepEndpoints();
    if (g_failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", g_failures);
        return EXIT_FAILURE;
    }
    std::puts("test_tracking: all checks passed");
    return EXIT_SUCCESS;
}
