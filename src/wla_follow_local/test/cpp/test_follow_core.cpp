// Native checks of FollowCore that need no Python reference. Equality with
// the Python engine (sim_runtime.process_step) is tested in test_follow_core.py.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include <wla_follow/follow_core.h>

namespace {

using wla::follow::FollowCore;
using wla::follow::Point2;
using wla::follow::StepPolicy;
using wla::follow::StepResult;
using wla::follow::StepSettings;

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

FollowCore::Options MakeOptions()
{
    FollowCore::Options options;
    options.m_mapResolution = .05;
    options.m_mapSize = 160;
    options.m_sweepRadius = std::hypot(.24, .24);
    return options;
}

void TestIdleStepKeepsTheMapOnly()
{
    FollowCore core(MakeOptions(), {0., 0., 0.}, 1.);
    StepSettings s;
    s.m_active = false;
    s.m_startupRadius = .5;
    const StepResult r = core.step({0., 0., 0.}, {.2, .1}, {}, std::nullopt, 2., 0, s);
    CHECK(!r.m_active && r.m_reason == "task_inactive" && r.m_command == (Point2{0., 0.}));
    // The startup prior is never used while idle.
    CHECK(!r.m_startupConsumed && !r.m_startupPrior);
    CHECK(r.m_grid.size() == 160u * 160u);
}

void TestStartupPriorOnceAndTranslationClock()
{
    FollowCore core(MakeOptions(), {0., 0., 0.}, 1.);
    StepSettings s;
    s.m_active = true;
    s.m_startupRadius = .5;
    s.m_taskMode = '4';
    s.m_goalRemaining = 1.;
    StepResult r = core.step({0., 0., 0.}, {0., 0.}, {}, std::nullopt, 2., 7, s);
    CHECK(r.m_startupConsumed && r.m_startupPrior && r.m_startupPrior->m_seededCells > 0);
    CHECK(!r.m_startupPrior->m_motionStartedS);
    // Rotation does not start the clock; translation does, once.
    r = core.step({0., 0., 1.}, {0., 0.}, {}, std::nullopt, 3., 7, s);
    CHECK(!r.m_startupPrior->m_motionStartedS);
    r = core.step({.04, 0., 1.}, {0., 0.}, {}, std::nullopt, 4., 7, s);
    CHECK(r.m_startupPrior->m_motionStartedS == 4. && r.m_startupPrior->m_expiresS == 16.);
    core.resetReference({0., 0., 0.}, 5.);
    r = core.step({0., 0., 0.}, {0., 0.}, {}, std::nullopt, 6., 8, s);
    CHECK(r.m_startupConsumed && r.m_startupPrior->m_referenceGeneration == 7);
}

void TestOrientIsBoundedAndPolicyLimited()
{
    FollowCore core(MakeOptions(), {0., 0., 0.}, 1.);
    StepSettings s;
    s.m_active = true;
    s.m_taskMode = '4';
    s.m_goalRemaining = -2.;
    StepResult r = core.step({0., 0., 0.}, {0., 0.}, {}, std::nullopt, 2., 0, s);
    // First change bounded by yaw_accel * step from rest: -0.3 rad/s, not -0.5.
    CHECK(r.m_reason == "orient" && r.m_plannerActivity == "in_place_turn");
    CHECK(r.m_command[0] == 0. && std::abs(r.m_command[1] + .3) < 1e-12);
    s.m_goalRemaining = .01;
    r = core.step({0., 0., 0.}, {0., 0.}, {}, std::nullopt, 2.1, 0, s);
    CHECK(r.m_command[1] == 0.);
    StepPolicy stop;
    stop.m_view.m_translationAllowed = false;
    stop.m_view.m_activeSearch = false;
    s.m_policy = stop;
    s.m_goalRemaining = 2.;
    r = core.step({0., 0., 0.}, {0., 0.}, {}, std::nullopt, 2.2, 0, s);
    CHECK(r.m_command == (Point2{0., 0.}));
}

void TestExternalCommandIsCommittedSeparately()
{
    FollowCore core(MakeOptions(), {0., 0., 0.}, 1.);
    StepSettings s;
    s.m_active = true;
    s.m_externalCommand = true;
    const StepResult r = core.step({0., 0., 0.}, {0., 0.}, {}, std::nullopt, 2., 0, s);
    CHECK(r.m_commandPending);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK(Throws([&] { (void)core.commit({nan, 0.}, 2., std::nullopt); }));
    CHECK(core.commit({.3, .1}, 2., std::nullopt) == (Point2{.3, .1}));
}

void TestPointGoalNeedsAGoal()
{
    FollowCore core(MakeOptions(), {0., 0., 0.}, 1.);
    StepSettings s;
    s.m_active = true;
    s.m_taskMode = '3';
    CHECK(Throws([&] { (void)core.step({0., 0., 0.}, {0., 0.}, {}, std::nullopt, 2., 0, s); }));
}

} // namespace

int main()
{
    TestIdleStepKeepsTheMapOnly();
    TestStartupPriorOnceAndTranslationClock();
    TestOrientIsBoundedAndPolicyLimited();
    TestExternalCommandIsCommittedSeparately();
    TestPointGoalNeedsAGoal();
    if (g_failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", g_failures);
        return EXIT_FAILURE;
    }
    std::puts("test_follow_core: all checks passed");
    return EXIT_SUCCESS;
}
