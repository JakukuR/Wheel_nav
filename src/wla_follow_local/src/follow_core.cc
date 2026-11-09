// FollowCore: sim_runtime.process_step / SimulationEngine.step in one C++
// owner. Every statement mirrors the Python engine (sim_runtime.py), which
// stays the numerical reference; test/test_follow_core.py compares the two.
#include <wla_follow/follow_core.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "map_view.h"
#include "numpy_blas.h"

namespace wla::follow {

namespace {

constexpr double TRAIL_RETENTION_S = 13.;
constexpr double POINT_ROUTE_REFRESH_S = .5;
constexpr double POINT_GOAL_MAX_V = .80;
constexpr double ORIENT_MIN_W = .35;
constexpr double ORIENT_MAX_W = .50;
constexpr double ORIENT_DONE_RAD = .05;
constexpr double DROP_ESTABLISHED_RADIUS_M = .6;  // RollingDepthMap.drop_established default
constexpr double RETIRE_WINDOW_S = .5;            // retire_person_trail defaults
constexpr double RETIRE_MAX_GAP_S = 2.;
constexpr double MAP_TTL_S = 12.;
constexpr double MAP_TARGET_CLEAR_TTL_S = .5;
constexpr double INF = std::numeric_limits<double>::infinity();

// np.clip of a scalar.
[[nodiscard]] double Clip(double x, double lo, double hi) noexcept
{
    return std::min(std::max(x, lo), hi);
}

// hybrid_controller.seed_command
[[nodiscard]] Point2 SeedCommand(const Point2& measured, const std::optional<Point2>& last,
                                 const std::optional<double>& last_at, double now)
{
    constexpr double MAX_AGE = .3, MAX_DV = .15, MAX_DW = .3;
    if (!last || !last_at || !(0 <= now - *last_at && now - *last_at <= MAX_AGE))
    {
        return measured;
    }
    if (std::abs((*last)[0] - measured[0]) > MAX_DV || std::abs((*last)[1] - measured[1]) > MAX_DW)
    {
        return measured;
    }
    return *last;
}

// hybrid_controller.limit_step
[[nodiscard]] Point2 LimitStep(const Point2& command, const Point2& seed, double dt, const Tuning& c)
{
    dt = Clip(dt, .02, c.m_step);
    const double w = Clip(command[1], seed[1] - c.m_yawAccel * dt, seed[1] + c.m_yawAccel * dt);
    // Stop or reverse while still moving the other way: stop now.
    if ((seed[0] > 0. && 0. >= command[0]) || (seed[0] < 0. && 0. <= command[0]))
    {
        return {0., w};
    }
    return {Clip(command[0], seed[0] - c.m_accel * dt, seed[0] + c.m_accel * dt), w};
}

// follow_session.TrackingPolicy.limit_command
[[nodiscard]] Point2 LimitCommand(Point2 command, const StepPolicy& policy) noexcept
{
    if (!policy.m_view.m_translationAllowed)
    {
        command[0] = 0.;
        if (!policy.m_view.m_activeSearch)
        {
            command[1] = 0.;
        }
    }
    return command;
}

// np.median of one column (partition, then the mean of the middle pair).
[[nodiscard]] double Median(std::vector<double> values)
{
    const std::size_t n = values.size();
    std::sort(values.begin(), values.end());
    if (n % 2 == 1)
    {
        return values[n / 2];
    }
    return (values[n / 2 - 1] + values[n / 2]) / 2.;
}

[[nodiscard]] bool StartsWith(const std::string& text, const char* prefix)
{
    return text.rfind(prefix, 0) == 0;
}

} // namespace

struct FollowCore::Impl
{
    Options m_options;
    Tuning m_baseTuning;  // SimulationEngine.tuning
    std::shared_ptr<RollingMap> m_map;
    double m_mapStamp{-INF};
    std::vector<std::array<double, 3>> m_targetTrail;  // (stamp, x, y)
    // Task state (a new SimulationEngine per task).
    std::unique_ptr<HybridFollower> m_controller;
    std::unique_ptr<PeopleTracks> m_people;
    std::unique_ptr<ApproachGoal> m_approach;
    double m_lastMeasurement{-INF};
    struct PointRoute
    {
        Point2 m_goal{};
        double m_plannedAt{0.};
        std::vector<Point2> m_path;
    };
    std::optional<PointRoute> m_pointRoute;
    double m_trailClearRadius{0.};
    std::int64_t m_trailRetired{0};
    std::optional<std::pair<std::int64_t, double>> m_trailChecked;
    std::optional<Point2> m_lastCommand;
    std::optional<double> m_lastCommandAt;
    // Process lifetime: _PROCESS_STARTUP_USED / _PROCESS_STARTUP_PRIOR.
    bool m_startupUsed{false};
    std::optional<StartupPrior> m_startupPrior;

    void newMap(const std::array<double, 3>& pose, double now)
    {
        RollingMap::Options o;
        o.m_resolution = m_options.m_mapResolution;
        o.m_size = m_options.m_mapSize;
        o.m_ttl = MAP_TTL_S;
        o.m_robotRadius = m_options.m_robotRadius;
        o.m_targetClearTtl = MAP_TARGET_CLEAR_TTL_S;
        m_map = std::make_shared<RollingMap>(o);
        m_map->slide(pose[0], pose[1]);
        m_map->rebuild(now, false);
        m_mapStamp = -INF;
        m_targetTrail.clear();
    }

    void resetTask()
    {
        m_baseTuning = m_options.m_tuning;
        m_controller = std::make_unique<HybridFollower>();
        m_people = std::make_unique<PeopleTracks>();
        m_approach = std::make_unique<ApproachGoal>();
        m_lastMeasurement = -INF;
        m_pointRoute.reset();
        m_trailClearRadius = 0.;
        m_trailRetired = 0;
        m_trailChecked.reset();
        m_lastCommand.reset();
        m_lastCommandAt.reset();
    }

    // SimulationEngine.update_map
    void updateMap(const std::array<double, 3>& pose, std::span<const StepPacket> packets, double now)
    {
        std::vector<const StepPacket*> order;
        order.reserve(packets.size());
        for (const StepPacket& p : packets)
        {
            order.push_back(&p);
        }
        std::stable_sort(order.begin(), order.end(),
                         [](const StepPacket* a, const StepPacket* b) { return a->m_stamp < b->m_stamp; });
        std::vector<double> trail = flatTrail();
        for (const StepPacket* packet : order)
        {
            if (packet->m_stamp > m_mapStamp)
            {
                CloudView cloud = packet->m_cloud;
                std::vector<double> kept;
                if (packet->m_targetPointsFromCluster)
                {
                    kept = m_map->dropEstablished(cloud.m_targetPointsOdom, trail, packet->m_stamp,
                                                  DROP_ESTABLISHED_RADIUS_M);
                    cloud.m_targetPointsOdom = kept;
                }
                const CloudView one[1] = {cloud};
                m_map->update(one, pose, packet->m_stamp, false);
                m_mapStamp = packet->m_stamp;
            }
        }
        m_map->slide(pose[0], pose[1]);
        m_map->setLocalFreeCenter(pose[0], pose[1]);
        if (!m_targetTrail.empty())
        {
            const double since = now - TRAIL_RETENTION_S;
            std::erase_if(m_targetTrail, [&](const std::array<double, 3>& row) { return !(row[0] >= since); });
            // Only new hits or a new trail point can create a match.
            const double last = m_targetTrail.empty() ? 0. : m_targetTrail.back()[0];
            const std::pair<std::int64_t, double> state{m_map->version(), last};
            if (!m_trailChecked || *m_trailChecked != state)
            {
                trail = flatTrail();
                m_trailRetired = m_map->retirePersonTrail(trail, now, m_trailClearRadius, RETIRE_WINDOW_S,
                                                          RETIRE_MAX_GAP_S);
                m_trailChecked = std::make_pair(m_map->version(), state.second);
            }
        }
        m_map->rebuild(now, true);
    }

    [[nodiscard]] std::vector<double> flatTrail() const
    {
        std::vector<double> flat;
        flat.reserve(3 * m_targetTrail.size());
        for (const auto& row : m_targetTrail)
        {
            flat.insert(flat.end(), row.begin(), row.end());
        }
        return flat;
    }

    // SimulationEngine._point_route
    [[nodiscard]] std::vector<Point2> pointRoute(const std::array<double, 3>& pose, const Point2& goal,
                                                 const Tuning& tuning, double now)
    {
        const Point2 here{pose[0], pose[1]};
        if (m_pointRoute)
        {
            const PointRoute& old = *m_pointRoute;
            const auto close = [](double a, double b) { return std::abs(a - b) <= 1e-08 + 1e-05 * std::abs(b); };
            if (close(old.m_goal[0], goal[0]) && close(old.m_goal[1], goal[1]) && now - old.m_plannedAt < POINT_ROUTE_REFRESH_S)
            {
                const std::vector<Point2>& path = old.m_path;
                std::size_t near = 0;
                double best = INF;
                for (std::size_t k = 0; k < path.size(); ++k)
                {
                    // np.argmin: the first minimum.
                    const double d = detail::RowNorm2(path[k][0] - here[0], path[k][1] - here[1]);
                    if (k == 0 || d < best)
                    {
                        best = d;
                        near = k;
                    }
                }
                std::vector<Point2> suffix{here};
                if (near + 1 < path.size())
                {
                    suffix.insert(suffix.end(), path.begin() + static_cast<std::ptrdiff_t>(near + 1), path.end());
                }
                else
                {
                    suffix.push_back(goal);
                }
                const detail::MapView view(*m_map);
                bool clear = true;
                for (const Point2& s : ApproachSamples(suffix))
                {
                    if (!(view.query(s[0], s[1]) >= tuning.m_radius))
                    {
                        clear = false;
                        break;
                    }
                }
                if (clear)
                {
                    return suffix;
                }
            }
        }
        std::optional<std::vector<Point2>> path = m_controller->detour().gridRoute(*m_map, here, goal, tuning);
        if (!path)
        {
            path = std::vector<Point2>{here, goal};
        }
        m_pointRoute = PointRoute{goal, now, *path};
        return *path;
    }

    // SimulationEngine.step
    void engineStep(const std::array<double, 3>& pose, const Point2& previous, std::span<const StepPacket> packets,
                    const std::optional<StepObservation>& observation, double now, const StepSettings& s,
                    StepResult& out)
    {
        const char mode = s.m_taskMode;
        if (s.m_active && (mode == '0' || mode == '1') && observation && observation->m_target)
        {
            const double stamp = observation->m_stamp;
            if (m_targetTrail.empty() || stamp > m_targetTrail.back()[0])
            {
                m_targetTrail.push_back({stamp, (*observation->m_target)[0], (*observation->m_target)[1]});
            }
        }
        updateMap(pose, packets, now);
        if (!s.m_active)
        {
            m_controller->reset();
            out.m_active = false;
            out.m_command = {0., 0.};
            out.m_state = "idle";
            out.m_reason = "task_inactive";
            out.m_tuning = m_baseTuning;
            return;
        }
        plan(pose, previous, observation, now, s, out);
    }

    // SimulationEngine._plan
    void plan(const std::array<double, 3>& pose, const Point2& previous,
              const std::optional<StepObservation>& observation, double now, const StepSettings& s, StepResult& out)
    {
        out.m_active = true;
        const char mode = s.m_taskMode;
        if (observation)
        {
            if (observation->m_target && observation->m_resolvedId)
            {
                m_people->drop(*observation->m_resolvedId);
            }
            if (m_lastMeasurement < observation->m_stamp && observation->m_stamp <= now)
            {
                (void)m_people->update(observation->m_people, observation->m_stamp);
                m_lastMeasurement = observation->m_stamp;
            }
        }
        const std::vector<PersonEstimate> tracks = m_people->predict(now);
        std::optional<Point2> estimate;
        Point2 velocity{0., 0.};
        std::vector<Point2> history;
        std::string state = "lost";
        const std::optional<TrackerSnapshot>& snapshot = s.m_tracker;
        if (snapshot)
        {
            const std::string& binding = snapshot->m_state;
            estimate = StartsWith(binding, "search_") ? snapshot->m_position : snapshot->m_prediction;
            velocity = snapshot->m_velocity;
            history = snapshot->m_history;
            state = binding == "tracking" ? "visible" : binding == "coasting" ? "coast" : binding;
            if (binding == "holding" && (mode == '0' || mode == '1'))
            {
                state = "coast";
            }
        }
        Tuning tuning = m_baseTuning;
        std::optional<std::vector<Point2>> route;
        std::optional<Point2> face;
        std::optional<ApproachReport> approach;
        if (mode == '1')
        {
            const double stop_gap = .95 + (snapshot ? snapshot->m_surfaceOffsetM : .28);
            tuning.m_desiredGap = stop_gap;
            tuning.m_targetStopGap = stop_gap;
            tuning.m_targetClearanceMax = 0.;
            tuning.m_maxV = .30;
            // Finite approach is to the current person, not a stale five-second trail.
            history.clear();
            velocity = {0., 0.};
        }
        else if (mode == '2')
        {
            tuning.m_radius = m_options.m_sweepRadius;
            tuning.m_desiredGap = 0.;
            tuning.m_targetStopGap = .025;
            tuning.m_targetHardGap = 0.;
            tuning.m_targetClearanceMax = 0.;
            tuning.m_maxV = .20;
            velocity = {0., 0.};
            history.clear();
            if (state == "visible" && s.m_objectSurface)
            {
                const std::vector<Point2>& surface = *s.m_objectSurface;
                approach = m_approach->select(pose, surface, *m_map, now);
                if (approach)
                {
                    route = approach->m_plan.m_path;
                    estimate = approach->m_plan.m_point;
                    std::vector<double> xs, ys;
                    xs.reserve(surface.size());
                    ys.reserve(surface.size());
                    for (const Point2& p : surface)
                    {
                        xs.push_back(p[0]);
                        ys.push_back(p[1]);
                    }
                    face = Point2{Median(std::move(xs)), Median(std::move(ys))};
                }
            }
            if (!approach)
            {
                state = "holding";
            }
        }
        else if (mode == '3')
        {
            tuning.m_desiredGap = 0.;
            tuning.m_targetStopGap = .05;
            tuning.m_targetHardGap = 0.;
            tuning.m_targetClearanceMax = 0.;
            tuning.m_maxV = POINT_GOAL_MAX_V;
            if (!s.m_goalPoint)
            {
                throw std::invalid_argument("missing_point_goal");
            }
            estimate = *s.m_goalPoint;
            velocity = {0., 0.};
            history.clear();
            state = "visible";
            route = pointRoute(pose, *estimate, tuning, now);
        }
        else if (mode == '4')
        {
            velocity = {0., 0.};
            history.clear();
            state = "visible";
        }
        const Point2 seed = SeedCommand(previous, m_lastCommand, m_lastCommandAt, now);
        Point2 command{};
        bool infeasible = false;
        CandidateReport candidates;
        std::vector<Point2> path;
        if (mode == '4')
        {
            // Turning in place never changes the circular footprint's clearance.
            const double remaining = s.m_goalRemaining;
            const double w = std::abs(remaining) <= ORIENT_DONE_RAD
                                 ? 0.
                                 : std::copysign(Clip(1.2 * std::abs(remaining), ORIENT_MIN_W, ORIENT_MAX_W), remaining);
            const double dt = m_lastCommandAt ? now - *m_lastCommandAt : tuning.m_step;
            command = LimitStep({0., w}, seed, dt, tuning);
        }
        else
        {
            CommandInput in;
            in.m_pose = pose;
            in.m_previous = seed;
            in.m_target = estimate;
            in.m_velocity = velocity;
            in.m_history = history;
            in.m_obstacles.reserve(tracks.size());
            for (const PersonEstimate& e : tracks)
            {
                in.m_obstacles.push_back({e.m_position, e.m_velocity, e.m_radius});
            }
            in.m_status = state;
            if (s.m_policy)
            {
                in.m_policy = s.m_policy->m_view;
            }
            in.m_policyTime = now;
            in.m_referencePath = route;
            in.m_facePoint = face;
            in.m_measured = previous;
            in.m_recovery = s.m_recovery;
            command = m_controller->command(*m_map, tuning, in);
            infeasible = m_controller->lastInfeasible();
            candidates = m_controller->lastCandidates();
            path = m_controller->lastReferencePath();
        }
        const bool has_candidates = candidates.m_kind != CandidateReport::Kind::Empty;
        if (has_candidates && !infeasible)
        {
            // Plans now run at control rate; the first change is bounded by the
            // time since the last issued command (zero/stop outputs stay exact).
            const double dt = m_lastCommandAt ? now - *m_lastCommandAt : tuning.m_step;
            command = LimitStep(command, seed, dt, tuning);
        }
        std::string reason;
        if (mode == '3')
        {
            reason = "point_goal";
        }
        else if (mode == '4')
        {
            reason = "orient";
        }
        else if (mode == '2' && !approach)
        {
            reason = "object_goal_unavailable";
        }
        else if (approach)
        {
            reason = approach->m_plan.m_fallback ? "nearest_reachable_local_position" : "object_clearance_goal";
        }
        else if (StartsWith(state, "search_"))
        {
            reason = state;
        }
        else if (snapshot && snapshot->m_state == "holding" && state == "coast")
        {
            reason = "holding_last_trusted";
        }
        else if (state == "coast")
        {
            reason = "tracker_prediction";
        }
        else
        {
            reason = "tracker_follow";
        }
        if (infeasible)
        {
            command = {0., 0.};
            reason = "no_feasible_candidate";
        }
        else if (s.m_recovery)
        {
            reason = "depth_coverage_recovery";
        }
        else if (candidates.m_kind == CandidateReport::Kind::Backing)
        {
            reason = "backing_out";
        }
        out.m_state = state;
        out.m_reason = reason;
        out.m_tuning = tuning;
        out.m_path = std::move(path);
        out.m_plannerActivity = has_candidates ? "candidate_evaluation" : mode == '4' ? "in_place_turn" : "waiting_target";
        out.m_candidates = std::move(candidates);
        out.m_approach = std::move(approach);
        out.m_estimate = estimate;
        out.m_velocityReference = velocity;
        out.m_historyCount = history.size();
        out.m_obstacleIds.reserve(tracks.size());
        for (const PersonEstimate& e : tracks)
        {
            out.m_obstacleIds.push_back(e.m_id);
        }
        if (s.m_externalCommand)
        {
            out.m_command = command;
            out.m_commandPending = true;
            return;
        }
        out.m_command = commit(command, now, s.m_policy);
    }

    Point2 commit(Point2 command, double now, const std::optional<StepPolicy>& policy)
    {
        if (policy)
        {
            command = LimitCommand(command, *policy);
        }
        if (!std::isfinite(command[0]) || !std::isfinite(command[1]))
        {
            throw std::invalid_argument("nonfinite_core_command");
        }
        m_lastCommand = command;
        m_lastCommandAt = now;
        return command;
    }
};

FollowCore::FollowCore(const Options& options, const std::array<double, 3>& pose, double now)
    : m_impl(std::make_unique<Impl>())
{
    m_impl->m_options = options;
    m_impl->newMap(pose, now);
    m_impl->resetTask();
}

FollowCore::~FollowCore() = default;

void FollowCore::resetTask()
{
    m_impl->resetTask();
}

void FollowCore::resetReference(const std::array<double, 3>& pose, double now)
{
    m_impl->newMap(pose, now);
    m_impl->resetTask();
}

void FollowCore::setStartup(bool consumed, const std::optional<StartupPrior>& prior)
{
    m_impl->m_startupUsed = consumed;
    m_impl->m_startupPrior = prior;
}

StepResult FollowCore::step(const std::array<double, 3>& pose, const Point2& previous,
                            std::span<const StepPacket> packets, const std::optional<StepObservation>& observation,
                            double now, std::int64_t reference_generation, const StepSettings& settings)
{
    Impl& e = *m_impl;
    RollingMap& map = *e.m_map;
    // process_step after its input checks.
    e.m_baseTuning.m_gridRouteAstar = settings.m_gridRouteAstar;
    map.setLocalFreeRadius(settings.m_localRadius);
    map.setTargetClearTtl(settings.m_targetClearTtl);
    e.m_trailClearRadius = settings.m_trailClearRadius;
    const double radius = settings.m_startupRadius;
    if (settings.m_active && settings.m_localRadius == 0 && radius > 0 && !e.m_startupUsed)
    {
        // User-authorized assumption, once at the first follow task, not during
        // idle warm-up. Never erase observed obstacles or renew on task changes.
        map.slide(pose[0], pose[1]);
        const int count = map.initialClearPatch(pose[0], pose[1], now, radius, true, true);
        StartupPrior prior;
        prior.m_radiusM = radius;
        prior.m_center = {pose[0], pose[1]};
        prior.m_stampS = now;
        prior.m_motionThresholdM = settings.m_motionThreshold;
        prior.m_referenceGeneration = reference_generation;
        prior.m_seededCells = count;
        e.m_startupPrior = prior;
        e.m_startupUsed = true;
    }
    if (e.m_startupPrior && e.m_startupPrior->m_referenceGeneration == reference_generation &&
        !e.m_startupPrior->m_motionStartedS)
    {
        // Translation comes from wheel odometry, never from a commanded velocity.
        StartupPrior& prior = *e.m_startupPrior;
        const bool moved = detail::BlasNorm2(pose[0] - prior.m_center[0], pose[1] - prior.m_center[1]) >=
                           prior.m_motionThresholdM;
        map.advanceStartupPrior(now, moved);
        if (moved)
        {
            prior.m_motionStartedS = now;
            prior.m_expiresS = now + map.ttl();
        }
    }
    StepResult out;
    e.engineStep(pose, previous, packets, observation, now, settings, out);
    // The map may have been replaced only by resetReference, never inside a step.
    out.m_startupPrior = e.m_startupPrior;
    out.m_startupConsumed = e.m_startupUsed;
    out.m_mapLower = map.lower();
    out.m_resolution = map.resolution();
    out.m_trailPoints = e.m_targetTrail.size();
    out.m_trailRetired = e.m_trailRetired;
    const std::size_t cells = static_cast<std::size_t>(map.size()) * static_cast<std::size_t>(map.size());
    const std::uint8_t* assumed = map.assumedFreeData();
    const std::uint8_t* occupied = map.occupiedData();
    const std::uint8_t* known = map.knownData();
    out.m_grid.resize(cells);
    std::int64_t assumed_cells = 0;
    for (std::size_t k = 0; k < cells; ++k)
    {
        assumed_cells += assumed[k] != 0;
        out.m_grid[k] = occupied[k] ? std::int8_t{100} : known[k] ? std::int8_t{0} : std::int8_t{-1};
    }
    out.m_assumedCells = assumed_cells;
    return out;
}

Point2 FollowCore::commit(const Point2& command, double now, const std::optional<StepPolicy>& policy)
{
    return m_impl->commit(command, now, policy);
}

const std::shared_ptr<RollingMap>& FollowCore::map() const noexcept
{
    return m_impl->m_map;
}

const Tuning& FollowCore::tuning() const noexcept
{
    return m_impl->m_baseTuning;
}

std::vector<std::int64_t> FollowCore::peopleIds() const
{
    return m_impl->m_people->ids();
}

} // namespace wla::follow
