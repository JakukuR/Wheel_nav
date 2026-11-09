#ifndef WLA_FOLLOW_INCLUDE_WLA_FOLLOW_FOLLOW_CORE_H
#define WLA_FOLLOW_INCLUDE_WLA_FOLLOW_FOLLOW_CORE_H

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <wla_follow/planning.h>
#include <wla_follow/rolling_map.h>
#include <wla_follow/tracking.h>

namespace wla::follow {

// One depth packet of a step: the map update of sim_runtime.SimulationEngine.update_map.
struct StepPacket
{
    double m_stamp{0.};
    CloudView m_cloud;
    // packet['target_points_from_cluster']: clearing candidates go through drop_established.
    bool m_targetPointsFromCluster{false};
};

// observation = (stamp, target, people, resolved_id) of process_step.
struct StepObservation
{
    double m_stamp{0.};
    std::optional<Point2> m_target;
    std::vector<Detection> m_people;  // detection order of the people dict
    std::optional<std::int64_t> m_resolvedId;
};

// follow_session.FollowTracker.snapshot() fields the engine reads.
struct TrackerSnapshot
{
    std::string m_state;
    std::optional<Point2> m_position;
    std::optional<Point2> m_prediction;
    Point2 m_velocity{};
    std::vector<Point2> m_history;
    double m_surfaceOffsetM{.28};
};

// follow_session.TrackingPolicy, its predicates evaluated at the step time.
struct StepPolicy
{
    PolicyView m_view;  // m_activeSearch is also rotation_allowed
};

// startup dict of process_step after validation.
struct StepSettings
{
    double m_startupRadius{0.};
    double m_localRadius{0.};
    double m_motionThreshold{.03};
    bool m_gridRouteAstar{false};
    double m_targetClearTtl{.5};
    double m_trailClearRadius{0.};
    bool m_active{false};
    char m_taskMode{'0'};
    bool m_recovery{false};
    std::optional<StepPolicy> m_policy;
    std::optional<TrackerSnapshot> m_tracker;
    std::optional<std::vector<Point2>> m_objectSurface;
    std::optional<Point2> m_goalPoint;  // goal['point'] (mode 3)
    double m_goalRemaining{0.};         // goal['remaining'] (mode 4)
    // A local planner outside the core (NeuPAN) replaces the command: the step
    // stops before the policy limit and commit(), which the caller then calls.
    bool m_externalCommand{false};
};

// _PROCESS_STARTUP_PRIOR of process_step (without the derived remaining/state).
struct StartupPrior
{
    double m_radiusM{0.};
    Point2 m_center{};
    double m_stampS{0.};
    std::optional<double> m_expiresS;
    std::optional<double> m_motionStartedS;
    double m_motionThresholdM{.03};
    std::int64_t m_referenceGeneration{0};
    std::int64_t m_seededCells{0};
};

struct StepResult
{
    bool m_active{false};
    Point2 m_command{};
    std::string m_state;  // target_state and recovery_state
    std::string m_reason;
    Tuning m_tuning;
    std::vector<Point2> m_path;
    CandidateReport m_candidates;
    std::optional<ApproachReport> m_approach;
    std::string m_plannerActivity;
    std::optional<Point2> m_estimate;
    Point2 m_velocityReference{};
    std::size_t m_historyCount{0};
    std::vector<std::int64_t> m_obstacleIds;
    bool m_commandPending{false};
    // Map and prior reports of process_step.
    std::array<std::int64_t, 2> m_mapLower{};
    double m_resolution{0.};
    std::optional<StartupPrior> m_startupPrior;
    bool m_startupConsumed{false};
    std::size_t m_trailPoints{0};
    std::int64_t m_trailRetired{0};
    std::int64_t m_assumedCells{0};
    std::vector<std::int8_t> m_grid;  // n*n: 100 occupied, 0 known free, -1 unknown
};

// The single owner of the numerical follow state: rolling map, people tracks,
// sampled MPC with its detour planner, approach goal, target trail, point route,
// last command and the startup prior. step() is sim_runtime.process_step after
// its input checks, statement for statement (the Python SimulationEngine stays
// the reference; results compare equal), and runs without the Python GIL.
class FollowCore final
{
public:
    struct Options
    {
        double m_mapResolution{.1};
        int m_mapSize{120};
        double m_robotRadius{.19};
        double m_sweepRadius{0.};  // approach_goal.SWEEP_RADIUS
        Tuning m_tuning;           // SimulationEngine's Tuning()
    };

    FollowCore(const Options& options, const std::array<double, 3>& pose, double now);
    ~FollowCore();

    FollowCore(const FollowCore&) = delete;
    FollowCore& operator=(const FollowCore&) = delete;
    FollowCore(FollowCore&&) = delete;
    FollowCore& operator=(FollowCore&&) = delete;

    // A new task: target/controller state reset, obstacle evidence kept.
    void resetTask();
    // A wheel reference reset: also a new map at the pose. The startup prior
    // bookkeeping (process lifetime) is kept.
    void resetReference(const std::array<double, 3>& pose, double now);
    void setStartup(bool consumed, const std::optional<StartupPrior>& prior);

    // process_step for one job; reference_generation is key[-1] as a token.
    [[nodiscard]] StepResult step(const std::array<double, 3>& pose, const Point2& previous,
                                  std::span<const StepPacket> packets,
                                  const std::optional<StepObservation>& observation, double now,
                                  std::int64_t reference_generation, const StepSettings& settings);
    // The rest of a step with m_externalCommand: policy limit and the issued command.
    [[nodiscard]] Point2 commit(const Point2& command, double now, const std::optional<StepPolicy>& policy);

    [[nodiscard]] const std::shared_ptr<RollingMap>& map() const noexcept;
    [[nodiscard]] const Tuning& tuning() const noexcept;
    [[nodiscard]] std::vector<std::int64_t> peopleIds() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace wla::follow

#endif // WLA_FOLLOW_INCLUDE_WLA_FOLLOW_FOLLOW_CORE_H
