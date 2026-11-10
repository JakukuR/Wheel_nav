#ifndef WLA_FOLLOW_INCLUDE_WLA_FOLLOW_PLANNING_H
#define WLA_FOLLOW_INCLUDE_WLA_FOLLOW_PLANNING_H

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <wla_follow/rolling_map.h>
#include <wla_follow/tracking.h>

namespace wla::follow {

// sim_core/common.py Tuning (one value per call; the engine changes it per mode).
struct Tuning
{
    double m_step{.1};
    int m_horizon{24};
    double m_maxV{1.2};
    double m_maxW{.9};
    double m_accel{.65};
    double m_yawAccel{3.};
    double m_radius{.19};
    double m_margin{.10};
    double m_referenceCornerM{.35};
    double m_marginWeight{400.};
    double m_desiredGap{.19 + .28 + .73};
    double m_targetRadius{.28};
    double m_targetStopGap{.19 + .28 + .69};
    double m_targetHardGap{.19 + .28 + .10};
    double m_targetClearanceMax{.50};
    double m_cameraHalfAngle{0.6747409422235527};
    double m_fovEdgeMargin{.05};
    double m_centeringWeight{6.};
    double m_fovSoftAngle{.45};
    double m_lateralBearingStart{.10};
    double m_lateralSlowdown{.80};
    bool m_gridRouteAstar{false};
    double m_gridRouteLookaheadM{.5};
    double m_turnSlowAngle{.3};
    double m_turnFirstAngle{1.};
    double m_backupSpeed{.2};
    double m_backupMinSpeed{.12};
    double m_backupStallS{2.};
    double m_chassisTauW{.50};
    double m_chassisTauV{.18};
    double m_chassisDeadbandLo{.2};
    double m_chassisDeadbandHi{.3};
    double m_backupMaxM{.6};
    double m_searchTurnRate{.6};
    double m_searchExitTurn{1.5};
    double m_searchSpeed{.4};
};

// What HybridFollower.command reads from follow_session.TrackingPolicy, with
// its two predicates already evaluated at the policy time.
struct PolicyView
{
    double m_searchArrivalM{.35};
    std::optional<double> m_searchHeading;
    std::optional<double> m_searchStarted;
    double m_searchSide{1.};
    bool m_translationAllowed{true};
    bool m_activeSearch{false};
};

struct Obstacle
{
    Point2 m_position{};
    Point2 m_velocity{};
    double m_radius{0.};
};

// DetourPlanner.last of the reference, as a tagged value.
struct DetourReport
{
    enum class Kind : std::uint8_t
    {
        None,      // last is None
        Inactive,  // {active: False}
        Holding,   // {active, side, waypoint, holding}
        Grid,      // {active, side: 'grid', waypoint, route, search}
        Nothing,   // {active, reason: 'no_clear_edge_nothing_to_look_at'}
        Looking,   // {active, side, looking, waypoint}
        Edge,      // {active, side, waypoint, score}
    };
    Kind m_kind{Kind::None};
    bool m_left{false};
    Point2 m_waypoint{};
    std::vector<Point2> m_route;
    bool m_astar{false};
    double m_score{0.};

    [[nodiscard]] bool active() const noexcept { return m_kind != Kind::None && m_kind != Kind::Inactive; }
    [[nodiscard]] bool looking() const noexcept { return m_kind == Kind::Looking; }
};

// HybridFollower.last_candidates of the reference, as a tagged value.
struct CandidateReport
{
    enum class Kind : std::uint8_t
    {
        Empty,     // {}
        Backing,   // backing out along the trail
        Recovery,  // recover(): turn in place / stop
        Looking,   // explicit turn to look at a flank
        Sampled,   // the sampled rollout family
        TurningToRoute,  // explicit turn in place to the detour waypoint
    };
    Kind m_kind{Kind::Empty};
    int m_safe{0};
    int m_feasible{0};
    int m_forward{0};
    std::array<int, 3> m_reasons{};
    int m_forwardTotal{0};
    int m_forwardKnown{0};
    int m_forwardStatic{0};
    int m_forwardPeople{0};
    double m_backed{0.};
    bool m_hasDetour{false};
    DetourReport m_detour;
};

struct CommandInput
{
    std::array<double, 3> m_pose{};
    Point2 m_previous{};
    std::optional<Point2> m_target;
    Point2 m_velocity{};
    std::vector<Point2> m_history;
    std::vector<Obstacle> m_obstacles;
    std::string m_status;
    std::optional<PolicyView> m_policy;
    std::optional<double> m_policyTime;
    std::optional<std::vector<Point2>> m_referencePath;
    std::optional<Point2> m_facePoint;
    std::optional<Point2> m_measured;
    bool m_recovery{false};
};

// sim_core/detour.py DetourPlanner: edge-point detour, grid route (scipy
// Dijkstra port), optional A* refinement and flank looking.
class DetourPlanner final
{
public:
    DetourPlanner();
    ~DetourPlanner();

    DetourPlanner(const DetourPlanner&) = delete;
    DetourPlanner& operator=(const DetourPlanner&) = delete;
    DetourPlanner(DetourPlanner&&) = delete;
    DetourPlanner& operator=(DetourPlanner&&) = delete;

    void reset();
    // plan(): a polyline (robot, waypoint(s), target) or nothing when not blocked.
    std::optional<std::vector<Point2>> plan(const RollingMap& map, const std::array<double, 3>& pose,
                                            const Point2& target, const Tuning& c,
                                            const std::optional<std::vector<Point2>>& reference);
    // _grid_route(): also used directly for point goals.
    [[nodiscard]] std::optional<std::vector<Point2>> gridRoute(const RollingMap& map, const Point2& p,
                                                               const Point2& target, const Tuning& c,
                                                               double window_m = 3.5,
                                                               double clearance_weight = 2.) const;
    bool lookReached(bool reached) noexcept;

    [[nodiscard]] const DetourReport& last() const noexcept;
    void setTrigger(bool path) noexcept;  // trigger 'path' (true) or 'line'

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

// sim_core/hybrid_controller.py HybridFollower: the sampled differential MPC.
class HybridFollower final
{
public:
    HybridFollower();
    ~HybridFollower();

    HybridFollower(const HybridFollower&) = delete;
    HybridFollower& operator=(const HybridFollower&) = delete;
    HybridFollower(HybridFollower&&) = delete;
    HybridFollower& operator=(HybridFollower&&) = delete;

    void reset();
    [[nodiscard]] Point2 command(const RollingMap& map, const Tuning& c, const CommandInput& input);

    [[nodiscard]] bool lastInfeasible() const noexcept;
    [[nodiscard]] const CandidateReport& lastCandidates() const noexcept;
    [[nodiscard]] const std::vector<Point2>& lastReferencePath() const noexcept;
    [[nodiscard]] int stuckCycles() const noexcept;
    [[nodiscard]] double backed() const noexcept;
    [[nodiscard]] const std::vector<Point2>& trail() const noexcept;
    [[nodiscard]] DetourPlanner& detour() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

// sim_core/hybrid_controller.py chassis_yaw and approach_tangent.
[[nodiscard]] double ChassisYaw(double w, const Tuning& c) noexcept;
[[nodiscard]] double ApproachTangent(const std::array<double, 3>& pose, const Point2& command,
                                     std::span<const Obstacle> tracks, const Point2& target, double radius);

// sim_core/approach_goal.py ApproachGoal: connected object-approach endpoint.
struct ApproachPlan
{
    Point2 m_point{};
    std::vector<Point2> m_path;
    double m_yaw{0.};
    double m_goalClearanceM{0.};
    bool m_fallback{false};
    std::int64_t m_reachableCells{0};
    std::int64_t m_generation{0};
};

struct ApproachReport
{
    ApproachPlan m_plan;
    double m_currentClearanceM{0.};
    double m_endpointDistanceM{0.};
    double m_stableS{0.};
};

class ApproachGoal final
{
public:
    ApproachGoal();
    ~ApproachGoal();

    ApproachGoal(const ApproachGoal&) = delete;
    ApproachGoal& operator=(const ApproachGoal&) = delete;
    ApproachGoal(ApproachGoal&&) = delete;
    ApproachGoal& operator=(ApproachGoal&&) = delete;

    std::optional<ApproachReport> select(const std::array<double, 3>& pose, std::span<const Point2> surface,
                                         const RollingMap& map, double now);

    [[nodiscard]] const std::optional<ApproachPlan>& plan() const noexcept;
    [[nodiscard]] double plannedAt() const noexcept;
    [[nodiscard]] const std::optional<Point2>& center() const noexcept;
    [[nodiscard]] std::int64_t generation() const noexcept;
    [[nodiscard]] std::optional<double> stableSince() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

// ApproachGoal._samples: every segment sampled at <= 2.5 cm.
[[nodiscard]] std::vector<Point2> ApproachSamples(std::span<const Point2> path);

} // namespace wla::follow

#endif // WLA_FOLLOW_INCLUDE_WLA_FOLLOW_PLANNING_H
