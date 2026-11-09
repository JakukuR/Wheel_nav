#ifndef WLA_FOLLOW_INCLUDE_WLA_FOLLOW_TRACKING_H
#define WLA_FOLLOW_INCLUDE_WLA_FOLLOW_TRACKING_H

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace wla::follow {

using Point2 = std::array<double, 2>;

// Accepted observations resampled on a fixed time grid; the C++ owner of
// sim_core/target_reference.py TargetHistory (same arithmetic, same order).
class TargetHistory final
{
public:
    struct Options
    {
        double m_windowS{5.};
        double m_periodS{.4};
        double m_maxGapS{.6};
    };

    struct Sample
    {
        double m_stamp{0.};
        Point2 m_point{};
    };

    // Throws std::invalid_argument("invalid_target_history_configuration").
    explicit TargetHistory(const Options& options);

    TargetHistory(const TargetHistory&) = delete;
    TargetHistory& operator=(const TargetHistory&) = delete;
    TargetHistory(TargetHistory&&) = delete;
    TargetHistory& operator=(TargetHistory&&) = delete;

    // Throws std::invalid_argument("invalid_target_history_observation").
    void update(const Point2& point, double stamp);
    [[nodiscard]] std::vector<Point2> points() const;

    [[nodiscard]] const Options& options() const noexcept { return m_options; }
    [[nodiscard]] std::size_t sampleCount() const noexcept { return m_samples.size(); }
    [[nodiscard]] std::optional<double> oldestStamp() const noexcept;
    [[nodiscard]] std::optional<double> lastObservationStamp() const noexcept;
    // Changes with every accepted observation (cache key for points()).
    [[nodiscard]] std::uint64_t revision() const noexcept { return m_revision; }

private:
    Options m_options;
    std::size_t m_capacity;
    std::deque<Sample> m_samples;
    std::optional<Sample> m_latest;
    double m_origin{0.};
    std::int64_t m_nextIndex{0};
    std::uint64_t m_revision{0};
};

enum class TrackState : std::uint8_t
{
    Lost,
    Coast,
    Visible,
};

[[nodiscard]] std::string_view TrackStateName(TrackState state) noexcept;

struct TrackUpdate
{
    std::optional<Point2> m_position;
    Point2 m_velocity{};
    TrackState m_state{TrackState::Lost};
};

// Filter state of a VisualTrack; a plain value, copied for predictions.
struct KalmanState
{
    bool m_initialised{false};
    std::array<double, 4> m_x{};             // x, y, vx, vy
    std::array<double, 16> m_covariance{     // row-major 4 x 4
        1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.};
    double m_time{0.};
    double m_seen{0.};
    int m_skips{0};
    int m_rejected{0};
};

// Constant-velocity Kalman track with innovation gate and coast decay; the
// C++ owner of sim_core/tracking.py VisualTrack. Matrix products and the 2x2
// solve reproduce numpy's reference BLAS/LAPACK rounding (fused multiply-add
// accumulation, LU with partial pivoting), so results are bit-identical.
class VisualTrack final
{
public:
    struct Options
    {
        double m_timeout{.8};
        TargetHistory::Options m_history{};
        double m_measStd{.025};
        double m_accelStd{.7};
        std::optional<double> m_gate{};
        int m_gateMaxSkips{2};
        std::optional<std::pair<double, double>> m_coastDecay{}; // (delay_s, tau_s)
    };

    explicit VisualTrack(const Options& options);

    VisualTrack(const VisualTrack&) = delete;
    VisualTrack& operator=(const VisualTrack&) = delete;
    VisualTrack(VisualTrack&&) = delete;
    VisualTrack& operator=(VisualTrack&&) = delete;

    // update(z, t) of the reference; z absent = no measurement this frame.
    TrackUpdate update(const std::optional<Point2>& z, double t);
    // predicted_position(t): constant-velocity extrapolation, state untouched.
    [[nodiscard]] std::optional<Point2> predictedPosition(double t) const;
    // copy(track).update(None, t): the same result, state untouched.
    [[nodiscard]] TrackUpdate predict(double t) const;

    [[nodiscard]] const Options& options() const noexcept { return m_options; }
    [[nodiscard]] const KalmanState& state() const noexcept { return m_state; }
    [[nodiscard]] const TargetHistory& trail() const noexcept { return m_trail; }

private:
    Options m_options;
    KalmanState m_state;
    TargetHistory m_trail;
};

struct Detection
{
    std::int64_t m_id{0};
    Point2 m_point{};
};

struct PersonEstimate
{
    std::int64_t m_id{0};
    Point2 m_position{};
    Point2 m_velocity{};
    double m_radius{0.};
    double m_age{0.};
};

// Other people as dynamic obstacles; the C++ owner of sim_core/tracking.py
// PeopleTracks. Tracks keep first-detection order, as the reference dict does.
class PeopleTracks final
{
public:
    static constexpr double MAX_AGE_S = 1.5;

    PeopleTracks();
    ~PeopleTracks();

    PeopleTracks(const PeopleTracks&) = delete;
    PeopleTracks& operator=(const PeopleTracks&) = delete;
    PeopleTracks(PeopleTracks&&) = delete;
    PeopleTracks& operator=(PeopleTracks&&) = delete;

    // Throws std::invalid_argument("duplicate_people_detection") on repeated ids.
    std::vector<PersonEstimate> update(std::span<const Detection> detections, double t);
    // What update({}, t) would return, leaving every track as it is.
    [[nodiscard]] std::vector<PersonEstimate> predict(double t) const;
    bool drop(std::int64_t id);
    [[nodiscard]] std::vector<std::int64_t> ids() const;

private:
    std::vector<std::pair<std::int64_t, std::unique_ptr<VisualTrack>>> m_tracks;
};

// sim_core/target_reference.py bezier_reference: C1 piecewise cubic Bezier.
// Throws std::invalid_argument("nonfinite_target_history").
[[nodiscard]] std::vector<Point2> BezierReference(std::span<const Point2> history);

// sim_core/target_reference.py bspline_corners: clamped cubic B-spline corners.
[[nodiscard]] std::vector<Point2> BsplineCorners(std::span<const Point2> path, double corner, double step);

} // namespace wla::follow

#endif // WLA_FOLLOW_INCLUDE_WLA_FOLLOW_TRACKING_H
