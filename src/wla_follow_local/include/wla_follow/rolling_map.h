#ifndef WLA_FOLLOW_INCLUDE_WLA_FOLLOW_ROLLING_MAP_H
#define WLA_FOLLOW_INCLUDE_WLA_FOLLOW_ROLLING_MAP_H

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace wla::follow {

// One depth head of a map update. Row-major arrays; N x 3 unless noted.
struct CloudView
{
    std::span<const double> m_points;            // sensor-frame points (z = height)
    std::span<const std::uint8_t> m_dynamic;     // N flags
    std::span<const double> m_pointsOdom;        // the same points in odom
    std::span<const double> m_targetPointsOdom;  // K x 3, may be empty
    std::span<const double> m_markingPointsOdom; // M x 3, used when m_hasMarking
    std::array<double, 3> m_origin{};            // sensor origin, sensor frame
    std::array<double, 3> m_originOdom{};        // sensor origin, odom
    bool m_hasMarking{false};
};

// Ring-buffer occupancy grid in continuous odom, the C++ owner of the state of
// the Python reference sim_core/rolling_depth_map.py (same equations, same
// order of operations; results compare element-wise equal).
//
// Ring arrays (log, stamp, ...) are indexed by slot a*n+b. Window arrays
// (cells, occupied, dist, ...) are indexed by window cell i*n+j, where the
// cell is (lower+i, lower+j) and its slot is ((lower+i) mod n, (lower+j) mod n).
class RollingMap final
{
public:
    struct Options
    {
        double m_resolution{.1};
        int m_size{120};
        double m_ttl{12.};
        double m_robotRadius{.19};
        double m_targetClearTtl{.5};
    };

    explicit RollingMap(const Options& options);
    ~RollingMap();

    RollingMap(const RollingMap&) = delete;
    RollingMap& operator=(const RollingMap&) = delete;
    RollingMap(RollingMap&&) = delete;
    RollingMap& operator=(RollingMap&&) = delete;

    void slide(double x, double y);
    [[nodiscard]] int initialClearPatch(double x, double y, double t, double radius,
                                        bool only_unknown, bool hold_until_motion);
    void advanceStartupPrior(double t, bool release);
    void update(std::span<const CloudView> clouds, const std::array<double, 3>& pose, double t, bool rebuild);
    // Rows of `points` (P x 3) kept as clearing candidates; trail rows are (stamp, x, y).
    [[nodiscard]] std::vector<double> dropEstablished(std::span<const double> points,
                                                      std::span<const double> trail,
                                                      double t, double radius) const;
    [[nodiscard]] int retirePersonTrail(std::span<const double> trail, double t, double radius,
                                        double window, double max_gap);
    // Hits within `radius` of (x, y) become unknown (rolling_depth_map.retire_disc).
    [[nodiscard]] int retireDisc(double x, double y, double radius, double t);
    void rebuild(double t, bool reuse);
    // Unique in-window ring slots of xy (Q x 2), in window-cell order.
    void slots(std::span<const double> xy, std::vector<std::int64_t>& a, std::vector<std::int64_t>& b) const;

    [[nodiscard]] double resolution() const noexcept;
    [[nodiscard]] int size() const noexcept;
    [[nodiscard]] double ttl() const noexcept;
    [[nodiscard]] double robotRadius() const noexcept;
    [[nodiscard]] double targetClearTtl() const noexcept;
    void setTargetClearTtl(double value);
    [[nodiscard]] double localFreeRadius() const noexcept;
    void setLocalFreeRadius(double value);
    [[nodiscard]] bool hasLocalFreeCenter() const noexcept;
    [[nodiscard]] std::array<double, 2> localFreeCenter() const noexcept;
    void setLocalFreeCenter(double x, double y);

    [[nodiscard]] std::int64_t version() const noexcept;
    [[nodiscard]] bool hasWindow() const noexcept;
    [[nodiscard]] std::array<std::int64_t, 2> lower() const noexcept;
    // Bumped whenever the window moves / the derived fields are recomputed.
    [[nodiscard]] std::uint64_t windowId() const noexcept;
    [[nodiscard]] std::uint64_t buildId() const noexcept;
    [[nodiscard]] bool hasFields() const noexcept;

    // Ring state (n*n, tags n*n*2), stable storage for the object's lifetime.
    [[nodiscard]] const double* logData() const noexcept;
    [[nodiscard]] const double* stampData() const noexcept;
    [[nodiscard]] const double* targetClearStampData() const noexcept;
    [[nodiscard]] const double* occupiedSinceData() const noexcept;
    [[nodiscard]] const std::int64_t* tagsData() const noexcept;
    // Derived window fields (n*n), valid when hasFields().
    [[nodiscard]] const std::uint8_t* sensorKnownData() const noexcept;
    [[nodiscard]] const std::uint8_t* occupiedData() const noexcept;
    [[nodiscard]] const std::uint8_t* knownData() const noexcept;
    [[nodiscard]] const std::uint8_t* assumedFreeData() const noexcept;
    [[nodiscard]] const double* distData() const noexcept;
    [[nodiscard]] const double* freeDistData() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

// Exact squared Euclidean distance of every nonzero cell of `foreground`
// (rows x cols) to the nearest zero cell. Every row and column must contain a
// zero cell; results are exact integers stored as double.
void SquaredDistanceTransform(const std::vector<std::uint8_t>& foreground, int rows, int cols,
                              std::vector<double>& out);

// numpy.interp for strictly increasing xp, bit for bit.
[[nodiscard]] double NumpyInterp(double x, const double* xp, const double* fp, std::size_t count, std::size_t stride);

} // namespace wla::follow

#endif // WLA_FOLLOW_INCLUDE_WLA_FOLLOW_ROLLING_MAP_H
