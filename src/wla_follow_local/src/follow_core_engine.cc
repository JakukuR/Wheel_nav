// Python binding of FollowCore (phase 4): process_step as one call. Inputs are
// converted while holding the GIL, the step runs without it, and the outputs
// come back as plain values; native_engine.py builds the result dict of
// SimulationEngine.step / process_step from them.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <wla_follow/follow_core.h>

namespace py = pybind11;
using wla::follow::ApproachReport;
using wla::follow::CandidateReport;
using wla::follow::CloudView;
using wla::follow::Detection;
using wla::follow::FollowCore;
using wla::follow::Point2;
using wla::follow::RollingMap;
using wla::follow::StartupPrior;
using wla::follow::StepObservation;
using wla::follow::StepPacket;
using wla::follow::StepPolicy;
using wla::follow::StepResult;
using wla::follow::StepSettings;
using wla::follow::TrackerSnapshot;
using wla::follow::Tuning;

// follow_core_planning.cc
py::dict ExportCandidateDict(const CandidateReport& report);
py::dict ExportApproachDict(const ApproachReport& report);
Tuning ExportTuningFrom(const py::handle& tuning);

namespace {

using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;
using BoolArray = py::array_t<bool, py::array::c_style | py::array::forcecast>;
using IntArray = py::array_t<std::int64_t, py::array::c_style | py::array::forcecast>;

[[nodiscard]] DoubleArray Rows(const py::handle& value, const char* name)
{
    DoubleArray array = DoubleArray::ensure(value);
    if (!array || array.size() % 3 != 0)
    {
        throw std::invalid_argument(std::string("invalid_map_rows:") + name);
    }
    return array;
}

[[nodiscard]] std::span<const double> Span(const DoubleArray& array)
{
    return {array.data(), static_cast<std::size_t>(array.size())};
}

[[nodiscard]] std::vector<Point2> Points(const py::handle& value, const char* what)
{
    DoubleArray array = DoubleArray::ensure(value);
    if (!array || array.size() % 2 != 0)
    {
        throw std::invalid_argument(std::string("invalid_points:") + what);
    }
    std::vector<Point2> points(static_cast<std::size_t>(array.size() / 2));
    if (!points.empty())
    {
        std::memcpy(points.data(), array.data(), points.size() * sizeof(Point2));
    }
    return points;
}

[[nodiscard]] Point2 Vector2(const py::handle& value, const char* what)
{
    DoubleArray array = DoubleArray::ensure(value);
    if (!array || array.size() < 2)
    {
        throw std::invalid_argument(what);
    }
    return {array.data()[0], array.data()[1]};
}

[[nodiscard]] std::optional<Point2> Optional2(const py::handle& value, const char* what)
{
    if (value.is_none())
    {
        return std::nullopt;
    }
    return Vector2(value, what);
}

[[nodiscard]] std::optional<double> OptionalDouble(const py::handle& value)
{
    if (value.is_none())
    {
        return std::nullopt;
    }
    return value.cast<double>();
}

[[nodiscard]] std::array<double, 3> Pose(const py::handle& value)
{
    DoubleArray array = DoubleArray::ensure(value);
    if (!array || array.size() < 3)
    {
        throw std::invalid_argument("invalid_pose");
    }
    return {array.data()[0], array.data()[1], array.data()[2]};
}

[[nodiscard]] py::array_t<double> Array(const std::vector<Point2>& points)
{
    py::array_t<double> out({static_cast<py::ssize_t>(points.size()), py::ssize_t(2)});
    if (!points.empty())
    {
        std::memcpy(out.mutable_data(), points.data(), points.size() * sizeof(Point2));
    }
    return out;
}

[[nodiscard]] py::array_t<double> Array(const Point2& p)
{
    return py::array_t<double>(2, p.data());
}

[[nodiscard]] py::object OptionalArray(const std::optional<Point2>& p)
{
    return p ? py::object(Array(*p)) : py::object(py::none());
}

// (search_arrival_m, search_heading, search_started, search_side,
//  translation_allowed(now), active_search(now)) or None
[[nodiscard]] std::optional<StepPolicy> Policy(const py::handle& value)
{
    if (value.is_none())
    {
        return std::nullopt;
    }
    const auto t = value.cast<py::tuple>();
    if (t.size() != 6)
    {
        throw std::invalid_argument("invalid_policy_view");
    }
    StepPolicy policy;
    policy.m_view.m_searchArrivalM = t[0].cast<double>();
    policy.m_view.m_searchHeading = OptionalDouble(t[1]);
    policy.m_view.m_searchStarted = OptionalDouble(t[2]);
    policy.m_view.m_searchSide = t[3].cast<double>();
    policy.m_view.m_translationAllowed = t[4].cast<bool>();
    policy.m_view.m_activeSearch = t[5].cast<bool>();
    return policy;
}

// (state, position, prediction, velocity_mps, history, surface_offset_m) or None
[[nodiscard]] std::optional<TrackerSnapshot> Tracker(const py::handle& value)
{
    if (value.is_none())
    {
        return std::nullopt;
    }
    const auto t = value.cast<py::tuple>();
    if (t.size() != 6)
    {
        throw std::invalid_argument("invalid_tracker_snapshot");
    }
    TrackerSnapshot s;
    s.m_state = t[0].cast<std::string>();
    s.m_position = Optional2(t[1], "invalid_tracker_position");
    s.m_prediction = Optional2(t[2], "invalid_tracker_prediction");
    s.m_velocity = Vector2(t[3], "invalid_tracker_velocity");
    s.m_history = Points(t[4], "tracker_history");
    s.m_surfaceOffsetM = t[5].cast<double>();
    return s;
}

// (stamp, target, ids, points, resolved_id) or None
[[nodiscard]] std::optional<StepObservation> Observation(const py::handle& value)
{
    if (value.is_none())
    {
        return std::nullopt;
    }
    const auto t = value.cast<py::tuple>();
    if (t.size() != 5)
    {
        throw std::invalid_argument("invalid_observation");
    }
    StepObservation o;
    o.m_stamp = t[0].cast<double>();
    o.m_target = Optional2(t[1], "invalid_observation_target");
    IntArray ids = IntArray::ensure(t[2]);
    const auto points = Points(t[3], "people");
    if (!ids || static_cast<std::size_t>(ids.size()) != points.size())
    {
        throw std::invalid_argument("invalid_people_detections");
    }
    o.m_people.resize(points.size());
    for (std::size_t k = 0; k < points.size(); ++k)
    {
        o.m_people[k] = {ids.data()[k], points[k]};
    }
    if (!t[4].is_none())
    {
        o.m_resolvedId = t[4].cast<std::int64_t>();
    }
    return o;
}

// The packets of one step with the arrays they reference kept alive.
struct Packets
{
    std::vector<DoubleArray> m_keep;
    std::vector<std::vector<std::uint8_t>> m_flags;
    std::vector<StepPacket> m_packets;

    explicit Packets(const py::list& items)
    {
        m_keep.reserve(4 * items.size());
        m_flags.reserve(items.size());
        for (const py::handle& item : items)
        {
            const auto pair = item.cast<py::tuple>();
            if (pair.size() != 2)
            {
                throw std::invalid_argument("invalid_packet");
            }
            const py::dict cloud = pair[1].cast<py::dict>();
            StepPacket packet;
            packet.m_stamp = pair[0].cast<double>();
            packet.m_targetPointsFromCluster =
                cloud.contains("target_points_from_cluster") && py::bool_(cloud["target_points_from_cluster"]);
            CloudView& view = packet.m_cloud;
            m_keep.push_back(Rows(cloud["points"], "points"));
            view.m_points = Span(m_keep.back());
            m_keep.push_back(Rows(cloud["points_odom"], "points_odom"));
            view.m_pointsOdom = Span(m_keep.back());
            if (cloud.contains("target_points_odom"))
            {
                m_keep.push_back(Rows(cloud["target_points_odom"], "target_points_odom"));
                view.m_targetPointsOdom = Span(m_keep.back());
            }
            else if (packet.m_targetPointsFromCluster)
            {
                throw std::invalid_argument("missing_target_points_odom");
            }
            if (cloud.contains("marking_points_odom"))
            {
                m_keep.push_back(Rows(cloud["marking_points_odom"], "marking_points_odom"));
                view.m_markingPointsOdom = Span(m_keep.back());
                view.m_hasMarking = true;
            }
            const BoolArray dynamic = BoolArray::ensure(cloud["dynamic"]);
            if (!dynamic)
            {
                throw std::invalid_argument("invalid_map_array:dynamic");
            }
            m_flags.emplace_back(dynamic.data(), dynamic.data() + dynamic.size());
            view.m_dynamic = m_flags.back();
            const DoubleArray origin = DoubleArray::ensure(cloud["origin"]);
            const DoubleArray origin_odom = DoubleArray::ensure(cloud["origin_odom"]);
            if (!origin || origin.size() < 3 || !origin_odom || origin_odom.size() < 2)
            {
                throw std::invalid_argument("invalid_map_origin");
            }
            view.m_origin = {origin.data()[0], origin.data()[1], origin.data()[2]};
            view.m_originOdom = {origin_odom.data()[0], origin_odom.data()[1],
                                 origin_odom.size() > 2 ? origin_odom.data()[2] : 0.};
            m_packets.push_back(packet);
        }
    }
};

[[nodiscard]] py::dict TuningDict(const Tuning& t)
{
    py::dict d;
    d["step"] = t.m_step;
    d["horizon"] = t.m_horizon;
    d["max_v"] = t.m_maxV;
    d["max_w"] = t.m_maxW;
    d["accel"] = t.m_accel;
    d["yaw_accel"] = t.m_yawAccel;
    d["radius"] = t.m_radius;
    d["margin"] = t.m_margin;
    d["reference_corner_m"] = t.m_referenceCornerM;
    d["margin_weight"] = t.m_marginWeight;
    d["desired_gap"] = t.m_desiredGap;
    d["target_radius"] = t.m_targetRadius;
    d["target_stop_gap"] = t.m_targetStopGap;
    d["target_hard_gap"] = t.m_targetHardGap;
    d["target_clearance_max"] = t.m_targetClearanceMax;
    d["camera_half_angle"] = t.m_cameraHalfAngle;
    d["fov_edge_margin"] = t.m_fovEdgeMargin;
    d["centering_weight"] = t.m_centeringWeight;
    d["fov_soft_angle"] = t.m_fovSoftAngle;
    d["lateral_bearing_start"] = t.m_lateralBearingStart;
    d["lateral_slowdown"] = t.m_lateralSlowdown;
    d["grid_route_planner"] = t.m_gridRouteAstar ? "astar" : "dijkstra";
    d["grid_route_lookahead_m"] = t.m_gridRouteLookaheadM;
    d["turn_slow_angle"] = t.m_turnSlowAngle;
    d["turn_first_angle"] = t.m_turnFirstAngle;
    d["backup_speed"] = t.m_backupSpeed;
    d["backup_min_speed"] = t.m_backupMinSpeed;
    d["backup_stall_s"] = t.m_backupStallS;
    d["chassis_tau_w"] = t.m_chassisTauW;
    d["chassis_tau_v"] = t.m_chassisTauV;
    d["chassis_deadband_lo"] = t.m_chassisDeadbandLo;
    d["chassis_deadband_hi"] = t.m_chassisDeadbandHi;
    d["backup_max_m"] = t.m_backupMaxM;
    d["search_turn_rate"] = t.m_searchTurnRate;
    d["search_exit_turn"] = t.m_searchExitTurn;
    d["search_speed"] = t.m_searchSpeed;
    return d;
}

[[nodiscard]] py::object PriorDict(const std::optional<StartupPrior>& prior)
{
    if (!prior)
    {
        return py::none();
    }
    py::dict d;
    d["radius_m"] = prior->m_radiusM;
    d["center_odom"] = py::make_tuple(prior->m_center[0], prior->m_center[1]);
    d["stamp_s"] = prior->m_stampS;
    d["expires_s"] = prior->m_expiresS ? py::object(py::float_(*prior->m_expiresS)) : py::object(py::none());
    d["motion_started_s"] =
        prior->m_motionStartedS ? py::object(py::float_(*prior->m_motionStartedS)) : py::object(py::none());
    d["motion_threshold_m"] = prior->m_motionThresholdM;
    d["reference_generation"] = prior->m_referenceGeneration;
    d["seeded_cells"] = prior->m_seededCells;
    return d;
}

[[nodiscard]] std::optional<StartupPrior> PriorFrom(const py::handle& value)
{
    if (value.is_none())
    {
        return std::nullopt;
    }
    const auto d = value.cast<py::dict>();
    StartupPrior p;
    p.m_radiusM = d["radius_m"].cast<double>();
    p.m_center = Vector2(d["center_odom"], "invalid_prior_center");
    p.m_stampS = d["stamp_s"].cast<double>();
    p.m_expiresS = OptionalDouble(d["expires_s"]);
    p.m_motionStartedS = OptionalDouble(d["motion_started_s"]);
    p.m_motionThresholdM = d["motion_threshold_m"].cast<double>();
    p.m_referenceGeneration = d["reference_generation"].cast<std::int64_t>();
    p.m_seededCells = d["seeded_cells"].cast<std::int64_t>();
    return p;
}

[[nodiscard]] py::dict ResultDict(StepResult& r)
{
    py::dict d;
    d["active"] = r.m_active;
    d["command"] = Array(r.m_command);
    d["command_pending"] = r.m_commandPending;
    d["state"] = r.m_state;
    d["reason"] = r.m_reason;
    d["tuning"] = TuningDict(r.m_tuning);
    d["path"] = Array(r.m_path);
    d["candidates"] = ExportCandidateDict(r.m_candidates);
    d["approach"] = r.m_approach ? py::object(ExportApproachDict(*r.m_approach)) : py::object(py::none());
    d["planner_activity"] = r.m_plannerActivity;
    d["estimate"] = OptionalArray(r.m_estimate);
    d["velocity_reference"] = Array(r.m_velocityReference);
    d["history_count"] = r.m_historyCount;
    d["obstacle_ids"] = py::array_t<std::int64_t>(static_cast<py::ssize_t>(r.m_obstacleIds.size()), r.m_obstacleIds.data());
    d["map_lower"] = py::array_t<std::int64_t>(2, r.m_mapLower.data());
    d["resolution"] = r.m_resolution;
    d["startup_prior"] = PriorDict(r.m_startupPrior);
    d["startup_consumed"] = r.m_startupConsumed;
    d["trail_points"] = r.m_trailPoints;
    d["trail_retired"] = r.m_trailRetired;
    d["assumed_cells"] = r.m_assumedCells;
    const auto n = static_cast<py::ssize_t>(std::llround(std::sqrt(static_cast<double>(r.m_grid.size()))));
    py::array_t<std::int8_t> grid({n, n});
    if (!r.m_grid.empty())
    {
        std::memcpy(grid.mutable_data(), r.m_grid.data(), r.m_grid.size());
    }
    d["grid"] = grid;
    return d;
}

} // namespace

void BindEngine(py::module_& m)
{
    py::class_<FollowCore>(m, "FollowCore")
        .def(py::init([](const py::handle& tuning, double resolution, int size, double robot_radius,
                         double sweep_radius, const py::handle& pose, double now) {
                 FollowCore::Options options;
                 options.m_mapResolution = resolution;
                 options.m_mapSize = size;
                 options.m_robotRadius = robot_radius;
                 options.m_sweepRadius = sweep_radius;
                 options.m_tuning = ExportTuningFrom(tuning);
                 return new FollowCore(options, Pose(pose), now);
             }),
             py::arg("tuning"), py::arg("resolution"), py::arg("size"), py::arg("robot_radius"),
             py::arg("sweep_radius"), py::arg("pose"), py::arg("now"))
        .def("reset_task", &FollowCore::resetTask)
        .def("reset_reference", [](FollowCore& self, const py::handle& pose, double now) {
            self.resetReference(Pose(pose), now);
        })
        .def("set_startup", [](FollowCore& self, bool consumed, const py::handle& prior) {
            self.setStartup(consumed, PriorFrom(prior));
        })
        .def("step",
             [](FollowCore& self, const py::handle& pose, const py::handle& previous, const py::list& packets,
                const py::handle& observation, double now, std::int64_t reference_generation,
                const py::dict& settings) {
                 const auto robot = Pose(pose);
                 const Point2 measured = Vector2(previous, "invalid_previous");
                 const Packets converted(packets);
                 const auto seen = Observation(observation);
                 StepSettings s;
                 s.m_startupRadius = settings["radius"].cast<double>();
                 s.m_localRadius = settings["local_radius"].cast<double>();
                 s.m_motionThreshold = settings["motion_threshold"].cast<double>();
                 s.m_gridRouteAstar = settings["grid_route_astar"].cast<bool>();
                 s.m_targetClearTtl = settings["target_clear_ttl_s"].cast<double>();
                 s.m_trailClearRadius = settings["trail_clear_radius_m"].cast<double>();
                 s.m_active = settings["active"].cast<bool>();
                 const auto mode = settings["task_mode"].cast<std::string>();
                 if (mode.size() != 1)
                 {
                     throw std::invalid_argument("invalid_task_mode");
                 }
                 s.m_taskMode = mode[0];
                 s.m_recovery = settings["recovery"].cast<bool>();
                 s.m_policy = Policy(settings["policy"]);
                 s.m_tracker = Tracker(settings["tracker"]);
                 if (!settings["object_surface"].is_none())
                 {
                     s.m_objectSurface = Points(settings["object_surface"], "object_surface");
                 }
                 s.m_goalPoint = Optional2(settings["goal_point"], "invalid_goal_point");
                 s.m_goalRemaining = settings["goal_remaining"].cast<double>();
                 s.m_externalCommand = settings["external_command"].cast<bool>();
                 StepResult result;
                 {
                     py::gil_scoped_release release;
                     result = self.step(robot, measured, converted.m_packets, seen, now, reference_generation, s);
                 }
                 return ResultDict(result);
             },
             py::arg("pose"), py::arg("previous"), py::arg("packets"), py::arg("observation"), py::arg("now"),
             py::arg("reference_generation"), py::arg("settings"))
        .def("commit", [](FollowCore& self, const py::handle& command, double now, const py::handle& policy) {
            return Array(self.commit(Vector2(command, "invalid_command"), now, Policy(policy)));
        })
        .def_property_readonly("map", [](const FollowCore& self) { return self.map(); })
        .def_property_readonly("grid_route_planner", [](const FollowCore& self) {
            return self.tuning().m_gridRouteAstar ? "astar" : "dijkstra";
        })
        .def("people_ids", &FollowCore::peopleIds);
}
