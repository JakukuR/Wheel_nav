// Python binding of the planning part of the C++ follow core (phase 3):
// HybridFollower (sampled MPC with its DetourPlanner), ApproachGoal and the
// helpers chassis_yaw / approach_tangent. Maps are the C++ RollingMap of
// native_map.NativeRollingDepthMap (its _core); Tuning is read from the Python
// dataclass by attribute. Diagnostics come back as dicts with the reference's
// keys and key order (native_planning.py rounds backed_m like the reference).
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <wla_follow/planning.h>

#include "numpy_ops.h"

namespace py = pybind11;
using wla::follow::ApproachGoal;
using wla::follow::ApproachReport;
using wla::follow::CandidateReport;
using wla::follow::CommandInput;
using wla::follow::DetourReport;
using wla::follow::HybridFollower;
using wla::follow::Obstacle;
using wla::follow::Point2;
using wla::follow::PolicyView;
using wla::follow::RollingMap;
using wla::follow::Tuning;

namespace {

using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;
using IntArray = py::array_t<std::int64_t, py::array::c_style | py::array::forcecast>;

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

[[nodiscard]] std::array<double, 3> Pose(const py::handle& value)
{
    DoubleArray array = DoubleArray::ensure(value);
    if (!array || array.size() < 3)
    {
        throw std::invalid_argument("invalid_pose");
    }
    return {array.data()[0], array.data()[1], array.data()[2]};
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

[[nodiscard]] py::list List(const Point2& p)
{
    py::list out;
    out.append(p[0]);
    out.append(p[1]);
    return out;
}

[[nodiscard]] const RollingMap& Map(const py::handle& value)
{
    return value.cast<const RollingMap&>();
}

// sim_core/common.py Tuning, by attribute.
[[nodiscard]] Tuning TuningFrom(const py::handle& c)
{
    const auto d = [&](const char* name) { return c.attr(name).cast<double>(); };
    Tuning t;
    t.m_step = d("step");
    t.m_horizon = c.attr("horizon").cast<int>();
    if (t.m_horizon < 1)
    {
        throw std::invalid_argument("invalid_tuning_horizon");
    }
    t.m_maxV = d("max_v");
    t.m_maxW = d("max_w");
    t.m_accel = d("accel");
    t.m_yawAccel = d("yaw_accel");
    t.m_radius = d("radius");
    t.m_margin = d("margin");
    t.m_referenceCornerM = d("reference_corner_m");
    t.m_marginWeight = d("margin_weight");
    t.m_desiredGap = d("desired_gap");
    t.m_targetRadius = d("target_radius");
    t.m_targetStopGap = d("target_stop_gap");
    t.m_targetHardGap = d("target_hard_gap");
    t.m_targetClearanceMax = d("target_clearance_max");
    t.m_cameraHalfAngle = d("camera_half_angle");
    t.m_fovEdgeMargin = d("fov_edge_margin");
    t.m_centeringWeight = d("centering_weight");
    t.m_fovSoftAngle = d("fov_soft_angle");
    t.m_lateralBearingStart = d("lateral_bearing_start");
    t.m_lateralSlowdown = d("lateral_slowdown");
    // getattr(c, 'grid_route_planner', 'dijkstra') == 'astar'
    t.m_gridRouteAstar = py::hasattr(c, "grid_route_planner") && py::str(c.attr("grid_route_planner")).cast<std::string>() == "astar";
    t.m_gridRouteLookaheadM = d("grid_route_lookahead_m");
    t.m_turnSlowAngle = d("turn_slow_angle");
    t.m_turnFirstAngle = d("turn_first_angle");
    t.m_backupSpeed = d("backup_speed");
    t.m_backupMinSpeed = d("backup_min_speed");
    t.m_backupStallS = d("backup_stall_s");
    t.m_chassisTauW = d("chassis_tau_w");
    t.m_chassisTauV = d("chassis_tau_v");
    t.m_chassisDeadbandLo = d("chassis_deadband_lo");
    t.m_chassisDeadbandHi = d("chassis_deadband_hi");
    t.m_backupMaxM = d("backup_max_m");
    t.m_searchTurnRate = d("search_turn_rate");
    t.m_searchExitTurn = d("search_exit_turn");
    t.m_searchSpeed = d("search_speed");
    return t;
}

[[nodiscard]] std::vector<Obstacle> Obstacles(const py::handle& positions, const py::handle& velocities,
                                              const py::handle& radii)
{
    const auto p = Points(positions, "obstacle_positions");
    const auto v = Points(velocities, "obstacle_velocities");
    DoubleArray r = DoubleArray::ensure(radii);
    if (!r || static_cast<std::size_t>(r.size()) != p.size() || v.size() != p.size())
    {
        throw std::invalid_argument("invalid_obstacles");
    }
    std::vector<Obstacle> out(p.size());
    for (std::size_t k = 0; k < p.size(); ++k)
    {
        out[k] = {p[k], v[k], r.data()[k]};
    }
    return out;
}

[[nodiscard]] py::object DetourDict(const DetourReport& last)
{
    using Kind = DetourReport::Kind;
    if (last.m_kind == Kind::None)
    {
        return py::none();
    }
    py::dict out;
    if (last.m_kind == Kind::Inactive)
    {
        out["active"] = false;
        return out;
    }
    out["active"] = true;
    const char* side = last.m_left ? "left" : "right";
    switch (last.m_kind)
    {
    case Kind::Holding:
        out["side"] = side;
        out["waypoint"] = List(last.m_waypoint);
        out["holding"] = true;
        break;
    case Kind::Grid:
    {
        out["side"] = "grid";
        out["waypoint"] = List(last.m_waypoint);
        py::list route;
        for (const Point2& p : last.m_route)
        {
            route.append(List(p));
        }
        out["route"] = route;
        out["search"] = last.m_astar ? "astar" : "dijkstra";
        break;
    }
    case Kind::Nothing:
        out["reason"] = "no_clear_edge_nothing_to_look_at";
        break;
    case Kind::Looking:
        out["side"] = side;
        out["looking"] = true;
        out["waypoint"] = List(last.m_waypoint);
        break;
    case Kind::Edge:
        out["side"] = side;
        out["waypoint"] = List(last.m_waypoint);
        out["score"] = last.m_score;
        break;
    default:
        break;
    }
    return out;
}

[[nodiscard]] py::dict CandidateDict(const CandidateReport& report)
{
    using Kind = CandidateReport::Kind;
    py::dict out;
    const auto detour = [&]() -> py::object {
        py::object d = DetourDict(report.m_detour);
        return d.is_none() ? py::object(py::dict()) : d;
    };
    switch (report.m_kind)
    {
    case Kind::Empty:
        break;
    case Kind::Backing:
        out["safe"] = 1;
        out["feasible"] = 1;
        out["forward"] = 0;
        out["backing"] = true;
        out["backed_m"] = report.m_backed;
        out["detour"] = detour();
        break;
    case Kind::Recovery:
        out["safe"] = 1;
        out["feasible"] = 1;
        out["forward"] = 0;
        out["recovery"] = true;
        out["backed_m"] = report.m_backed;
        break;
    case Kind::Looking:
        out["safe"] = 1;
        out["feasible"] = 1;
        out["forward"] = 0;
        out["looking"] = true;
        out["detour"] = detour();
        break;
    case Kind::TurningToRoute:
        out["safe"] = 1;
        out["feasible"] = 1;
        out["forward"] = 0;
        out["turning_to_route"] = true;
        out["detour"] = detour();
        break;
    case Kind::Sampled:
    {
        out["safe"] = report.m_safe;
        out["feasible"] = report.m_feasible;
        out["forward"] = report.m_forward;
        py::list reasons;
        for (const int r : report.m_reasons)
        {
            reasons.append(r);
        }
        out["reasons"] = reasons;
        out["forward_total"] = report.m_forwardTotal;
        out["forward_known"] = report.m_forwardKnown;
        out["forward_static"] = report.m_forwardStatic;
        out["forward_people"] = report.m_forwardPeople;
        if (report.m_hasDetour)
        {
            out["detour"] = detour();
        }
        break;
    }
    }
    return out;
}

[[nodiscard]] py::dict ApproachDict(const ApproachReport& report)
{
    const auto& p = report.m_plan;
    py::dict out;
    out["point"] = Array(p.m_point);
    out["path"] = Array(p.m_path);
    out["yaw"] = p.m_yaw;
    out["goal_clearance_m"] = p.m_goalClearanceM;
    out["fallback"] = p.m_fallback;
    out["reachable_cells"] = p.m_reachableCells;
    out["generation"] = p.m_generation;
    out["current_clearance_m"] = report.m_currentClearanceM;
    out["endpoint_distance_m"] = report.m_endpointDistanceM;
    out["stable_s"] = report.m_stableS;
    return out;
}

} // namespace

// For the FollowCore binding (follow_core_engine.cc).
py::dict ExportCandidateDict(const CandidateReport& report)
{
    return CandidateDict(report);
}

py::dict ExportApproachDict(const ApproachReport& report)
{
    return ApproachDict(report);
}

Tuning ExportTuningFrom(const py::handle& tuning)
{
    return TuningFrom(tuning);
}

void BindPlanning(py::module_& m)
{
    py::class_<HybridFollower>(m, "HybridFollower")
        .def(py::init<>())
        .def("reset", &HybridFollower::reset)
        .def("command",
             [](HybridFollower& self, const py::handle& map, const py::handle& tuning, const py::handle& pose,
                const py::handle& previous, const py::handle& target, const py::handle& velocity,
                const py::handle& history, const py::handle& obstacle_positions,
                const py::handle& obstacle_velocities, const py::handle& obstacle_radii, const std::string& status,
                const py::handle& policy, const py::handle& policy_time, const py::handle& reference_path,
                const py::handle& face_point, const py::handle& measured, bool recovery) {
                 CommandInput in;
                 in.m_pose = Pose(pose);
                 in.m_previous = Vector2(previous, "invalid_previous");
                 in.m_target = Optional2(target, "invalid_target");
                 in.m_velocity = Vector2(velocity, "invalid_velocity");
                 in.m_history = Points(history, "history");
                 in.m_obstacles = Obstacles(obstacle_positions, obstacle_velocities, obstacle_radii);
                 in.m_status = status;
                 if (!policy.is_none())
                 {
                     // (search_arrival_m, search_heading, search_started, search_side,
                     //  translation_allowed(policy_time), active_search(policy_time))
                     const auto t = policy.cast<py::tuple>();
                     if (t.size() != 6)
                     {
                         throw std::invalid_argument("invalid_policy_view");
                     }
                     PolicyView view;
                     view.m_searchArrivalM = t[0].cast<double>();
                     view.m_searchHeading = OptionalDouble(t[1]);
                     view.m_searchStarted = OptionalDouble(t[2]);
                     view.m_searchSide = t[3].cast<double>();
                     view.m_translationAllowed = t[4].cast<bool>();
                     view.m_activeSearch = t[5].cast<bool>();
                     in.m_policy = view;
                 }
                 in.m_policyTime = OptionalDouble(policy_time);
                 if (!reference_path.is_none())
                 {
                     in.m_referencePath = Points(reference_path, "reference_path");
                 }
                 in.m_facePoint = Optional2(face_point, "invalid_face_point");
                 in.m_measured = Optional2(measured, "invalid_measured");
                 in.m_recovery = recovery;
                 const Tuning c = TuningFrom(tuning);
                 const RollingMap& rolling = Map(map);
                 Point2 out;
                 {
                     py::gil_scoped_release release;
                     out = self.command(rolling, c, in);
                 }
                 return Array(out);
             },
             py::arg("map"), py::arg("tuning"), py::arg("pose"), py::arg("previous"), py::arg("target"),
             py::arg("velocity"), py::arg("history"), py::arg("obstacle_positions"), py::arg("obstacle_velocities"),
             py::arg("obstacle_radii"), py::arg("status"), py::arg("policy"), py::arg("policy_time"),
             py::arg("reference_path"), py::arg("face_point"), py::arg("measured"), py::arg("recovery"))
        .def("grid_route",
             [](HybridFollower& self, const py::handle& map, const py::handle& p, const py::handle& target,
                const py::handle& tuning, double window_m, double clearance_weight) -> py::object {
                 const Point2 a = Vector2(p, "invalid_point");
                 const Point2 b = Vector2(target, "invalid_target");
                 const Tuning c = TuningFrom(tuning);
                 const RollingMap& rolling = Map(map);
                 std::optional<std::vector<Point2>> route;
                 {
                     py::gil_scoped_release release;
                     route = self.detour().gridRoute(rolling, a, b, c, window_m, clearance_weight);
                 }
                 return route ? py::object(Array(*route)) : py::object(py::none());
             },
             py::arg("map"), py::arg("p"), py::arg("target"), py::arg("tuning"), py::arg("window_m") = 3.5,
             py::arg("clearance_weight") = 2.)
        .def("set_detour_trigger", [](HybridFollower& self, const std::string& trigger) {
            if (trigger != "line" && trigger != "path")
            {
                throw std::invalid_argument("invalid_detour_trigger");
            }
            self.detour().setTrigger(trigger == "path");
        })
        .def_property_readonly("last_infeasible", &HybridFollower::lastInfeasible)
        .def_property_readonly("last_candidates", [](const HybridFollower& self) { return CandidateDict(self.lastCandidates()); })
        .def_property_readonly("last_reference_path", [](const HybridFollower& self) { return Array(self.lastReferencePath()); })
        .def_property_readonly("stuck_cycles", &HybridFollower::stuckCycles)
        .def_property_readonly("backed", &HybridFollower::backed)
        .def_property_readonly("trail", [](const HybridFollower& self) { return Array(self.trail()); })
        .def_property_readonly("detour_last", [](HybridFollower& self) { return DetourDict(self.detour().last()); });

    py::class_<ApproachGoal>(m, "ApproachGoal")
        .def(py::init<>())
        .def("select",
             [](ApproachGoal& self, const py::handle& pose, const py::handle& surface, const py::handle& map,
                double now) -> py::object {
                 const auto p = Pose(pose);
                 const auto s = Points(surface, "surface");
                 const RollingMap& rolling = Map(map);
                 std::optional<ApproachReport> report;
                 {
                     py::gil_scoped_release release;
                     report = self.select(p, s, rolling, now);
                 }
                 return report ? py::object(ApproachDict(*report)) : py::object(py::none());
             },
             py::arg("pose"), py::arg("surface"), py::arg("map"), py::arg("now"))
        .def_property_readonly("planned_at", &ApproachGoal::plannedAt)
        .def_property_readonly("generation", &ApproachGoal::generation)
        .def_property_readonly("stable_since", &ApproachGoal::stableSince)
        .def_property_readonly("center", [](const ApproachGoal& self) -> py::object {
            return self.center() ? py::object(Array(*self.center())) : py::object(py::none());
        })
        .def_property_readonly("has_plan", [](const ApproachGoal& self) { return self.plan().has_value(); });

    m.def("approach_samples", [](const py::handle& path) { return Array(wla::follow::ApproachSamples(Points(path, "path"))); });
    m.def("chassis_yaw", [](double w, const py::handle& tuning) { return wla::follow::ChassisYaw(w, TuningFrom(tuning)); });
    m.def("approach_tangent",
          [](const py::handle& pose, const py::handle& command, const py::handle& positions,
             const py::handle& velocities, const py::handle& radii, const py::handle& target, double radius) {
              const auto obstacles = Obstacles(positions, velocities, radii);
              return wla::follow::ApproachTangent(Pose(pose), Vector2(command, "invalid_command"), obstacles,
                                                  Vector2(target, "invalid_target"), radius);
          });
    // Test hooks: the numpy argsort and scipy dijkstra ports.
    m.def("_argsort", [](const py::handle& values) {
        DoubleArray array = DoubleArray::ensure(values);
        if (!array || array.ndim() != 1)
        {
            throw std::invalid_argument("invalid_argsort_input");
        }
        const std::vector<double> v(array.data(), array.data() + array.size());
        const auto order = wla::follow::detail::ArgSort(v);
        return py::array_t<std::int64_t>(static_cast<py::ssize_t>(order.size()), order.data());
    });
    m.def("_dijkstra", [](int n, const py::handle& src, const py::handle& dst, const py::handle& weights, int source) {
        IntArray s = IntArray::ensure(src), d = IntArray::ensure(dst);
        DoubleArray w = DoubleArray::ensure(weights);
        if (!s || !d || !w || s.size() != d.size() || s.size() != w.size())
        {
            throw std::invalid_argument("invalid_graph");
        }
        const std::vector<int> a(s.data(), s.data() + s.size()), b(d.data(), d.data() + d.size());
        const std::vector<double> c(w.data(), w.data() + w.size());
        std::vector<double> distance;
        std::vector<int> predecessor;
        wla::follow::detail::Dijkstra(wla::follow::detail::BuildCsr(n, a, b, c), source, distance, predecessor);
        return py::make_tuple(py::array_t<double>(static_cast<py::ssize_t>(distance.size()), distance.data()),
                              py::array_t<int>(static_cast<py::ssize_t>(predecessor.size()), predecessor.data()));
    });
}
