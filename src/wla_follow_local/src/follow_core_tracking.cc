// Python binding of the tracking part of the C++ follow core (phase 2):
// TargetHistory, VisualTrack, PeopleTracks and the two reference curves.
// Inputs are validated here (shape, finiteness where the reference checks it);
// C++ exceptions become Python ValueError / LinAlgError-like errors.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <wla_follow/tracking.h>

namespace py = pybind11;
using wla::follow::Detection;
using wla::follow::PeopleTracks;
using wla::follow::PersonEstimate;
using wla::follow::Point2;
using wla::follow::TargetHistory;
using wla::follow::TrackUpdate;
using wla::follow::VisualTrack;

namespace {

using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;

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
    if (!array || array.ndim() != 1 || array.size() != 2)
    {
        throw std::invalid_argument(what);
    }
    return {array.data()[0], array.data()[1]};
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

[[nodiscard]] py::tuple Result(const TrackUpdate& update)
{
    return py::make_tuple(update.m_position ? py::object(Array(*update.m_position)) : py::object(py::none()),
                          Array(update.m_velocity), std::string(wla::follow::TrackStateName(update.m_state)));
}

[[nodiscard]] py::tuple Estimates(const std::vector<PersonEstimate>& estimates)
{
    const auto n = static_cast<py::ssize_t>(estimates.size());
    py::array_t<std::int64_t> ids(n);
    py::array_t<double> position({n, py::ssize_t(2)}), velocity({n, py::ssize_t(2)}), radius(n), age(n);
    auto* i = ids.mutable_data();
    auto* p = position.mutable_data();
    auto* v = velocity.mutable_data();
    auto* r = radius.mutable_data();
    auto* a = age.mutable_data();
    for (py::ssize_t k = 0; k < n; ++k)
    {
        const PersonEstimate& e = estimates[static_cast<std::size_t>(k)];
        i[k] = e.m_id;
        p[2 * k] = e.m_position[0];
        p[2 * k + 1] = e.m_position[1];
        v[2 * k] = e.m_velocity[0];
        v[2 * k + 1] = e.m_velocity[1];
        r[k] = e.m_radius;
        a[k] = e.m_age;
    }
    return py::make_tuple(ids, position, velocity, radius, age);
}

[[nodiscard]] py::object Optional(const std::optional<double>& value)
{
    return value ? py::object(py::float_(*value)) : py::object(py::none());
}

} // namespace

void BindTracking(py::module_& m)
{
    py::class_<TargetHistory>(m, "TargetHistory")
        .def(py::init([](double window_s, double period_s, double max_gap_s) {
                 return new TargetHistory(TargetHistory::Options{window_s, period_s, max_gap_s});
             }),
             py::arg("window_s") = 5., py::arg("period_s") = .4, py::arg("max_gap_s") = .6)
        .def("update", [](TargetHistory& self, const py::handle& point, double stamp) {
            self.update(Vector2(point, "invalid_target_history_observation"), stamp);
        }, py::arg("point"), py::arg("stamp"))
        .def("points", [](const TargetHistory& self) { return Array(self.points()); },
             "Sampled points and the live endpoint as a new (n, 2) array")
        .def_property_readonly("window_s", [](const TargetHistory& self) { return self.options().m_windowS; })
        .def_property_readonly("period_s", [](const TargetHistory& self) { return self.options().m_periodS; })
        .def_property_readonly("max_gap_s", [](const TargetHistory& self) { return self.options().m_maxGapS; })
        .def_property_readonly("sample_count", &TargetHistory::sampleCount)
        .def_property_readonly("oldest_stamp", [](const TargetHistory& self) { return Optional(self.oldestStamp()); })
        .def_property_readonly("last_observation_stamp",
                               [](const TargetHistory& self) { return Optional(self.lastObservationStamp()); })
        .def_property_readonly("revision", &TargetHistory::revision);

    py::class_<VisualTrack>(m, "VisualTrack")
        .def(py::init([](double timeout, double history_window_s, double history_sample_period_s,
                         double history_max_gap_s, double meas_std, double accel_std, std::optional<double> gate,
                         int gate_max_skips, std::optional<std::pair<double, double>> coast_decay) {
                 VisualTrack::Options options;
                 options.m_timeout = timeout;
                 options.m_history = {history_window_s, history_sample_period_s, history_max_gap_s};
                 options.m_measStd = meas_std;
                 options.m_accelStd = accel_std;
                 options.m_gate = gate;
                 options.m_gateMaxSkips = gate_max_skips;
                 options.m_coastDecay = coast_decay;
                 return new VisualTrack(options);
             }),
             py::arg("timeout"), py::arg("history_window_s"), py::arg("history_sample_period_s"),
             py::arg("history_max_gap_s"), py::arg("meas_std"), py::arg("accel_std"), py::arg("gate"),
             py::arg("gate_max_skips"), py::arg("coast_decay"))
        .def("update", [](VisualTrack& self, const py::handle& z, double t) {
            std::optional<Point2> measurement;
            if (!z.is_none())
            {
                measurement = Vector2(z, "invalid_track_measurement");
            }
            return Result(self.update(measurement, t));
        }, py::arg("z"), py::arg("t"))
        .def("predict", [](const VisualTrack& self, double t) { return Result(self.predict(t)); }, py::arg("t"))
        .def("predicted_position", [](const VisualTrack& self, double t) -> py::object {
            const auto p = self.predictedPosition(t);
            return p ? py::object(Array(*p)) : py::object(py::none());
        }, py::arg("t"))
        .def_property_readonly("x", [](const VisualTrack& self) -> py::object {
            if (!self.state().m_initialised)
            {
                return py::none();
            }
            return py::array_t<double>(4, self.state().m_x.data());
        }, "A new array (x, y, vx, vy), None before the first measurement")
        .def_property_readonly("P", [](const VisualTrack& self) {
            return py::array_t<double>({4, 4}, self.state().m_covariance.data());
        })
        .def_property_readonly("time", [](const VisualTrack& self) -> py::object {
            return self.state().m_initialised ? py::object(py::float_(self.state().m_time)) : py::object(py::none());
        })
        .def_property_readonly("seen", [](const VisualTrack& self) -> py::object {
            return self.state().m_initialised ? py::object(py::float_(self.state().m_seen)) : py::object(py::none());
        })
        .def_property_readonly("skips", [](const VisualTrack& self) { return self.state().m_skips; })
        .def_property_readonly("rejected", [](const VisualTrack& self) { return self.state().m_rejected; })
        .def_property_readonly("trail", &VisualTrack::trail, py::return_value_policy::reference_internal);

    py::class_<PeopleTracks>(m, "PeopleTracks")
        .def(py::init<>())
        .def("update", [](PeopleTracks& self, const py::handle& ids, const py::handle& points, double t) {
            const auto id_array = py::array_t<std::int64_t, py::array::c_style | py::array::forcecast>::ensure(ids);
            const std::vector<Point2> xy = Points(points, "people");
            if (!id_array || static_cast<std::size_t>(id_array.size()) != xy.size())
            {
                throw std::invalid_argument("invalid_people_detections");
            }
            std::vector<Detection> detections(xy.size());
            for (std::size_t k = 0; k < xy.size(); ++k)
            {
                detections[k] = {id_array.data()[k], xy[k]};
            }
            return Estimates(self.update(detections, t));
        }, py::arg("ids"), py::arg("points"), py::arg("t"),
           "(ids, positions, velocities, radii, ages) of the tracks kept, in track order")
        .def("predict", [](const PeopleTracks& self, double t) { return Estimates(self.predict(t)); }, py::arg("t"))
        .def("drop", &PeopleTracks::drop, py::arg("id"))
        .def("ids", &PeopleTracks::ids);

    m.def("bezier_reference", [](const py::handle& history) {
        const auto points = Points(history, "history");
        return Array(wla::follow::BezierReference(points));
    }, py::arg("history"));
    m.def("bspline_corners", [](const py::handle& path, double corner, double step) {
        const auto points = Points(path, "path");
        return Array(wla::follow::BsplineCorners(points, corner, step));
    }, py::arg("path"), py::arg("corner") = .35, py::arg("step") = .05);
}
