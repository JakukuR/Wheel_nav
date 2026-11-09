// Python binding of the C++ follow core (_wla_follow_core). Ring state is
// exposed as read-only views kept alive by the owning object; derived fields
// are returned as new arrays (the Python reference replaces them on every
// rebuild, so a caller holding one must not see it change).
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <wla_follow/rolling_map.h>

namespace py = pybind11;
using wla::follow::CloudView;
using wla::follow::RollingMap;

void BindTracking(py::module_& m);  // follow_core_tracking.cc
void BindPlanning(py::module_& m);  // follow_core_planning.cc

namespace {

using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;
using BoolArray = py::array_t<bool, py::array::c_style | py::array::forcecast>;

[[nodiscard]] DoubleArray Rows(const py::handle& value, const char* name)
{
    DoubleArray array = DoubleArray::ensure(value);
    if (!array)
    {
        throw std::invalid_argument(std::string("invalid_map_array:") + name);
    }
    if (array.size() % 3 != 0)
    {
        throw std::invalid_argument(std::string("invalid_map_rows:") + name);
    }
    return array;
}

[[nodiscard]] std::span<const double> Span(const DoubleArray& array)
{
    return {array.data(), static_cast<std::size_t>(array.size())};
}

template <typename T>
[[nodiscard]] py::array View(const py::object& owner, const T* data, std::vector<py::ssize_t> shape)
{
    std::vector<py::ssize_t> strides(shape.size());
    py::ssize_t stride = sizeof(T);
    for (std::size_t k = shape.size(); k-- > 0;)
    {
        strides[k] = stride;
        stride *= shape[k];
    }
    py::array view(py::dtype::of<T>(), shape, strides, data, owner);
    view.attr("setflags")(py::arg("write") = false);
    return view;
}

template <typename T, typename Out = T>
[[nodiscard]] py::array Copy(const T* data, py::ssize_t n)
{
    py::array_t<Out> out({n, n});
    auto* dst = out.mutable_data();
    for (py::ssize_t k = 0; k < n * n; ++k)
    {
        dst[k] = static_cast<Out>(data[k]);
    }
    return out;
}

void RequireFields(const RollingMap& map)
{
    if (!map.hasFields())
    {
        throw py::attribute_error("rolling map fields are not built yet");
    }
}

void RequireWindow(const RollingMap& map)
{
    if (!map.hasWindow())
    {
        throw py::attribute_error("rolling map window is not initialised yet");
    }
}

[[nodiscard]] std::array<double, 2> Point2(const py::handle& value)
{
    DoubleArray array = DoubleArray::ensure(value);
    if (!array || array.size() < 2)
    {
        throw std::invalid_argument("invalid_map_position");
    }
    return {array.data()[0], array.data()[1]};
}

} // namespace

PYBIND11_MODULE(_wla_follow_core, m)
{
    m.doc() = "C++ follow core: rolling depth map (phase 1), tracking (phase 2) and planning (phase 3) of PY_CPP_ARCHITECTURE.md";

    py::class_<RollingMap>(m, "RollingMap")
        .def(py::init([](double resolution, int size, double ttl, double robot_radius, double target_clear_ttl) {
                 RollingMap::Options options;
                 options.m_resolution = resolution;
                 options.m_size = size;
                 options.m_ttl = ttl;
                 options.m_robotRadius = robot_radius;
                 options.m_targetClearTtl = target_clear_ttl;
                 return new RollingMap(options);
             }),
             py::arg("resolution"), py::arg("size"), py::arg("ttl"), py::arg("robot_radius"), py::arg("target_clear_ttl"))
        .def("slide", [](RollingMap& self, const py::handle& position) {
            const auto p = Point2(position);
            self.slide(p[0], p[1]);
        })
        .def("initial_clear_patch", [](RollingMap& self, const py::handle& position, double t, double radius,
                                       bool only_unknown, bool hold_until_motion) {
            const auto p = Point2(position);
            return self.initialClearPatch(p[0], p[1], t, radius, only_unknown, hold_until_motion);
        })
        .def("advance_startup_prior", &RollingMap::advanceStartupPrior, py::arg("t"), py::arg("release"))
        .def("update", [](RollingMap& self, const py::list& clouds, const py::handle& pose, double t, bool rebuild) {
            DoubleArray pose_array = DoubleArray::ensure(pose);
            if (!pose_array || pose_array.size() < 2)
            {
                throw std::invalid_argument("invalid_map_pose");
            }
            const std::array<double, 3> robot{pose_array.data()[0], pose_array.data()[1],
                                              pose_array.size() > 2 ? pose_array.data()[2] : 0.};
            // Keep every converted array alive until the update returns.
            std::vector<DoubleArray> keep;
            std::vector<BoolArray> keep_flags;
            std::vector<std::vector<std::uint8_t>> flags;
            std::vector<CloudView> views;
            keep.reserve(4 * clouds.size());
            keep_flags.reserve(clouds.size());
            flags.reserve(clouds.size());
            for (const py::handle& item : clouds)
            {
                const py::dict cloud = py::reinterpret_borrow<py::dict>(item);
                CloudView view;
                keep.push_back(Rows(cloud["points"], "points"));
                view.m_points = Span(keep.back());
                keep.push_back(Rows(cloud["points_odom"], "points_odom"));
                view.m_pointsOdom = Span(keep.back());
                if (cloud.contains("target_points_odom"))
                {
                    keep.push_back(Rows(cloud["target_points_odom"], "target_points_odom"));
                    view.m_targetPointsOdom = Span(keep.back());
                }
                if (cloud.contains("marking_points_odom"))
                {
                    keep.push_back(Rows(cloud["marking_points_odom"], "marking_points_odom"));
                    view.m_markingPointsOdom = Span(keep.back());
                    view.m_hasMarking = true;
                }
                keep_flags.push_back(BoolArray::ensure(cloud["dynamic"]));
                if (!keep_flags.back())
                {
                    throw std::invalid_argument("invalid_map_array:dynamic");
                }
                const bool* dynamic = keep_flags.back().data();
                flags.emplace_back(dynamic, dynamic + keep_flags.back().size());
                view.m_dynamic = flags.back();
                const DoubleArray origin = DoubleArray::ensure(cloud["origin"]);
                const DoubleArray origin_odom = DoubleArray::ensure(cloud["origin_odom"]);
                if (!origin || origin.size() < 3 || !origin_odom || origin_odom.size() < 2)
                {
                    throw std::invalid_argument("invalid_map_origin");
                }
                view.m_origin = {origin.data()[0], origin.data()[1], origin.data()[2]};
                view.m_originOdom = {origin_odom.data()[0], origin_odom.data()[1],
                                     origin_odom.size() > 2 ? origin_odom.data()[2] : 0.};
                views.push_back(view);
            }
            py::gil_scoped_release release;
            self.update(views, robot, t, rebuild);
        }, py::arg("clouds"), py::arg("pose"), py::arg("t"), py::arg("rebuild"))
        .def("drop_established", [](const RollingMap& self, const py::handle& points, const py::handle& trail,
                                    double t, double radius) {
            const DoubleArray p = Rows(points, "points");
            const DoubleArray r = Rows(trail, "trail");
            std::vector<double> kept;
            {
                py::gil_scoped_release release;
                kept = self.dropEstablished(Span(p), Span(r), t, radius);
            }
            py::array_t<double> out({static_cast<py::ssize_t>(kept.size() / 3), py::ssize_t(3)});
            if (!kept.empty())
            {
                std::memcpy(out.mutable_data(), kept.data(), kept.size() * sizeof(double));
            }
            return out;
        })
        .def("retire_person_trail", [](RollingMap& self, const py::handle& trail, double t, double radius,
                                       double window, double max_gap) {
            const DoubleArray r = Rows(trail, "trail");
            py::gil_scoped_release release;
            return self.retirePersonTrail(Span(r), t, radius, window, max_gap);
        })
        .def("rebuild", [](RollingMap& self, double t, bool reuse) {
            py::gil_scoped_release release;
            self.rebuild(t, reuse);
        }, py::arg("t"), py::arg("reuse") = false)
        .def("slots", [](const RollingMap& self, const py::handle& xy) {
            DoubleArray array = DoubleArray::ensure(xy);
            if (!array || array.size() % 2 != 0)
            {
                throw std::invalid_argument("invalid_slots_input");
            }
            std::vector<std::int64_t> a, b;
            self.slots({array.data(), static_cast<std::size_t>(array.size())}, a, b);
            return py::make_tuple(py::array_t<std::int64_t>(static_cast<py::ssize_t>(a.size()), a.data()),
                                  py::array_t<std::int64_t>(static_cast<py::ssize_t>(b.size()), b.data()));
        })
        .def_property_readonly("res", &RollingMap::resolution)
        .def_property_readonly("n", &RollingMap::size)
        .def_property_readonly("ttl", &RollingMap::ttl)
        .def_property_readonly("robot_radius", &RollingMap::robotRadius)
        .def_property("target_clear_ttl", &RollingMap::targetClearTtl, &RollingMap::setTargetClearTtl)
        .def_property("local_assumed_free_radius", &RollingMap::localFreeRadius, &RollingMap::setLocalFreeRadius)
        .def_property("local_assumed_free_center",
            [](const RollingMap& self) -> py::object {
                if (!self.hasLocalFreeCenter())
                {
                    return py::none();
                }
                const auto c = self.localFreeCenter();
                return py::array_t<double>(2, c.data());
            },
            [](RollingMap& self, const py::handle& value) {
                const auto c = Point2(value);
                self.setLocalFreeCenter(c[0], c[1]);
            })
        .def_property_readonly("version", &RollingMap::version)
        .def_property_readonly("window_id", &RollingMap::windowId)
        .def_property_readonly("build_id", &RollingMap::buildId)
        .def_property_readonly("has_window", &RollingMap::hasWindow)
        .def_property_readonly("has_fields", &RollingMap::hasFields)
        .def_property_readonly("lower", [](const RollingMap& self) {
            const auto l = self.lower();
            return py::array_t<std::int64_t>(2, l.data());
        })
        .def("ring_views", [](py::object self_object) {
            const auto& self = self_object.cast<const RollingMap&>();
            const py::ssize_t n = self.size();
            return py::make_tuple(View(self_object, self.logData(), {n, n}),
                                  View(self_object, self.stampData(), {n, n}),
                                  View(self_object, self.targetClearStampData(), {n, n}),
                                  View(self_object, self.occupiedSinceData(), {n, n}),
                                  View(self_object, self.tagsData(), {n, n, 2}));
        }, "Read-only (log, stamp, target_clear_stamp, occupied_since, tags) views of the ring state")
        .def("window_arrays", [](const RollingMap& self) {
            RequireWindow(self);
            const auto n = static_cast<py::ssize_t>(self.size());
            const auto lower = self.lower();
            py::array_t<std::int64_t> cells({n, n, py::ssize_t(2)});
            py::array_t<std::int64_t> ix({n, n}), iy({n, n});
            auto* c = cells.mutable_data();
            auto* x = ix.mutable_data();
            auto* y = iy.mutable_data();
            for (py::ssize_t i = 0; i < n; ++i)
            {
                for (py::ssize_t j = 0; j < n; ++j)
                {
                    const std::int64_t cx = i + lower[0], cy = j + lower[1];
                    c[2 * (i * n + j)] = cx;
                    c[2 * (i * n + j) + 1] = cy;
                    x[i * n + j] = ((cx % n) + n) % n;
                    y[i * n + j] = ((cy % n) + n) % n;
                }
            }
            return py::make_tuple(cells, ix, iy);
        }, "New (cells, ix, iy) arrays of the current window")
        .def("fields", [](const RollingMap& self) {
            RequireFields(self);
            const auto n = static_cast<py::ssize_t>(self.size());
            return py::make_tuple(Copy<std::uint8_t, bool>(self.sensorKnownData(), n),
                                  Copy<std::uint8_t, bool>(self.occupiedData(), n),
                                  Copy<std::uint8_t, bool>(self.knownData(), n),
                                  Copy<std::uint8_t, bool>(self.assumedFreeData(), n),
                                  Copy<double>(self.distData(), n),
                                  Copy<double>(self.freeDistData(), n));
        }, "New (sensor_known, occupied, known, assumed_free, dist, free_dist) arrays");

    BindTracking(m);
    BindPlanning(m);
}
