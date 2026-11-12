#include "wla_ground/geometry.h"

#include <Eigen/Eigenvalues>
#include <tinyxml2.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <set>
#include <sstream>
#include <stdexcept>

namespace wla::ground {
namespace {
constexpr double UNIT_TOLERANCE = 1e-6;
void Require(bool condition, const std::string& reason) {
  if (!condition) throw std::invalid_argument(reason);
}
Eigen::Vector3d Vector(const char* text) {
  if (text == nullptr) return Eigen::Vector3d::Zero();
  std::istringstream input(text);
  Eigen::Vector3d value;
  std::string extra;
  Require(bool(input >> value.x() >> value.y() >> value.z()) && !(input >> extra) &&
      value.allFinite(), "invalid_urdf_vector");
  return value;
}
std::string Attribute(const tinyxml2::XMLElement* element, const char* name) {
  Require(element != nullptr && element->Attribute(name) != nullptr, "missing_urdf_attribute");
  return element->Attribute(name);
}
Eigen::Isometry3d Origin(const tinyxml2::XMLElement* element) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  if (element != nullptr) {
    out.translation() = Vector(element->Attribute("xyz"));
    out.linear() = FromRpy(Vector(element->Attribute("rpy")));
  }
  return out;
}
void CheckNormal(const Eigen::Vector3d& value) {
  Require(value.allFinite() && std::abs(value.norm()-1.0) < UNIT_TOLERANCE, "invalid_plane_normal");
}
void CheckTransform(const Eigen::Isometry3d& transform) {
  Require(transform.matrix().allFinite() &&
      (transform.linear().transpose()*transform.linear()-Eigen::Matrix3d::Identity()).norm() < UNIT_TOLERANCE &&
      std::abs(transform.linear().determinant()-1.0) < UNIT_TOLERANCE &&
      (transform.matrix().row(3)-Eigen::RowVector4d(0, 0, 0, 1)).norm() < UNIT_TOLERANCE,
      "invalid_rigid_transform");
}
}  // namespace

Eigen::Quaterniond CheckedQuaternion(const Eigen::Quaterniond& value) {
  Require(value.coeffs().allFinite() && std::abs(value.norm()-1.0) < 0.05, "invalid_quaternion");
  return value.normalized();
}
Eigen::Matrix3d FromRpy(const Eigen::Vector3d& rpy) {
  Require(rpy.allFinite(), "nonfinite_rpy");
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}
double CylinderSupport(const Wheel& wheel, const Eigen::Vector3d& normal) {
  CheckNormal(normal);
  Require(wheel.center.allFinite() && wheel.axis.allFinite() &&
      std::abs(wheel.axis.norm()-1.0) < UNIT_TOLERANCE &&
      std::isfinite(wheel.radius) && wheel.radius > 0.0 &&
      std::isfinite(wheel.halfWidth) && wheel.halfWidth >= 0.0, "invalid_wheel_geometry");
  const double cosine = std::clamp(std::abs(normal.dot(wheel.axis)), 0.0, 1.0);
  return wheel.radius*std::sqrt(std::max(0.0, 1.0-cosine*cosine)) + wheel.halfWidth*cosine;
}
HeightEstimate EstimateFlatGround(const std::array<Wheel, 2>& wheels,
    const Eigen::Quaterniond& body_to_world, const std::array<Contact, 2>& contacts,
    double max_height_difference) {
  Require(std::isfinite(max_height_difference) && max_height_difference > 0.0, "invalid_height_difference_limit");
  HeightEstimate out;
  out.plane.normal = CheckedQuaternion(body_to_world).conjugate()*Eigen::Vector3d::UnitZ();
  std::vector<int> selected;
  bool unknown = false;
  for (int side = 0; side < 2; ++side) {
    Require(contacts[side] == Contact::Ground || contacts[side] == Contact::Unknown ||
        contacts[side] == Contact::Air, "invalid_contact_state");
    out.wheelHeights[side] = CylinderSupport(wheels[side], out.plane.normal)-out.plane.normal.dot(wheels[side].center);
    if (contacts[side] == Contact::Ground) selected.push_back(side);
    unknown = unknown || contacts[side] == Contact::Unknown;
  }
  if (selected.empty() && !unknown) { out.reason = "airborne_no_height_observation"; return out; }
  if (selected.empty()) {
    for (int side = 0; side < 2; ++side) if (contacts[side] == Contact::Unknown) selected.push_back(side);
  } else { out.contactKnown = true; }
  for (int side : selected) out.plane.offset += out.wheelHeights[side]/double(selected.size());
  if (selected.size() == 2 && std::abs(out.wheelHeights[0]-out.wheelHeights[1]) > max_height_difference) {
    out.reason = "flat_ground_contact_height_disagreement"; return out;
  }
  if (!std::isfinite(out.plane.offset) || out.plane.offset <= 0.0) {
    out.reason = "nonpositive_body_height"; return out;
  }
  out.hasEstimate = true;
  out.reason = out.contactKnown ? "flat_ground_contact_model" : "contact_unknown_flat_ground_candidate";
  return out;
}
Plane FitGroundPlane(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& up,
    double max_residual, double max_tilt) {
  CheckNormal(up);
  Require(points.size() >= 3 && points.size() <= 4, "need_three_or_four_tof_points");
  Require(std::isfinite(max_residual) && max_residual > 0.0 &&
      std::isfinite(max_tilt) && max_tilt > 0.0 && max_tilt < 1.5707963267948966, "invalid_plane_limits");
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& point : points) { Require(point.allFinite(), "nonfinite_tof_point"); mean += point/double(points.size()); }
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& point : points) covariance += (point-mean)*(point-mean).transpose()/double(points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  Require(solver.info() == Eigen::Success && solver.eigenvalues()[1] > 1e-8, "degenerate_tof_plane");
  Plane out;
  out.normal = solver.eigenvectors().col(0);
  if (out.normal.dot(up) < 0.0) out.normal = -out.normal;
  Require(out.normal.dot(up) >= std::cos(max_tilt), "tof_plane_tilt_exceeded");
  out.offset = -out.normal.dot(mean);
  for (const auto& point : points) out.residual = std::max(out.residual, std::abs(out.normal.dot(point)+out.offset));
  Require(out.residual <= max_residual && out.offset > 0.0, "tof_plane_residual_or_height_invalid");
  return out;
}
Plane TransformPlane(const Plane& body_plane, const Eigen::Isometry3d& body_from_sensor) {
  CheckNormal(body_plane.normal); CheckTransform(body_from_sensor);
  Require(std::isfinite(body_plane.offset), "nonfinite_plane_offset");
  return {body_from_sensor.linear().transpose()*body_plane.normal,
      body_plane.offset+body_plane.normal.dot(body_from_sensor.translation()), body_plane.residual};
}
double VerticalHeight(const Plane& body_plane, const Eigen::Quaterniond& body_to_world) {
  CheckNormal(body_plane.normal);
  const double nz = (CheckedQuaternion(body_to_world)*body_plane.normal).z();
  Require(std::isfinite(body_plane.offset) && body_plane.offset > 0.0 && nz > 1e-3, "ground_not_below_body");
  return body_plane.offset/nz;
}

struct WheelModel::Impl {
  struct Joint {
    std::string name, parent, child, type;
    Eigen::Isometry3d origin;
    Eigen::Vector3d axis;
  };
  std::vector<Joint> chain;
  std::string root;
  std::array<std::string, 2> wheelLinks;
  std::array<Eigen::Isometry3d, 2> collisionOrigins;
  std::array<double, 2> radii{}, halfWidths{};
  Eigen::Isometry3d bodyFromModel;
};
WheelModel::WheelModel(const std::string& urdf_path, const std::array<std::string, 2>& wheel_links,
    const Eigen::Isometry3d& body_from_model) : m_impl(std::make_unique<Impl>()) {
  CheckTransform(body_from_model);
  m_impl->bodyFromModel = body_from_model;
  m_impl->wheelLinks = wheel_links;
  tinyxml2::XMLDocument doc;
  Require(doc.LoadFile(urdf_path.c_str()) == tinyxml2::XML_SUCCESS, "urdf_load_failed");
  const auto* robot = doc.FirstChildElement("robot");
  Require(robot != nullptr, "invalid_urdf");
  std::set<std::string> links, children;
  for (auto* link = robot->FirstChildElement("link"); link != nullptr; link = link->NextSiblingElement("link")) {
    const auto name = Attribute(link, "name");
    Require(links.insert(name).second, "duplicate_urdf_link");
    for (int side = 0; side < 2; ++side) if (name == wheel_links[side]) {
      const auto* collision = link->FirstChildElement("collision");
      Require(collision != nullptr, "missing_wheel_collision");
      const auto* geometry = collision->FirstChildElement("geometry");
      const auto* cylinder = geometry != nullptr ? geometry->FirstChildElement("cylinder") : nullptr;
      Require(cylinder != nullptr, "wheel_requires_analytic_cylinder");
      m_impl->radii[side] = cylinder->DoubleAttribute("radius");
      m_impl->halfWidths[side] = cylinder->DoubleAttribute("length")/2.0;
      Require(m_impl->radii[side] > 0.0 && m_impl->halfWidths[side] > 0.0 &&
          std::isfinite(m_impl->radii[side]) && std::isfinite(m_impl->halfWidths[side]), "invalid_cylinder_dimensions");
      m_impl->collisionOrigins[side] = Origin(collision->FirstChildElement("origin"));
    }
  }
  std::vector<Impl::Joint> pending;
  std::set<std::string> joint_names;
  for (auto* joint = robot->FirstChildElement("joint"); joint != nullptr; joint = joint->NextSiblingElement("joint")) {
    Impl::Joint value;
    value.name = Attribute(joint, "name"); value.type = Attribute(joint, "type");
    Require(joint_names.insert(value.name).second, "duplicate_urdf_joint");
    Require(value.type == "fixed" || value.type == "revolute" || value.type == "continuous", "unsupported_joint_type");
    value.parent = Attribute(joint->FirstChildElement("parent"), "link");
    value.child = Attribute(joint->FirstChildElement("child"), "link");
    Require(links.contains(value.parent) && links.contains(value.child) && children.insert(value.child).second, "invalid_urdf_parent");
    value.origin = Origin(joint->FirstChildElement("origin"));
    const auto* axis = joint->FirstChildElement("axis");
    value.axis = axis != nullptr ? Vector(axis->Attribute("xyz")) : Eigen::Vector3d::UnitX();
    if (value.type != "fixed") { Require(value.axis.norm() > 1e-9, "invalid_joint_axis"); value.axis.normalize(); }
    pending.push_back(value);
  }
  std::vector<std::string> roots;
  std::set_difference(links.begin(), links.end(), children.begin(), children.end(), std::back_inserter(roots));
  Require(roots.size() == 1 && pending.size()+1 == links.size(), "urdf_requires_single_tree");
  m_impl->root = roots[0];
  std::set<std::string> ready{roots[0]};
  while (!pending.empty()) {
    bool progress = false;
    for (auto it = pending.begin(); it != pending.end();) {
      if (ready.contains(it->parent)) {
        ready.insert(it->child); m_impl->chain.push_back(*it); it = pending.erase(it); progress = true;
      } else ++it;
    }
    Require(progress, "disconnected_or_cyclic_urdf");
  }
  for (int side = 0; side < 2; ++side) Require(links.contains(wheel_links[side]) && m_impl->radii[side] > 0.0, "missing_wheel_link");
}
WheelModel::~WheelModel() = default;
std::array<Wheel, 2> WheelModel::evaluate(const std::map<std::string, double>& positions) const {
  std::map<std::string, Eigen::Isometry3d> transforms{{m_impl->root, m_impl->bodyFromModel}};
  for (const auto& joint : m_impl->chain) {
    Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
    if (joint.type != "fixed") {
      const auto it = positions.find(joint.name);
      Require(it != positions.end() && std::isfinite(it->second), "missing_or_nonfinite_joint:"+joint.name);
      motion.linear() = Eigen::AngleAxisd(it->second, joint.axis).toRotationMatrix();
    }
    transforms.emplace(joint.child, transforms.at(joint.parent)*joint.origin*motion);
  }
  std::array<Wheel, 2> out;
  for (int side = 0; side < 2; ++side) {
    const Eigen::Isometry3d collision = transforms.at(m_impl->wheelLinks[side])*m_impl->collisionOrigins[side];
    out[side] = {collision.translation(), collision.linear()*Eigen::Vector3d::UnitZ(), m_impl->radii[side], m_impl->halfWidths[side]};
  }
  return out;
}

struct AttitudeBuffer::Impl {
  struct Sample { double stamp; Eigen::Quaterniond orientation; };
  std::deque<Sample> samples;
  double maxSpan;
  std::size_t capacity;
};
AttitudeBuffer::AttitudeBuffer(double max_span_s, std::size_t capacity) : m_impl(std::make_unique<Impl>()) {
  Require(std::isfinite(max_span_s) && max_span_s > 0.0 && capacity >= 2 && capacity <= 10000, "invalid_attitude_buffer");
  m_impl->maxSpan = max_span_s; m_impl->capacity = capacity;
}
AttitudeBuffer::~AttitudeBuffer() = default;
void AttitudeBuffer::reset() noexcept { m_impl->samples.clear(); }
void AttitudeBuffer::push(double stamp, const Eigen::Quaterniond& orientation) {
  Require(std::isfinite(stamp) && stamp > 0.0, "invalid_attitude_stamp");
  const auto q = CheckedQuaternion(orientation);
  Require(m_impl->samples.empty() || stamp > m_impl->samples.back().stamp, "nonincreasing_attitude_stamp");
  m_impl->samples.push_back({stamp, q});
  while (m_impl->samples.size() > m_impl->capacity) m_impl->samples.pop_front();
}
Eigen::Quaterniond AttitudeBuffer::sample(double stamp, double& bracket_span) const {
  Require(std::isfinite(stamp) && !m_impl->samples.empty(), "missing_attitude");
  const auto& samples = m_impl->samples;
  Require(stamp >= samples.front().stamp && stamp <= samples.back().stamp, "attitude_not_bracketed");
  const auto it = std::lower_bound(samples.begin(), samples.end(), stamp,
      [](const Impl::Sample& value, double time) { return value.stamp < time; });
  if (it->stamp == stamp) { bracket_span = 0.0; return it->orientation; }
  const auto before = std::prev(it);
  bracket_span = it->stamp-before->stamp;
  Require(bracket_span <= m_impl->maxSpan, "attitude_gap_exceeded");
  return before->orientation.slerp((stamp-before->stamp)/bracket_span, it->orientation).normalized();
}
}  // namespace wla::ground
