#ifndef WLA_GROUND_INCLUDE_GEOMETRY_H
#define WLA_GROUND_INCLUDE_GEOMETRY_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace wla::ground {
enum class Contact : unsigned char { Unknown = 0, Ground = 1, Air = 2 };
struct Wheel {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitY();
  double radius = 0.0;
  double halfWidth = 0.0;
};
struct Plane {
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double offset = 0.0;  // n.dot(point) + offset = 0; normal points upward.
  double residual = 0.0;
};
struct HeightEstimate {
  Plane plane;
  std::array<double, 2> wheelHeights{};
  bool contactKnown = false;
  bool hasEstimate = false;
  std::string reason;
};
[[nodiscard]] Eigen::Quaterniond CheckedQuaternion(const Eigen::Quaterniond& value);
[[nodiscard]] Eigen::Matrix3d FromRpy(const Eigen::Vector3d& rpy);
[[nodiscard]] double CylinderSupport(const Wheel& wheel, const Eigen::Vector3d& normal);
[[nodiscard]] HeightEstimate EstimateFlatGround(const std::array<Wheel, 2>& wheels,
    const Eigen::Quaterniond& body_to_world, const std::array<Contact, 2>& contacts,
    double max_height_difference);
[[nodiscard]] Plane FitGroundPlane(const std::vector<Eigen::Vector3d>& points,
    const Eigen::Vector3d& up, double max_residual, double max_tilt);
[[nodiscard]] Plane TransformPlane(const Plane& body_plane,
    const Eigen::Isometry3d& body_from_sensor);
[[nodiscard]] double VerticalHeight(const Plane& body_plane,
    const Eigen::Quaterniond& body_to_world);

// Sole owner of immutable parsed model; never loads dynamics or hardware APIs.
class WheelModel {
 public:
  explicit WheelModel(const std::string& urdf_path,
      const std::array<std::string, 2>& wheel_links,
      const Eigen::Isometry3d& body_from_model);
  ~WheelModel();
  WheelModel(const WheelModel&) = delete;
  WheelModel& operator=(const WheelModel&) = delete;
  WheelModel(WheelModel&&) = delete;
  WheelModel& operator=(WheelModel&&) = delete;
  [[nodiscard]] std::array<Wheel, 2> evaluate(const std::map<std::string, double>& positions) const;
 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

// Bounded history. No extrapolation; queries require bracketing samples.
class AttitudeBuffer {
 public:
  explicit AttitudeBuffer(double max_span_s, std::size_t capacity = 600);
  ~AttitudeBuffer();
  AttitudeBuffer(const AttitudeBuffer&) = delete;
  AttitudeBuffer& operator=(const AttitudeBuffer&) = delete;
  AttitudeBuffer(AttitudeBuffer&&) = delete;
  AttitudeBuffer& operator=(AttitudeBuffer&&) = delete;
  void reset() noexcept;
  void push(double stamp, const Eigen::Quaterniond& orientation);
  [[nodiscard]] Eigen::Quaterniond sample(double stamp, double& bracket_span) const;
 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};
}  // namespace wla::ground
#endif
