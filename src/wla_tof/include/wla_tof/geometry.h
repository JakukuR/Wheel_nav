#ifndef WLA_TOF_GEOMETRY_H
#define WLA_TOF_GEOMETRY_H
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <map>
#include <string>
#include <vector>
namespace wla::tof {
struct GroundPlane {
  Eigen::Vector3d normal=Eigen::Vector3d::UnitZ();
  double offset{};
  std::vector<double> wheelHeights;
  bool available{};
  std::string reason="waiting_for_inputs";
};
struct GeometryState {
  nlohmann::json frames=nlohmann::json::array();
  GroundPlane ground;
};
class RobotGeometry {
 public:
  explicit RobotGeometry(const nlohmann::json& model);
  [[nodiscard]] GeometryState evaluate(const std::map<std::string,double>& positions,
      const Eigen::Quaterniond& body_to_world,const std::string& support_mode,double max_difference) const;
 private:
  nlohmann::json m_model;
};
[[nodiscard]] double HeightAboveGround(const GroundPlane& plane,const Eigen::Vector3d& point);
}  // namespace wla::tof
#endif
