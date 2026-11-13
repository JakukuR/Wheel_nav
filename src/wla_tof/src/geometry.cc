#include "wla_tof/geometry.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
namespace wla::tof {
namespace {
Eigen::Vector3d Vector(const nlohmann::json& value) {
  if(!value.is_array() || value.size()!=3) throw std::invalid_argument("geometry_vector_size");
  Eigen::Vector3d out(value[0].get<double>(),value[1].get<double>(),value[2].get<double>());
  if(!out.allFinite()) throw std::invalid_argument("geometry_nonfinite_vector");
  return out;
}
Eigen::Isometry3d Pose(const nlohmann::json& item) {
  const auto r=Vector(item.at("rpy"));Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  t.translation()=Vector(item.at("xyz"));
  t.linear()=(Eigen::AngleAxisd(r.z(),Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(r.y(),Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(r.x(),Eigen::Vector3d::UnitX())).toRotationMatrix();return t;
}
nlohmann::json Frame(const std::string& name,const std::string& parent,const std::string& kind,const Eigen::Isometry3d& t) {
  const auto p=t.translation();const Eigen::Quaterniond q(t.linear());
  return {{"name",name},{"parent",parent},{"kind",kind},{"p",{p.x(),p.y(),p.z()}},{"q",{q.x(),q.y(),q.z(),q.w()}}};
}
}
RobotGeometry::RobotGeometry(const nlohmann::json& model):m_model(model) {
  if(model.at("root")!="base_Link" || model.at("wheels").size()!=2 || model.at("joints").size()!=8)
    throw std::invalid_argument("invalid_robot_geometry");
  std::map<std::string,double> zero;std::set<std::string> children;
  for(const auto& j:model.at("joints")) {
    const std::string name=j.at("name");
    if(!zero.emplace(name,0.).second || !children.insert(j.at("child").get<std::string>()).second) throw std::invalid_argument("duplicate_geometry_joint");
    if(j.at("type")!="revolute" && j.at("type")!="continuous") throw std::invalid_argument("unsupported_joint_type");
  }
  (void)evaluate(zero,Eigen::Quaterniond::Identity(),"unknown",.05);
}
GeometryState RobotGeometry::evaluate(const std::map<std::string,double>& positions,
    const Eigen::Quaterniond& orientation,const std::string& support_mode,double max_difference) const {
  if(!orientation.coeffs().allFinite() || std::abs(orientation.norm()-1.)>.05 || !std::isfinite(max_difference) || max_difference<=0)
    throw std::invalid_argument("invalid_geometry_attitude_or_limit");
  if(support_mode!="both" && support_mode!="left" && support_mode!="right" && support_mode!="unknown" && support_mode!="air")
    throw std::invalid_argument("invalid_support_mode");
  GeometryState out;std::map<std::string,Eigen::Isometry3d> transforms{{"base_Link",Eigen::Isometry3d::Identity()}};
  out.frames.push_back(Frame("base_Link","","imu",transforms.at("base_Link")));
  std::set<std::string> done;
  while(done.size()<m_model.at("joints").size()) {
    bool progress=false;
    for(const auto& j:m_model.at("joints")) {
      const std::string name=j.at("name"),parent=j.at("parent"),child=j.at("child");
      if(done.contains(name) || !transforms.contains(parent)) continue;
      const auto it=positions.find(name);
      if(it==positions.end() || !std::isfinite(it->second)) throw std::invalid_argument("missing_joint:"+name);
      const auto axis=Vector(j.at("axis"));if(axis.norm()<1e-9) throw std::invalid_argument("zero_joint_axis");
      auto t=transforms.at(parent)*Pose(j);t.rotate(Eigen::AngleAxisd(it->second,axis.normalized()));transforms.emplace(child,t);
      auto frame=Frame(child,parent,"joint",t);frame["joint"]=name;frame["angle"]=it->second;out.frames.push_back(frame);
      done.insert(name);progress=true;
    }
    if(!progress) throw std::invalid_argument("geometry_tree_not_connected");
  }
  for(const auto& s:m_model.at("sensors")) out.frames.push_back(Frame(s.at("name"),"base_Link",s.at("kind"),Pose(s)));
  out.ground.normal=orientation.normalized().conjugate()*Eigen::Vector3d::UnitZ();
  for(const auto& w:m_model.at("wheels")) {
    const Eigen::Isometry3d t=transforms.at(w.at("link").get<std::string>())*Pose(w);
    const auto axis=t.linear()*Eigen::Vector3d::UnitZ();const double a=std::clamp(std::abs(out.ground.normal.dot(axis)),0.,1.);
    const double radius=w.at("radius"),half_width=w.at("half_width");
    if(!std::isfinite(radius) || radius<=0 || !std::isfinite(half_width) || half_width<0) throw std::invalid_argument("invalid_wheel_dimensions");
    const double support=radius*std::sqrt(std::max(0.,1-a*a))+half_width*a;
    out.ground.wheelHeights.push_back(support-out.ground.normal.dot(t.translation()));
    auto frame=Frame(w.at("link").get<std::string>()+"_collision",w.at("link"),"wheel",t);
    frame["radius"]=radius;frame["half_width"]=half_width;out.frames.push_back(frame);
  }
  auto& g=out.ground;
  if(support_mode=="unknown" || support_mode=="air") {g.reason="wheel_contact_not_confirmed";return out;}
  if(support_mode=="both" && std::abs(g.wheelHeights[0]-g.wheelHeights[1])>max_difference) {g.reason="wheel_heights_disagree_on_flat_ground";return out;}
  g.offset=support_mode=="both"?(g.wheelHeights[0]+g.wheelHeights[1])*.5:g.wheelHeights[support_mode=="left"?0:1];
  if(g.offset<=0 || !std::isfinite(g.offset)) {g.reason="ground_not_below_body";return out;}
  g.available=true;g.reason="operator_confirmed_"+support_mode+"_contact_flat_ground";return out;
}
double HeightAboveGround(const GroundPlane& plane,const Eigen::Vector3d& point) {
  if(!plane.available || !point.allFinite()) throw std::invalid_argument("ground_or_point_unavailable");
  return plane.normal.dot(point)+plane.offset;
}
}  // namespace wla::tof
