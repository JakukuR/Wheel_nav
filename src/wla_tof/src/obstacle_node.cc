#include "wla_tof/geometry.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cstring>
#include <deque>
#include <fstream>
#include <limits>
#include <optional>
namespace wla::tof {
namespace {
using Json=nlohmann::json;
struct ImuSample { double stamp{};Eigen::Quaterniond q; };
}
class ObstacleNode final:public rclcpp::Node {
 public:
  ObstacleNode():Node("tof_obstacles") {
    const auto path=declare_parameter<std::string>("geometry_file","");std::ifstream file(path);
    if(!file) throw std::invalid_argument("geometry_file_missing");
    file>>m_model;m_geometry=std::make_unique<RobotGeometry>(m_model);
    m_support=declare_parameter<std::string>("support_mode","unknown");
    m_minHeight=declare_parameter<double>("obstacle_min_height_m",.05);
    m_maxHeight=declare_parameter<double>("obstacle_max_height_m",.50);
    m_maxAge=declare_parameter<double>("max_input_age_s",.2);
    m_maxSkew=declare_parameter<double>("max_input_skew_s",.10);
    m_maxDifference=declare_parameter<double>("max_wheel_height_difference_m",.05);
    if(!std::isfinite(m_minHeight) || !std::isfinite(m_maxHeight) || m_minHeight<0 || m_minHeight>=m_maxHeight ||
        !std::isfinite(m_maxAge) || m_maxAge<=0 || m_maxAge>1 || !std::isfinite(m_maxSkew) || m_maxSkew<=0 || m_maxSkew>m_maxAge)
      throw std::invalid_argument("invalid_obstacle_limits");
    if(m_support!="both" && m_support!="left" && m_support!="right" && m_support!="unknown" && m_support!="air") throw std::invalid_argument("invalid_support_mode");
    m_cloudPub=create_publisher<sensor_msgs::msg::PointCloud2>("/r680/tof/points",rclcpp::SensorDataQoS());
    m_statePub=create_publisher<std_msgs::msg::String>("/robot_state/geometry",rclcpp::SensorDataQoS());
    m_statusPub=create_publisher<std_msgs::msg::String>("/r680/tof/ground_status",10);
    m_imuSub=create_subscription<sensor_msgs::msg::Imu>("/imu/data",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Imu::ConstSharedPtr m){
      const double t=rclcpp::Time(m->header.stamp).seconds();const auto& o=m->orientation;Eigen::Quaterniond q(o.w,o.x,o.y,o.z);
      if(m->header.frame_id!="base_Link" || m->orientation_covariance[0]==-1 || !fresh(t) || !q.coeffs().allFinite() || std::abs(q.norm()-1.)>.05 || (!m_imus.empty() && t<=m_imus.back().stamp)) {m_error="invalid_imu";return;}
      m_imus.push_back({t,q.normalized()});while(m_imus.size()>200) m_imus.pop_front();
    });
    m_jointSub=create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,[this](sensor_msgs::msg::JointState::ConstSharedPtr m){
      m_jointStamp=0;m_positions.clear();
      if(m->header.frame_id!="base_Link" || m->name.size()!=m->position.size()) {m_error="invalid_joint_array";return;}
      const double t=rclcpp::Time(m->header.stamp).seconds();if(!fresh(t)) {m_error="stale_joints";return;}
      for(size_t i=0;i<m->name.size();++i) if(!std::isfinite(m->position[i]) || !m_positions.emplace(m->name[i],m->position[i]).second) {m_error="invalid_joint_value";return;}
      m_jointStamp=t;
    });
    m_cloudSub=create_subscription<sensor_msgs::msg::PointCloud2>("/r680/tof/points_raw",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){filter(*m);});
    m_timer=create_wall_timer(std::chrono::milliseconds(50),[this]{publishGeometry();});
  }
  ObstacleNode(const ObstacleNode&)=delete;ObstacleNode& operator=(const ObstacleNode&)=delete;
  ObstacleNode(ObstacleNode&&)=delete;ObstacleNode& operator=(ObstacleNode&&)=delete;
 private:
  bool fresh(double t) const {const double age=now().seconds()-t;return t>0 && age>=-.01 && age<=m_maxAge;}
  std::pair<GeometryState,Eigen::Quaterniond> geometryAt(double stamp) {
    if(!fresh(stamp) || !fresh(m_jointStamp) || std::abs(stamp-m_jointStamp)>m_maxSkew) throw std::invalid_argument("missing_stale_or_skewed_joints");
    if(m_imus.empty()) throw std::invalid_argument("missing_imu");
    const auto it=std::min_element(m_imus.begin(),m_imus.end(),[stamp](const auto& a,const auto& b){return std::abs(a.stamp-stamp)<std::abs(b.stamp-stamp);});
    if(!fresh(it->stamp) || std::abs(it->stamp-stamp)>m_maxSkew) throw std::invalid_argument("stale_or_skewed_imu");
    m_geometryImuStamp=it->stamp;
    return {m_geometry->evaluate(m_positions,it->q,m_support,m_maxDifference),it->q};
  }
  void filter(const sensor_msgs::msg::PointCloud2& input) {
    try {
      const auto state=geometryAt(rclcpp::Time(input.header.stamp).seconds()).first;
      if(!state.ground.available) throw std::invalid_argument(state.ground.reason);
      if(input.header.frame_id!="base_Link" || input.is_bigendian || input.height!=1 || input.point_step!=24 ||
          input.row_step!=input.width*input.point_step || input.data.size()!=input.row_step || input.width>256) throw std::invalid_argument("unexpected_cloud_layout");
      const auto field=[&](const char* name,uint32_t offset,uint8_t type){return std::any_of(input.fields.begin(),input.fields.end(),[&](const auto& f){return f.name==name && f.offset==offset && f.datatype==type && f.count==1;});};
      if(!field("x",0,7) || !field("y",4,7) || !field("z",8,7) || !field("sensor_id",20,2)) throw std::invalid_argument("unexpected_cloud_fields");
      auto out=input;out.data.clear();m_inputPoints=input.width;m_groundRemoved=0;m_highRemoved=0;
      for(size_t i=0;i<input.width;++i) {
        const auto* p=input.data.data()+i*24;float xyz[3];std::memcpy(xyz,p,12);
        if(p[20]>=4) throw std::invalid_argument("invalid_sensor_id");
        const double h=HeightAboveGround(state.ground,Eigen::Vector3d(xyz[0],xyz[1],xyz[2]));
        if(h<m_minHeight) {++m_groundRemoved;continue;}
        if(h>m_maxHeight) {++m_highRemoved;continue;}
        out.data.insert(out.data.end(),p,p+24);
      }
      out.width=out.data.size()/24;out.row_step=out.data.size();m_outputPoints=out.width;m_cloudPub->publish(out);
      m_lastFilter=now().seconds();m_filterError.clear();
    } catch(const std::exception& e) {m_filterError=e.what();}
  }
  void publishGeometry() {
    Json status={{"backend","cpp"},{"method","joint_fk_imu_flat_ground"},{"support_mode",m_support},
      {"contact_source","operator_confirmed_test_setup_not_contact_sensor"},{"geometry_calibrated",false},
      {"min_height_m",m_minHeight},{"max_height_m",m_maxHeight},{"input_points",m_inputPoints},{"output_points",m_outputPoints},
      {"ground_removed",m_groundRemoved},{"above_max_removed",m_highRemoved},{"filtered_available",fresh(m_lastFilter) && m_filterError.empty()},
      {"filter_error",m_filterError},{"sample_time_basis","host_receive_approximation_no_hardware_sync"}};
    Json geometry={{"schema","robot-geometry/v1"},{"frame","base_Link"},{"stamp",now().seconds()},
      {"available",false},{"geometry_calibrated",false},{"source_sha256",m_model.at("source_sha256")}};
    try {
      const auto [state,q]=geometryAt(m_jointStamp);const auto& g=state.ground;
      geometry["available"]=true;geometry["frames"]=state.frames;
      geometry["joint_stamp"]=m_jointStamp;geometry["imu_stamp"]=m_geometryImuStamp;
      geometry["source_stamp"]=std::min(m_jointStamp,m_geometryImuStamp);
      geometry["ground"]={{"available",g.available},{"normal",{g.normal.x(),g.normal.y(),g.normal.z()}},{"offset_m",g.available?Json(g.offset):Json(nullptr)},
        {"wheel_heights_m",g.wheelHeights},{"reason",g.reason}};
      status["ground"]=geometry["ground"];status["geometry_available"]=true;m_error.clear();
    } catch(const std::exception& e) {m_error=e.what();status["geometry_available"]=false;}
    geometry["error"]=m_error;status["geometry_error"]=m_error;
    std_msgs::msg::String msg;msg.data=geometry.dump();m_statePub->publish(msg);msg.data=status.dump();m_statusPub->publish(msg);
  }
  Json m_model;std::unique_ptr<RobotGeometry> m_geometry;std::map<std::string,double> m_positions;std::deque<ImuSample> m_imus;
  std::string m_support,m_error,m_filterError="waiting_for_cloud";
  double m_jointStamp{},m_geometryImuStamp{},m_lastFilter{},m_minHeight{},m_maxHeight{},m_maxAge{},m_maxSkew{},m_maxDifference{};
  size_t m_inputPoints{},m_outputPoints{},m_groundRemoved{},m_highRemoved{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloudPub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_statePub,m_statusPub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_jointSub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloudSub;rclcpp::TimerBase::SharedPtr m_timer;
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<wla::tof::ObstacleNode>());}
  catch(const std::exception& e) {RCLCPP_FATAL(rclcpp::get_logger("tof_obstacles"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
