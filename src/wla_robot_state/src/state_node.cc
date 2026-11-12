#include "wla_robot_state/state.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>

namespace wla::state {
class StateNode final : public rclcpp::Node {
 public:
  StateNode() : Node("robot_state_adapter") {
    m_maxAge = declare_parameter<double>("max_input_age_s", .1);
    if (!std::isfinite(m_maxAge) || m_maxAge <= 0.0 || m_maxAge > 1.0) throw std::invalid_argument("invalid_source_age");
    m_bodyFrame = declare_parameter<std::string>("imu_frame", "base_Link");
    m_jointsPublisher = create_publisher<sensor_msgs::msg::JointState>(declare_parameter<std::string>("joint_topic", "/joint_states"), 10);
    m_attitudePublisher = create_publisher<geometry_msgs::msg::QuaternionStamped>("/robot_state/attitude", rclcpp::SensorDataQoS());
    m_rpyPublisher = create_publisher<geometry_msgs::msg::Vector3Stamped>("/robot_state/rpy", rclcpp::SensorDataQoS());
    m_statusPublisher = create_publisher<std_msgs::msg::String>("/robot_state/status", 10);
    m_stateSubscription = create_subscription<std_msgs::msg::String>(declare_parameter<std::string>("state_topic", "/motor35_powered/state"), 10,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          try {
            const auto q = ParseJointFeedback(msg->data);
            const auto value = nlohmann::json::parse(msg->data);
            m_attitudeSource = value.value("attitude_source", std::string("unknown"));
            sensor_msgs::msg::JointState out;
            out.header.stamp = now(); out.header.frame_id = m_bodyFrame;
            out.name.assign(JOINT_NAMES.begin(), JOINT_NAMES.end()); out.position.assign(q.begin(), q.end());
            // Source has no qd, effort or original per-joint ROS sampling stamps.
            m_jointsPublisher->publish(out); m_lastJointReceive = now().seconds(); ++m_jointCount; m_jointError.clear();
          } catch (const std::exception& error) { m_lastJointReceive = 0.0; m_jointError = error.what(); }
        });
    m_imuSubscription = create_subscription<sensor_msgs::msg::Imu>(declare_parameter<std::string>("imu_topic", "/imu/data"), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          try {
            const double stamp = rclcpp::Time(msg->header.stamp).seconds();
            if (msg->header.frame_id != m_bodyFrame || msg->orientation_covariance[0] == -1.0 ||
                !fresh(stamp) || stamp <= m_lastImuStamp) throw std::invalid_argument("invalid_imu_frame_or_timestamp");
            const auto q = CheckOrientation(Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z));
            if (!m_haveStartYaw) {
              const auto rotation = q.toRotationMatrix();
              if (std::hypot(rotation(0,0), rotation(1,0)) < .2) throw std::invalid_argument("startup_heading_near_vertical");
              m_worldToStart = Eigen::Quaterniond(Eigen::AngleAxisd(-ToRpy(q).z(), Eigen::Vector3d::UnitZ()));
              m_haveStartYaw = true;
            }
            const auto relative = (m_worldToStart*q).normalized();
            const auto rpy = ToRpy(relative);
            geometry_msgs::msg::QuaternionStamped attitude;
            attitude.header = msg->header; attitude.header.frame_id = "imu_start_gravity";
            attitude.quaternion.x = relative.x(); attitude.quaternion.y = relative.y();
            attitude.quaternion.z = relative.z(); attitude.quaternion.w = relative.w();
            geometry_msgs::msg::Vector3Stamped euler; euler.header = attitude.header;
            euler.vector.x = rpy.x(); euler.vector.y = rpy.y(); euler.vector.z = rpy.z();
            m_attitudePublisher->publish(attitude); m_rpyPublisher->publish(euler);
            m_lastImuStamp = stamp; ++m_imuCount; m_imuError.clear(); m_imuRejected = false;
          } catch (const std::exception& error) { m_imuRejected = true; m_imuError = error.what(); }
        });
    m_timer = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      std_msgs::msg::String msg;
      msg.data = nlohmann::json{{"backend", "cpp"}, {"joint_state_available", fresh(m_lastJointReceive)},
          {"attitude_available", fresh(m_lastImuStamp) && !m_imuRejected}, {"joint_messages", m_jointCount}, {"imu_messages", m_imuCount},
          {"joint_timestamp_basis", "controller_state_receive"}, {"imu_timestamp_basis", "preserved_source_header"},
          {"attitude_source", m_attitudeSource}, {"body_frame", m_bodyFrame}, {"attitude_reference", "imu_start_gravity"},
          {"joint_error", m_jointError}, {"imu_error", m_imuError}, {"position_estimated", false}, {"tf_published", false}}.dump();
      m_statusPublisher->publish(msg);
    });
  }
  StateNode(const StateNode&) = delete;
  StateNode& operator=(const StateNode&) = delete;
  StateNode(StateNode&&) = delete;
  StateNode& operator=(StateNode&&) = delete;
 private:
  [[nodiscard]] bool fresh(double stamp) const {
    const double age = get_clock()->now().seconds()-stamp;
    return stamp > 0.0 && age >= -.002 && age <= m_maxAge;
  }
  double m_maxAge = .1, m_lastJointReceive = 0.0, m_lastImuStamp = 0.0;
  bool m_haveStartYaw = false, m_imuRejected = false;
  Eigen::Quaterniond m_worldToStart = Eigen::Quaterniond::Identity();
  std::string m_bodyFrame, m_jointError, m_imuError, m_attitudeSource = "unknown";
  unsigned long long m_jointCount = 0, m_imuCount = 0;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_jointsPublisher;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr m_attitudePublisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr m_rpyPublisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_statusPublisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_stateSubscription;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscription;
  rclcpp::TimerBase::SharedPtr m_timer;
};
}  // namespace wla::state
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<wla::state::StateNode>()); }
  catch (const std::exception& error) { RCLCPP_FATAL(rclcpp::get_logger("robot_state_adapter"), "%s", error.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
