#include "wla_ground/geometry.h"
#include "wla_ground_estimation/msg/ground_estimate.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>

namespace wla::ground {
namespace {
using Json = nlohmann::json;
using GroundMessage = wla_ground_estimation::msg::GroundEstimate;
constexpr std::array<const char*, 8> MOTOR_NAMES = {"abad_L_Joint", "abad_R_Joint",
    "hip_L_Joint", "hip_R_Joint", "knee_L_Joint", "knee_R_Joint", "wheel_L_Joint", "wheel_R_Joint"};
geometry_msgs::msg::Vector3 ToVector(const Eigen::Vector3d& value) {
  geometry_msgs::msg::Vector3 out; out.x = value.x(); out.y = value.y(); out.z = value.z(); return out;
}
Eigen::Vector3d FromJsonVector(const Json& value) {
  if (!value.is_array() || value.size() != 3) throw std::invalid_argument("invalid_mount_vector");
  Eigen::Vector3d out(value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>());
  if (!out.allFinite()) throw std::invalid_argument("nonfinite_mount_vector");
  return out;
}
}  // namespace

class GroundNode final : public rclcpp::Node {
 public:
  GroundNode() : Node("ground_estimator") {
    m_bodyFrame = declare_parameter<std::string>("body_frame", "base_link");
    m_imuFrame = declare_parameter<std::string>("imu_frame", "base_Link");
    m_source = declare_parameter<std::string>("ground_source", "kinematic_flat");
    if (m_source != "kinematic_flat" && m_source != "tof_plane") throw std::invalid_argument("invalid_ground_source");
    m_axesVerified = declare_parameter<bool>("imu_axes_verified", false);
    m_mountsVerified = declare_parameter<bool>("mount_geometry_verified", false);
    m_tofAxesVerified = declare_parameter<bool>("tof_axes_verified", false);
    m_sampleTimesVerified = declare_parameter<bool>("sample_times_verified", false);
    m_maxAge = declare_parameter<double>("max_input_age_s", 0.10);
    m_maxImuSpan = declare_parameter<double>("max_imu_bracket_s", 0.02);
    m_maxTofSkew = declare_parameter<double>("max_tof_skew_s", 0.01);
    m_maxHeightDifference = declare_parameter<double>("max_wheel_height_difference_m", 0.03);
    m_maxPlaneResidual = declare_parameter<double>("max_tof_plane_residual_m", 0.015);
    m_maxPlaneTilt = declare_parameter<double>("max_ground_tilt_rad", 0.61);
    for (double value : {m_maxAge, m_maxImuSpan, m_maxTofSkew, m_maxHeightDifference, m_maxPlaneResidual, m_maxPlaneTilt}) {
      if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument("invalid_positive_parameter");
    }
    if (m_maxAge > 1.0 || m_maxImuSpan > m_maxAge || m_maxTofSkew > m_maxAge || m_maxPlaneTilt >= 1.5707963267948966)
      throw std::invalid_argument("invalid_time_or_tilt_budget");
    const auto xyz = declare_parameter<std::vector<double>>("body_from_model_xyz_m", {0.020, 0.0, -0.0005});
    const auto rpy = declare_parameter<std::vector<double>>("body_from_model_rpy_rad", {0.0, 0.0, 0.0});
    if (xyz.size() != 3 || rpy.size() != 3) throw std::invalid_argument("invalid_model_transform");
    Eigen::Isometry3d body_from_model = Eigen::Isometry3d::Identity();
    body_from_model.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    body_from_model.linear() = FromRpy(Eigen::Vector3d(rpy[0], rpy[1], rpy[2]));
    m_model = std::make_unique<WheelModel>(declare_parameter<std::string>("model_urdf", ""),
        std::array<std::string, 2>{"wheel_L_Link", "wheel_R_Link"}, body_from_model);
    m_attitudes = std::make_unique<AttitudeBuffer>(m_maxImuSpan);
    const auto mounts_file = declare_parameter<std::string>("sensor_mounts_json", "");
    std::ifstream stream(mounts_file);
    if (!stream) throw std::invalid_argument("cannot_open_sensor_mounts");
    const Json mounts = Json::parse(stream);
    if (mounts.at("parent_frame") != m_bodyFrame) throw std::invalid_argument("mount_parent_frame_mismatch");
    for (const auto& [name, value] : mounts.at("sensors").items()) {
      if (!value.at("installed").get<bool>()) continue;
      const auto& q = value.at("quaternion_xyzw");
      if (q.size() != 4 || value.at("parent") != m_bodyFrame) throw std::invalid_argument("invalid_mount_frame");
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      transform.translation() = FromJsonVector(value.at("xyz_m"));
      transform.linear() = CheckedQuaternion(Eigen::Quaterniond(q.at(3).get<double>(), q.at(0).get<double>(),
          q.at(1).get<double>(), q.at(2).get<double>())).toRotationMatrix();
      m_mounts.emplace(name, transform);
    }
    m_publisher = create_publisher<GroundMessage>("/ground_estimation/plane", 10);
    m_statusPublisher = create_publisher<std_msgs::msg::String>("/ground_estimation/status", 10);
    m_sensorPoses = create_publisher<geometry_msgs::msg::PoseArray>("/ground_estimation/sensor_poses", 10);
    m_imuSubscription = create_subscription<sensor_msgs::msg::Imu>(declare_parameter<std::string>("imu_topic", "/imu/data"),
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          try {
            const double stamp = rclcpp::Time(msg->header.stamp).seconds();
            if (msg->header.frame_id != m_imuFrame || msg->orientation_covariance[0] == -1.0 ||
                !fresh(stamp, now().seconds())) throw std::invalid_argument("invalid_imu_frame_orientation_or_time");
            m_attitudes->push(stamp, Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z));
            ++m_imuCount;
          } catch (const std::exception& error) { m_attitudes->reset(); invalidate(error.what()); }
        });
    m_jointSubscription = create_subscription<sensor_msgs::msg::JointState>(declare_parameter<std::string>("joint_topic", "/joint_states"),
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          try {
            if (msg->name.size() != msg->position.size()) throw std::invalid_argument("joint_state_size_mismatch");
            JointInput input; input.stamp = rclcpp::Time(msg->header.stamp).seconds(); input.standard = true;
            for (std::size_t i = 0; i < msg->name.size(); ++i)
              if (!input.positions.emplace(msg->name[i], msg->position[i]).second) throw std::invalid_argument("duplicate_joint_name");
            acceptJoints(std::move(input)); m_lastStandardReceive = now().seconds();
          } catch (const std::exception& error) { m_joints.reset(); invalidate(error.what()); }
        });
    m_stateSubscription = create_subscription<std_msgs::msg::String>(declare_parameter<std::string>("state_topic", "/motor35_powered/state"),
        10, [this](std_msgs::msg::String::ConstSharedPtr msg) { receiveState(msg->data); });
    const auto tof_names = declare_parameter<std::vector<std::string>>("tof_frames",
        {"front_tof_L", "front_tof_R", "back_tof_L", "back_tof_R"});
    const auto tof_topics = declare_parameter<std::vector<std::string>>("tof_topics",
        {"/tof/front_left/range", "/tof/front_right/range", "/tof/back_left/range", "/tof/back_right/range"});
    if (tof_names.size() != 4 || tof_topics.size() != 4) throw std::invalid_argument("need_four_tof_bindings");
    for (std::size_t i = 0; i < 4; ++i) {
      if (!m_mounts.contains(tof_names[i])) throw std::invalid_argument("unknown_tof_mount");
      const auto name = tof_names[i];
      m_tofSubscriptions.push_back(create_subscription<sensor_msgs::msg::Range>(tof_topics[i], rclcpp::SensorDataQoS(),
          [this, name](sensor_msgs::msg::Range::ConstSharedPtr msg) {
            m_ranges.erase(name);
            const double stamp = rclcpp::Time(msg->header.stamp).seconds();
            if (msg->header.frame_id != name || !fresh(stamp, now().seconds()) ||
                !std::isfinite(msg->range) || !std::isfinite(msg->min_range) || !std::isfinite(msg->max_range) ||
                msg->min_range < 0.0F || msg->max_range <= msg->min_range ||
                msg->range < msg->min_range || msg->range > msg->max_range) return;
            // Range +X axis must be established physically before tof_axes_verified.
            m_ranges.emplace(name, RangeInput{stamp, m_mounts.at(name)*Eigen::Vector3d(msg->range, 0.0, 0.0)});
          }));
    }
    m_timer = create_wall_timer(std::chrono::milliseconds(20), [this]() { update(); });
  }
  GroundNode(const GroundNode&) = delete;
  GroundNode& operator=(const GroundNode&) = delete;
  GroundNode(GroundNode&&) = delete;
  GroundNode& operator=(GroundNode&&) = delete;

 private:
  struct JointInput { double stamp = 0.0; bool standard = false; std::map<std::string, double> positions; };
  struct RangeInput { double stamp; Eigen::Vector3d point; };
  [[nodiscard]] bool fresh(double stamp, double time) const noexcept {
    return std::isfinite(stamp) && stamp > 0.0 && time-stamp >= -0.002 && time-stamp <= m_maxAge;
  }
  void invalidate(const std::string& reason) { m_lastError = reason; }
  void acceptJoints(JointInput input) {
    if (!fresh(input.stamp, now().seconds())) throw std::invalid_argument("stale_or_future_joint_input");
    if (m_joints && input.stamp <= m_joints->stamp) throw std::invalid_argument("nonincreasing_joint_input");
    (void)m_model->evaluate(input.positions);  // Checks required names and finite values.
    m_joints = std::move(input); ++m_jointCount;
  }
  void receiveState(const std::string& text) {
    try {
      const auto value = Json::parse(text);
      const auto session = value.at("session").get<std::string>();
      if (!m_session.empty() && session != m_session) { m_joints.reset(); m_attitudes->reset(); m_contactTime = 0.0; }
      m_session = session;
      m_contacts = {Contact::Unknown, Contact::Unknown}; m_contactTime = now().seconds();
      if (value.contains("support_observer")) {
        const auto& support = value.at("support_observer");
        if (support.value("enabled", 0.0) != 0.0 && support.value("calibration_verified", 0.0) != 0.0) {
          for (int side = 0; side < 2; ++side) {
            const std::string prefix = side == 0 ? "L_" : "R_";
            const int state = support.at(prefix+"state").get<int>();
            if (support.at(prefix+"valid").get<double>() != 0.0 && (state == 1 || state == 2))
              m_contacts[side] = static_cast<Contact>(state);
          }
        }
      }
      if (now().seconds()-m_lastStandardReceive <= m_maxAge) return;
      if (!value.at("ready").get<bool>() || !value.at("gate").get<std::string>().empty())
        throw std::invalid_argument("controller_feedback_not_ready");
      const auto& positions = value.at("q_rad");
      if (positions.size() != MOTOR_NAMES.size()) throw std::invalid_argument("motor_q_array_size");
      JointInput input; input.stamp = now().seconds();
      for (std::size_t i = 0; i < MOTOR_NAMES.size(); ++i) input.positions.emplace(MOTOR_NAMES[i], positions.at(i).get<double>());
      acceptJoints(std::move(input));  // Receipt approximation, never labeled sampled time.
    } catch (const std::exception& error) { m_contacts = {Contact::Unknown, Contact::Unknown}; m_joints.reset(); invalidate(error.what()); }
  }
  void update() {
    const double time = now().seconds();
    if (m_lastNow > 0.0 && time < m_lastNow) { m_joints.reset(); m_attitudes->reset(); m_ranges.clear(); invalidate("ros_clock_went_backward"); }
    m_lastNow = time;
    GroundMessage out; out.header.frame_id = m_bodyFrame; out.header.stamp = now();
    out.method = m_source; out.reason = m_lastError.empty() ? "waiting_for_inputs" : m_lastError;
    out.imu_axes_verified = m_axesVerified;
    out.vertical_height_m = std::numeric_limits<double>::quiet_NaN();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    try {
      if (!m_joints) throw std::invalid_argument(out.reason);
      const auto& input = *m_joints;
      out.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(input.stamp*1e9)));
      out.input_age_s = time-input.stamp;
      out.timing_basis = input.standard ? "joint_header_and_imu_header" : "controller_state_receipt_approximate";
      out.timing_verified = input.standard && m_sampleTimesVerified;
      if (!fresh(input.stamp, time)) throw std::invalid_argument("stale_joint_input");
      orientation = m_attitudes->sample(input.stamp, out.imu_bracket_span_s);
      auto contacts = fresh(m_contactTime, time) ? m_contacts : std::array<Contact, 2>{Contact::Unknown, Contact::Unknown};
      for (int side = 0; side < 2; ++side) out.contact_state[side] = static_cast<unsigned char>(contacts[side]);
      const auto wheels = m_model->evaluate(input.positions);
      const auto flat = EstimateFlatGround(wheels, orientation, contacts, m_maxHeightDifference);
      out.wheel_heights_m = flat.wheelHeights;
      Plane plane;
      bool supported = false;
      if (m_source == "kinematic_flat") {
        out.has_estimate = flat.hasEstimate; out.reason = flat.reason;
        plane = flat.plane; supported = flat.contactKnown;
      } else {
        if (!m_tofAxesVerified) throw std::invalid_argument("tof_axes_not_verified");
        std::vector<Eigen::Vector3d> points;
        double oldest = input.stamp, newest = input.stamp;
        for (const auto& [name, range] : m_ranges) {
          (void)name;
          if (fresh(range.stamp, time) && std::abs(range.stamp-input.stamp) <= m_maxTofSkew) {
            points.push_back(range.point); oldest = std::min(oldest, range.stamp); newest = std::max(newest, range.stamp);
          }
        }
        if (newest-oldest > m_maxTofSkew) throw std::invalid_argument("tof_time_skew_exceeded");
        plane = FitGroundPlane(points, orientation.conjugate()*Eigen::Vector3d::UnitZ(), m_maxPlaneResidual, m_maxPlaneTilt);
        out.has_estimate = true; supported = true; out.reason = "tof_local_plane_approximate_time_group";
      }
      if (out.has_estimate) {
        out.normal = ToVector(plane.normal); out.offset_m = plane.offset;
        out.vertical_height_m = VerticalHeight(plane, orientation); out.fit_residual_m = plane.residual;
        for (const auto& [name, mount] : m_mounts) {
          const auto sensor_plane = TransformPlane(plane, mount);
          out.sensor_names.push_back(name); out.sensor_ground_normals.push_back(ToVector(sensor_plane.normal));
          out.sensor_ground_offsets_m.push_back(sensor_plane.offset);
        }
        out.valid = supported && m_axesVerified && m_mountsVerified && out.timing_verified;
        if (!m_axesVerified) out.reason += ";imu_axes_not_verified";
        if (!m_mountsVerified) out.reason += ";drawing_mount_geometry_not_verified";
        if (!out.timing_verified) out.reason += ";sample_times_not_verified";
      }
    } catch (const std::exception& error) { out.valid = false; out.has_estimate = false; out.reason = error.what(); }
    m_publisher->publish(out);
    std_msgs::msg::String status;
    status.data = Json{{"backend", "cpp"}, {"has_estimate", out.has_estimate}, {"valid", out.valid},
        {"method", out.method}, {"reason", out.reason}, {"imu_count", m_imuCount}, {"joint_count", m_jointCount},
        {"timing_basis", out.timing_basis}, {"height_m", out.has_estimate ? Json(out.vertical_height_m) : Json(nullptr)},
        {"wheel_heights_m", out.wheel_heights_m}, {"contact", out.contact_state},
        {"input_age_s", out.input_age_s}, {"imu_bracket_span_s", out.imu_bracket_span_s},
        {"sensor_names", out.sensor_names}, {"local_reference_only", true}, {"tf_published", false}}.dump();
    m_statusPublisher->publish(status);
    if (out.valid) {
      geometry_msgs::msg::PoseArray poses; poses.header = out.header; poses.header.frame_id = "ground_estimation/local_reference";
      for (const auto& name : out.sensor_names) {
        const auto& mount = m_mounts.at(name);
        const Eigen::Vector3d position = orientation*mount.translation()+Eigen::Vector3d(0.0, 0.0, out.vertical_height_m);
        const Eigen::Quaterniond q = orientation*Eigen::Quaterniond(mount.linear());
        geometry_msgs::msg::Pose pose;
        pose.position.x = position.x(); pose.position.y = position.y(); pose.position.z = position.z();
        pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
        poses.poses.push_back(pose);
      }
      m_sensorPoses->publish(poses);
    }
  }
  std::string m_bodyFrame, m_imuFrame, m_source, m_session, m_lastError;
  bool m_axesVerified = false, m_mountsVerified = false, m_tofAxesVerified = false, m_sampleTimesVerified = false;
  double m_maxAge = 0.1, m_maxImuSpan = 0.02, m_maxTofSkew = 0.01;
  double m_maxHeightDifference = 0.03, m_maxPlaneResidual = 0.015, m_maxPlaneTilt = 0.61;
  double m_lastStandardReceive = -1.0, m_contactTime = 0.0, m_lastNow = 0.0;
  unsigned long long m_imuCount = 0, m_jointCount = 0;
  std::unique_ptr<WheelModel> m_model;
  std::unique_ptr<AttitudeBuffer> m_attitudes;
  std::optional<JointInput> m_joints;
  std::array<Contact, 2> m_contacts{Contact::Unknown, Contact::Unknown};
  std::map<std::string, Eigen::Isometry3d> m_mounts;
  std::map<std::string, RangeInput> m_ranges;
  rclcpp::Publisher<GroundMessage>::SharedPtr m_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_statusPublisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_sensorPoses;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscription;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_jointSubscription;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_stateSubscription;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> m_tofSubscriptions;
  rclcpp::TimerBase::SharedPtr m_timer;
};
}  // namespace wla::ground

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<wla::ground::GroundNode>()); }
  catch (const std::exception& error) { RCLCPP_FATAL(rclcpp::get_logger("ground_estimator"), "%s", error.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown();
  return 0;
}
