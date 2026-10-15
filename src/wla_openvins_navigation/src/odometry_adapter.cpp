#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "wla_openvins_navigation/adapter.hpp"

// Interface conversion only. Estimation, image scheduling and initialization
// belong to the unmodified upstream run_subscribe_msckf executable.
class OdometryAdapter final : public rclcpp::Node {
public:
  OdometryAdapter() : Node("openvins_odometry_adapter") {
    world_ = declare_parameter<std::string>("odom_frame", "d455_floor_odom");
    base_ = declare_parameter<std::string>("base_frame", "r680_mapping_floor");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    const auto p = declare_parameter<std::vector<double>>("p_base_in_imu", {0., 0., 0.});
    const auto q = declare_parameter<std::vector<double>>("q_base_to_imu_xyzw", {0., 0., 0., 1.});
    if (p.size() != 3 || q.size() != 4) throw std::invalid_argument("invalid T_imu_base size");
    Eigen::Quaterniond rotation(q[3], q[0], q[1], q[2]);
    if (!rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1.) > 1e-4)
      throw std::invalid_argument("invalid T_imu_base rotation");
    T_IB_.setIdentity();
    T_IB_.linear() = rotation.toRotationMatrix();
    T_IB_.translation() = Eigen::Map<const Eigen::Vector3d>(p.data());
    if (!T_IB_.translation().allFinite()) throw std::invalid_argument("invalid T_imu_base translation");
    output_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    input_ = create_subscription<nav_msgs::msg::Odometry>("odomimu", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { convert(*m); });
  }

private:
  void convert(const nav_msgs::msg::Odometry &m) {
    // Upstream pose covariance is [position_G, attitude_error_I]; twist is
    // [linear_velocity_I, angular_velocity_I]. Undo only the published ordering
    // before applying the existing rigid IMU-to-base transform.
    wla_openvins::State x;
    const auto &q = m.pose.pose.orientation;
    const auto &p = m.pose.pose.position;
    const auto &v = m.twist.twist.linear;
    const auto &w = m.twist.twist.angular;
    x << q.x, q.y, q.z, q.w, p.x, p.y, p.z, v.x, v.y, v.z, w.x, w.y, w.z;
    wla_openvins::Covariance covariance = wla_openvins::Covariance::Zero();
    for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) {
      covariance((r + 3) % 6, (c + 3) % 6) = m.pose.covariance[6*r+c];
      covariance(r+6, c+6) = m.twist.covariance[6*r+c];
    }
    try {
      const auto b = wla_openvins::adapt(x, covariance, T_IB_);
      nav_msgs::msg::Odometry out;
      out.header = m.header;  // Preserve the actual upstream sample timestamp.
      out.header.frame_id = world_;
      out.child_frame_id = base_;
      auto &op = out.pose.pose.position;
      op.x = b.p.x(); op.y = b.p.y(); op.z = b.p.z();
      auto &oq = out.pose.pose.orientation;
      oq.x = b.q.x(); oq.y = b.q.y(); oq.z = b.q.z(); oq.w = b.q.w();
      auto &ov = out.twist.twist.linear;
      ov.x = b.v.x(); ov.y = b.v.y(); ov.z = b.v.z();
      auto &ow = out.twist.twist.angular;
      ow.x = b.w.x(); ow.y = b.w.y(); ow.z = b.w.z();
      for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) {
        out.pose.covariance[6*r+c] = b.covariance(r,c);
        out.twist.covariance[6*r+c] = b.covariance(r+6,c+6);
      }
      output_->publish(out);
      if (publish_tf_) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = out.header;
        transform.child_frame_id = base_;
        transform.transform.translation.x = op.x;
        transform.transform.translation.y = op.y;
        transform.transform.translation.z = op.z;
        transform.transform.rotation = oq;
        tf_->sendTransform(transform);
      }
    } catch (const std::exception &e) {
      // A malformed message cannot be converted; never latch or reset OpenVINS.
      RCLCPP_ERROR(get_logger(), "Cannot transform upstream odometry: %s", e.what());
    }
  }
  std::string world_, base_;
  bool publish_tf_;
  Eigen::Isometry3d T_IB_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr input_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryAdapter>());
  rclcpp::shutdown();
}
