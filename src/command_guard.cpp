#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "wla_r680_navigation/vio_init_policy.hpp"
#include "wla_r680_navigation/degraded_odometry.hpp"

using namespace std::chrono_literals;

class CommandGuard final : public rclcpp::Node
{
public:
  CommandGuard()
  : Node("r680_command_guard")
  {
    timeout_s_ = declare_parameter<double>("timeout_s", 0.30);
    health_timeout_s_ = declare_parameter<double>("health_timeout_s", 0.50);
    forward_max_ = declare_parameter<double>("forward_max", 1.20);
    reverse_max_ = declare_parameter<double>("reverse_max", 0.25);
    angular_max_ = declare_parameter<double>("angular_max", 1.50);
    hardware_output_enabled_ = declare_parameter<bool>("hardware_output_enabled", false);
    require_mission_permission_ = declare_parameter<bool>("require_mission_permission", true);
    init_mode_enabled_ = declare_parameter<bool>("initialization_mode_enabled", false);
    degraded_enabled_ = declare_parameter<bool>("degraded_mode_enabled", false);
    degraded_v_ = declare_parameter<double>("degraded_linear_max", 0.15);
    degraded_w_ = declare_parameter<double>("degraded_angular_max", 0.30);
    if(!std::isfinite(degraded_v_+degraded_w_) || degraded_v_<=0 || degraded_v_>0.2 ||
      degraded_w_<=0 || degraded_w_>0.4) throw std::invalid_argument("invalid degraded motion envelope");
    if(degraded_enabled_) degraded_sub_=create_subscription<std_msgs::msg::Bool>(
      "/r680_nav/odom_degraded", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        degraded_=msg->data;degraded_seen_=std::chrono::steady_clock::now();
      });
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/r680_nav/cmd_vel_collision_checked");
    const auto health_topic = declare_parameter<std::string>(
      "health_topic", "/r680_nav/localization_ready");
    const auto permission_topic = declare_parameter<std::string>(
      "mission_permission_topic", "/r680_nav/mission_motion_allowed");
    const auto raw_topic = declare_parameter<std::string>(
      "raw_output_topic", "/r680_nav/chassis_cmd_vel");
    const auto preview_topic = declare_parameter<std::string>(
      "preview_output_topic", "/r680_nav/cmd_vel_safe_preview");

    for (const auto value : {timeout_s_, health_timeout_s_, forward_max_, reverse_max_, angular_max_}) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("command guard limits must be finite and positive");
      }
    }

    raw_pub_ = create_publisher<geometry_msgs::msg::Twist>(raw_topic, 1);
    preview_pub_ = create_publisher<geometry_msgs::msg::Twist>(preview_topic, 1);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic, 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        command_ = *msg;
        command_received_ = std::chrono::steady_clock::now();
      });
    health_sub_ = create_subscription<std_msgs::msg::Bool>(
      health_topic, 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        healthy_ = msg->data;
        health_received_ = std::chrono::steady_clock::now();
      });
    permission_sub_ = create_subscription<std_msgs::msg::Bool>(
      permission_topic, 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        mission_allowed_ = msg->data;
        permission_received_ = std::chrono::steady_clock::now();
      });
    timer_ = create_wall_timer(20ms, std::bind(&CommandGuard::tick, this));
    if (init_mode_enabled_) {
      init_state_sub_ = create_subscription<std_msgs::msg::String>(
        "/r680_nav/vio_init_state", 1, [this](std_msgs::msg::String::ConstSharedPtr msg) {
          init_state_ = msg->data; init_state_received_ = std::chrono::steady_clock::now();
        });
      init_permit_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/r680_nav/vio_init_permit", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          init_permitted_ = msg->data; init_permit_received_ = std::chrono::steady_clock::now();
        });
      init_request_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/r680_nav/vio_init_request", 1, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
          init_request_ = *msg; init_request_received_ = std::chrono::steady_clock::now();
        });
    }

    RCLCPP_WARN(
      get_logger(), "raw chassis output is %s; raw topic is %s",
      hardware_output_enabled_ ? "ENABLED" : "DISABLED", raw_topic.c_str());
  }

private:
  void tick()
  {
    const auto now = std::chrono::steady_clock::now();
    const auto command_age = std::chrono::duration<double>(now - command_received_).count();
    const auto health_age = std::chrono::duration<double>(now - health_received_).count();
    const auto permission_age = std::chrono::duration<double>(now - permission_received_).count();
    const bool mission_permitted = !require_mission_permission_ ||
      (mission_allowed_ && permission_age <= health_timeout_s_);
    const bool permitted = healthy_ && mission_permitted &&
      command_age <= timeout_s_ && health_age <= health_timeout_s_ &&
      (!init_mode_enabled_ || (init_state_ == "succeeded" &&
      std::chrono::duration<double>(now - init_state_received_).count() <= 0.20));

    geometry_msgs::msg::Twist output;
    if (permitted && std::isfinite(command_.linear.x) && std::isfinite(command_.angular.z)) {
      output.linear.x = std::clamp(command_.linear.x, -reverse_max_, forward_max_);
      output.angular.z = std::clamp(command_.angular.z, -angular_max_, angular_max_);
    }
    if (init_mode_enabled_ && init_state_ == "moving") {
      const double stamp_age = (get_clock()->now() - rclcpp::Time(init_request_.header.stamp)).seconds();
      const bool bootstrap_permitted = init_permitted_ &&
        std::chrono::duration<double>(now - init_state_received_).count() <= 0.20 &&
        std::chrono::duration<double>(now - init_permit_received_).count() <= 0.20 &&
        std::chrono::duration<double>(now - init_request_received_).count() <= 0.20 &&
        stamp_age >= -0.1 && stamp_age <= 0.20 &&
        init_request_.header.frame_id == "r680_mapping_floor" && command_age <= 0.20;
      if (bootstrap_permitted) {
        output.linear.x = std::max(0.0, wla_r680_navigation::bound_init_component(
          command_.linear.x, init_request_.twist.linear.x, 0.08));
        output.angular.z = wla_r680_navigation::bound_init_component(
          command_.angular.z, init_request_.twist.angular.z, 0.20);
      }
    }
    if(degraded_enabled_) {
      if(std::chrono::duration<double>(now-degraded_seen_).count()>0.20) output=geometry_msgs::msg::Twist();
      else if(degraded_) {
        // Preserve requested curvature; only reduce the collision-checked command.
        const double scale=wla_r680_navigation::degradedCommandScale(
          output.linear.x,output.angular.z,degraded_v_,degraded_w_);
        output.linear.x*=scale;output.angular.z*=scale;
      }
    }
    preview_pub_->publish(output);
    if (hardware_output_enabled_) {
      raw_pub_->publish(output);
    }
  }

  double timeout_s_{};
  double health_timeout_s_{};
  double forward_max_{};
  double reverse_max_{};
  double angular_max_{};
  bool hardware_output_enabled_{false};
  bool degraded_enabled_{false},degraded_{false};double degraded_v_{},degraded_w_{};
  std::chrono::steady_clock::time_point degraded_seen_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr degraded_sub_;
  bool init_mode_enabled_{false}, init_permitted_{false};
  std::string init_state_;
  geometry_msgs::msg::TwistStamped init_request_;
  std::chrono::steady_clock::time_point init_state_received_{}, init_permit_received_{}, init_request_received_{};
  bool require_mission_permission_{true};
  bool mission_allowed_{false};
  bool healthy_{false};
  geometry_msgs::msg::Twist command_{};
  std::chrono::steady_clock::time_point command_received_{};
  std::chrono::steady_clock::time_point health_received_{};
  std::chrono::steady_clock::time_point permission_received_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr preview_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr init_permit_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr init_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr init_request_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandGuard>());
  rclcpp::shutdown();
  return 0;
}
