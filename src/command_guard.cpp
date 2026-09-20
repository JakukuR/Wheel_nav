#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class CommandGuard final : public rclcpp::Node
{
public:
  CommandGuard()
  : Node("r680_command_guard")
  {
    timeout_s_ = declare_parameter<double>("timeout_s", 0.30);
    health_timeout_s_ = declare_parameter<double>("health_timeout_s", 0.50);
    forward_max_ = declare_parameter<double>("forward_max", 1.00);
    reverse_max_ = declare_parameter<double>("reverse_max", 0.25);
    angular_max_ = declare_parameter<double>("angular_max", 1.50);
    hardware_output_enabled_ = declare_parameter<bool>("hardware_output_enabled", false);
    require_mission_permission_ = declare_parameter<bool>("require_mission_permission", true);
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
      command_age <= timeout_s_ && health_age <= health_timeout_s_;

    geometry_msgs::msg::Twist output;
    if (permitted && std::isfinite(command_.linear.x) && std::isfinite(command_.angular.z)) {
      output.linear.x = std::clamp(command_.linear.x, -reverse_max_, forward_max_);
      output.angular.z = std::clamp(command_.angular.z, -angular_max_, angular_max_);
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
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandGuard>());
  rclcpp::shutdown();
  return 0;
}
