#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

class GoalApproachLimiter final : public rclcpp::Node
{
public:
  GoalApproachLimiter()
  : Node("r680_goal_approach_limiter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "r680_mapping_floor");
    reaction_delay_ = declare_parameter<double>("reaction_delay", 0.65);
    deceleration_ = declare_parameter<double>("deceleration", 0.80);
    xy_tolerance_ = declare_parameter<double>("xy_tolerance", 0.12);
    minimum_speed_ = declare_parameter<double>("minimum_speed", 0.08);
    maximum_speed_ = declare_parameter<double>("maximum_speed", 1.20);
    goal_timeout_ = declare_parameter<double>("goal_timeout", 1.5);
    tf_max_age_ = declare_parameter<double>("tf_max_age", 0.4);
    const auto path_topic = declare_parameter<std::string>("path_topic", "/plan");
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/r680_nav/cmd_vel_controller");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/cmd_vel_nav");
    const auto limit_topic = declare_parameter<std::string>(
      "limit_topic", "/r680_nav/goal_approach_speed_limit");

    if (base_frame_.empty() || reaction_delay_ < 0.0 || deceleration_ <= 0.0 ||
      xy_tolerance_ <= 0.0 || minimum_speed_ <= 0.0 ||
      minimum_speed_ > maximum_speed_ || goal_timeout_ <= 0.0 || tf_max_age_ <= 0.0)
    {
      throw std::invalid_argument("invalid goal approach limiter parameters");
    }

    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, 1);
    limit_pub_ = create_publisher<std_msgs::msg::Float64>(limit_topic, 1);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr path) {
        if (path->poses.empty()) {return;}
        const auto & last = path->poses.back();
        goal_frame_ = last.header.frame_id.empty() ? path->header.frame_id :
          last.header.frame_id;
        goal_x_ = last.pose.position.x;
        goal_y_ = last.pose.position.y;
        last_goal_ = std::chrono::steady_clock::now();
      });
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic, rclcpp::QoS(1).reliable(),
      std::bind(&GoalApproachLimiter::command_callback, this, std::placeholders::_1));
  }

private:
  void command_callback(geometry_msgs::msg::Twist::ConstSharedPtr command)
  {
    auto output = *command;
    double cap = maximum_speed_;
    const auto now = std::chrono::steady_clock::now();
    const bool fresh_goal = last_goal_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - last_goal_).count() <= goal_timeout_;
    if (fresh_goal && !goal_frame_.empty()) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          goal_frame_, base_frame_, tf2::TimePointZero);
        const double tf_age = (
          get_clock()->now() - rclcpp::Time(transform.header.stamp)).seconds();
        if (tf_age >= -0.05 && tf_age <= tf_max_age_) {
          const double distance = std::hypot(
            goal_x_ - transform.transform.translation.x,
            goal_y_ - transform.transform.translation.y);
          if (std::isfinite(distance)) {
            const double distance_to_brake = std::max(0.0, distance - xy_tolerance_);
            const double delay_speed = deceleration_ * reaction_delay_;
            const double brake_cap = std::sqrt(
              delay_speed * delay_speed +
              2.0 * deceleration_ * distance_to_brake) - delay_speed;
            cap = distance <= xy_tolerance_ ? 0.0 :
              std::clamp(brake_cap, minimum_speed_, maximum_speed_);
            output.linear.x = std::clamp(output.linear.x, -cap, cap);
          }
        }
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Cannot apply terminal speed cap: %s", error.what());
      }
    }
    // Keep angular velocity intact so final yaw alignment is not slowed by Nav2 speed limits.
    command_pub_->publish(output);
    std_msgs::msg::Float64 limit;
    limit.data = cap;
    limit_pub_->publish(limit);
  }

  std::string base_frame_;
  std::string goal_frame_;
  double reaction_delay_{};
  double deceleration_{};
  double xy_tolerance_{};
  double minimum_speed_{};
  double maximum_speed_{};
  double goal_timeout_{};
  double tf_max_age_{};
  double goal_x_{};
  double goal_y_{};
  std::chrono::steady_clock::time_point last_goal_{};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr limit_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalApproachLimiter>());
  rclcpp::shutdown();
  return 0;
}
