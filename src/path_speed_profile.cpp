#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

namespace
{
struct Point2
{
  double x;
  double y;
};

double distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double curvature(const Point2 & a, const Point2 & b, const Point2 & c)
{
  const double ab = distance(a, b);
  const double bc = distance(b, c);
  const double ac = distance(a, c);
  const double denominator = ab * bc * ac;
  if (denominator < 1.0e-6) {return 0.0;}
  const double cross = (b.x - a.x) * (c.y - a.y) -
    (b.y - a.y) * (c.x - a.x);
  return 2.0 * std::abs(cross) / denominator;
}
}  // namespace

class PathSpeedProfile final : public rclcpp::Node
{
public:
  PathSpeedProfile()
  : Node("r680_path_speed_profile")
  {
    max_speed_ = declare_parameter<double>("max_speed", 1.2);
    min_curve_speed_ = declare_parameter<double>("min_curve_speed", 0.18);
    lateral_acceleration_max_ = declare_parameter<double>("lateral_acceleration_max", 0.65);
    angular_velocity_max_ = declare_parameter<double>("angular_velocity_max", 1.50);
    deceleration_max_ = declare_parameter<double>("deceleration_max", 0.90);
    sample_distance_ = declare_parameter<double>("sample_distance", 0.10);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 2.0);
    goal_stop_distance_ = declare_parameter<double>("goal_stop_distance", 0.35);
    path_timeout_ = declare_parameter<double>("path_timeout", 0.50);
    const double publish_rate = declare_parameter<double>("publish_rate", 10.0);
    const auto path_topic = declare_parameter<std::string>("path_topic", "/local_plan");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/speed_limit");
    const auto debug_topic = declare_parameter<std::string>(
      "debug_topic", "/r680_nav/path_speed_limit");

    if (max_speed_ <= 0.0 || min_curve_speed_ <= 0.0 ||
      min_curve_speed_ > max_speed_ || lateral_acceleration_max_ <= 0.0 ||
      angular_velocity_max_ <= 0.0 || deceleration_max_ <= 0.0 ||
      sample_distance_ <= 0.0 || lookahead_distance_ <= sample_distance_ ||
      goal_stop_distance_ < 0.0 || path_timeout_ <= 0.0 || publish_rate <= 0.0)
    {
      throw std::invalid_argument("invalid path speed profile parameters");
    }

    speed_limit_pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>(output_topic, 1);
    debug_pub_ = create_publisher<std_msgs::msg::Float64>(debug_topic, 1);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(1).reliable(),
      std::bind(&PathSpeedProfile::path_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate)),
      std::bind(&PathSpeedProfile::publish_limit, this));
  }

private:
  void path_callback(nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    current_limit_ = compute_limit(*msg);
    last_path_ = std::chrono::steady_clock::now();
  }

  double compute_limit(const nav_msgs::msg::Path & path) const
  {
    std::vector<Point2> points;
    std::vector<double> arc;
    points.reserve(path.poses.size());
    arc.reserve(path.poses.size());
    bool includes_goal = false;
    for (size_t pose_index = 0; pose_index < path.poses.size(); ++pose_index) {
      const auto & pose = path.poses[pose_index];
      const Point2 point{pose.pose.position.x, pose.pose.position.y};
      if (points.empty()) {
        points.push_back(point);
        arc.push_back(0.0);
        includes_goal = path.poses.size() == 1U;
        continue;
      }
      const bool is_last_pose = pose_index + 1U == path.poses.size();
      const double step = distance(points.back(), point);
      if (step < sample_distance_ && !is_last_pose) {continue;}
      const double next_arc = arc.back() + step;
      points.push_back(point);
      arc.push_back(next_arc);
      includes_goal = is_last_pose;
      if (next_arc >= lookahead_distance_ && !is_last_pose) {break;}
    }
    if (points.size() < 3U) {return max_speed_;}

    std::vector<double> profile(points.size(), max_speed_);
    for (size_t i = 1; i + 1 < points.size(); ++i) {
      const double kappa = curvature(points[i - 1], points[i], points[i + 1]);
      if (kappa < 1.0e-3) {continue;}
      const double curve_limit = std::min(
        angular_velocity_max_ / kappa,
        std::sqrt(lateral_acceleration_max_ / kappa));
      profile[i] = std::clamp(curve_limit, min_curve_speed_, max_speed_);
    }
    profile.front() = profile[1];
    profile.back() = profile[profile.size() - 2U];

    if (includes_goal && arc.back() <= goal_stop_distance_) {
      profile.back() = 0.0;
    }
    for (size_t i = profile.size() - 1U; i > 0U; --i) {
      const double ds = std::max(0.0, arc[i] - arc[i - 1U]);
      const double braking_limit = std::sqrt(
        profile[i] * profile[i] + 2.0 * deceleration_max_ * ds);
      profile[i - 1U] = std::min(profile[i - 1U], braking_limit);
    }
    return std::clamp(profile.front(), min_curve_speed_, max_speed_);
  }

  void publish_limit()
  {
    const auto now = std::chrono::steady_clock::now();
    const bool fresh = last_path_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - last_path_).count() <= path_timeout_;
    nav2_msgs::msg::SpeedLimit limit;
    limit.header.stamp = get_clock()->now();
    limit.percentage = false;
    limit.speed_limit = fresh ? current_limit_ : 0.0;
    speed_limit_pub_->publish(limit);
    std_msgs::msg::Float64 debug;
    debug.data = limit.speed_limit;
    debug_pub_->publish(debug);
  }

  double max_speed_{};
  double min_curve_speed_{};
  double lateral_acceleration_max_{};
  double angular_velocity_max_{};
  double deceleration_max_{};
  double sample_distance_{};
  double lookahead_distance_{};
  double goal_stop_distance_{};
  double path_timeout_{};
  double current_limit_{0.0};
  std::chrono::steady_clock::time_point last_path_{};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathSpeedProfile>());
  rclcpp::shutdown();
  return 0;
}
