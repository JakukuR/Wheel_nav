#include <chrono>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class InterfaceMonitor final : public rclcpp::Node
{
public:
  InterfaceMonitor()
  : Node("r680_navigation_interface_monitor")
  {
    freshness_s_ = declare_parameter<double>("freshness_s", 0.50);
    require_points_ = declare_parameter<bool>("require_obstacle_points", true);
    const auto rgb = declare_parameter<std::string>("rgb_topic", "/r680/d455/color/image_raw");
    const auto depth = declare_parameter<std::string>(
      "depth_topic", "/r680/d455/aligned_depth_to_color/image_raw");
    const auto info = declare_parameter<std::string>(
      "camera_info_topic", "/r680/d455/color/camera_info");
    const auto odom = declare_parameter<std::string>("odom_topic", "/d455_slam/odom");
    const auto points = declare_parameter<std::string>(
      "obstacle_points_topic", "/r680_nav/d455/points");

    const auto qos = rclcpp::SensorDataQoS();
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rgb, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr) {rgb_seen_ = now_steady();});
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr) {depth_seen_ = now_steady();});
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info, qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (msg->k[0] > 0.0 && msg->k[4] > 0.0) {info_seen_ = now_steady();}
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom, qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        if (!msg->header.frame_id.empty() && !msg->child_frame_id.empty()) {odom_seen_ = now_steady();}
      });
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points, qos, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        // A well-formed empty obstacle cloud is a valid clear scene after ground filtering.
        if (!msg->header.frame_id.empty() && msg->point_step > 0) {points_seen_ = now_steady();}
      });
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/r680_nav/localization_ready", 1);
    timer_ = create_wall_timer(100ms, std::bind(&InterfaceMonitor::tick, this));
  }

private:
  using Clock = std::chrono::steady_clock;
  static Clock::time_point now_steady() {return Clock::now();}
  bool fresh(const Clock::time_point & stamp, const Clock::time_point & now) const
  {
    return stamp.time_since_epoch().count() != 0 &&
           std::chrono::duration<double>(now - stamp).count() <= freshness_s_;
  }
  void tick()
  {
    const auto stamp = now_steady();
    std_msgs::msg::Bool ready;
    ready.data = fresh(rgb_seen_, stamp) && fresh(depth_seen_, stamp) &&
      fresh(info_seen_, stamp) && fresh(odom_seen_, stamp) &&
      (!require_points_ || fresh(points_seen_, stamp));
    ready_pub_->publish(ready);
  }

  double freshness_s_{0.5};
  bool require_points_{true};
  Clock::time_point rgb_seen_{}, depth_seen_{}, info_seen_{}, odom_seen_{}, points_seen_{};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_, depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterfaceMonitor>());
  rclcpp::shutdown();
  return 0;
}
