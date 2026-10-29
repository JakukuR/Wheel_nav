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
    require_chassis_odom_ = declare_parameter<bool>("require_chassis_odom", false);
    require_vo_watchdog_ = declare_parameter<bool>("require_vo_watchdog", false);
    require_vio_health_ = declare_parameter<bool>("require_vio_health", false);
    require_motion_permission_=declare_parameter<bool>("require_motion_permission",false);
    allow_bounded_degraded_=declare_parameter<bool>("allow_bounded_degraded",false);
    motion_permission_sub_=create_subscription<std_msgs::msg::Bool>(
      declare_parameter<std::string>("motion_permission_topic","/r680_nav/continuous_odom_healthy"),
      1,[this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        motion_permission_=msg->data;motion_permission_seen_=now_steady();
      });
    degraded_sub_=create_subscription<std_msgs::msg::Bool>("/r680_nav/odom_degraded",1,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        degraded_=msg->data;degraded_seen_=now_steady();
      });
    require_mapping_odom_ = declare_parameter<bool>("require_mapping_odom", false);
    const auto rgb = declare_parameter<std::string>("rgb_topic", "/r680/d455/color/image_raw");
    const auto depth = declare_parameter<std::string>(
      "depth_topic", "/r680/d455/aligned_depth_to_color/image_raw");
    const auto info = declare_parameter<std::string>(
      "camera_info_topic", "/r680/d455/color/camera_info");
    const auto odom = declare_parameter<std::string>("odom_topic", "/d455_slam/odom");
    const auto points = declare_parameter<std::string>(
      "obstacle_points_topic", "/r680_nav/d455/points");
    const auto chassis_odom = declare_parameter<std::string>(
      "chassis_odom_topic", "/wheel/odom");

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
    chassis_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      chassis_odom, qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        if (!msg->header.frame_id.empty() && !msg->child_frame_id.empty()) {
          chassis_odom_seen_ = now_steady();
        }
      });
    vo_watchdog_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/r680_nav/vo_watchdog_healthy", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        vo_watchdog_healthy_ = msg->data;
        vo_watchdog_seen_ = now_steady();
      });
    vo_odometry_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/r680_nav/vo_odometry_healthy", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        vo_odometry_healthy_ = msg->data;
        vo_odometry_seen_ = now_steady();
      });
    vio_health_sub_ = create_subscription<std_msgs::msg::Bool>(
      declare_parameter<std::string>("vio_health_topic", "/r680_nav/vio_tracking_healthy"),
      1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        vio_healthy_ = msg->data; vio_seen_ = now_steady();
      });
    mapping_health_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/r680_nav/mapping_odom_ready", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        mapping_healthy_ = msg->data; mapping_seen_ = now_steady();
      });
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/r680_nav/localization_ready", 1);
    // Same inputs without the verified map match: odometry is enough to move
    // mapless (navigation_map_mode) while relocalization continues.
    motion_pub_ = create_publisher<std_msgs::msg::Bool>("/r680_nav/motion_ready", 1);
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
    const bool common = fresh(rgb_seen_, stamp) && fresh(depth_seen_, stamp) &&
      fresh(info_seen_, stamp) && fresh(odom_seen_, stamp) &&
      (!require_points_ || fresh(points_seen_, stamp)) &&
      (!require_chassis_odom_ || fresh(chassis_odom_seen_, stamp)) &&
      (!require_vio_health_ || (vio_healthy_ && fresh(vio_seen_, stamp)) ||
        (allow_bounded_degraded_ && require_motion_permission_ && degraded_ &&
         fresh(degraded_seen_, stamp) && motion_permission_ && fresh(motion_permission_seen_, stamp))) &&
      (!require_motion_permission_ || (motion_permission_ && fresh(motion_permission_seen_, stamp))) &&
      (!require_mapping_odom_ || (mapping_healthy_ && fresh(mapping_seen_, stamp)));
    std_msgs::msg::Bool ready, motion;
    ready.data = common &&
      (!require_vo_watchdog_ || (vo_watchdog_healthy_ && fresh(vo_watchdog_seen_, stamp)));
    motion.data = common &&
      (!require_vo_watchdog_ || (vo_odometry_healthy_ && fresh(vo_odometry_seen_, stamp)));
    ready_pub_->publish(ready);
    motion_pub_->publish(motion);
  }

  bool require_motion_permission_{false}, motion_permission_{false};
  bool allow_bounded_degraded_{false}, degraded_{false};
  Clock::time_point motion_permission_seen_{},degraded_seen_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_permission_sub_,degraded_sub_;
  double freshness_s_{0.5};
  bool require_points_{true};
  bool require_chassis_odom_{false};
  bool require_vo_watchdog_{false};
  bool vo_watchdog_healthy_{false}, vo_odometry_healthy_{false};
  Clock::time_point vo_odometry_seen_{};
  bool require_vio_health_{false}, vio_healthy_{false};
  Clock::time_point vio_seen_{};
  bool require_mapping_odom_{false}, mapping_healthy_{false};
  Clock::time_point mapping_seen_{};
  Clock::time_point rgb_seen_{}, depth_seen_{}, info_seen_{}, odom_seen_{}, points_seen_{};
  Clock::time_point chassis_odom_seen_{}, vo_watchdog_seen_{};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_, depth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr chassis_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vo_watchdog_sub_, vo_odometry_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vio_health_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mapping_health_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_, motion_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterfaceMonitor>());
  rclcpp::shutdown();
  return 0;
}
