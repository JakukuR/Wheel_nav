#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <unistd.h>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rtabmap_msgs/msg/odom_info.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class VoWatchdog final : public rclcpp::Node
{
public:
  VoWatchdog() : Node("r680_vo_watchdog"), started_(Clock::now())
  {
    startup_grace_s_ = declare_parameter<double>("startup_grace_s", 12.0);
    stale_s_ = declare_parameter<double>("stale_s", 1.5);
    lost_limit_ = declare_parameter<int>("lost_limit", 3);
    jump_linear_mps_ = declare_parameter<double>("jump_linear_mps", 2.5);
    jump_angular_rps_ = declare_parameter<double>("jump_angular_rps", 3.0);
    wheel_window_s_ = declare_parameter<double>("wheel_window_s", 0.5);
    wheel_translation_error_m_ = declare_parameter<double>("wheel_translation_error_m", 0.45);
    wheel_yaw_error_rad_ = declare_parameter<double>("wheel_yaw_error_rad", 0.65);
    wheel_mismatch_limit_ = declare_parameter<int>("wheel_mismatch_limit", 3);
    compare_wheel_yaw_ = declare_parameter<bool>("compare_wheel_yaw", false);
    restart_enabled_ = declare_parameter<bool>("restart_enabled", true);
    const auto vo_topic = declare_parameter<std::string>("vo_topic", "/r680_nav/vo_odom");
    const auto info_topic = declare_parameter<std::string>("info_topic", "/d455_slam/odom_info");
    const auto wheel_topic = declare_parameter<std::string>("wheel_topic", "/wheel/odom");

    if (startup_grace_s_ <= 0 || stale_s_ <= 0 || lost_limit_ < 1 ||
      jump_linear_mps_ <= 0 || jump_angular_rps_ <= 0 || wheel_window_s_ <= 0 ||
      wheel_translation_error_m_ <= 0 || wheel_yaw_error_rad_ <= 0 ||
      wheel_mismatch_limit_ < 1) {
      throw std::invalid_argument("invalid VO watchdog thresholds");
    }

    health_pub_ = create_publisher<std_msgs::msg::Bool>("/r680_nav/vo_watchdog_healthy", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/r680_nav/vo_watchdog_status", 10);
    vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      vo_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onVo(*msg);});
    info_sub_ = create_subscription<rtabmap_msgs::msg::OdomInfo>(
      info_topic, rclcpp::SensorDataQoS(),
      [this](rtabmap_msgs::msg::OdomInfo::ConstSharedPtr msg) {onInfo(*msg);});
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        if (finitePose(*msg)) {
          wheel_ = poseOf(*msg);
          wheel_seen_ = Clock::now();
        }
      });
    timer_ = create_wall_timer(100ms, [this]() {tick();});
    RCLCPP_INFO(get_logger(), "VO watchdog armed; restart=%s", restart_enabled_ ? "on" : "off");
  }

private:
  using Clock = std::chrono::steady_clock;
  struct Pose {double x, y, yaw;};

  static bool finitePose(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    return std::isfinite(p.x) && std::isfinite(p.y) &&
           std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z) && std::isfinite(q.w) &&
           (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) > 0.5;
  }

  static Pose poseOf(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    return {p.x, p.y, std::atan2(2.0*(q.w*q.z + q.x*q.y),
      1.0 - 2.0*(q.y*q.y + q.z*q.z))};
  }

  static double angleDifference(double a, double b)
  {
    return std::atan2(std::sin(a-b), std::cos(a-b));
  }

  static double distance(const Pose & a, const Pose & b)
  {
    return std::hypot(a.x-b.x, a.y-b.y);
  }

  static double age(const Clock::time_point & t)
  {
    if (t.time_since_epoch().count() == 0) {return 1e9;}
    return std::chrono::duration<double>(Clock::now()-t).count();
  }

  void onInfo(const rtabmap_msgs::msg::OdomInfo & msg)
  {
    if (fault_latched_) {return;}
    info_seen_ = Clock::now();
    lost_count_ = msg.lost ? lost_count_ + 1 : 0;
    if (lost_count_ >= lost_limit_) {fault("visual tracking lost");}
  }

  void onVo(const nav_msgs::msg::Odometry & msg)
  {
    if (fault_latched_) {return;}
    if (!finitePose(msg)) {fault("non-finite VO pose"); return;}
    const auto stamp = rclcpp::Time(msg.header.stamp);
    const auto pose = poseOf(msg);
    if (last_vo_) {
      const double dt = (stamp - last_stamp_).seconds();
      if (dt > 0.0 && dt < 0.5) {
        if (distance(pose, *last_vo_) > 0.15 + jump_linear_mps_*dt ||
          std::abs(angleDifference(pose.yaw, last_vo_->yaw)) > 0.20 + jump_angular_rps_*dt) {
          fault("VO pose discontinuity");
          return;
        }
      }
    }
    vo_seen_ = Clock::now();
    last_vo_ = pose;
    last_stamp_ = stamp;

    // Compare movement over windows, not single frames; wheel odometry is an
    // independent diagnostic, not a replacement for camera localization.
    if (age(wheel_seen_) > 0.30) {window_vo_.reset(); mismatch_count_ = 0; return;}
    if (!window_vo_) {
      window_vo_ = pose;
      window_wheel_ = wheel_;
      window_started_ = Clock::now();
      return;
    }
    if (age(window_started_) < wheel_window_s_) {return;}
    const bool mismatch =
      std::abs(distance(pose, *window_vo_) - distance(wheel_, window_wheel_)) >
        wheel_translation_error_m_ ||
      (compare_wheel_yaw_ &&
       std::abs(angleDifference(
         angleDifference(pose.yaw, window_vo_->yaw),
         angleDifference(wheel_.yaw, window_wheel_.yaw))) > wheel_yaw_error_rad_);
    mismatch_count_ = mismatch ? mismatch_count_ + 1 : 0;
    window_vo_ = pose;
    window_wheel_ = wheel_;
    window_started_ = Clock::now();
    if (mismatch_count_ >= wheel_mismatch_limit_) {fault("VO/wheel motion disagreement");}
  }

  static std::vector<pid_t> voPids()
  {
    std::vector<pid_t> result;
    for (const auto & entry : fs::directory_iterator("/proc")) {
      const auto name = entry.path().filename().string();
      if (name.empty() || !std::all_of(name.begin(), name.end(), ::isdigit)) {continue;}
      std::error_code ec;
      const auto exe = fs::read_symlink(entry.path() / "exe", ec).string();
      if (ec || exe.find("/rtabmap_odom/rgbd_odometry") == std::string::npos) {continue;}
      std::ifstream command(entry.path() / "cmdline", std::ios::binary);
      std::string args((std::istreambuf_iterator<char>(command)), std::istreambuf_iterator<char>());
      if (args.find("__ns:=/d455_vo") != std::string::npos &&
          args.find("__node:=rgbd_odometry") != std::string::npos) {
        result.push_back(static_cast<pid_t>(std::stoi(name)));
      }
    }
    return result;
  }

  void fault(const std::string & why)
  {
    if (fault_latched_) {return;}
    fault_latched_ = true;
    reason_ = why;
    std_msgs::msg::Bool unhealthy;
    unhealthy.data = false;
    health_pub_->publish(unhealthy);
    RCLCPP_ERROR(get_logger(), "VO fault: %s; motion stays locked until full bringup restart",
      why.c_str());
    if (!restart_enabled_) {return;}
    const auto pids = voPids();
    if (pids.size() != 1) {
      RCLCPP_ERROR(get_logger(), "expected exactly one /d455_vo/rgbd_odometry process, found %zu",
        pids.size());
      return;
    }
    vo_pid_ = pids.front();
    if (::kill(vo_pid_, SIGTERM) == 0) {
      RCLCPP_WARN(get_logger(), "sent SIGTERM to VO pid %d; launch will respawn it", vo_pid_);
    } else {
      RCLCPP_ERROR(get_logger(), "could not signal VO pid %d", vo_pid_);
    }
  }

  void tick()
  {
    if (!fault_latched_ && age(started_) > startup_grace_s_ &&
      (age(vo_seen_) > stale_s_ || age(info_seen_) > stale_s_)) {
      fault("VO odometry or tracking status timed out");
    }
    const bool healthy = !fault_latched_ && age(vo_seen_) <= stale_s_ &&
      age(info_seen_) <= stale_s_ && lost_count_ == 0;
    std_msgs::msg::Bool health;
    health.data = healthy;
    health_pub_->publish(health);
    std_msgs::msg::String status;
    status.data = fault_latched_ ? "fault_latched: " + reason_ :
      (healthy ? "healthy" : "waiting_for_vo");
    status_pub_->publish(status);
  }

  double startup_grace_s_{}, stale_s_{}, jump_linear_mps_{}, jump_angular_rps_{};
  double wheel_window_s_{}, wheel_translation_error_m_{}, wheel_yaw_error_rad_{};
  int lost_limit_{}, wheel_mismatch_limit_{}, lost_count_{0}, mismatch_count_{0};
  bool compare_wheel_yaw_{false}, restart_enabled_{true}, fault_latched_{false};
  pid_t vo_pid_{-1};
  std::string reason_;
  Clock::time_point started_, vo_seen_{}, info_seen_{}, wheel_seen_{}, window_started_{};
  Pose wheel_{}, window_wheel_{};
  std::optional<Pose> last_vo_, window_vo_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_, wheel_sub_;
  rclcpp::Subscription<rtabmap_msgs::msg::OdomInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoWatchdog>());
  rclcpp::shutdown();
  return 0;
}
